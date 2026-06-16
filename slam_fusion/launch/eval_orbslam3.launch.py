"""
=============================================================================
@Project : Semantic SLAM Evaluation Framework
@Desc    : Evaluation framework for comparing Visual vs LIDAR SLAM 
           algorithms (ORB-SLAM3, RTAB-Map, Cartographer) augmented 
           with zero-shot semantic segmentation (SAM2 / DeepLabV3).
=============================================================================
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sam_sim_dir     = get_package_share_directory('sam_simulation')
    slam_fusion_dir = get_package_share_directory('slam_fusion')

    run_mode_arg = DeclareLaunchArgument(
        'run_mode',
        default_value='dataset',
        description='Mode to run: dataset or simulation'
    )

    perception_model_arg = DeclareLaunchArgument(
        'perception_model',
        default_value='sam2',
        description='Perception model to use: sam2 or deeplabv3'
    )

    use_sim_time = PythonExpression(["'true' if '", LaunchConfiguration('run_mode'), "' == 'simulation' else 'false'"])

    default_dataset_path = '/workspace/datasets/TUM/rgbd_dataset_freiburg1_desk'
    if not os.path.exists(default_dataset_path):
        default_dataset_path = os.path.expanduser('~/sam_slam_ws/datasets/TUM/rgbd_dataset_freiburg1_desk')

    # ==========================
    # DATA SOURCES
    # ==========================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sam_sim_dir, 'launch', 'gazebo_indoor.launch.py')
        ),
        condition=LaunchConfigurationEquals('run_mode', 'simulation')
    )

    dataset_player = TimerAction(
        period=5.0,
        condition=LaunchConfigurationEquals('run_mode', 'dataset'),
        actions=[Node(
            package='sam_perception',
            executable='dataset_node',
            name='dataset_publisher',
            output='screen',
            prefix='xterm -T "Dataset Publisher" -hold -e',
            parameters=[{
                'dataset_path': default_dataset_path,
                'dataset_type': 'tum_fr1',
                'fps': 5.0,
                'loop': True
            }]
        )]
    )

    # ==========================
    # PERCEPTION
    # ==========================
    sam2_node = Node(
        package='sam_perception',
        executable='sam2_node',
        name='sam2_perception',
        output='screen',
        prefix='xterm -T "SAM2 Perception" -hold -e',
        parameters=[{'keyframe_interval': 3, 'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('perception_model', 'sam2')
    )

    deeplabv3_node = Node(
        package='sam_perception',
        executable='deeplabv3_node',
        name='deeplabv3_perception',
        output='screen',
        prefix='xterm -T "DeepLabV3 Perception" -hold -e',
        parameters=[{'keyframe_interval': 3, 'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('perception_model', 'deeplabv3')
    )

    # ==========================
    # SLAM BACKEND: ORB-SLAM3
    # ==========================
    orbslam3_dataset_node = Node(
        package='orbslam3',
        executable='rgbd',
        name='orb_slam3_rgbd',
        output='screen',
        prefix='xterm -T "ORB-SLAM3 Dataset" -hold -e',
        arguments=[
            '/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            '/ros2_ws/src/ORB_SLAM3/Examples/RGB-D/TUM1.yaml'
        ],
        remappings=[
            ('camera/rgb',   '/camera/color/image_raw'),
            ('camera/depth', '/camera/depth/image_raw')
        ],
        parameters=[{'use_sim_time': False}],
        condition=LaunchConfigurationEquals('run_mode', 'dataset')
    )

    orbslam3_sim_node = Node(
        package='orbslam3',
        executable='rgbd',
        name='orb_slam3_rgbd',
        output='screen',
        prefix='xterm -T "ORB-SLAM3 Sim" -hold -e',
        arguments=[
            '/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            '/ros2_ws/src/slam_fusion/config/g1_gazebo_camera.yaml'
        ],
        remappings=[
            ('camera/rgb',   '/camera/color/image_raw'),
            ('camera/depth', '/camera/depth/image_raw')
        ],
        parameters=[{'use_sim_time': True}],
        condition=LaunchConfigurationEquals('run_mode', 'simulation')
    )

    # ==========================
    # FUSION & EVALUATION
    # ==========================
    fusion_node = Node(
        package='slam_fusion',
        executable='semantic_fusion_node',
        name='semantic_fusion',
        output='screen',
        prefix='xterm -T "Semantic Fusion" -hold -e',
        parameters=[{
            'voxel_leaf_size': 0.05,
            'pixel_step': 2,
            'depth_min': 0.1,
            'depth_max': 10.0,
            'use_sim_time': use_sim_time,
        }]
    )

    traj_filename = ['/ros2_ws/orbslam3_', LaunchConfiguration('perception_model'), '_estimate.txt']
    csv_filename = ['/ros2_ws/orbslam3_', LaunchConfiguration('perception_model'), '_performance.csv']

    trajectory_exporter_node = Node(
        package='slam_fusion',
        executable='trajectory_exporter.py',
        name='trajectory_exporter',
        output='screen',
        prefix='xterm -T "Trajectory Exporter" -hold -e',
        parameters=[{
            'output_file': traj_filename,
            'use_sim_time': use_sim_time
        }]
    )

    performance_monitor_node = Node(
        package='slam_fusion',
        executable='performance_monitor.py',
        name='performance_monitor',
        output='screen',
        prefix='xterm -T "Performance Monitor" -hold -e',
        parameters=[{
            'output_csv': csv_filename,
            'use_sim_time': use_sim_time
        }]
    )

    semantic_evaluator_node = Node(
        package='slam_fusion',
        executable='semantic_evaluator_node.py',
        name='semantic_evaluator',
        output='screen',
        prefix='xterm -T "Semantic Evaluator" -hold -e',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gazebo_gt_exporter = Node(
        package='slam_fusion',
        executable='gazebo_groundtruth_exporter.py',
        name='gazebo_groundtruth_exporter',
        output='screen',
        prefix='xterm -T "Gazebo GT Exporter" -hold -e',
        parameters=[{'output_file': '/ros2_ws/gazebo_groundtruth.txt'}],
        condition=LaunchConfigurationEquals('run_mode', 'simulation')
    )

    # RViz2
    rviz_config = PathJoinSubstitution([FindPackageShare('slam_fusion'), 'rviz', 'mapping.rviz'])
    rviz_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        )]
    )

    return LaunchDescription([
        run_mode_arg,
        perception_model_arg,
        gazebo_launch,
        dataset_player,
        sam2_node,
        deeplabv3_node,
        orbslam3_dataset_node,
        orbslam3_sim_node,
        fusion_node,
        trajectory_exporter_node,
        gazebo_gt_exporter,
        performance_monitor_node,
        semantic_evaluator_node,
        rviz_node
    ])
