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
                'dataset_path': '/workspace/datasets/TUM/rgbd_dataset_freiburg1_desk',
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
    # SLAM BACKEND: CARTOGRAPHER
    # ==========================
    cartographer_config_dir = os.path.join(slam_fusion_dir, 'config')
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        prefix='xterm -T "Cartographer" -hold -e',
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_g1.lua',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_grid_node',
        output='screen',
        prefix='xterm -T "Cartographer Grid" -hold -e',
        parameters=[{'resolution': 0.05, 'use_sim_time': use_sim_time}]
    )

    tf_to_odom_node = Node(
        package='slam_fusion',
        executable='tf_to_odom.py',
        name='tf_to_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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

    trajectory_exporter_node = Node(
        package='slam_fusion',
        executable='trajectory_exporter.py',
        name='trajectory_exporter',
        output='screen',
        prefix='xterm -T "Trajectory Exporter" -hold -e',
        parameters=[{
            'output_file': 'cartographer_estimate.txt',
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
            'output_csv': 'cartographer_performance.csv',
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
        cartographer_node,
        cartographer_grid_node,
        tf_to_odom_node,
        fusion_node,
        trajectory_exporter_node,
        performance_monitor_node,
        semantic_evaluator_node,
        rviz_node
    ])
