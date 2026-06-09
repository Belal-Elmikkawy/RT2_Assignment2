import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sam_sim_dir     = get_package_share_directory('sam_simulation')
    slam_fusion_dir = get_package_share_directory('slam_fusion')

    perception_model_arg = DeclareLaunchArgument(
        'perception_model',
        default_value='sam2',
        description='Perception model to use: sam2 or deeplabv3'
    )

    # Gazebo simulation with the G1 humanoid robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sam_sim_dir, 'launch', 'gazebo_indoor.launch.py')
        )
    )

    # SAM2 foundation-model segmentation — publishes per-pixel instance masks
    sam2_node = Node(
        package='sam_perception',
        executable='sam2_node',
        name='sam2_perception',
        output='screen',
        prefix='xterm -T "SAM2 Perception" -hold -e',
        parameters=[{'keyframe_interval': 3, 'use_sim_time': True}],
        condition=LaunchConfigurationEquals('perception_model', 'sam2')
    )

    # DeepLabV3 Segmentation
    deeplabv3_node = Node(
        package='sam_perception',
        executable='deeplabv3_node',
        name='deeplabv3_perception',
        output='screen',
        prefix='xterm -T "DeepLabV3 Perception" -hold -e',
        parameters=[{'keyframe_interval': 3, 'use_sim_time': True}],
        condition=LaunchConfigurationEquals('perception_model', 'deeplabv3')
    )

    # RTAB-Map SLAM (Mapping Node)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        prefix='xterm -T "RTAB-Map SLAM" -hold -e',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'use_sim_time': True,
            'publish_tf': False,
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom',            '/odom'),
        ],
        arguments=['-d']
    )

    # Cartographer LiDAR SLAM — builds an occupancy grid and publishes the map→odom TF
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
        parameters=[{'use_sim_time': True}]
    )

    # Cartographer occupancy grid publisher — converts internal map to ROS OccupancyGrid
    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_grid_node',
        output='screen',
        prefix='xterm -T "Cartographer Grid" -hold -e',
        parameters=[{'resolution': 0.05, 'use_sim_time': True}]
    )

    # Semantic fusion — projects SAM2 masks onto the 3D point cloud using SLAM odometry
    fusion_node = Node(
        package='slam_fusion',
        executable='semantic_fusion_node',
        name='semantic_fusion',
        output='screen',
        prefix='xterm -T "Semantic Fusion" -hold -e',
        parameters=[{
            'voxel_leaf_size': 0.05,
            'pixel_step':      2,
            'depth_min':       0.1,
            'depth_max':       10.0,
            'use_sim_time':    True,
        }]
    )

    # ORB-SLAM3 (Running simultaneously with RTAB-Map and Cartographer)
    # The vocabulary and camera config paths are set for the Docker container workspace
    orb_libs = '/ros2_ws/src/ORB_SLAM3/lib:/ros2_ws/src/ORB_SLAM3/Thirdparty/DBoW2/lib:/ros2_ws/src/ORB_SLAM3/Thirdparty/g2o/lib'
    current_ld = os.environ.get('LD_LIBRARY_PATH', '')
    
    orbslam3_node = Node(
        package='orbslam3',
        executable='rgbd',
        name='orb_slam3_rgbd',
        output='screen',
        arguments=[
            '/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            '/ros2_ws/src/slam_fusion/config/g1_gazebo_camera.yaml'
        ],
        remappings=[
            ('camera/rgb',   '/camera/color/image_raw'),
            ('camera/depth', '/camera/depth/image_raw')
        ],
        parameters=[{'use_sim_time': True}]
    )

    # RViz2 — delayed so the map TF frame is already available when it starts
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('slam_fusion'), 'rviz', 'mapping.rviz']
    )
    rviz_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        )]
    )

    # Evaluation: Semantic Consistency Evaluator
    semantic_evaluator = Node(
        package='slam_fusion',
        executable='semantic_evaluator_node.py',
        name='semantic_evaluator',
        output='screen',
        prefix='xterm -T "Semantic Evaluator" -hold -e',
        remappings=[
            ('/slam/odom', '/odom')
        ],
        parameters=[{'use_sim_time': True}]
    )

    # Evaluation: Trajectory Exporter (Writes slam_trajectory.txt for evo)
    trajectory_exporter = Node(
        package='slam_fusion',
        executable='trajectory_exporter.py',
        name='trajectory_exporter',
        output='screen',
        prefix='xterm -T "Trajectory Exporter" -hold -e',
        parameters=[
            {'output_file': 'slam_trajectory.txt'},
            {'use_sim_time': True}
        ],
        remappings=[
            ('/slam/odom', '/odom')
        ]
    )

    return LaunchDescription([
        perception_model_arg,
        gazebo_launch,
        sam2_node,
        deeplabv3_node,
        rtabmap_slam,
        cartographer_node,
        cartographer_grid_node,
        fusion_node,
        orbslam3_node,
        semantic_evaluator,
        rviz_node,
    ])
