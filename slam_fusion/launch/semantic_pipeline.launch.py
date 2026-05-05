import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sam_sim_dir     = get_package_share_directory('sam_simulation')
    slam_fusion_dir = get_package_share_directory('slam_fusion')

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
        parameters=[{'keyframe_interval': 3}]
    )

    # RTAB-Map visual odometry — estimates robot pose from the RGB-D stream
    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[{
            'frame_id':    'base_link',
            'approx_sync': True,
            'publish_tf':  True,
            'queue_size':  20,
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom',            '/slam/odom'),
        ]
    )

    # Cartographer LiDAR SLAM — builds an occupancy grid and publishes the map→odom TF
    cartographer_config_dir = os.path.join(slam_fusion_dir, 'config')
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_g1.lua',
        ]
    )

    # Cartographer occupancy grid publisher — converts internal map to ROS OccupancyGrid
    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_grid_node',
        output='screen',
        parameters=[{'resolution': 0.05}]
    )

    # TF-to-Odometry bridge — converts Cartographer's map→base_link TF to /slam/odom
    tf_to_odom_node = Node(
        package='slam_fusion',
        executable='tf_to_odom',
        name='tf_to_odom',
        output='screen'
    )

    # Semantic fusion — projects SAM2 masks onto the 3D point cloud using SLAM odometry
    fusion_node = Node(
        package='slam_fusion',
        executable='semantic_fusion_node',
        name='semantic_fusion',
        output='screen',
        parameters=[{
            'voxel_leaf_size': 0.05,
            'pixel_step':      2,
            'depth_min':       0.1,
            'depth_max':       10.0,
        }]
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
        )]
    )

    return LaunchDescription([
        gazebo_launch,
        sam2_node,
        rtabmap_odom,
        cartographer_node,
        cartographer_grid_node,
        tf_to_odom_node,
        fusion_node,
        rviz_node,
    ])
