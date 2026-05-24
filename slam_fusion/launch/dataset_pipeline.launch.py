import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    perception_model_arg = DeclareLaunchArgument(
        'perception_model',
        default_value='sam2',
        description='Perception model to use: sam2 or deeplabv3'
    )


    #  Dataset Player (Delayed by 12 seconds to wait for ORB-SLAM3 Vocabulary to load)
    dataset_player = TimerAction(
        period=12.0,
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

    # 2. SAM2 Segmentation
    sam2_node = Node(
        package='sam_perception',
        executable='sam2_node',
        name='sam2_perception',
        output='screen',
        prefix='xterm -T "SAM2 Perception" -hold -e',
        parameters=[{'keyframe_interval': 3}],
        condition=LaunchConfigurationEquals('perception_model', 'sam2')
    )

    # DeepLabv3 Segmentation
    deeplabv3_node = Node(
        package='sam_perception',
        executable='deeplabv3_node',
        name='deeplabv3_perception',
        output='screen',
        prefix='xterm -T "DeepLabV3 Perception" -hold -e',
        parameters=[{'keyframe_interval': 3}],
        condition=LaunchConfigurationEquals('perception_model', 'deeplabv3')
    )

    # 3. RTAB-Map Odometry (using camera frame instead of base_link)
    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        prefix='xterm -T "RTAB-Map Odometry" -hold -e',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'approx_sync': True,
            'publish_tf': True,
            'Vis/MaxFeatures': '1500',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '1',
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom',            '/slam/odom'),
        ]
    )
        # ---------------------------------------------------------
    # ORB-SLAM3 (Running simultaneously with RTAB-Map)
    orbslam3_node = Node(
        package='orbslam3',
        executable='rgbd',
        name='orb_slam3_rgbd',
        output='screen',
        prefix='xterm -T "ORB-SLAM3" -hold -e',
        arguments=[
            '/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            '/ros2_ws/src/ORB_SLAM3/Examples/RGB-D/TUM1.yaml'
        ],
        remappings=[
            ('camera/rgb',   '/camera/color/image_raw'),
            ('camera/depth', '/camera/depth/image_raw')
        ]
    )

    #RTAB-map SLAM (Mapping Node)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        prefix='xterm -T "RTAB-Map SLAM" -hold -e',
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom', '/slam/odom'),
        ],
        arguments=['-d']
    )
    # ---------------------------------------------------------


    # 4. Semantic Fusion Node (with our new tf2 fixes)
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
        }]
    )

    # 5. RViz2
    rviz_config = PathJoinSubstitution([FindPackageShare('slam_fusion'), 'rviz', 'mapping.rviz'])
    rviz_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )]
    )

    return LaunchDescription([
        perception_model_arg,
        dataset_player,
        sam2_node,
        deeplabv3_node,
        rtabmap_odom,
        rtabmap_slam,
        orbslam3_node,
        fusion_node,
        rviz_node
    ])
