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


    # Dataset Player (Delayed to accommodate ORB-SLAM3 Vocabulary loading)
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

    # SAM2 Segmentation Node
    sam2_node = Node(
        package='sam_perception',
        executable='sam2_node',
        name='sam2_perception',
        output='screen',
        prefix='xterm -T "SAM2 Perception" -hold -e',
        parameters=[{'keyframe_interval': 3}],
        condition=LaunchConfigurationEquals('perception_model', 'sam2')
    )

    # DeepLabv3 Segmentation Node
    deeplabv3_node = Node(
        package='sam_perception',
        executable='deeplabv3_node',
        name='deeplabv3_perception',
        output='screen',
        prefix='xterm -T "DeepLabV3 Perception" -hold -e',
        parameters=[{'keyframe_interval': 3}],
        condition=LaunchConfigurationEquals('perception_model', 'deeplabv3')
    )

    # RTAB-Map Odometry (Utilizing camera frame rather than base_link)
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
    # ORB-SLAM3 Tracking (Executes concurrently with RTAB-Map)
    orbslam3_node = Node(
        package='orbslam3',
        executable='rgbd',
        name='orb_slam3_rgbd',
        output='screen',
        #prefix='xterm -T "ORB-SLAM3" -hold -e',
        arguments=[
            '/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            '/ros2_ws/src/ORB_SLAM3/Examples/RGB-D/TUM1.yaml'
        ],
        remappings=[
            ('camera/rgb',   '/camera/color/image_raw'),
            ('camera/depth', '/camera/depth/image_raw')
        ]
    )

    # RTAB-Map SLAM Backend
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


    # Semantic Fusion Node for 3D Map Reconstruction
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

    # Trajectory Exporter Node
    trajectory_exporter_node = Node(
        package='slam_fusion',
        executable='trajectory_exporter.py',
        name='trajectory_exporter',
        output='screen',
        prefix='xterm -T "Trajectory Exporter" -hold -e',
        parameters=[{'output_file': 'slam_trajectory.txt'}]
    )

    # Semantic Evaluator Node
    semantic_evaluator_node = Node(
        package='slam_fusion',
        executable='semantic_evaluator_node.py',
        name='semantic_evaluator',
        output='screen',
        prefix='xterm -T "Semantic Evaluator" -hold -e'
    )

    # RViz2 Visualization Node
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
        trajectory_exporter_node,
        semantic_evaluator_node,
        rviz_node
    ])
