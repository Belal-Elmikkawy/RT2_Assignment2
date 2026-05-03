"""
Semantic SLAM Pipeline Launch File  (ROS 2 Jazzy / Ubuntu 24.04)
───────────────────────────────────────────────────────────────────────────
Launches the complete SAM2 + RTAB-Map semantic SLAM pipeline:

  virtual_camera     — dataset_node           (RGB + Depth + CameraInfo)
  sam2_perception    — sam2_node              (per-pixel instance masks)
  static_tf          — tf2_ros                (base_link → camera frame)
  rgbd_odometry      — rtabmap_odom           (visual odometry / pose)
  semantic_fusion    — semantic_fusion_node   (3D semantic point cloud)
  rviz2              — rviz2                  (live visualizer)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch Arguments ─────────────────────────────────────────────────
    declare_dataset_path = DeclareLaunchArgument(
        'dataset_path',
        default_value='/workspace/datasets/TUM/rgbd_dataset_freiburg1_desk',
        description='Absolute path to the RGB-D dataset root folder'
    )
    declare_fps = DeclareLaunchArgument(
        'fps',
        default_value='15.0',
        description='Playback rate in frames per second'
    )
    declare_dataset_type = DeclareLaunchArgument(
        'dataset_type',
        default_value='tum_fr1',
        description='Dataset intrinsic profile: tum_fr1 | tum_fr2 | tum_fr3 | scannet'
    )
    declare_loop = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='If true, replay the dataset continuously'
    )
    declare_checkpoint = DeclareLaunchArgument(
        'sam2_checkpoint',
        default_value='checkpoints/sam2.1_hiera_small.pt',
        description='Path to SAM2 model checkpoint'
    )
    declare_kf_interval = DeclareLaunchArgument(
        'keyframe_interval',
        default_value='3',
        description='Run SAM2 on every N-th incoming RGB frame'
    )
    declare_voxel = DeclareLaunchArgument(
        'voxel_leaf_size',
        default_value='0.05',
        description='Voxel grid leaf size in metres (0 = disabled)'
    )

    # ── Nodes ─────────────────────────────────────────────────────────────

    # Dataset Publisher (virtual camera)
    dataset_node = Node(
        package='sam_perception',
        executable='dataset_node',
        name='virtual_camera',
        output='screen',
        respawn=True,
        parameters=[{
            'dataset_path':         LaunchConfiguration('dataset_path'),
            'fps':                  LaunchConfiguration('fps'),
            'dataset_type':         LaunchConfiguration('dataset_type'),
            'loop':                 LaunchConfiguration('loop'),
        }]
    )

    # SAM2 Perception Node
    sam2_node = Node(
        package='sam_perception',
        executable='sam2_node',
        name='sam2_perception',
        output='screen',
        respawn=True,
        parameters=[{
            'checkpoint':        LaunchConfiguration('sam2_checkpoint'),
            'keyframe_interval': LaunchConfiguration('keyframe_interval'),
        }]
    )

    # Static TF: base_link → camera_color_optical_frame
    #    (Standard ROS camera convention: X-right, Y-down, Z-forward)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0', '0', '0',              # translation x y z
            '0', '0', '0', '1',         # quaternion x y z w  (identity — camera at base)
            'base_link',
            'camera_color_optical_frame'
        ]
    )

    # RTAB-Map RGBD Odometry (visual odometry backend)
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
            # Increase queue size to tolerate SAM2 inference delays
            'queue_size':  20,
        }],
        remappings=[
            ('rgb/image',        '/camera/color/image_raw'),
            ('depth/image',      '/camera/depth/image_raw'),
            ('rgb/camera_info',  '/camera/color/camera_info'),
            ('odom',             '/slam/odom'),
        ]
    )

    # Semantic Fusion Node (C++)
    fusion_node = Node(
        package='slam_fusion',
        executable='semantic_fusion_node',
        name='semantic_fusion',
        output='screen',
        parameters=[{
            'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
            'pixel_step':      2,
            'depth_min':       0.1,
            'depth_max':       10.0,
        }]
    )

    # RViz2 — started 5 s after other nodes so the map frame is already published
    rviz_config = PathJoinSubstitution([
        FindPackageShare('slam_fusion'), 'rviz', 'mapping.rviz'
    ])
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        declare_dataset_path,
        declare_fps,
        declare_dataset_type,
        declare_loop,
        declare_checkpoint,
        declare_kf_interval,
        declare_voxel,
        # Log
        LogInfo(msg='Starting SAM2 + RTAB-Map Semantic SLAM pipeline (ROS 2 Humble)...'),
        # Nodes (order matters: data source first)
        dataset_node,
        static_tf,
        rtabmap_odom,
        sam2_node,
        fusion_node,
        rviz_node,    # delayed 5 s so 'map' frame is ready
    ])