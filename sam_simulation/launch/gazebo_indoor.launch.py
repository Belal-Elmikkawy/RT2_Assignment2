import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('sam_simulation')

    # Start Gazebo Classic with the indoor world and the ROS 2 factory plugin
    world_file = os.path.join(pkg_dir, 'worlds', 'indoor_room.world')
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    # Load the official Unitree G1 URDF and inject Gazebo sensor plugins at runtime
    g1_urdf_path = os.path.join(
        get_package_share_directory('g1_description'), 'g1_23dof.urdf'
    )
    with open(g1_urdf_path, 'r') as f:
        robot_desc = f.read()

    # Convert package:// URIs to absolute file:// URIs so Gazebo can find the meshes
    # without needing GAZEBO_MODEL_PATH configured
    g1_share_dir = get_package_share_directory('g1_description')
    robot_desc = robot_desc.replace(
        'package://g1_description/meshes', 
        f'file://{g1_share_dir}/meshes'
    )

    # Convert all moving joints to fixed joints! 
    # This locks the robot in a rigid standing pose and allows robot_state_publisher 
    # to publish the entire TF tree statically without needing joint_state_publisher.
    robot_desc = robot_desc.replace('type="revolute"', 'type="fixed"')
    robot_desc = robot_desc.replace('type="continuous"', 'type="fixed"')

    import re
    # Strip all complex STL collision meshes from the URDF to prevent Gazebo from crashing
    # when initializing physics for the massive merged static body.
    robot_desc = re.sub(r'<collision>.*?</collision>', '', robot_desc, flags=re.DOTALL)

    # Sensor plugins and Planar Move plugin for keyboard control
    sensor_plugins = """
  <gazebo reference="d435_link">
    <sensor name="camera" type="depth">
      <always_on>true</always_on>
      <update_rate>15.0</update_rate>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>~/image_raw:=/camera/color/image_raw</remapping>
          <remapping>~/depth/image_raw:=/camera/depth/image_raw</remapping>
          <remapping>~/camera_info:=/camera/color/camera_info</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_depth_optical_frame</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="mid360_link">
    <sensor name="lidar" type="ray">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>10.0</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples><resolution>1</resolution>
            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range><min>0.12</min><max>10.0</max></range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros><remapping>~/out:=/scan</remapping></ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>mid360_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo>
    <plugin name="object_controller" filename="libgazebo_ros_planar_move.so">
      <ros><remapping>cmd_vel:=cmd_vel</remapping><remapping>odom:=odom</remapping></ros>
      <update_rate>100.0</update_rate>
      <publish_rate>10.0</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>
    </plugin>
  </gazebo>

  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="500.0"/>
      <inertia ixx="1000.0" ixy="0.0" ixz="0.0" iyy="1000.0" iyz="0.0" izz="1000.0"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry><box size="0.4 0.4 0.1"/></geometry>
    </collision>
  </link>
  <joint name="base_link_to_pelvis" type="fixed">
    <parent link="base_link"/>
    <child link="pelvis"/>
    <origin xyz="0 0 0.8" rpy="0 0 0"/>
  </joint>
"""
    robot_desc = robot_desc.replace('</robot>', sensor_plugins + '\n</robot>')

    # Publish the combined robot description (G1 body + sensor plugins)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawn the G1 robot — delayed 5 s to ensure Gazebo's /spawn_entity service is ready
    # We use ExecuteProcess with /usr/bin/python3 to bypass the Conda Python 3.11 environment
    spawn_entity = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                '/usr/bin/python3',
                '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py',
                '-topic', 'robot_description',
                '-entity', 'g1_robot',
                '-x', '0.0', '-y', '0.0', '-z', '0.0'
            ],
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
