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

    import re
    # Strip all complex STL collision meshes from the URDF to prevent Gazebo from crashing
    robot_desc = re.sub(r'<collision>.*?</collision>', '', robot_desc, flags=re.DOTALL)

    animated_joints = [
        'left_hip_pitch_joint', 'left_knee_joint', 'left_ankle_pitch_joint',
        'right_hip_pitch_joint', 'right_knee_joint', 'right_ankle_pitch_joint',
        'left_shoulder_pitch_joint', 'left_elbow_joint',
        'right_shoulder_pitch_joint', 'right_elbow_joint'
    ]

    # Convert all moving joints to fixed EXCEPT the animated ones.
    # This prevents the waist, spine, and sideways hip joints from going loose and flapping 
    # around under centrifugal force when turning.
    def freeze_unused_joints(match):
        joint_name = match.group(1)
        if joint_name in animated_joints:
            return match.group(0) # Keep it revolute/continuous
        else:
            return match.group(0).replace('type="revolute"', 'type="fixed"').replace('type="continuous"', 'type="fixed"')

    robot_desc = re.sub(r'<joint\s+name="([^"]+)"\s+type="(?:revolute|continuous)">', freeze_unused_joints, robot_desc)

    # Add ros2_control tags for the walking animation
    ros2_control_xml = '<ros2_control name="GazeboSystem" type="system"><hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>'
    for j in animated_joints:
        ros2_control_xml += f'<joint name="{j}"><command_interface name="position"/><state_interface name="position"/></joint>'
    ros2_control_xml += '</ros2_control>'

    controllers_yaml = os.path.join(get_package_share_directory('sam_simulation'), 'config', 'g1_controllers.yaml')


    # Sensor plugins and Planar Move plugin for keyboard control
    sensor_plugins = """
  <gazebo reference="d435_link">
    <sensor name="camera" type="depth">
      <always_on>true</always_on>
      <update_rate>15.0</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <camera_name>camera</camera_name>
        <ros>
          <remapping>camera/image_raw:=/camera/color/image_raw</remapping>
          <remapping>camera/camera_info:=/camera/color/camera_info</remapping>
          <remapping>camera/depth/image_raw:=/camera/depth/image_raw</remapping>
          <remapping>camera/depth/camera_info:=/camera/depth/camera_info</remapping>
          <remapping>camera/points:=/camera/depth/points</remapping>
        </ros>
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
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>{controllers_yaml}</parameters>
    </plugin>
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
    # Use f-string to inject the parameters path into the plugin xml
    sensor_plugins = sensor_plugins.format(controllers_yaml=controllers_yaml)
    robot_desc = robot_desc.replace('</robot>', ros2_control_xml + sensor_plugins + '\n</robot>')

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

    # Controller managers to actuate the joints in Gazebo
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
    )

    # Gait animator node to convert cmd_vel into walking joint commands
    gait_animator_node = Node(
        package='sam_simulation',
        executable='gait_animator.py',
        name='gait_animator'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        gait_animator_node
    ])
