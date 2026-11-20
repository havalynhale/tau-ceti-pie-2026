"""Start the micro-ROS agent, the robot model, and RViz.

    ros2 launch tau_ceti display.launch.py              # robot plugged in on /dev/ttyUSB0
    ros2 launch tau_ceti display.launch.py port:=/dev/ttyUSB1
    ros2 launch tau_ceti display.launch.py agent:=false  # agent already running elsewhere

What a launch file is: a recipe that starts several ROS programs at once, so you
don't need a terminal for each one. This one starts four:

  1. micro_ros_agent        - talks to the ESP32 over USB and relays its topics
                              (/cmd, /status, /joint_states) onto the ROS network.
  2. robot_state_publisher  - has the 3D model of the robot (built below). Every
                              time /joint_states arrives, it works out where every
                              leg piece is in space and publishes that as "TF"
                              (= the position + rotation of each part).
  3. imu_tf.py              - takes the body tilt from /imu and tells TF how the
                              body sits relative to the floor ("world").
  4. rviz2                  - the 3D viewer. Reads the model + TF and draws it.

Data flow:  ESP32 -> /joint_states -> robot_state_publisher -> TF (legs)   -> RViz
            ESP32 -> /imu          -> imu_tf.py             -> TF (tilt)   -> RViz
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same numbers as the firmware (mm -> m, because ROS always uses meters).
# If you change the robot, change BOTH this file and the .ino.
L1 = 0.11825          # thigh
L2 = 0.08181          # shin
R = 0.07698           # body center -> hip yaw axis
COXA = 0.055          # hip yaw axis -> thigh joint
LEG_ANGLE = [72, 288, 144, 216]   # b1..b4, degrees around the body (0 = front, +x)
STAND_H = 0.170


def box_link(name, length, size, rgba):
    """One leg piece drawn as a box. It starts at its joint and sticks out
    along +x, so the next joint sits at x = length."""
    return f"""
  <link name="{name}">
    <visual>
      <origin xyz="{length / 2} 0 0"/>
      <geometry><box size="{length} {size} {size}"/></geometry>
      <material name="{name}_mat"><color rgba="{rgba}"/></material>
    </visual>
  </link>"""


def build_urdf():
    """Build the robot model as a URDF (the standard ROS robot-description format).

    A URDF is a tree of LINKS (rigid pieces) connected by JOINTS (where they
    rotate). Ours, for each of the 4 legs:

      base_link (body disc)
        └─ hip_yaw joint  (turns left/right, around vertical z)
            └─ coxa       (55 mm piece from hip axis to thigh joint)
                └─ thigh joint (tilts up/down)
                    └─ thigh   (L1)
                        └─ knee joint (bends)
                            └─ shin    (L2)
                                └─ foot (black ball at the tip)

    Joint conventions match the firmware IK exactly, so the angles the ESP32
    sends can be used directly:
      hip_yaw = yaw away from the radial direction (+ = counter-clockwise from above)
      thigh   = thetaT, thigh angle above horizontal
      knee    = thetaK, how far the shin bends down from straight
    """
    parts = ['<?xml version="1.0"?>', '<robot name="tau_ceti">']

    # The body: a disc, plus a small orange bar pointing at the FRONT (+x),
    # which is the direction the robot walks.
    parts.append(f"""
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="{R + 0.012}" length="0.02"/></geometry>
      <material name="body"><color rgba="0.25 0.3 0.4 1"/></material>
    </visual>
    <visual>
      <origin xyz="{R * 0.6} 0 0.015"/>
      <geometry><box size="0.04 0.012 0.01"/></geometry>
      <material name="front"><color rgba="1 0.6 0.1 1"/></material>
    </visual>
  </link>""")

    # Leg colors in RViz: b1 red, b2 green, b3 blue, b4 yellow
    leg_colors = ["0.9 0.3 0.3 1", "0.3 0.8 0.4 1", "0.3 0.5 0.95 1", "0.9 0.8 0.3 1"]
    for i, ang in enumerate(LEG_ANGLE):
        b = f"b{i + 1}"
        v = math.radians(ang)
        c = leg_colors[i]
        parts.append(box_link(f"{b}_coxa", COXA, 0.02, "0.6 0.6 0.6 1"))
        parts.append(box_link(f"{b}_thigh", L1, 0.018, c))
        parts.append(box_link(f"{b}_shin", L2, 0.014, c))
        # Joint notes:
        #   hip_yaw origin: put the hip on the body edge at LEG_ANGLE, rotated so
        #                   the leg's "straight out" points away from the center.
        #   thigh axis 0 -1 0: positive angle lifts the thigh UP (matches thetaT).
        #   knee  axis 0  1 0: positive angle bends the shin DOWN (matches thetaK).
        # Joint names (b1_hip_yaw, b1_thigh, b1_knee...) must match initJointMsg() in the .ino.
        parts.append(f"""
  <link name="{b}_foot">
    <visual>
      <geometry><sphere radius="0.009"/></geometry>
      <material name="{b}_foot_mat"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>
  <joint name="{b}_hip_yaw" type="revolute">
    <parent link="base_link"/><child link="{b}_coxa"/>
    <origin xyz="{R * math.cos(v):.5f} {R * math.sin(v):.5f} 0" rpy="0 0 {v:.5f}"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="1" velocity="5"/>
  </joint>
  <joint name="{b}_thigh" type="revolute">
    <parent link="{b}_coxa"/><child link="{b}_thigh"/>
    <origin xyz="{COXA} 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-3.1416" upper="3.1416" effort="1" velocity="5"/>
  </joint>
  <joint name="{b}_knee" type="revolute">
    <parent link="{b}_thigh"/><child link="{b}_shin"/>
    <origin xyz="{L1} 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.1416" upper="3.1416" effort="1" velocity="5"/>
  </joint>
  <joint name="{b}_foot_fixed" type="fixed">
    <parent link="{b}_shin"/><child link="{b}_foot"/>
    <origin xyz="{L2} 0 0"/>
  </joint>""")
    parts.append("</robot>")
    return "\n".join(parts)


def generate_launch_description():
    """ROS calls this to find out what to start."""
    share = get_package_share_directory("tau_ceti")
    # Options you can change from the command line, e.g. port:=/dev/ttyUSB1
    agent = LaunchConfiguration("agent")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument("agent", default_value="true",
                              description="Start the micro-ROS serial agent"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="ESP32 serial device"),

        # 1. The bridge to the ESP32. Only ONE of these may run at a time:
        #    two agents on the same USB port fight and the robot goes silent.
        #    additional_env: makes the agent find the other ROS programs on this
        #    PC directly (see config/localhost_dds.xml for why).
        Node(
            package="micro_ros_agent", executable="micro_ros_agent",
            arguments=["serial", "--dev", port, "-b", "115200"],
            additional_env={"FASTRTPS_DEFAULT_PROFILES_FILE":
                            os.path.join(share, "config", "localhost_dds.xml")},
            condition=IfCondition(agent), output="screen",
        ),
        # 2. Model + joint angles -> positions of every part (TF)
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": build_urdf()}],
        ),
        # 3. IMU tilt -> body tilt in RViz (world -> base_link)
        Node(package="tau_ceti", executable="imu_tf.py"),
        # Game controller: receives sticks/buttons from ros2/gamepad_windows.py
        # (run that on Windows) and publishes /cmd_vel + /cmd. Harmless if unused.
        Node(package="tau_ceti", executable="gamepad_bridge.py", output="screen"),
        # 4. The 3D viewer, opened with our saved layout (rviz/tau_ceti.rviz).
        #    If you rearrange RViz and do File > Save Config, it saves to that file.
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "tau_ceti.rviz")],
        ),
    ])
