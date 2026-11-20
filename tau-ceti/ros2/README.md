# Tau Ceti on ROS 2 (Jazzy + micro-ROS)

Firmware: `inverse_v2_balance_ros/` (ESP32 + PCA9685 + MPU-6050/6500 IMU). Same IK and gait as `inverse_v2_balance/`, but driven over ROS, with IMU tilt safety and optional body leveling.

```
ESP32 (IK + gait + servos + IMU) --USB serial--> micro_ros_agent --> ROS 2 graph
   subscribes  /cmd           std_msgs/String   g stand, w/r/a/d/,/. walk directions, s stop, x relax, p print, l leveling, c level, z straight legs
   subscribes  /cmd_vel       geometry_msgs/Twist   linear.x fwd, linear.y left, angular.z turn left (-1..1) -> omnidirectional crawl
gamepad_windows.py (Windows) --UDP 9870--> gamepad_bridge.py (WSL) -> /cmd_vel + /cmd
   publishes   /joint_states  sensor_msgs/JointState   12 joints, radians, 2 Hz idle / 10 Hz moving
   publishes   /imu           sensor_msgs/Imu   tilt vs level + accel + gyro, robot frame, 10 Hz
   publishes   /status        std_msgs/String   what used to go to the Serial Monitor
robot_state_publisher: model + /joint_states -> TF (legs)
imu_tf.py: /imu -> TF world -> base_link (body tilt)
rviz2: draws it
```

## Wiring (ESP32)
| Device | SDA | SCL | VCC | GND |
|---|---|---|---|---|
| PCA9685 | GPIO13 | GPIO14 | 3V3 | GND |
| MPU-6050/6500 | GPIO21 | GPIO22 | 3V3 | GND |

IMU mounted chip-up, X arrow to the robot's left, Y arrow backward (change `imuToRobot()` if remounted).
Servo power (V+) stays on its own supply. Never use GPIO6-11. `i2c_check/` tests the wiring.

## Run
```
# Windows (admin PowerShell), once per plug-in, with a WSL terminal open:
usbipd attach --wsl --busid 2-6

# WSL:
ros2 launch tau_ceti display.launch.py     # agent + model + IMU tilt + RViz
ros2 run tau_ceti teleop.py                # second terminal: g / w / s / p / l / c keys
```
Keep the robot still for ~2 s after power-on (IMU calibration).

## Network note (WSL)
On some networks WSL's mirrored networking drops DDS multicast, so ROS programs can't find each other.
Fixed with `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in `~/.bashrc`, plus `config/localhost_dds.xml`
for the micro-ROS agent (the launch file sets `FASTRTPS_DEFAULT_PROFILES_FILE` for it).

## Workspace
`~/tau_ceti_ws/src/tau_ceti` is a symlink to `ros2/tau_ceti` in this repo. Rebuild after adding files:
`cd ~/tau_ceti_ws && colcon build --symlink-install`.

The robot dimensions are in two places: the firmware constants and `launch/display.launch.py`. Change both together.
