#!/usr/bin/python3
# System Python on purpose: ROS Jazzy is built for 3.12, and a conda python3 on PATH breaks rclpy.
"""Tilts the robot body in RViz using the IMU.

Listens to /imu (from the ESP32) and publishes the TF transform  world -> base_link:
  - rotation    = the body tilt the IMU measured
  - translation = body STAND_H above the floor (just so the feet sit near the grid)

TF is ROS's "where is everything" system. robot_state_publisher already says
where each leg piece is relative to the body (base_link). This node adds where
the body is relative to the world/floor. RViz's Fixed Frame is "world", so a
tilted body now shows up tilted.

If no /imu arrives (IMU unplugged, old firmware), it publishes a level body so
RViz still works.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

STAND_H = 0.170   # m, same as the firmware's STAND_H


class ImuTf(Node):
    def __init__(self):
        super().__init__("imu_tf")
        self.br = TransformBroadcaster(self)
        self.q = (0.0, 0.0, 0.0, 1.0)   # x, y, z, w (= level)
        # The ESP32 sends /imu "best effort", so we must listen best effort too
        # (qos_profile_sensor_data). A "reliable" listener would hear nothing.
        self.create_subscription(Imu, "imu", self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.05, self.send)   # 20 times a second

    def on_imu(self, msg):
        o = msg.orientation
        self.q = (o.x, o.y, o.z, o.w)

    def send(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.z = STAND_H
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = self.q
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = ImuTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
