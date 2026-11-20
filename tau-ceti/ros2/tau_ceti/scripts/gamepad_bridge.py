#!/usr/bin/python3
# System Python on purpose: ROS Jazzy is built for 3.12, and a conda python3 on PATH breaks rclpy.
"""Game controller bridge (runs in Ubuntu, started by display.launch.py).

ros2/gamepad_windows.py (on Windows) reads the controller and sends its stick
and button values here over UDP (localhost:9870). This node turns them into:
  /cmd_vel  geometry_msgs/Twist  - the standard ROS "move like this" message:
            linear.x = forward, linear.y = left, angular.z = turn left (each -1..1)
  /cmd      std_msgs/String      - button presses, as the same letters teleop.py sends

Safety: if the Windows script stops (closed, crashed, controller unplugged),
we send "stop" (all zeros) right away, and the ESP32 also stops on its own if
/cmd_vel goes quiet.
"""
import json
import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

PORT = 9870
TIMEOUT_S = 0.5


class GamepadBridge(Node):
    def __init__(self):
        super().__init__("gamepad_bridge")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_pub = self.create_publisher(String, "cmd", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", PORT))
        self.sock.setblocking(False)
        self.last = None          # last stick values
        self.last_time = None     # when the last packet arrived
        self.connected = False
        self.create_timer(0.1, self.tick)    # 10 Hz: plenty for a slow crawl, lighter on the USB link
        self.get_logger().info(f"Waiting for gamepad_windows.py on UDP {PORT}...")

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        # Read everything that arrived since last time
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                break
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            self.last, self.last_time = msg, now
            for letter in msg.get("press", []):
                self.cmd_pub.publish(String(data=letter))
                self.get_logger().info(f"button -> {letter}")

        fresh = self.last_time is not None and now - self.last_time < TIMEOUT_S
        if fresh != self.connected:
            self.connected = fresh
            self.get_logger().info("Controller connected." if fresh else "Controller lost: stopping.")
        t = Twist()
        if fresh:
            t.linear.x = float(self.last.get("vx", 0))
            t.linear.y = float(self.last.get("vy", 0))
            t.angular.z = float(self.last.get("wz", 0))
            self.vel_pub.publish(t)
        elif self.last is not None:
            self.vel_pub.publish(t)   # all zeros = stop
            self.last = None


def main():
    rclpy.init()
    node = GamepadBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
