#!/usr/bin/python3
# System Python on purpose: ROS Jazzy is built for 3.12, and a conda python3 on PATH breaks rclpy.
"""Keyboard control for tau_ceti. Replaces the Arduino Serial Monitor.

    ros2 run tau_ceti teleop.py

Keys: g = stand, w = walk, s = stop, p = print state,
      l = leveling on/off, c = calibrate level, q = quit.
Messages from the robot (/status) print as they arrive.

How it works: this is a small ROS node (program) that
  - PUBLISHES each key you press as a text message on /cmd  -> the ESP32 receives it
  - SUBSCRIBES to /status and prints whatever the ESP32 says back
"""
import select
import sys
import termios
import tty

import rclpy                      # the ROS 2 Python library
from rclpy.node import Node
from std_msgs.msg import String   # message type: just a text field called .data

KEYS = {"x": "EMERGENCY RELAX (all servos limp)",
        "g": "stand", "w": "walk forward", "r": "walk backward",
        "a": "turn left", "d": "turn right (or 'done' after z)",
        ",": "walk sideways left", ".": "walk sideways right",
        "s": "stop", "p": "print",
        "]": "walk faster", "[": "walk slower",
        "m": "more balance margin", "n": "less balance margin (less rocking)",
        "l": "leveling on/off", "c": "calibrate level (robot must be flat + still)",
        "z": "assembly pose: every leg straight out and flat",
        # calibration keys (after z)
        "1": "select leg b1", "2": "select leg b2", "3": "select leg b3", "4": "select leg b4",
        "h": "select hip", "t": "select thigh", "k": "select knee",
        "+": "+1 deg", "=": "+1 deg", "-": "-1 deg", "d": "done: print offsets"}


class Teleop(Node):
    def __init__(self):
        super().__init__("tau_ceti_teleop")   # our name on the ROS network
        # Publisher: we send on /cmd. The 10 = keep up to 10 unsent messages queued.
        self.pub = self.create_publisher(String, "cmd", 10)
        # Subscriber: every message on /status calls self.on_status
        self.create_subscription(String, "status", self.on_status, 10)

    def on_status(self, msg):
        # \r and \r\n because the terminal is in "cbreak" mode (see main)
        print(f"\r[robot] {msg.data}", end="\r\n", flush=True)

    def send(self, key):
        self.pub.publish(String(data=key))
        print(f"\r> {key} ({KEYS[key]})", end="\r\n", flush=True)


def main():
    rclpy.init()
    node = Teleop()
    # Normally a terminal waits for Enter before giving us your typing.
    # cbreak mode hands over each key the moment it's pressed. We save the old
    # settings so we can put the terminal back to normal when we quit.
    old = termios.tcgetattr(sys.stdin)
    print("x = EMERGENCY RELAX (all servos limp)")
    print("g = stand | w fwd  r back  a/d turn  ,/. sideways | s = stop | p = print | q = quit")
    print("] / [ = faster / slower | m / n = more / less balance margin")
    print("l = leveling | c = calibrate level | z = legs straight (horns)")
    print("after z:  1-4 = leg | h/t/k = hip/thigh/knee | + or = / - = move 1 deg | d = print offsets")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            # Let ROS deliver any /status messages (waits at most 50 ms)
            rclpy.spin_once(node, timeout_sec=0.05)
            # Was a key pressed? (select checks without blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == "q":
                    break
                if key in KEYS:
                    node.send(key)
    except KeyboardInterrupt:   # Ctrl+C
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)   # restore the terminal
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
