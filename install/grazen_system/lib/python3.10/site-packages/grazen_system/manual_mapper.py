#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import subprocess
import os

MSG = """
---------------------------
   MANUAL MAPPING MODE
---------------------------
   w
 a s d    (Drive)
   x

p : SAVE MAP NOW
space : Force Stop
CTRL-C : Quit
---------------------------
"""

class ManualMapper(Node):
    def __init__(self):
        super().__init__('manual_mapper')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.declare_parameter('map_path', '')
        self.map_path = self.get_parameter('map_path').value
        self.speed = 0.2
        self.turn = 1.0

    def save_map(self):
        if not self.map_path:
            self.get_logger().error("No map path provided!")
            return
            
        self.get_logger().info(f"💾 SAVING MAP TO: {self.map_path}...")
        try:
            # Run the map saver CLI command
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", self.map_path],
                check=True
            )
            self.get_logger().info("✅ MAP SAVED SUCCESSFULLY!")
        except Exception as e:
            # Log the error without using sudo
            self.get_logger().error(f"❌ Failed to save map: {e}")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print(MSG)
        x, th = 0.0, 0.0
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w':
                    x = self.speed
                    th = 0.0
                elif key == 'x':
                    x = -self.speed
                    th = 0.0
                elif key == 'a':
                    x = 0.0
                    th = self.turn
                elif key == 'd':
                    x = 0.0
                    th = -self.turn
                elif key == ' ' or key == 's':
                    x = 0.0
                    th = 0.0
                elif key == 'p': # SAVE KEY
                    x = 0.0
                    th = 0.0
                    self.save_map()
                elif key == '\x03': # Ctrl-C
                    break
                else:
                    x = 0.0
                    th = 0.0

                twist = Twist()
                twist.linear.x = float(x)
                twist.angular.z = float(th)
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            self.publisher_.publish(twist)

settings = None

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ManualMapper()
    node.run()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()