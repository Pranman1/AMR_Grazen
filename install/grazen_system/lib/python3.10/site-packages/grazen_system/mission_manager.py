#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
import os
import time

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        self.declare_parameter('waypoints_file', '') 
        self.file_path = self.get_parameter('waypoints_file').value
        
        self.navigator = BasicNavigator()
        
        # TF Buffer to check if we are localized
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def is_localized(self):
        """Checks if the map->base_link transform exists (meaning AMCL is working)"""
        try:
            # Use Time() to get the latest available transform
            # 0 seconds means "latest", but we need to be careful with timeouts
            return self.tf_buffer.can_transform('map', 'base_link', Time(), timeout=Duration(seconds=1.0))
        except Exception:
            return False

    def read_waypoints(self):
        if not os.path.exists(self.file_path):
            self.get_logger().error(f"Waypoints file not found: {self.file_path}")
            return []
        
        points = []
        with open(self.file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'): continue
                parts = line.split()
                try:
                    points.append((float(parts[0]), float(parts[1])))
                except ValueError: pass
        return points

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    def run(self):
        # 1. Wait for Nav2 to come online
        self.get_logger().info("Waiting for Nav2 to start...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 Active.")

        # 2. Wait for Localization
        self.get_logger().info("🛑 WAITING FOR LOCALIZATION...")
        self.get_logger().info("👉 Please use '2D Pose Estimate' in RViz to align the robot.")
        
        while rclpy.ok():
            # We must spin the node briefly to allow the TF buffer to fill!
            # This was the MISSING PIECE in previous versions.
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.is_localized():
                self.get_logger().info("✅ Robot Localized! Starting Mission...")
                break
            self.get_logger().info("   Waiting for transform map->base_link...", throttle_duration_sec=2.0)
            
        # 3. Read Mission
        waypoints = self.read_waypoints()
        if not waypoints:
            self.get_logger().warn("No waypoints found. Mission Aborted.")
            return

        # 4. Get Initial Position
        # We need to spin one more time to ensure we get the fresh transform
        rclpy.spin_once(self, timeout_sec=0.1)
        
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            self.get_logger().info(f"⚓ STARTING POSITION: ({current_x:.2f}, {current_y:.2f})")
        except Exception as e:
            self.get_logger().error(f"Failed to get starting position: {e}")
            current_x, current_y = 0.0, 0.0

        # 5. Execute Sequential Moves (each waypoint relative to PREVIOUS position)
        for i, (dx, dy) in enumerate(waypoints):
            # Calculate target relative to CURRENT position (not initial anchor)
            target_x = current_x + dx
            target_y = current_y + dy
            
            self.get_logger().info(f"--- Move {i+1}: ({current_x:.2f}, {current_y:.2f}) → ({target_x:.2f}, {target_y:.2f}) [Δx={dx:.2f}, Δy={dy:.2f}] ---")
            
            goal = self.create_pose(target_x, target_y)
            self.navigator.goToPose(goal)

            while not self.navigator.isTaskComplete():
                pass

            if self.navigator.getResult() == TaskResult.SUCCEEDED:
                self.get_logger().info(f"✅ Move {i+1} Complete!")
                # Update current position for next move (use commanded target, not AMCL estimate)
                current_x = target_x
                current_y = target_y
            else:
                self.get_logger().error(f"❌ Move {i+1} Failed!")
                # On failure, break out (don't continue with wrong position)
                break

        self.get_logger().info("🏁 Mission Complete.")

def main():
    rclpy.init()
    node = MissionManager()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()