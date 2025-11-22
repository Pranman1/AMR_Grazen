#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetEntityState
import os
import time

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # 1. Load Configuration from Parameters
        self.declare_parameter('mode', 'sim')       # 'sim' or 'real'
        self.declare_parameter('waypoints_file', '') 
        
        self.mode = self.get_parameter('mode').value
        self.file_path = self.get_parameter('waypoints_file').value
        
        self.navigator = BasicNavigator()

    def get_sim_pose(self):
        """ [SIM ONLY] Asks Gazebo where the robot is. """
        client = self.create_client(GetEntityState, '/get_entity_state')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Gazebo service not found!")
            return None
        req = GetEntityState.Request()
        req.name = 'burger'
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().state.pose if future.result().success else None

    def initialize_localization(self):
        """ Handles the 'Where am I?' problem based on Mode. """
        if self.mode == 'sim':
            self.get_logger().info(" [SIM MODE] Auto-detecting start pose from Gazebo...")
            pose = self.get_sim_pose()
            if pose:
                init_pose = PoseStamped()
                init_pose.header.frame_id = 'map'
                init_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                init_pose.pose = pose
                self.navigator.setInitialPose(init_pose)
                self.get_logger().info(f"Set Pose to: ({pose.position.x:.2f}, {pose.position.y:.2f})")
            else:
                self.get_logger().error("Could not detect robot. defaulting to 0,0")
                self.navigator.setInitialPose(self.create_pose(0,0))
        else:
            self.get_logger().info(" [REAL MODE] Waiting for manual 2D Pose Estimate in RViz...")
            # In real mode, we just wait for the user to click in RViz
            # Nav2 is considered 'active' once it gets that pose.

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

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

    def run(self):
        # 1. Setup
        self.initialize_localization()
        
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is Active!")

        # 2. Read Mission
        waypoints = self.read_waypoints()
        if not waypoints:
            self.get_logger().warn("No waypoints found. Mission Aborted.")
            return

        # 3. Execute Mission (Relative Movement Logic)
        # Note: This assumes waypoints are relative (Move 1m North), 
        # matching your previous logic.
        
        for i, (dx, dy) in enumerate(waypoints):
            self.get_logger().info(f"--- Executing Move {i+1}: Go X+{dx}, Y+{dy} ---")
            
            # Get current location from Nav2 feedback
            feedback = self.navigator.getFeedback()
            start_pose = feedback.current_pose.pose if feedback else None
            
            # If feedback isn't ready, wait briefly
            if not start_pose:
                time.sleep(1.0) 
                feedback = self.navigator.getFeedback()
                start_pose = feedback.current_pose.pose if feedback else None

            if start_pose:
                target_x = start_pose.position.x + dx
                target_y = start_pose.position.y + dy
                
                goal = self.create_pose(target_x, target_y)
                self.navigator.goToPose(goal)

                while not self.navigator.isTaskComplete():
                    pass

                if self.navigator.getResult() == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"Move {i+1} Complete!")
                else:
                    self.get_logger().error(f"Move {i+1} Failed!")
            else:
                self.get_logger().error("Could not determine current location. Skipping move.")

        self.get_logger().info("Mission Complete.")

def main():
    rclpy.init()
    node = MissionManager()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()