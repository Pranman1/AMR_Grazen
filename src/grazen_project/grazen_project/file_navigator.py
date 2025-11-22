#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetEntityState
import sys
import os
import time

def create_pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0 
    return pose

def teleport_robot(node, x, y):
    """ Teleports the physical robot in Gazebo """
    # Create a temporary client to talk to Gazebo
    client = node.create_client(SetEntityState, '/set_entity_state')
    
    # Wait for Gazebo to be ready
    if not client.wait_for_service(timeout_sec=2.0):
        print("❌ ERROR: Gazebo /set_entity_state service not found!")
        return False

    req = SetEntityState.Request()
    # --- CONFIRMED MODEL NAME ---
    req.state.name = 'burger' 
    # ----------------------------
    req.state.pose.position.x = float(x)
    req.state.pose.position.y = float(y)
    req.state.pose.position.z = 0.30  # Drop from 30cm to avoid floor clipping
    req.state.pose.orientation.w = 1.0
    
    print(f"Attempting to teleport 'burger' to ({x}, {y})...")
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    result = future.result()
    if result.success:
        print("✅ Gazebo Teleport Successful")
    else:
        print("❌ Gazebo Teleport Failed (Check model name?)")
        
    return result.success

def read_waypoints(filepath):
    points = []
    if not os.path.exists(filepath):
        print(f"Error: File not found at {filepath}")
        return []

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('['):
                continue
            parts = line.split()
            try:
                x = float(parts[0])
                y = float(parts[1])
                points.append((x, y))
            except ValueError:
                pass 
    return points

def main():
    rclpy.init()
    
    # We need a raw node for the service client
    # (BasicNavigator hides its node, so we make a temp one for teleporting)
    temp_node = rclpy.create_node('teleporter')
    navigator = BasicNavigator()

    # --- CONFIGURATION ---
    waypoint_file = '/home/pranav/waypoints.txt' 
    # ---------------------

    print(f"Reading waypoints from: {waypoint_file}")
    waypoints = read_waypoints(waypoint_file)
    
    if not waypoints:
        print("No valid waypoints found!")
        return

    # 1. GET START POINT
    start_x, start_y = waypoints[0]
    print(f"Start Point: ({start_x}, {start_y})")

    # 2. TELEPORT (The Missing Step)
    teleport_robot(temp_node, start_x, start_y)
    time.sleep(1.0) # Let physics settle
    temp_node.destroy_node() # Clean up temp node

    # 3. RESET NAV2 (The Localization Step)
    print("Resetting Nav2 Localization...")
    initial_pose = create_pose(start_x, start_y)
    navigator.setInitialPose(initial_pose)
    
    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active()

    # 4. DRIVE LOOP (Start from index 1)
    for i, (x, y) in enumerate(waypoints[1:], start=1):
        print(f"\nDriving to Waypoint {i+1}: ({x}, {y})...")
        
        goal_pose = create_pose(x, y)
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            pass

        if navigator.getResult() == TaskResult.SUCCEEDED:
            print(f"✅ Reached Waypoint {i+1}")
        else:
            print(f"❌ Failed to reach Waypoint {i+1}")

    print("\nMission Complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()