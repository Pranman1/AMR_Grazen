#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetEntityState
import math
import time

def create_pose(x, y, yaw=0.0):
    """Helper to create a PoseStamped message."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def teleport_robot(navigator, x, y, yaw=0.0):
    """
    Teleport the robot using the /set_entity_state service.
    """
    # Create a client attached to the navigator's internal node
    # (BasicNavigator wraps a node, we can access it via navigator)
    # If that access is restricted, we create a temporary node or use rclpy direct
    
    # Simplest way: create a raw node just for this service call
    service_node = rclpy.create_node('service_client_temp')
    client = service_node.create_client(SetEntityState, '/set_entity_state')
    
    if not client.wait_for_service(timeout_sec=2.0):
        print("Warning: /set_entity_state service not available. Robot will not teleport.")
        service_node.destroy_node()
        return False

    req = SetEntityState.Request()
    req.state.name = 'burger' # Verify this name in Gazebo
    req.state.pose.position.x = float(x)
    req.state.pose.position.y = float(y)
    req.state.pose.position.z = 0.05
    req.state.pose.orientation.z = math.sin(yaw / 2.0)
    req.state.pose.orientation.w = math.cos(yaw / 2.0)
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(service_node, future)
    
    result = future.result()
    service_node.destroy_node()
    return result.success

def main(args=None):
    rclpy.init(args=args)
    
    navigator = BasicNavigator()
    
    # --- CONFIGURATION ---
    A_x, A_y = -2.0, -0.5
    B_x, B_y = 1.5, 1.0 
    # ---------------------

    A = create_pose(A_x, A_y)
    B = create_pose(B_x, B_y)

    print("=" * 50)
    print(f"Starting Navigation: A({A_x},{A_y}) -> B({B_x},{B_y}) -> A")
    print("=" * 50)

    # 1. Teleport Robot
    print("[1/4] Teleporting robot to A...")
    teleport_robot(navigator, A_x, A_y)
    time.sleep(1.0) 

    # 2. Set Initial Pose (Reset AMCL)
    print("[2/4] Setting Initial Pose for Nav2...")
    navigator.setInitialPose(A)
    
    # 3. Wait for Nav2
    print("[3/4] Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    
    # 4. Go to B
    print("[4/4] Going to B...")
    navigator.goToPose(B)
    
    while not navigator.isTaskComplete():
        pass

    if navigator.getResult() == TaskResult.SUCCEEDED:
        print("✅ Reached B!")
        time.sleep(1.0)
        
        # 5. Go back to A
        print("Returning to A...")
        navigator.goToPose(A)
        while not navigator.isTaskComplete():
            pass
            
        if navigator.getResult() == TaskResult.SUCCEEDED:
            print("✅ Returned to A!")
        else:
            print("❌ Failed return trip.")
    else:
        print("❌ Failed to reach B.")

    # Do NOT shutdown lifecycle, so you can run again
    rclpy.shutdown()

if __name__ == '__main__':
    main()