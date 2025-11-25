#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetEntityState
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import sys
import os
import time

# --- CONFIGURATION ---
# TRUE = Real Robot (You must manually localize in RViz first)
# FALSE = Simulation (Script asks Gazebo where the robot is)
REAL_ROBOT = False  

WAYPOINT_FILE = '/home/pranav/waypoints.txt' 
# ---------------------

class LocationReader(Node):
    def __init__(self):
        params = []
        if not REAL_ROBOT:
            params.append(Parameter('use_sim_time', Parameter.Type.BOOL, True))
        
        super().__init__('location_reader', parameter_overrides=params)
        
        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self):
        try:
            now = rclpy.time.Time() 
            if not self.tf_buffer.can_transform('map', 'base_link', now, timeout=Duration(seconds=1.0)):
                return None
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=Duration(seconds=1.0))
            return [trans.transform.translation.x, trans.transform.translation.y]
        except Exception:
            return None

def get_true_sim_pose(node):
    """ [SIM ONLY] Asks Gazebo where the 'burger' actually is. """
    client = node.create_client(GetEntityState, '/get_entity_state')
    if not client.wait_for_service(timeout_sec=1.0):
        print("⚠️  Gazebo service not found. Defaulting to (0,0).")
        return None

    req = GetEntityState.Request()
    req.name = 'burger' # Matches your Gazebo model name
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()
    
    if res.success:
        return res.state.pose
    return None

def create_pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0 
    return pose

def read_relative_commands(filepath):
    moves = []
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
                dx = float(parts[0])
                dy = float(parts[1])
                moves.append((dx, dy))
            except ValueError:
                pass 
    return moves

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    tf_node = LocationReader()
    
    print(f"Reading relative commands from: {WAYPOINT_FILE}")
    moves = read_relative_commands(WAYPOINT_FILE)
    
    if not moves:
        print("No commands found.")
        return

    # --- INTELLIGENT INITIALIZATION ---
    if not REAL_ROBOT:
        print("[SIM MODE] Auto-detecting robot position...")
        # Create temp node to ask Gazebo for truth
        temp_node = rclpy.create_node('temp_locator')
        true_pose = get_true_sim_pose(temp_node)
        temp_node.destroy_node()
        
        if true_pose:
            print(f"   Found robot at: ({true_pose.position.x:.2f}, {true_pose.position.y:.2f})")
            # Seed Nav2 with the ACTUAL position
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose = true_pose
            navigator.setInitialPose(initial_pose)
        else:
            print("   Could not find robot in Gazebo. Using default (0,0).")
            navigator.setInitialPose(create_pose(0,0))
    else:
        print("[REAL MODE] Assuming robot is already localized manually.")

    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    print("Nav2 is Ready!")

    # --- EXECUTION ---
    for i, (dx, dy) in enumerate(moves):
        print(f"\n--- Move {i+1}: Relative Go X+{dx}, Y+{dy} ---")

        current_pos = None
        retries = 0
        while current_pos is None and retries < 20:
            rclpy.spin_once(tf_node, timeout_sec=0.2)
            current_pos = tf_node.get_current_pose()
            if current_pos is None:
                retries += 1
                time.sleep(0.2)
            
        if current_pos is None:
            print("❌ CRITICAL: Robot is lost (No TF). Stopping.")
            break

        curr_x, curr_y = current_pos
        print(f"Current: ({curr_x:.2f}, {curr_y:.2f})")

        target_x = curr_x + dx
        target_y = curr_y + dy
        print(f"Target:  ({target_x:.2f}, {target_y:.2f})")

        goal_pose = create_pose(target_x, target_y)
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            pass

        if navigator.getResult() == TaskResult.SUCCEEDED:
            print(f"✅ Move {i+1} Complete")
        else:
            print(f"❌ Move {i+1} Failed")

    print("\nMission Complete!")
    tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()