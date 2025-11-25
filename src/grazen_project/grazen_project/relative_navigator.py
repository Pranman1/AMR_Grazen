#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetEntityState  # <--- Added this
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import sys
import os
import time

# --- CONFIGURATION ---
REAL_ROBOT_MODE = False 
WAYPOINT_FILE = '/home/pranav/waypoints.txt' 
# ---------------------

class LocationReader(Node):
    def __init__(self):
        params = []
        if not REAL_ROBOT_MODE:
            params.append(Parameter('use_sim_time', Parameter.Type.BOOL, True))
        
        super().__init__('location_reader', parameter_overrides=params)
        
        # FIX: Tell buffer to use Sim Time clock
        self.tf_buffer = Buffer(clock=self.get_clock()) 
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self):
        try:
            now = rclpy.time.Time() 
            if not self.tf_buffer.can_transform('map', 'base_link', now, timeout=Duration(seconds=1.0)):
                self.get_logger().warn("Transform map->base_link not available yet...")
                return None

            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=Duration(seconds=1.0))
            return [trans.transform.translation.x, trans.transform.translation.y]
        except Exception as e:
            self.get_logger().warn(f"TF Error: {e}")
            return None

def create_pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0 
    return pose

# --- NEW: TELEPORT FUNCTION ADDED BACK ---
def teleport_robot(node, x, y):
    client = node.create_client(SetEntityState, '/set_entity_state')
    if not client.wait_for_service(timeout_sec=2.0):
        print("Warning: /set_entity_state service not available.")
        return False

    req = SetEntityState.Request()
    req.state.name = 'burger' 
    req.state.pose.position.x = float(x)
    req.state.pose.position.y = float(y)
    req.state.pose.position.z = 0.30
    req.state.pose.orientation.w = 1.0
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result().success
# -----------------------------------------

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
    
    # Create temp node for teleport service
    temp_node = rclpy.create_node('teleporter')

    print(f"Reading relative commands from: {WAYPOINT_FILE}")
    moves = read_relative_commands(WAYPOINT_FILE)
    
    if not moves:
        print("No commands found.")
        return

    # 2. SETUP & TELEPORT
    if not REAL_ROBOT_MODE:
        print("Sim Mode: Teleporting to (0,0)...")
        # Move Body
        teleport_robot(temp_node, 0.0, 0.0)
        time.sleep(1.0)
        
        print("Resetting Nav2 Localization...")
        # Move Mind
        navigator.setInitialPose(create_pose(0,0))
        
    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    print("Nav2 is Ready!")

    # 3. Execute Relative Moves
    for i, (dx, dy) in enumerate(moves):
        print(f"\n--- Move {i+1} ---")
        print(f"Command: Go North {dx}m, East {dy}m")

        current_pos = None
        retries = 0
        while current_pos is None and retries < 10:
            rclpy.spin_once(tf_node, timeout_sec=0.5)
            current_pos = tf_node.get_current_pose()
            if current_pos is None:
                print(f"Waiting for robot location (Attempt {retries+1}/10)...")
                retries += 1
            
        if current_pos is None:
            print("❌ CRITICAL: Robot is lost (No TF). Stopping.")
            break

        curr_x, curr_y = current_pos
        print(f"Current Location: ({curr_x:.2f}, {curr_y:.2f})")

        target_x = curr_x + dx
        target_y = curr_y + dy
        print(f"Calculated Goal: ({target_x:.2f}, {target_y:.2f})")

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
    temp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()