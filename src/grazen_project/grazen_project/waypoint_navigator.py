#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetEntityState
import math
import time
from visualization_msgs.msg import Marker, MarkerArray

# --- NEW FUNCTION FOR ERROR CALCULATION ---
def get_distance_error(target_pose, actual_pose):
    """Calculates Euclidean distance between two PoseStamped objects."""
    tx = target_pose.pose.position.x
    ty = target_pose.pose.position.y
    
    ax = actual_pose.pose.position.x
    ay = actual_pose.pose.position.y
    
    # Distance formula: sqrt((x2-x1)^2 + (y2-y1)^2)
    error = math.sqrt((tx - ax)**2 + (ty - ay)**2)
    return error

def create_pose(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def teleport_robot(navigator, x, y, yaw=0.0):
    service_node = rclpy.create_node('service_client_temp')
    client = service_node.create_client(SetEntityState, '/set_entity_state')
    
    if not client.wait_for_service(timeout_sec=2.0):
        print("Warning: /set_entity_state service not available. Robot will not teleport.")
        service_node.destroy_node()
        return False

    req = SetEntityState.Request()
    req.state.name = 'burger' # Ensure this matches Gazebo!
    req.state.pose.position.x = float(x)
    req.state.pose.position.y = float(y)
    req.state.pose.position.z = 0.30 # Drop from height to avoid floor clipping
    req.state.pose.orientation.z = math.sin(yaw / 2.0)
    req.state.pose.orientation.w = math.cos(yaw / 2.0)
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(service_node, future)
    
    result = future.result()
    service_node.destroy_node()
    return result.success


def publish_markers(node, pose_a, pose_b):
    """Publishes A (Green) and B (Red) markers to RViz."""
    publisher = node.create_publisher(MarkerArray, '/waypoint_markers', 10)
    markers = MarkerArray()

    def create_marker(id, pose, r, g, b, text):
        m = Marker()
        m.header.frame_id = "map"
        m.id = id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3
        m.color.a = 1.0; m.color.r = r; m.color.g = g; m.color.b = b
        return m

    markers.markers.append(create_marker(0, pose_a, 0.0, 1.0, 0.0, "A"))
    markers.markers.append(create_marker(1, pose_b, 1.0, 0.0, 0.0, "B"))

    # Publish a few times to ensure RViz gets it
    for _ in range(5):
        publisher.publish(markers)
        time.sleep(0.1)



def main(args=None):
    print("Starting Navigation with Error Tracking")
    rclpy.init(args=args)
    navigator = BasicNavigator()
    
    # --- CONFIGURATION ---
    A_x, A_y = -2.0, -0.5
    B_x, B_y = 3.8, -0.6 
    # ---------------------

    A = create_pose(A_x, A_y)
    B = create_pose(B_x, B_y)

    

    print("=" * 50)
    print(f"Starting Navigation with Error Tracking")
    print("=" * 50)

    # 1. Teleport
    print("[1/4] Teleporting...")
    teleport_robot(navigator, A_x, A_y)
    time.sleep(1.0) 
    
    # 2. Reset Nav2
    print("[2/4] Resetting Nav2 Pose...")
    navigator.setInitialPose(A)
    navigator.waitUntilNav2Active()
    
    # 3. Go to B
    print("[3/4] Going to B...")
    navigator.goToPose(B)
    
    # --- TRACKING LOOP ---
    final_pose_b = None
    while not navigator.isTaskComplete():
        # Capture the latest feedback from Nav2
        feedback = navigator.getFeedback()
        if feedback:
            final_pose_b = feedback.current_pose
            # Optional: Print live distance remaining
            # print(f'Distance remaining: {feedback.distance_remaining:.2f}m', end='\r')

    if navigator.getResult() == TaskResult.SUCCEEDED and final_pose_b:
        # CALCULATE ERROR
        error_b = get_distance_error(B, final_pose_b)
        print(f"\n✅ Reached B!")
        print(f"   Target: ({B_x}, {B_y})")
        print(f"   Actual: ({final_pose_b.pose.position.x:.3f}, {final_pose_b.pose.position.y:.3f})")
        print(f"   Error (Drift): {error_b:.4f} meters") # 
    else:
        print("❌ Failed to reach B.")

    time.sleep(1.0)

    # 4. Return to A
    print("\n[4/4] Returning to A...")
    navigator.goToPose(A)
    
    final_pose_a = None
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            final_pose_a = feedback.current_pose

    if navigator.getResult() == TaskResult.SUCCEEDED and final_pose_a:
        # CALCULATE ERROR
        error_a = get_distance_error(A, final_pose_a)
        print(f"\n✅ Returned to A!")
        print(f"   Target: ({A_x}, {A_y})")
        print(f"   Actual: ({final_pose_a.pose.position.x:.3f}, {final_pose_a.pose.position.y:.3f})")
        print(f"   Error (Drift): {error_a:.4f} meters")
        
        print("-" * 30)
        print(f"Total Round Trip Error: {error_b + error_a:.4f} meters")
    else:
        print("❌ Failed return trip.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()