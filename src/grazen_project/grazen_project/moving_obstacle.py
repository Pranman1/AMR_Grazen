#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState
import math
import time

# --- CONFIGURATION ---
OBSTACLE_NAME = "dynamic_block"
MOVE_SPEED = 0.5  # How fast it moves
MOVE_RANGE = 1.5   # How far it slides (meters)
CENTER_X = 1.0     # Center of the slide
CENTER_Y = 0.0     # Center of the slide
# ---------------------

# A Simple Red Box Model (SDF Format)
BOX_SDF = """
<sdf version='1.6'>
  <model name='dynamic_block'>
    <pose>0 0 0.5 0 0 0</pose>
    <link name='link'>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.16</ixx> <ixy>0</ixy> <ixz>0</ixz>
          <iyy>0.16</iyy> <iyz>0</iyz> <izz>0.16</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('obstacle_puppeteer')

    # 1. Create Clients
    spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
    move_client = node.create_client(SetEntityState, '/set_entity_state')

    print("Waiting for Gazebo...")
    spawn_client.wait_for_service()
    move_client.wait_for_service()

    # 2. Spawn the Box (if not already there)
    print("Spawning Obstacle Box...")
    spawn_req = SpawnEntity.Request()
    spawn_req.name = OBSTACLE_NAME
    spawn_req.xml = BOX_SDF
    
    # We try to spawn. If it fails (already exists), we just ignore and start moving it.
    future = spawn_client.call_async(spawn_req)
    rclpy.spin_until_future_complete(node, future)

    # 3. The Animation Loop
    print("Starting Animation Loop (Ctrl+C to stop)")
    start_time = time.time()
    
    try:
        while rclpy.ok():
            # Calculate smooth sine wave motion
            now = time.time() - start_time
            # y = Center + (Range * sin(Speed * time))
            new_y = CENTER_Y + (MOVE_RANGE * math.sin(MOVE_SPEED * now))
            
            # Create State Request
            state_req = SetEntityState.Request()
            state_req.state.name = OBSTACLE_NAME
            state_req.state.pose.position.x = float(CENTER_X)
            state_req.state.pose.position.y = float(new_y)
            state_req.state.pose.position.z = 0.5 # Keep it on the floor
            
            # Keep rotation zero
            state_req.state.pose.orientation.w = 1.0
            
            # Send Update (Async to be fast)
            move_client.call_async(state_req)
            
            # Run at ~30Hz for smoothness
            time.sleep(0.033)

    except KeyboardInterrupt:
        print("\nStopping Animation.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()