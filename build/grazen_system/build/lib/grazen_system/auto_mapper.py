#!/usr/bin/env python3
"""
AUTO MAPPER NODE

Purpose:
    Monitors explore_lite during autonomous mapping and automatically
    saves the map when exploration is complete (no more frontiers).

How it works:
    1. Subscribes to /explore/frontiers (visualization markers from explore_lite)
    2. Tracks time since last frontier was seen
    3. If no frontiers for 'completion_timeout' seconds → exploration complete
    4. Triggers map save using nav2_map_server
    5. Optionally shuts down the exploration

Usage:
    Launched automatically with: mode:=real/sim task:=map auto_map:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
import subprocess
import time
import os
import signal


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')
        
        # Parameters
        self.declare_parameter('map_path', '')
        self.declare_parameter('completion_timeout', 30.0)  # Seconds with no frontiers = done
        self.declare_parameter('save_interval', 60.0)       # Periodic save every N seconds (0 = disabled)
        
        self.map_path = self.get_parameter('map_path').value
        self.completion_timeout = self.get_parameter('completion_timeout').value
        self.save_interval = self.get_parameter('save_interval').value
        
        # State tracking
        self.last_frontier_time = time.time()
        self.last_save_time = time.time()
        self.exploration_complete = False
        self.frontier_count = 0
        
        # QoS for visualization markers (match explore_lite's publisher)
        marker_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to explore_lite's frontier markers
        self.subscription = self.create_subscription(
            MarkerArray,
            '/explore/frontiers',
            self.frontier_callback,
            marker_qos
        )
        
        # Timer to check exploration status
        self.timer = self.create_timer(2.0, self.check_status)
        
        self.get_logger().info(f'🗺️ Auto Mapper initialized')
        self.get_logger().info(f'   Map will save to: {self.map_path}')
        self.get_logger().info(f'   Completion timeout: {self.completion_timeout}s')
        if self.save_interval > 0:
            self.get_logger().info(f'   Periodic save every: {self.save_interval}s')

    def frontier_callback(self, msg):
        """Called when explore_lite publishes frontier markers."""
        # Count actual frontier markers (not delete markers)
        active_frontiers = [m for m in msg.markers if m.action != 2]  # action=2 is DELETE
        self.frontier_count = len(active_frontiers)
        
        if self.frontier_count > 0:
            self.last_frontier_time = time.time()

    def check_status(self):
        """Periodically check if exploration is complete."""
        if self.exploration_complete:
            return
            
        current_time = time.time()
        time_since_frontier = current_time - self.last_frontier_time
        time_since_save = current_time - self.last_save_time
        
        # Periodic save (backup during exploration)
        if self.save_interval > 0 and time_since_save >= self.save_interval:
            self.get_logger().info(f'💾 Periodic save (backup)...')
            self.save_map(suffix='_backup')
            self.last_save_time = current_time
        
        # Check for completion
        if time_since_frontier >= self.completion_timeout:
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'✅ EXPLORATION COMPLETE!')
            self.get_logger().info(f'   No frontiers detected for {self.completion_timeout}s')
            self.get_logger().info('=' * 50)
            
            self.exploration_complete = True
            self.save_map()
            
            self.get_logger().info('🏁 Auto mapping finished. Shutting down in 3 seconds...')
            time.sleep(3)
            
            # Send SIGINT to entire process group (simulates Ctrl+C, stops all nodes)
            os.killpg(os.getpgid(os.getpid()), signal.SIGINT)
        else:
            # Status update
            self.get_logger().info(
                f'📍 Exploring... Frontiers: {self.frontier_count} | '
                f'Time since last frontier: {time_since_frontier:.1f}s / {self.completion_timeout}s',
                throttle_duration_sec=10.0
            )

    def save_map(self, suffix=''):
        """Save the map using nav2_map_server."""
        if not self.map_path:
            self.get_logger().error("No map path provided!")
            return False
            
        save_path = f"{self.map_path}{suffix}"
        self.get_logger().info(f'💾 SAVING MAP TO: {save_path}')
        
        try:
            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", save_path],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                self.get_logger().info(f'✅ Map saved successfully: {save_path}.yaml')
                return True
            else:
                self.get_logger().error(f'❌ Map save failed: {result.stderr}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('❌ Map save timed out!')
            return False
        except Exception as e:
            self.get_logger().error(f'❌ Map save error: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Auto Mapper...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

