#!/usr/bin/env python3
"""
LASER SCAN RESAMPLER NODE

Purpose: 
    Convert variable-count laser scans (250-256 readings) from real LDS-02
    into fixed-count scans (360 readings) for SLAM Toolbox compatibility.

Problem:
    - Real LDS-02: produces 250-256 readings/scan (motor timing variations)
    - SLAM Toolbox: locks to first scan's count, rejects mismatches
    - Result: intermittent mapping (works/stops/works/stops)

Solution:
    - Subscribe to /scan (variable count)
    - Resample via linear interpolation to 360 readings (fixed count)
    - Publish to /scan_filtered (constant count)
    - SLAM Toolbox uses /scan_filtered → continuous mapping

Method:
    Linear interpolation between adjacent measurements.
    Example: 250 input readings → 360 output readings
        Input angles:  [0°, 1.44°, 2.88°, 4.32°, ...]
        Output angles: [0°, 1.0°, 2.0°, 3.0°, 4.0°, ...]
        Values at 1°, 2°, 3° computed via interpolation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
import numpy as np


class ScanResampler(Node):
    def __init__(self):
        super().__init__('scan_resampler')
        
        # Target: 360 readings (1° resolution for full 360° coverage)
        self.target_num_readings = 360
        
        # QoS Profile for sensor data (matches LDS-02 publisher settings)
        # BEST_EFFORT reliability is standard for laser scanners (low latency, tolerates packet loss)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match sensor's QoS
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to raw variable-count scans (with sensor-compatible QoS)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos  # Use BEST_EFFORT to match LDS-02
        )
        
        # Publish fixed-count resampled scans (also use sensor QoS for compatibility)
        self.publisher = self.create_publisher(LaserScan, 'scan_filtered', sensor_qos)
        
        self.get_logger().info(f'Scan Resampler initialized: resampling to {self.target_num_readings} readings')

    def scan_callback(self, msg):
        """
        Resample incoming scan to fixed number of readings via interpolation.
        
        Process:
        1. Extract input angles and ranges
        2. Create target angles (evenly spaced, fixed count)
        3. Interpolate ranges at target angles
        4. Publish resampled scan
        """
        
        # Input scan parameters
        num_input = len(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        
        # Create input angle array
        input_angles = np.linspace(angle_min, angle_max, num_input)
        input_ranges = np.array(msg.ranges)
        
        # Create target angle array (fixed 360 readings)
        target_angles = np.linspace(angle_min, angle_max, self.target_num_readings)
        
        # Interpolate: estimate range values at target angles from input data
        # np.interp does linear interpolation between known points
        target_ranges = np.interp(target_angles, input_angles, input_ranges)
        
        # Create output message (copy most fields, update ranges)
        resampled_msg = LaserScan()
        resampled_msg.header = msg.header
        resampled_msg.angle_min = angle_min
        resampled_msg.angle_max = angle_max
        resampled_msg.angle_increment = (angle_max - angle_min) / (self.target_num_readings - 1)
        resampled_msg.time_increment = msg.time_increment
        resampled_msg.scan_time = msg.scan_time
        resampled_msg.range_min = msg.range_min
        resampled_msg.range_max = msg.range_max
        resampled_msg.ranges = target_ranges.tolist()
        resampled_msg.intensities = []  # Optional, not needed for SLAM
        
        self.publisher.publish(resampled_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanResampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

