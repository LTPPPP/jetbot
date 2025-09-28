#!/usr/bin/env python3
"""
Simple Opposite Detector for LIDAR data
Detects obstacles and provides basic navigation guidance
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


class SimpleOppositeDetector:
    """
    Simple detector that analyzes LIDAR data to provide navigation guidance
    """
    
    def __init__(self):
        self.latest_scan = None
        self.min_distance = 0.5  # Minimum safe distance in meters
        self.front_angle_range = 60  # Degrees to check in front
        
    def callback(self, scan_msg):
        """
        Callback function for LIDAR scan data
        
        Args:
            scan_msg (LaserScan): ROS LaserScan message
        """
        self.latest_scan = scan_msg
        
    def get_front_distance(self):
        """
        Get minimum distance in front of the robot
        
        Returns:
            float: Minimum distance in meters, or float('inf') if no valid readings
        """
        if self.latest_scan is None:
            return float('inf')
            
        ranges = np.array(self.latest_scan.ranges)
        angles = np.arange(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            self.latest_scan.angle_increment
        )
        
        # Convert angle range to radians
        front_angle_rad = np.deg2rad(self.front_angle_range / 2)
        
        # Find indices for front-facing measurements
        front_indices = np.where(np.abs(angles) <= front_angle_rad)[0]
        
        if len(front_indices) == 0:
            return float('inf')
            
        front_ranges = ranges[front_indices]
        
        # Filter out invalid readings
        valid_ranges = front_ranges[
            (front_ranges > self.latest_scan.range_min) & 
            (front_ranges < self.latest_scan.range_max) &
            (~np.isinf(front_ranges)) &
            (~np.isnan(front_ranges))
        ]
        
        if len(valid_ranges) == 0:
            return float('inf')
            
        return np.min(valid_ranges)
    
    def get_left_distance(self, angle_deg=45):
        """
        Get distance at specified angle to the left
        
        Args:
            angle_deg (float): Angle in degrees (positive is left)
            
        Returns:
            float: Distance in meters at that angle
        """
        return self._get_distance_at_angle(-angle_deg)
    
    def get_right_distance(self, angle_deg=45):
        """
        Get distance at specified angle to the right
        
        Args:
            angle_deg (float): Angle in degrees (positive is right)
            
        Returns:
            float: Distance in meters at that angle
        """
        return self._get_distance_at_angle(angle_deg)
    
    def _get_distance_at_angle(self, target_angle_deg):
        """
        Get distance measurement at specific angle
        
        Args:
            target_angle_deg (float): Target angle in degrees
            
        Returns:
            float: Distance in meters, or float('inf') if no valid reading
        """
        if self.latest_scan is None:
            return float('inf')
            
        ranges = np.array(self.latest_scan.ranges)
        angles = np.arange(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            self.latest_scan.angle_increment
        )
        
        target_angle_rad = np.deg2rad(target_angle_deg)
        
        # Find closest angle
        angle_diffs = np.abs(angles - target_angle_rad)
        closest_idx = np.argmin(angle_diffs)
        
        distance = ranges[closest_idx]
        
        # Check if reading is valid
        if (distance > self.latest_scan.range_min and 
            distance < self.latest_scan.range_max and
            not np.isinf(distance) and 
            not np.isnan(distance)):
            return distance
        else:
            return float('inf')
    
    def is_obstacle_detected(self):
        """
        Check if obstacle is detected in front
        
        Returns:
            bool: True if obstacle detected within minimum distance
        """
        front_distance = self.get_front_distance()
        return front_distance < self.min_distance
    
    def get_best_direction(self):
        """
        Analyze LIDAR data and suggest best direction
        
        Returns:
            str: 'forward', 'left', 'right', or 'stop'
        """
        front_dist = self.get_front_distance()
        left_dist = self.get_left_distance()
        right_dist = self.get_right_distance()
        
        rospy.loginfo(f"Distances - Front: {front_dist:.2f}m, Left: {left_dist:.2f}m, Right: {right_dist:.2f}m")
        
        # If front is clear, move forward
        if front_dist > self.min_distance:
            return 'forward'
        
        # If obstacle in front, choose side with more space
        if left_dist > right_dist and left_dist > self.min_distance:
            return 'left'
        elif right_dist > self.min_distance:
            return 'right'
        else:
            return 'stop'
    
    def get_scan_data(self):
        """
        Get raw scan data for advanced processing
        
        Returns:
            dict: Dictionary containing scan data and metadata
        """
        if self.latest_scan is None:
            return {
                'ranges': [],
                'angles': [],
                'range_min': 0.0,
                'range_max': 0.0,
                'angle_min': 0.0,
                'angle_max': 0.0
            }
        
        angles = np.arange(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            self.latest_scan.angle_increment
        )
        
        return {
            'ranges': list(self.latest_scan.ranges),
            'angles': list(angles),
            'range_min': self.latest_scan.range_min,
            'range_max': self.latest_scan.range_max,
            'angle_min': self.latest_scan.angle_min,
            'angle_max': self.latest_scan.angle_max,
            'scan_time': self.latest_scan.scan_time
        }