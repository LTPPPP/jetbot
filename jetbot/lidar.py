#!/usr/bin/env python3
"""
RPLIDAR integration for JetBot
Provides LIDAR sensor interface using rplidar_ros package
"""

import rospy
import numpy as np
import traitlets
from traitlets.config.configurable import SingletonConfigurable
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import threading
import time


class Lidar(SingletonConfigurable):
    """
    RPLIDAR sensor interface for JetBot
    Provides laser scan data and basic obstacle detection
    """
    
    # Traitlets for scan data
    ranges = traitlets.List(default_value=[]).tag(config=True)
    angles = traitlets.List(default_value=[]).tag(config=True)
    range_min = traitlets.Float(default_value=0.0).tag(config=True)
    range_max = traitlets.Float(default_value=0.0).tag(config=True)
    angle_min = traitlets.Float(default_value=0.0).tag(config=True)
    angle_max = traitlets.Float(default_value=0.0).tag(config=True)
    scan_time = traitlets.Float(default_value=0.0).tag(config=True)
    
    # Obstacle detection
    obstacle_detected = traitlets.Bool(default_value=False).tag(config=True)
    min_obstacle_distance = traitlets.Float(default_value=0.3).tag(config=True)  # 30cm
    obstacle_angle_range = traitlets.Float(default_value=60.0).tag(config=True)  # ±30 degrees
    
    def __init__(self, *args, **kwargs):
        super(Lidar, self).__init__(*args, **kwargs)
        
        try:
            rospy.init_node('jetbot_lidar', anonymous=True)
        except rospy.exceptions.ROSException:
            # Node already initialized
            pass
            
        # Subscribe to laser scan topic
        self.scan_subscriber = rospy.Subscriber(
            '/scan', 
            LaserScan, 
            self._scan_callback,
            queue_size=1
        )
        
        # Service clients for motor control
        try:
            rospy.wait_for_service('/start_motor', timeout=5)
            rospy.wait_for_service('/stop_motor', timeout=5)
            self.start_motor_service = rospy.ServiceProxy('/start_motor', Empty)
            self.stop_motor_service = rospy.ServiceProxy('/stop_motor', Empty)
            self.motor_control_available = True
        except rospy.ROSException:
            rospy.logwarn("RPLIDAR motor control services not available")
            self.motor_control_available = False
        
        # Data processing thread
        self._running = True
        self._data_lock = threading.Lock()
        
        rospy.loginfo("JetBot LIDAR interface initialized")
    
    def _scan_callback(self, scan_msg):
        """
        Callback function for laser scan messages
        """
        with self._data_lock:
            # Update scan data
            self.ranges = list(scan_msg.ranges)
            self.range_min = scan_msg.range_min
            self.range_max = scan_msg.range_max
            self.angle_min = scan_msg.angle_min
            self.angle_max = scan_msg.angle_max
            self.scan_time = scan_msg.scan_time
            
            # Calculate angles for each measurement
            if len(scan_msg.ranges) > 0:
                angle_increment = scan_msg.angle_increment
                self.angles = [
                    scan_msg.angle_min + i * angle_increment 
                    for i in range(len(scan_msg.ranges))
                ]
                
                # Check for obstacles in front
                self._check_obstacles()
    
    def _check_obstacles(self):
        """
        Check for obstacles in the front facing direction
        """
        if not self.ranges or not self.angles:
            return
            
        obstacle_found = False
        front_angle_rad = np.deg2rad(self.obstacle_angle_range / 2)
        
        for i, (distance, angle) in enumerate(zip(self.ranges, self.angles)):
            # Skip invalid readings
            if np.isinf(distance) or np.isnan(distance) or distance <= 0:
                continue
                
            # Check if angle is in front-facing range
            if abs(angle) <= front_angle_rad:
                if distance < self.min_obstacle_distance:
                    obstacle_found = True
                    break
        
        self.obstacle_detected = obstacle_found
    
    def get_front_distance(self, angle_range=30):
        """
        Get minimum distance in front of the robot
        
        Args:
            angle_range (float): Angle range in degrees to check (±angle_range/2)
            
        Returns:
            float: Minimum distance in meters, or float('inf') if no valid readings
        """
        with self._data_lock:
            if not self.ranges or not self.angles:
                return float('inf')
            
            front_distances = []
            front_angle_rad = np.deg2rad(angle_range / 2)
            
            for distance, angle in zip(self.ranges, self.angles):
                if (not np.isinf(distance) and not np.isnan(distance) and 
                    distance > 0 and abs(angle) <= front_angle_rad):
                    front_distances.append(distance)
            
            return min(front_distances) if front_distances else float('inf')
    
    def get_distance_at_angle(self, target_angle_deg):
        """
        Get distance measurement at specific angle
        
        Args:
            target_angle_deg (float): Target angle in degrees
            
        Returns:
            float: Distance in meters, or float('inf') if no valid reading
        """
        with self._data_lock:
            if not self.ranges or not self.angles:
                return float('inf')
            
            target_angle_rad = np.deg2rad(target_angle_deg)
            
            # Find closest angle
            min_diff = float('inf')
            closest_distance = float('inf')
            
            for distance, angle in zip(self.ranges, self.angles):
                angle_diff = abs(angle - target_angle_rad)
                if angle_diff < min_diff and not np.isinf(distance) and not np.isnan(distance):
                    min_diff = angle_diff
                    closest_distance = distance
            
            return closest_distance
    
    def get_scan_data(self):
        """
        Get current scan data
        
        Returns:
            dict: Dictionary containing ranges, angles, and metadata
        """
        with self._data_lock:
            return {
                'ranges': list(self.ranges),
                'angles': list(self.angles),
                'range_min': self.range_min,
                'range_max': self.range_max,
                'angle_min': self.angle_min,
                'angle_max': self.angle_max,
                'scan_time': self.scan_time,
                'obstacle_detected': self.obstacle_detected
            }
    
    def start_motor(self):
        """
        Start RPLIDAR motor
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.motor_control_available:
            rospy.logwarn("Motor control not available")
            return False
            
        try:
            self.start_motor_service()
            rospy.loginfo("RPLIDAR motor started")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to start motor: {e}")
            return False
    
    def stop_motor(self):
        """
        Stop RPLIDAR motor
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.motor_control_available:
            rospy.logwarn("Motor control not available")
            return False
            
        try:
            self.stop_motor_service()
            rospy.loginfo("RPLIDAR motor stopped")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to stop motor: {e}")
            return False
    
    def __del__(self):
        """
        Cleanup when object is destroyed
        """
        self._running = False
        if hasattr(self, 'scan_subscriber'):
            self.scan_subscriber.unregister()


# Global instance for easy access
_lidar_instance = None

def get_lidar():
    """
    Get global LIDAR instance
    
    Returns:
        Lidar: Global LIDAR instance
    """
    global _lidar_instance
    if _lidar_instance is None:
        _lidar_instance = Lidar()
    return _lidar_instance