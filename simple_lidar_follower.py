#!/usr/bin/env python3
"""
Simplified ROS LIDAR Follower for JetBot
This version focuses on basic LIDAR-based navigation without complex dependencies
"""

import rospy
import numpy as np
import time
from enum import Enum

from jetbot import Robot
from sensor_msgs.msg import LaserScan
from opposite_detector import SimpleOppositeDetector
from map_navigator import MapNavigator

class RobotState(Enum):
    DRIVING_STRAIGHT = 1
    TURNING_LEFT = 2
    TURNING_RIGHT = 3
    STOPPING = 4
    OBSTACLE_AVOIDANCE = 5

class SimpleLidarFollower:
    def __init__(self):
        rospy.loginfo("Initializing Simple LIDAR Follower...")
        self.setup_parameters()
        self.initialize_hardware()
        
        # Initialize detector and navigator
        self.detector = SimpleOppositeDetector()
        try:
            self.navigator = MapNavigator("map.json")
        except Exception as e:
            rospy.logwarn(f"Navigator initialization failed: {e}")
            self.navigator = None
        
        # Subscribe to LIDAR data
        rospy.Subscriber('/scan', LaserScan, self.detector.callback)
        rospy.loginfo("Subscribed to /scan topic")
        
        self.current_state = RobotState.DRIVING_STRAIGHT
        self.state_start_time = rospy.get_time()
        
        rospy.loginfo("Simple LIDAR Follower initialized successfully")

    def setup_parameters(self):
        """Setup navigation parameters"""
        self.BASE_SPEED = 0.16
        self.TURN_SPEED = 0.2
        self.OBSTACLE_DISTANCE = 0.5  # meters
        self.TURN_DURATION = 1.0  # seconds
        self.FRONT_SCAN_ANGLE = 60  # degrees
        
        rospy.loginfo("Parameters configured")

    def initialize_hardware(self):
        """Initialize JetBot hardware"""
        try:
            self.robot = Robot()
            rospy.loginfo("JetBot hardware initialized")
        except Exception as e:
            rospy.logwarn(f"Hardware initialization failed: {e}")
            # Use mock robot for testing
            from unittest.mock import Mock
            self.robot = Mock()
            rospy.logwarn("Using mock robot for testing")

    def get_navigation_decision(self):
        """
        Analyze LIDAR data and decide on navigation action
        
        Returns:
            tuple: (action, data) where action is the recommended action
        """
        if self.detector.latest_scan is None:
            return 'stop', {'reason': 'no_scan_data'}
        
        front_dist = self.detector.get_front_distance()
        left_dist = self.detector.get_left_distance()
        right_dist = self.detector.get_right_distance()
        
        # Log distances
        rospy.loginfo_throttle(
            1.0,  # Log every 1 second
            f"Distances - Front: {front_dist:.2f}m, "
            f"Left: {left_dist:.2f}m, Right: {right_dist:.2f}m"
        )
        
        # Decision logic
        if front_dist > self.obstacle_distance:
            return 'forward', {
                'front_dist': front_dist,
                'reason': 'clear_path'
            }
        else:
            # Obstacle detected, choose best direction to turn
            if left_dist > right_dist and left_dist > self.obstacle_distance:
                return 'turn_left', {
                    'left_dist': left_dist,
                    'right_dist': right_dist,
                    'reason': 'obstacle_ahead_turn_left'
                }
            elif right_dist > self.obstacle_distance:
                return 'turn_right', {
                    'left_dist': left_dist,
                    'right_dist': right_dist,
                    'reason': 'obstacle_ahead_turn_right'
                }
            else:
                return 'stop', {
                    'front_dist': front_dist,
                    'left_dist': left_dist,
                    'right_dist': right_dist,
                    'reason': 'no_clear_path'
                }

    def execute_action(self, action, data):
        """
        Execute the navigation action
        
        Args:
            action (str): Action to execute
            data (dict): Additional data about the action
        """
        current_time = rospy.get_time()
        
        if action == 'forward':
            if self.current_state != RobotState.DRIVING_STRAIGHT:
                rospy.loginfo(f"Moving forward - {data.get('reason', '')}")
                self.current_state = RobotState.DRIVING_STRAIGHT
                self.state_start_time = current_time
            
            self.robot.forward(self.BASE_SPEED)
            
        elif action == 'turn_left':
            if (self.current_state != RobotState.TURNING_LEFT or 
                current_time - self.state_start_time > self.TURN_DURATION):
                rospy.loginfo(f"Turning left - {data.get('reason', '')}")
                self.current_state = RobotState.TURNING_LEFT
                self.state_start_time = current_time
            
            if current_time - self.state_start_time < self.TURN_DURATION:
                self.robot.left(self.TURN_SPEED)
            else:
                # Turn completed, go back to straight
                self.current_state = RobotState.DRIVING_STRAIGHT
                self.state_start_time = current_time
                
        elif action == 'turn_right':
            if (self.current_state != RobotState.TURNING_RIGHT or 
                current_time - self.state_start_time > self.TURN_DURATION):
                rospy.loginfo(f"Turning right - {data.get('reason', '')}")
                self.current_state = RobotState.TURNING_RIGHT
                self.state_start_time = current_time
            
            if current_time - self.state_start_time < self.TURN_DURATION:
                self.robot.right(self.TURN_SPEED)
            else:
                # Turn completed, go back to straight
                self.current_state = RobotState.DRIVING_STRAIGHT
                self.state_start_time = current_time
                
        elif action == 'stop':
            if self.current_state != RobotState.STOPPING:
                rospy.loginfo(f"Stopping - {data.get('reason', '')}")
                self.current_state = RobotState.STOPPING
                self.state_start_time = current_time
            
            self.robot.stop()
        
        else:
            rospy.logwarn(f"Unknown action: {action}")
            self.robot.stop()

    def run(self):
        """
        Main navigation loop
        """
        rospy.loginfo("Starting navigation loop...")
        
        rate = rospy.Rate(10)  # 10 Hz
        
        # Wait for initial LIDAR data
        rospy.loginfo("Waiting for LIDAR data...")
        while not rospy.is_shutdown() and self.detector.latest_scan is None:
            rate.sleep()
        
        rospy.loginfo("LIDAR data received, starting navigation")
        
        try:
            while not rospy.is_shutdown():
                # Get navigation decision
                action, data = self.get_navigation_decision()
                
                # Execute action
                self.execute_action(action, data)
                
                # Sleep to maintain loop rate
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Navigation interrupted by user")
        except Exception as e:
            rospy.logerr(f"Navigation error: {e}")
        finally:
            # Stop the robot
            self.robot.stop()
            rospy.loginfo("Robot stopped")

    def shutdown(self):
        """
        Shutdown the follower
        """
        rospy.loginfo("Shutting down LIDAR follower...")
        self.robot.stop()


def main():
    """
    Main function
    """
    try:
        # Initialize ROS
        rospy.init_node('simple_lidar_follower', anonymous=True)
        
        # Create follower
        follower = SimpleLidarFollower()
        
        # Setup shutdown handler
        rospy.on_shutdown(follower.shutdown)
        
        # Run navigation
        follower.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        rospy.logerr(f"Main function error: {e}")
    finally:
        rospy.loginfo("Simple LIDAR Follower finished")


if __name__ == '__main__':
    main()