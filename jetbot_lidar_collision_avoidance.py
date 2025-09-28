#!/usr/bin/env python3
"""
JetBot LIDAR collision avoidance example
Uses RPLIDAR sensor for obstacle detection and navigation
"""

import time
import numpy as np
from jetbot import Robot, get_lidar
import rospy


class LidarCollisionAvoidance:
    """
    LIDAR-based collision avoidance for JetBot
    """
    
    def __init__(self, 
                 base_speed=0.3,
                 obstacle_distance=0.5,
                 turn_speed=0.2,
                 scan_angle=60):
        """
        Initialize collision avoidance system
        
        Args:
            base_speed (float): Forward movement speed
            obstacle_distance (float): Distance threshold for obstacle detection
            turn_speed (float): Turning speed when avoiding obstacles
            scan_angle (float): Angle range to scan in front (degrees)
        """
        self.robot = Robot()
        self.lidar = get_lidar()
        
        self.base_speed = base_speed
        self.obstacle_distance = obstacle_distance
        self.turn_speed = turn_speed
        self.scan_angle = scan_angle
        
        self.running = False
        
        # Start LIDAR motor
        if not self.lidar.start_motor():
            rospy.logwarn("Could not start LIDAR motor, continuing anyway...")
    
    def get_navigation_decision(self):
        """
        Analyze LIDAR data and decide navigation action
        
        Returns:
            tuple: (action, turn_direction) where action is 'forward', 'turn', or 'stop'
                  and turn_direction is 'left', 'right', or None
        """
        # Get distances in different directions
        front_distance = self.lidar.get_front_distance(self.scan_angle)
        left_distance = self.lidar.get_distance_at_angle(-45)  # 45 degrees left
        right_distance = self.lidar.get_distance_at_angle(45)  # 45 degrees right
        
        rospy.loginfo(f"Distances - Front: {front_distance:.2f}m, "
                     f"Left: {left_distance:.2f}m, Right: {right_distance:.2f}m")
        
        # If no valid readings, stop
        if np.isinf(front_distance) and np.isinf(left_distance) and np.isinf(right_distance):
            return 'stop', None
        
        # If front is clear, move forward
        if front_distance > self.obstacle_distance:
            return 'forward', None
        
        # If obstacle in front, decide which way to turn
        if left_distance > right_distance:
            return 'turn', 'left'
        else:
            return 'turn', 'right'
    
    def execute_action(self, action, turn_direction):
        """
        Execute the navigation action
        
        Args:
            action (str): Action to take ('forward', 'turn', 'stop')
            turn_direction (str): Direction to turn ('left', 'right', or None)
        """
        if action == 'forward':
            rospy.loginfo("Moving forward")
            self.robot.forward(self.base_speed)
        elif action == 'turn':
            if turn_direction == 'left':
                rospy.loginfo("Turning left")
                self.robot.left(self.turn_speed)
            elif turn_direction == 'right':
                rospy.loginfo("Turning right")
                self.robot.right(self.turn_speed)
        elif action == 'stop':
            rospy.loginfo("Stopping - no clear path")
            self.robot.stop()
        else:
            self.robot.stop()
    
    def run(self, duration=None):
        """
        Run collision avoidance behavior
        
        Args:
            duration (float): How long to run (None for indefinite)
        """
        rospy.loginfo("Starting LIDAR collision avoidance")
        self.running = True
        
        start_time = time.time()
        
        try:
            while self.running and not rospy.is_shutdown():
                # Check duration limit
                if duration and (time.time() - start_time) > duration:
                    break
                
                # Get navigation decision
                action, turn_direction = self.get_navigation_decision()
                
                # Execute action
                self.execute_action(action, turn_direction)
                
                # Small delay to prevent too rapid changes
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            rospy.loginfo("Collision avoidance interrupted")
        finally:
            self.stop()
    
    def stop(self):
        """
        Stop collision avoidance behavior
        """
        rospy.loginfo("Stopping collision avoidance")
        self.running = False
        self.robot.stop()
    
    def __del__(self):
        """
        Cleanup when destroyed
        """
        self.stop()
        if hasattr(self, 'lidar'):
            self.lidar.stop_motor()


def main():
    """
    Main function for running collision avoidance
    """
    try:
        # Initialize ROS
        rospy.init_node('jetbot_lidar_collision_avoidance', anonymous=True)
        
        # Create collision avoidance system
        collision_avoidance = LidarCollisionAvoidance(
            base_speed=0.2,
            obstacle_distance=0.6,
            turn_speed=0.15,
            scan_angle=90
        )
        
        # Wait for LIDAR data
        rospy.loginfo("Waiting for LIDAR data...")
        time.sleep(2)
        
        # Run collision avoidance
        collision_avoidance.run()
        
    except Exception as e:
        rospy.logerr(f"Error in collision avoidance: {e}")
    finally:
        rospy.loginfo("Collision avoidance finished")


if __name__ == '__main__':
    main()