#!/usr/bin/env python3
"""
JetBot ROS-based LIDAR collision avoidance
Uses ROS topics for motor control instead of direct hardware control
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class ROSLidarCollisionAvoidance:
    """
    LIDAR-based collision avoidance for JetBot using ROS topics
    """
    
    def __init__(self):
        rospy.init_node('jetbot_lidar_collision_avoidance', anonymous=True)
        
        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)
        self.angular_speed = rospy.get_param('~angular_speed', 0.8)
        self.obstacle_distance = rospy.get_param('~obstacle_distance', 0.6)
        self.scan_angle = rospy.get_param('~scan_angle', 60)  # degrees
        self.rate = rospy.get_param('~rate', 10)  # Hz
        
        # State variables
        self.latest_scan = None
        self.running = False
        
        # ROS Publishers - try different topic patterns
        self.cmd_vel_pub = None
        self.motor_left_pub = None
        self.motor_right_pub = None
        
        # Try to find the right motor control method
        self.setup_motor_publishers()
        
        # ROS Subscribers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Control rate
        self.rate_obj = rospy.Rate(self.rate)
        
        rospy.loginfo("ROS LIDAR Collision Avoidance initialized")
        rospy.loginfo(f"Linear speed: {self.linear_speed}, Angular speed: {self.angular_speed}")
        rospy.loginfo(f"Obstacle distance: {self.obstacle_distance}m, Scan angle: ±{self.scan_angle/2}°")
    
    def setup_motor_publishers(self):
        """Setup motor control publishers based on available topics"""
        
        # Method 1: Try cmd_vel (most common for mobile robots)
        try:
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            rospy.loginfo("Using /cmd_vel topic for motor control")
            return
        except:
            pass
        
        # Method 2: Try individual motor topics (based on your topic list)
        try:
            self.motor_left_pub = rospy.Publisher('/motor/lvel', Float64, queue_size=1)
            self.motor_right_pub = rospy.Publisher('/motor/rvel', Float64, queue_size=1)
            rospy.loginfo("Using individual motor velocity topics (/motor/lvel, /motor/rvel)")
            return
        except:
            pass
        
        # Method 3: Try alternative motor topics
        try:
            self.motor_left_pub = rospy.Publisher('/motor/lset', Float64, queue_size=1)
            self.motor_right_pub = rospy.Publisher('/motor/rset', Float64, queue_size=1)
            rospy.loginfo("Using individual motor set topics (/motor/lset, /motor/rset)")
            return
        except:
            pass
        
        rospy.logwarn("Could not find suitable motor control topics!")
    
    def scan_callback(self, scan_msg):
        """Callback for LIDAR scan data"""
        self.latest_scan = scan_msg
    
    def get_front_distance(self, angle_range_deg=None):
        """Get minimum distance in front of the robot"""
        if self.latest_scan is None:
            return float('inf')
        
        if angle_range_deg is None:
            angle_range_deg = self.scan_angle
        
        angle_range_rad = math.radians(angle_range_deg / 2)
        ranges = np.array(self.latest_scan.ranges)
        
        # Calculate angles for each measurement
        angles = np.linspace(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            len(ranges)
        )
        
        # Find measurements in front (within angle range)
        front_mask = np.abs(angles) <= angle_range_rad
        front_ranges = ranges[front_mask]
        
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
    
    def get_side_distances(self):
        """Get distances on left and right sides"""
        if self.latest_scan is None:
            return float('inf'), float('inf')
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            len(ranges)
        )
        
        # Left side (positive angles, around +90 degrees)
        left_angle_target = math.radians(90)
        left_tolerance = math.radians(30)
        left_mask = np.abs(angles - left_angle_target) <= left_tolerance
        left_ranges = ranges[left_mask]
        left_valid = left_ranges[
            (left_ranges > self.latest_scan.range_min) &
            (left_ranges < self.latest_scan.range_max) &
            (~np.isinf(left_ranges)) &
            (~np.isnan(left_ranges))
        ]
        left_distance = np.min(left_valid) if len(left_valid) > 0 else float('inf')
        
        # Right side (negative angles, around -90 degrees)
        right_angle_target = math.radians(-90)
        right_tolerance = math.radians(30)
        right_mask = np.abs(angles - right_angle_target) <= right_tolerance
        right_ranges = ranges[right_mask]
        right_valid = right_ranges[
            (right_ranges > self.latest_scan.range_min) &
            (right_ranges < self.latest_scan.range_max) &
            (~np.isinf(right_ranges)) &
            (~np.isnan(right_ranges))
        ]
        right_distance = np.min(right_valid) if len(right_valid) > 0 else float('inf')
        
        return left_distance, right_distance
    
    def publish_cmd_vel(self, linear_x, angular_z):
        """Publish velocity command using cmd_vel topic"""
        if self.cmd_vel_pub is not None:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            return True
        return False
    
    def publish_motor_velocities(self, left_vel, right_vel):
        """Publish individual motor velocities"""
        success = False
        
        # Try velocity topics first
        if self.motor_left_pub is not None and self.motor_right_pub is not None:
            try:
                self.motor_left_pub.publish(Float64(left_vel))
                self.motor_right_pub.publish(Float64(right_vel))
                success = True
            except:
                pass
        
        return success
    
    def move_forward(self):
        """Move robot forward"""
        rospy.loginfo("Moving forward")
        
        # Method 1: Try cmd_vel
        if self.publish_cmd_vel(self.linear_speed, 0.0):
            return
        
        # Method 2: Try individual motor control
        if self.publish_motor_velocities(self.linear_speed, self.linear_speed):
            return
        
        rospy.logwarn("Could not send forward command")
    
    def turn_left(self):
        """Turn robot left"""
        rospy.loginfo("Turning left")
        
        # Method 1: Try cmd_vel
        if self.publish_cmd_vel(0.0, self.angular_speed):
            return
        
        # Method 2: Try individual motor control (differential drive)
        if self.publish_motor_velocities(-self.angular_speed/2, self.angular_speed/2):
            return
        
        rospy.logwarn("Could not send left turn command")
    
    def turn_right(self):
        """Turn robot right"""
        rospy.loginfo("Turning right")
        
        # Method 1: Try cmd_vel
        if self.publish_cmd_vel(0.0, -self.angular_speed):
            return
        
        # Method 2: Try individual motor control (differential drive)
        if self.publish_motor_velocities(self.angular_speed/2, -self.angular_speed/2):
            return
        
        rospy.logwarn("Could not send right turn command")
    
    def stop(self):
        """Stop robot"""
        rospy.loginfo("Stopping")
        
        # Method 1: Try cmd_vel
        if self.publish_cmd_vel(0.0, 0.0):
            return
        
        # Method 2: Try individual motor control
        if self.publish_motor_velocities(0.0, 0.0):
            return
        
        rospy.logwarn("Could not send stop command")
    
    def get_navigation_decision(self):
        """Analyze LIDAR data and decide navigation action"""
        front_distance = self.get_front_distance()
        left_distance, right_distance = self.get_side_distances()
        
        rospy.loginfo(
            f"Distances - Front: {front_distance:.2f}m, "
            f"Left: {left_distance:.2f}m, Right: {right_distance:.2f}m"
        )
        
        # If no valid readings, stop
        if (np.isinf(front_distance) and 
            np.isinf(left_distance) and 
            np.isinf(right_distance)):
            return 'stop'
        
        # If front is clear, move forward
        if front_distance > self.obstacle_distance:
            return 'forward'
        
        # If obstacle in front, decide which way to turn
        if left_distance > right_distance:
            return 'left'
        else:
            return 'right'
    
    def execute_action(self, action):
        """Execute the navigation action"""
        if action == 'forward':
            self.move_forward()
        elif action == 'left':
            self.turn_left()
        elif action == 'right':
            self.turn_right()
        elif action == 'stop':
            self.stop()
        else:
            self.stop()
    
    def run(self):
        """Main control loop"""
        rospy.loginfo("Starting ROS LIDAR collision avoidance")
        
        # Wait for initial scan data
        rospy.loginfo("Waiting for LIDAR data...")
        while self.latest_scan is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo("LIDAR data received, starting navigation")
        self.running = True
        
        try:
            while not rospy.is_shutdown() and self.running:
                # Get navigation decision
                action = self.get_navigation_decision()
                
                # Execute action
                self.execute_action(action)
                
                # Sleep according to rate
                self.rate_obj.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation interrupted")
        except KeyboardInterrupt:
            rospy.loginfo("Navigation stopped by user")
        finally:
            self.stop()
            rospy.loginfo("Navigation stopped")
    
    def shutdown(self):
        """Shutdown the navigation system"""
        rospy.loginfo("Shutting down collision avoidance")
        self.running = False
        self.stop()


def main():
    """Main function"""
    try:
        collision_avoidance = ROSLidarCollisionAvoidance()
        collision_avoidance.run()
    except Exception as e:
        rospy.logerr(f"Error in collision avoidance: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()