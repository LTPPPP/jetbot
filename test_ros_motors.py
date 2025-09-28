#!/usr/bin/env python3
"""
Simple ROS motor control test
Test different methods to control JetBot motors through ROS
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time


class ROSMotorTest:
    def __init__(self):
        rospy.init_node('jetbot_motor_test', anonymous=True)
        
        # Publishers for different control methods
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.motor_left_vel_pub = rospy.Publisher('/motor/lvel', Float64, queue_size=1)
        self.motor_right_vel_pub = rospy.Publisher('/motor/rvel', Float64, queue_size=1)
        self.motor_left_set_pub = rospy.Publisher('/motor/lset', Float64, queue_size=1)
        self.motor_right_set_pub = rospy.Publisher('/motor/rset', Float64, queue_size=1)
        
        rospy.loginfo("Motor test node initialized")
        time.sleep(1)  # Wait for publishers to connect
    
    def test_cmd_vel(self):
        """Test using cmd_vel topic"""
        rospy.loginfo("=== Testing cmd_vel topic ===")
        
        # Move forward
        rospy.loginfo("Moving forward for 2 seconds...")
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        
        for _ in range(20):  # 2 seconds at 10 Hz
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        rospy.loginfo("Stopping...")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(1)
        
        # Turn right
        rospy.loginfo("Turning right for 1 second...")
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        
        for _ in range(10):  # 1 second at 10 Hz
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(1)
    
    def test_individual_motors_vel(self):
        """Test using individual motor velocity topics"""
        rospy.loginfo("=== Testing individual motor velocity topics ===")
        
        # Move forward
        rospy.loginfo("Moving forward for 2 seconds...")
        left_vel = Float64(0.2)
        right_vel = Float64(0.2)
        
        for _ in range(20):
            self.motor_left_vel_pub.publish(left_vel)
            self.motor_right_vel_pub.publish(right_vel)
            time.sleep(0.1)
        
        # Stop
        rospy.loginfo("Stopping...")
        left_vel.data = 0.0
        right_vel.data = 0.0
        self.motor_left_vel_pub.publish(left_vel)
        self.motor_right_vel_pub.publish(right_vel)
        time.sleep(1)
        
        # Turn right (left motor faster)
        rospy.loginfo("Turning right for 1 second...")
        left_vel.data = 0.3
        right_vel.data = -0.3
        
        for _ in range(10):
            self.motor_left_vel_pub.publish(left_vel)
            self.motor_right_vel_pub.publish(right_vel)
            time.sleep(0.1)
        
        # Stop
        left_vel.data = 0.0
        right_vel.data = 0.0
        self.motor_left_vel_pub.publish(left_vel)
        self.motor_right_vel_pub.publish(right_vel)
        time.sleep(1)
    
    def test_individual_motors_set(self):
        """Test using individual motor set topics"""
        rospy.loginfo("=== Testing individual motor set topics ===")
        
        # Move forward
        rospy.loginfo("Moving forward for 2 seconds...")
        left_set = Float64(0.2)
        right_set = Float64(0.2)
        
        for _ in range(20):
            self.motor_left_set_pub.publish(left_set)
            self.motor_right_set_pub.publish(right_set)
            time.sleep(0.1)
        
        # Stop
        rospy.loginfo("Stopping...")
        left_set.data = 0.0
        right_set.data = 0.0
        self.motor_left_set_pub.publish(left_set)
        self.motor_right_set_pub.publish(right_set)
        time.sleep(1)
        
        # Turn right
        rospy.loginfo("Turning right for 1 second...")
        left_set.data = 0.3
        right_set.data = -0.3
        
        for _ in range(10):
            self.motor_left_set_pub.publish(left_set)
            self.motor_right_set_pub.publish(right_set)
            time.sleep(0.1)
        
        # Stop
        left_set.data = 0.0
        right_set.data = 0.0
        self.motor_left_set_pub.publish(left_set)
        self.motor_right_set_pub.publish(right_set)
        time.sleep(1)
    
    def run_tests(self):
        """Run all motor control tests"""
        rospy.loginfo("Starting motor control tests")
        rospy.loginfo("Make sure the robot has space to move!")
        rospy.loginfo("Starting in 3 seconds...")
        time.sleep(3)
        
        try:
            # Test cmd_vel
            self.test_cmd_vel()
            time.sleep(2)
            
            # Test individual motor velocity
            self.test_individual_motors_vel()
            time.sleep(2)
            
            # Test individual motor set
            self.test_individual_motors_set()
            
            rospy.loginfo("All tests completed!")
            
        except KeyboardInterrupt:
            rospy.loginfo("Test interrupted by user")
        except Exception as e:
            rospy.logerr(f"Test error: {e}")
        finally:
            # Make sure robot stops
            self.stop_all_motors()
    
    def stop_all_motors(self):
        """Stop all motors using all methods"""
        rospy.loginfo("Stopping all motors...")
        
        # cmd_vel
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Individual motors
        zero = Float64(0.0)
        self.motor_left_vel_pub.publish(zero)
        self.motor_right_vel_pub.publish(zero)
        self.motor_left_set_pub.publish(zero)
        self.motor_right_set_pub.publish(zero)


def main():
    try:
        motor_test = ROSMotorTest()
        motor_test.run_tests()
    except rospy.ROSInterruptException:
        rospy.loginfo("Motor test node interrupted")
    except Exception as e:
        rospy.logerr(f"Motor test error: {e}")


if __name__ == '__main__':
    main()