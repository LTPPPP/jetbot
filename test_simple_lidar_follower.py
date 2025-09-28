#!/usr/bin/env python3
"""
Simplified LIDAR Follower Runner
Quick test script for running LIDAR-based navigation
"""

import rospy
import time
from jetbot import Robot
from opposite_detector import SimpleOppositeDetector


def main():
    """Simple LIDAR follower test"""
    print("🤖 Starting Simple LIDAR Follower Test...")
    
    try:
        # Initialize ROS
        rospy.init_node('lidar_follower_test', anonymous=True)
        
        # Initialize components
        robot = Robot()

        detector = SimpleOppositeDetector()
        
        # Subscribe to LIDAR
        rospy.Subscriber('/scan', LaserScan, detector.callback)
        
        # Start LIDAR motor
        lidar.start_motor()
        time.sleep(2)
        
        print("✅ Initialized. Starting navigation...")
        
        # Parameters
        BASE_SPEED = 0.2
        TURN_SPEED = 0.15
        OBSTACLE_THRESHOLD = 0.5
        
        # Main loop
        rate = rospy.Rate(5)  # 5 Hz
        
        while not rospy.is_shutdown():
            # Get LIDAR data
            obstacles = detector.detect_obstacles()
            
            if obstacles:
                front_dist = obstacles['front_distance']
                print(f"Front: {front_dist:.2f}m")
                
                if obstacles['front_clear']:
                    print("Going forward")
                    robot.forward(BASE_SPEED)
                elif obstacles['left_clear']:
                    print("Turning left")
                    robot.left(TURN_SPEED)
                elif obstacles['right_clear']:
                    print("Turning right") 
                    robot.right(TURN_SPEED)
                else:
                    print("All blocked, stopping")
                    robot.stop()
            else:
                print("No LIDAR data")
                robot.stop()
            
            rate.sleep()
            
    except KeyboardInterrupt:
        print("🛑 Stopped by user")
    finally:
        robot.stop()

        print("✅ Cleanup complete")


if __name__ == '__main__':
    main()