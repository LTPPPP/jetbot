#!/usr/bin/env python3
"""
JetBot LIDAR Integration Setup Guide

This script helps you set up RPLIDAR with JetBot by checking dependencies,
providing setup instructions, and testing the integration.
"""

import subprocess
import sys
import os
import time


def run_command(cmd, description=""):
    """Run a shell command and return the result"""
    print(f"\n{'='*50}")
    if description:
        print(f"EXECUTING: {description}")
    print(f"COMMAND: {cmd}")
    print(f"{'='*50}")
    
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ SUCCESS")
            if result.stdout:
                print(f"OUTPUT:\n{result.stdout}")
        else:
            print("✗ FAILED")
            if result.stderr:
                print(f"ERROR:\n{result.stderr}")
        return result.returncode == 0
    except Exception as e:
        print(f"✗ EXCEPTION: {e}")
        return False


def check_ros_installation():
    """Check if ROS is installed and working"""
    print("\n" + "="*60)
    print("CHECKING ROS INSTALLATION")
    print("="*60)
    
    # Check if ROS is sourced
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print(f"✓ ROS {ros_distro} detected")
    else:
        print("✗ ROS not detected. Please install ROS and source the setup file")
        print("  For Ubuntu 18.04: sudo apt install ros-melodic-desktop-full")
        print("  For Ubuntu 20.04: sudo apt install ros-noetic-desktop-full")
        print("  Then: source /opt/ros/<distro>/setup.bash")
        return False
    
    # Check roscore
    if run_command("pgrep -f roscore", "Checking if roscore is running"):
        print("✓ roscore is already running")
    else:
        print("ℹ roscore is not running. You'll need to start it with 'roscore'")
    
    return True


def check_rplidar_ros_package():
    """Check if rplidar_ros package is installed"""
    print("\n" + "="*60)
    print("CHECKING RPLIDAR_ROS PACKAGE")
    print("="*60)
    
    # Try to find the package
    if run_command("rospack find rplidar_ros", "Looking for rplidar_ros package"):
        print("✓ rplidar_ros package found")
        return True
    else:
        print("✗ rplidar_ros package not found")
        print("\nTo install rplidar_ros package:")
        print("1. cd ~/catkin_ws/src")
        print("2. git clone https://github.com/Slamtec/rplidar_ros.git")
        print("3. cd ~/catkin_ws")
        print("4. catkin_make")
        print("5. source ~/catkin_ws/devel/setup.bash")
        return False


def check_hardware_connection():
    """Check for RPLIDAR hardware connection"""
    print("\n" + "="*60)
    print("CHECKING HARDWARE CONNECTION")
    print("="*60)
    
    # Check for USB serial devices
    devices = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
    found_devices = []
    
    for device in devices:
        if os.path.exists(device):
            found_devices.append(device)
    
    if found_devices:
        print(f"✓ Serial devices found: {found_devices}")
        
        # Check permissions
        for device in found_devices:
            if os.access(device, os.R_OK | os.W_OK):
                print(f"✓ {device} has read/write permissions")
            else:
                print(f"✗ {device} lacks permissions. Run: sudo chmod 666 {device}")
        return True
    else:
        print("✗ No serial devices found")
        print("Please check:")
        print("1. RPLIDAR is connected via USB")
        print("2. USB cable is working")
        print("3. RPLIDAR power is on")
        return False


def setup_jetbot_lidar():
    """Setup JetBot LIDAR integration"""
    print("\n" + "="*60)
    print("SETTING UP JETBOT LIDAR INTEGRATION")
    print("="*60)
    
    # Add lidar module to JetBot package init
    jetbot_init_path = os.path.join(os.path.dirname(__file__), "jetbot", "__init__.py")
    
    if os.path.exists(jetbot_init_path):
        with open(jetbot_init_path, 'r') as f:
            content = f.read()
        
        if 'from .lidar import Lidar, get_lidar' not in content:
            with open(jetbot_init_path, 'a') as f:
                f.write('\n# LIDAR integration\nfrom .lidar import Lidar, get_lidar\n')
            print("✓ Added LIDAR to JetBot __init__.py")
        else:
            print("✓ LIDAR already in JetBot __init__.py")
    else:
        print(f"✗ Could not find {jetbot_init_path}")


def test_lidar_integration():
    """Test the LIDAR integration"""
    print("\n" + "="*60)
    print("TESTING LIDAR INTEGRATION")
    print("="*60)
    
    print("Starting test sequence...")
    print("1. Make sure roscore is running: roscore")
    print("2. In another terminal, launch RPLIDAR: roslaunch jetbot jetbot_rplidar_a1.launch")
    print("3. Run the test below")
    
    test_code = '''
import rospy
import time
from jetbot import get_lidar

# Initialize
rospy.init_node('lidar_test', anonymous=True)
lidar = get_lidar()

# Start motor
print("Starting LIDAR motor...")
lidar.start_motor()
time.sleep(2)

# Wait for data
print("Waiting for scan data...")
timeout = 10
start_time = time.time()

while time.time() - start_time < timeout:
    scan_data = lidar.get_scan_data()
    if scan_data['ranges']:
        print(f"✓ Received {len(scan_data['ranges'])} range measurements")
        print(f"  Range: {scan_data['range_min']:.2f}m to {scan_data['range_max']:.2f}m")
        print(f"  Front distance: {lidar.get_front_distance():.2f}m")
        print(f"  Obstacle detected: {scan_data['obstacle_detected']}")
        break
    time.sleep(0.5)
else:
    print("✗ No LIDAR data received within timeout")

print("Test completed")
'''
    
    print("\nTest code saved to test_lidar.py")
    with open("test_lidar.py", "w") as f:
        f.write(test_code)


def main():
    """Main setup function"""
    print("JetBot RPLIDAR Integration Setup")
    print("=" * 60)
    
    # Check ROS
    if not check_ros_installation():
        print("\n❌ Please install and configure ROS first")
        return False
    
    # Check rplidar_ros package
    if not check_rplidar_ros_package():
        print("\n❌ Please install rplidar_ros package first")
        return False
    
    # Check hardware
    hardware_ok = check_hardware_connection()
    
    # Setup integration
    setup_jetbot_lidar()
    
    # Create test
    test_lidar_integration()
    
    # Summary
    print("\n" + "="*60)
    print("SETUP SUMMARY")
    print("="*60)
    
    if hardware_ok:
        print("✓ All checks passed! You can now use RPLIDAR with JetBot")
        print("\nNext steps:")
        print("1. Start roscore: roscore")
        print("2. Launch RPLIDAR: roslaunch jetbot jetbot_rplidar_a1.launch")
        print("3. Run collision avoidance: python3 jetbot_lidar_collision_avoidance.py")
        print("4. Or test basic functionality: python3 test_lidar.py")
    else:
        print("⚠️  Hardware connection issues detected")
        print("Please check RPLIDAR connection and try again")
    
    print("\n📁 Files created:")
    print("  - jetbot/lidar.py (LIDAR interface)")
    print("  - jetbot_lidar_collision_avoidance.py (collision avoidance)")
    print("  - launch/jetbot_rplidar_a1.launch (A1 config)")
    print("  - launch/jetbot_rplidar_a3.launch (A3 config)")
    print("  - launch/jetbot_lidar_visualization.launch (with RViz)")
    print("  - rviz/jetbot_lidar.rviz (RViz config)")
    print("  - test_lidar.py (test script)")


if __name__ == "__main__":
    main()