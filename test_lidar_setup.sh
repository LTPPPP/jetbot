#!/bin/bash

# Test script for RPLiDAR integration with JetBot
echo "=== Testing RPLiDAR Integration with JetBot ==="

# Check if ROS is installed
if ! command -v roscore &> /dev/null; then
    echo "❌ ROS not found! Please install ROS first."
    exit 1
fi

echo "✅ ROS found"

# Check if rplidar_ros package exists
if [ ! -d "/opt/ros/*/share/rplidar_ros" ] && [ ! -d "~/catkin_ws/src/rplidar_ros" ]; then
    echo "❌ rplidar_ros package not found!"
    echo "Please install with: sudo apt-get install ros-melodic-rplidar-ros"
    exit 1
fi

echo "✅ rplidar_ros package found"

# Check Python dependencies
echo "Checking Python dependencies..."

python3 -c "
import sys
missing = []

try:
    import rospy
    print('✅ rospy found')
except ImportError:
    missing.append('rospy')
    print('❌ rospy not found')

try:
    import numpy
    print('✅ numpy found')
except ImportError:
    missing.append('numpy')
    print('❌ numpy not found')

try:
    from jetbot import Robot
    print('✅ jetbot found')
except ImportError:
    missing.append('jetbot')
    print('❌ jetbot not found')

try:
    from sensor_msgs.msg import LaserScan
    print('✅ sensor_msgs found')
except ImportError:
    missing.append('sensor_msgs')
    print('❌ sensor_msgs not found')

if missing:
    print(f'\\n❌ Missing dependencies: {missing}')
    sys.exit(1)
else:
    print('\\n✅ All Python dependencies found')
"

if [ $? -ne 0 ]; then
    echo "Please install missing dependencies and try again."
    exit 1
fi

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "To test the RPLiDAR integration:"
echo ""
echo "1. Start ROS core:"
echo "   roscore"
echo ""
echo "2. In another terminal, launch RPLiDAR (choose your model):"
echo "   # For A1:"
echo "   roslaunch jetbot_lidar_a1.launch"
echo "   # For A3:"  
echo "   roslaunch jetbot_lidar_a3.launch"
echo ""
echo "3. In another terminal, test LIDAR data:"
echo "   rostopic echo /scan"
echo ""
echo "4. Run the simple navigation:"
echo "   cd $(pwd)"
echo "   python3 simple_lidar_follower.py"
echo ""
echo "5. Optional - View in RViz:"
echo "   rosrun rviz rviz -d jetbot_lidar_config.rviz"
echo ""
echo "=== Files created ==="
echo "✅ jetbot/lidar.py - Core LIDAR interface"
echo "✅ simple_lidar_follower.py - Simplified navigation"
echo "✅ jetbot_lidar_a1.launch - A1 LIDAR launch file"
echo "✅ jetbot_lidar_a3.launch - A3 LIDAR launch file"
echo "✅ jetbot_lidar_config.rviz - RViz configuration"
echo "✅ opposite_detector.py - LIDAR data analyzer"
echo "✅ map_navigator.py - Navigation system"
echo "✅ map.json - Navigation map"
echo ""