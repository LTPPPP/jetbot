#!/usr/bin/env python3
"""
Dependency checker and installer for ros_lidar_follower.py
Kiểm tra và cài đặt các dependencies cần thiết
"""

import subprocess
import sys
import os
import importlib


def check_and_install_package(package_name, import_name=None, pip_name=None):
    """
    Check if a package is installed, if not, install it
    
    Args:
        package_name (str): Name of the package for display
        import_name (str): Name used for import (default: package_name)
        pip_name (str): Name used for pip install (default: package_name)
    
    Returns:
        bool: True if package is available
    """
    if import_name is None:
        import_name = package_name
    if pip_name is None:
        pip_name = package_name
    
    try:
        importlib.import_module(import_name)
        print(f"✓ {package_name} is already installed")
        return True
    except ImportError:
        print(f"✗ {package_name} not found. Installing...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", pip_name])
            print(f"✓ {package_name} installed successfully")
            return True
        except subprocess.CalledProcessError:
            print(f"✗ Failed to install {package_name}")
            return False


def check_ros_dependencies():
    """Check ROS-specific dependencies"""
    print("\n" + "="*50)
    print("CHECKING ROS DEPENDENCIES")
    print("="*50)
    
    # Check if ROS is available
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print(f"✓ ROS {ros_distro} detected")
    else:
        print("✗ ROS not detected. Make sure to source ROS setup.bash")
        return False
    
    try:
        import rospy
        print("✓ rospy available")
    except ImportError:
        print("✗ rospy not available")
        return False
    
    try:
        from sensor_msgs.msg import LaserScan, Image
        print("✓ sensor_msgs available")
    except ImportError:
        print("✗ sensor_msgs not available")
        return False
    
    return True


def check_python_dependencies():
    """Check and install Python package dependencies"""
    print("\n" + "="*50)
    print("CHECKING PYTHON DEPENDENCIES")
    print("="*50)
    
    dependencies = [
        ("OpenCV", "cv2", "opencv-python"),
        ("NumPy", "numpy", "numpy"),
        ("ONNX Runtime", "onnxruntime", "onnxruntime"),
        ("PyZbar", "pyzbar", "pyzbar"),
        ("Paho MQTT", "paho.mqtt.client", "paho-mqtt"),
        ("Enum", "enum", None),  # Built-in in Python 3.4+
    ]
    
    all_ok = True
    
    for package_name, import_name, pip_name in dependencies:
        if pip_name is None:
            # Built-in package, just check import
            try:
                importlib.import_module(import_name)
                print(f"✓ {package_name} (built-in) is available")
            except ImportError:
                print(f"✗ {package_name} not available (this shouldn't happen)")
                all_ok = False
        else:
            if not check_and_install_package(package_name, import_name, pip_name):
                all_ok = False
    
    return all_ok


def check_jetbot_integration():
    """Check JetBot integration"""
    print("\n" + "="*50)
    print("CHECKING JETBOT INTEGRATION")  
    print("="*50)
    
    try:
        from jetbot import Robot
        print("✓ JetBot Robot class available")
    except ImportError:
        print("✗ JetBot Robot not available")
        return False
    
    # Check for LIDAR integration
    try:
        from jetbot import get_lidar
        print("✓ JetBot LIDAR integration available")
    except ImportError:
        print("✗ JetBot LIDAR integration not available")
        print("  Run setup_jetbot_lidar.py first")
        return False
    
    return True


def check_project_files():
    """Check required project files"""
    print("\n" + "="*50)
    print("CHECKING PROJECT FILES")
    print("="*50)
    
    required_files = [
        "opposite_detector.py",
        "map_navigator.py", 
        "ros_lidar_follower.py",
        "jetbot/lidar.py"
    ]
    
    all_files_exist = True
    
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"✓ {file_path} exists")
        else:
            print(f"✗ {file_path} missing")
            all_files_exist = False
    
    return all_files_exist


def check_optional_files():
    """Check optional files and suggest creation"""
    print("\n" + "="*50)
    print("CHECKING OPTIONAL FILES")
    print("="*50)
    
    optional_files = {
        "models/best.onnx": "YOLO model file - needed for sign detection",
        "map.json": "Map configuration file",
        "MQTT broker": "MQTT broker for communication (mosquitto)"
    }
    
    for file_path, description in optional_files.items():
        if file_path == "MQTT broker":
            # Check if mosquitto is running
            try:
                result = subprocess.run(["pgrep", "-f", "mosquitto"], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    print(f"✓ MQTT broker (mosquitto) is running")
                else:
                    print(f"ℹ MQTT broker not running - {description}")
            except:
                print(f"ℹ Cannot check MQTT broker status")
        else:
            if os.path.exists(file_path):
                print(f"✓ {file_path} exists - {description}")
            else:
                print(f"ℹ {file_path} missing - {description}")


def create_minimal_config_files():
    """Create minimal configuration files if they don't exist"""
    print("\n" + "="*50)
    print("CREATING MINIMAL CONFIG FILES")
    print("="*50)
    
    # Create minimal map.json if it doesn't exist
    if not os.path.exists("map.json"):
        minimal_map = {
            "nodes": {
                "start": {"x": 0, "y": 0},
                "goal": {"x": 5, "y": 5}
            },
            "edges": [
                {"from": "start", "to": "goal", "weight": 1.0}
            ],
            "start_node": "start",
            "end_node": "goal"
        }
        
        import json
        with open("map.json", "w") as f:
            json.dump(minimal_map, f, indent=2)
        print("✓ Created minimal map.json")
    else:
        print("✓ map.json already exists")
    
    # Create models directory
    os.makedirs("models", exist_ok=True)
    print("✓ Created/verified models directory")


def test_basic_functionality():
    """Test basic functionality"""
    print("\n" + "="*50)
    print("TESTING BASIC FUNCTIONALITY")
    print("="*50)
    
    try:
        # Test LIDAR integration
        print("Testing LIDAR integration...")
        from jetbot import get_lidar
        lidar = get_lidar()
        print("✓ LIDAR integration test passed")
        
        # Test Robot
        print("Testing Robot integration...")
        from jetbot import Robot
        robot = Robot()
        print("✓ Robot integration test passed")
        
        # Test opposite detector
        print("Testing Opposite Detector...")
        from opposite_detector import SimpleOppositeDetector
        detector = SimpleOppositeDetector()
        print("✓ Opposite Detector test passed")
        
        # Test map navigator
        print("Testing Map Navigator...")
        from map_navigator import MapNavigator
        if os.path.exists("map.json"):
            navigator = MapNavigator("map.json")
            print("✓ Map Navigator test passed")
        else:
            print("ℹ Map Navigator test skipped (no map.json)")
        
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False


def main():
    """Main function"""
    print("JetBot ROS LIDAR Follower - Dependency Checker")
    print("=" * 60)
    
    # Check all dependencies
    ros_ok = check_ros_dependencies()
    python_ok = check_python_dependencies() 
    jetbot_ok = check_jetbot_integration()
    files_ok = check_project_files()
    
    # Check optional files
    check_optional_files()
    
    # Create minimal configs
    create_minimal_config_files()
    
    # Overall status
    print("\n" + "="*60)
    print("OVERALL STATUS")
    print("="*60)
    
    if ros_ok and python_ok and jetbot_ok and files_ok:
        print("✅ ALL DEPENDENCIES SATISFIED!")
        print("ros_lidar_follower.py should be able to run")
        
        # Test basic functionality
        if test_basic_functionality():
            print("\n✅ BASIC FUNCTIONALITY TESTS PASSED!")
            
            print("\n🚀 TO RUN ros_lidar_follower.py:")
            print("1. Start roscore: roscore")
            print("2. Launch RPLIDAR: roslaunch jetbot jetbot_rplidar_a1.launch") 
            print("3. Start camera (if needed): roslaunch jetbot camera.launch")
            print("4. Run the follower: python3 ros_lidar_follower.py")
        else:
            print("\n⚠️  Some functionality tests failed")
    else:
        print("❌ SOME DEPENDENCIES ARE MISSING")
        print("Please resolve the issues above before running ros_lidar_follower.py")
        
        if not ros_ok:
            print("  - Fix ROS installation/configuration")
        if not python_ok:
            print("  - Install missing Python packages")
        if not jetbot_ok:
            print("  - Run setup_jetbot_lidar.py")
        if not files_ok:
            print("  - Check missing project files")


if __name__ == "__main__":
    main()