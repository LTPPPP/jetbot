#!/usr/bin/env python3
"""
Install dependencies for ros_lidar_follower.py
This script checks and installs required Python packages
"""

import subprocess
import sys
import os

def install_package(package_name, pip_name=None):
    """Install a package using pip"""
    if pip_name is None:
        pip_name = package_name
    
    try:
        __import__(package_name)
        print(f"✓ {package_name} already installed")
        return True
    except ImportError:
        print(f"Installing {pip_name}...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", pip_name])
            print(f"✓ {pip_name} installed successfully")
            return True
        except subprocess.CalledProcessError as e:
            print(f"✗ Failed to install {pip_name}: {e}")
            return False

def check_system_packages():
    """Check system-level dependencies"""
    print("Checking system packages...")
    
    # Check for ROS
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print(f"✓ ROS {ros_distro} detected")
    else:
        print("⚠️  ROS not detected. Please install ROS and source setup.bash")
        return False
    
    return True

def main():
    """Main installation function"""
    print("=" * 60)
    print("INSTALLING DEPENDENCIES FOR ros_lidar_follower.py")
    print("=" * 60)
    
    # Check system packages first
    if not check_system_packages():
        print("❌ System packages check failed")
        return
    
    # List of required packages
    required_packages = [
        ("cv2", "opencv-python"),
        ("numpy", "numpy"),
        ("onnxruntime", "onnxruntime"),
        ("pyzbar", "pyzbar[scripts]"),
        ("paho.mqtt.client", "paho-mqtt"),
        ("enum", None),  # Built-in since Python 3.4
    ]
    
    print("\nInstalling Python packages...")
    
    failed_packages = []
    
    for package_name, pip_name in required_packages:
        if pip_name is None:  # Built-in package
            print(f"✓ {package_name} is built-in")
            continue
            
        if not install_package(package_name, pip_name):
            failed_packages.append(pip_name)
    
    # Install ROS-specific packages via apt if needed
    print("\nChecking ROS packages...")
    ros_packages = [
        "sensor_msgs",
        "std_msgs", 
        "geometry_msgs"
    ]
    
    for pkg in ros_packages:
        try:
            __import__(f"{pkg}.msg")
            print(f"✓ {pkg} available")
        except ImportError:
            print(f"⚠️  {pkg} not found. Install with: sudo apt install ros-$ROS_DISTRO-{pkg.replace('_', '-')}")
    
    # Check for additional system dependencies
    print("\nAdditional system dependencies:")
    print("For QR code detection: sudo apt install libzbar0")
    print("For camera support: sudo apt install v4l-utils")
    print("For MQTT: sudo apt install mosquitto mosquitto-clients")
    
    print("\n" + "=" * 60)
    print("INSTALLATION SUMMARY")
    print("=" * 60)
    
    if failed_packages:
        print(f"❌ Failed to install: {', '.join(failed_packages)}")
        print("Please install these manually or check your internet connection")
    else:
        print("✅ All Python packages installed successfully!")
    
    print("\nNext steps:")
    print("1. Make sure roscore is running: roscore")
    print("2. Launch RPLIDAR: roslaunch jetbot jetbot_rplidar_a1.launch")
    print("3. Run the follower: python3 ros_lidar_follower.py")
    
    # Check if models directory exists
    models_dir = "models"
    if os.path.exists(models_dir):
        print(f"\n📁 Models directory found: {models_dir}")
        if not os.path.exists(os.path.join(models_dir, "best.onnx")):
            print("⚠️  YOLO model (best.onnx) not found in models/")
            print("   The code will run but YOLO detection will be disabled")
    else:
        print(f"\n📁 Creating models directory: {models_dir}")
        os.makedirs(models_dir, exist_ok=True)

if __name__ == "__main__":
    main()