#!/usr/bin/env python3
"""
Fixed version of ros_lidar_follower.py with error handling and fallbacks
"""

import rospy
import cv2
import numpy as np
import json
import time
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from jetbot import Robot
import threading
import traceback

# Import các module local
try:
    from opposite_detector import OppositeDetector
    from map_navigator import MapNavigator
    from jetbot.lidar import Lidar, get_lidar
except ImportError as e:
    print(f"[ERROR] Import failed: {e}")
    exit(1)

class JetBotEventDrivenController:
    def __init__(self):
        """Khởi tạo controller"""
        rospy.loginfo("Đang khởi tạo JetBot Event-Driven Controller...")
        
        # Khởi tạo ROS node
        rospy.init_node('jetbot_lidar_follower', anonymous=True)
        
        # Hardware
        self.robot = Robot()
        rospy.loginfo("Phần cứng JetBot (động cơ) đã được khởi tạo.")
        
        # LIDAR
        try:
            self.lidar = get_lidar()
            rospy.loginfo("LIDAR đã được khởi tạo.")
        except Exception as e:
            rospy.logerr(f"Không thể khởi tạo LIDAR: {e}")
            self.lidar = None
        
        # YOLO model (mock if not available)
        try:
            # Try to import actual YOLO
            import onnxruntime
            from yolo_detector import YOLODetector
            self.yolo = YOLODetector('models/best.onnx')
            rospy.loginfo("YOLO model loaded successfully.")
        except Exception as e:
            rospy.logwarn(f"YOLO model not available: {e}")
            from mock_yolo import MockYOLO
            self.yolo = MockYOLO('models/best.onnx')
            rospy.loginfo("Using mock YOLO detector.")
        
        # Navigation
        try:
            with open('map.json', 'r') as f:
                map_data = json.load(f)
            self.navigator = MapNavigator(map_data)
            rospy.loginfo("Map navigator initialized.")
        except Exception as e:
            rospy.logerr(f"Failed to load map: {e}")
            # Create simple default map
            default_map = {
                "nodes": {
                    "1": {"x": 0, "y": 0},
                    "2": {"x": 1, "y": 0},
                    "3": {"x": 2, "y": 0}
                },
                "edges": [{"from": "1", "to": "2"}, {"from": "2", "to": "3"}]
            }
            self.navigator = MapNavigator(default_map)
        
        # Opposite detector
        self.opposite_detector = OppositeDetector()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # State variables
        self.current_image = None
        self.current_scan = None
        self.image_lock = threading.Lock()
        self.scan_lock = threading.Lock()
        
        # Navigation state
        self.current_goal = None
        self.path = []
        self.path_index = 0
        
        # Control parameters
        self.base_speed = 0.2
        self.turn_speed = 0.15
        self.obstacle_threshold = 0.5
        
        # Set initial path
        self.set_path([2, 5, 9, 8, 11])
        
        # ROS subscribers
        self.setup_subscribers()
        
        rospy.loginfo("Khởi tạo hoàn tất. Sẵn sàng hoạt động.")
    
    def setup_subscribers(self):
        """Setup ROS subscribers"""
        # LIDAR subscriber
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        
        # Camera subscriber - try multiple common camera topics
        camera_topics = ['/csi_cam_0/image_raw', '/camera/image_raw', '/usb_cam/image_raw', '/image_raw']
        
        self.camera_sub = None
        for topic in camera_topics:
            try:
                # Check if topic exists
                topics = rospy.get_published_topics()
                topic_names = [t[0] for t in topics]
                if topic in topic_names:
                    self.camera_sub = rospy.Subscriber(topic, Image, self.image_callback)
                    rospy.loginfo(f"Subscribed to camera topic: {topic}")
                    break
            except Exception as e:
                continue
        
        if self.camera_sub is None:
            rospy.logwarn("No camera topics found. Running in LIDAR-only mode.")
        
        rospy.loginfo("Đã đăng ký vào các topic.")
    
    def set_path(self, node_list):
        """Set navigation path"""
        if len(node_list) >= 2:
            start_node = node_list[0]
            goal_node = node_list[-1]
            
            rospy.loginfo(f"Đang lập kế hoạch từ node {start_node} đến {goal_node}...")
            
            try:
                path = self.navigator.plan_path(str(start_node), str(goal_node))
                path_ints = [int(x) for x in path]
                rospy.loginfo(f"Path found: {path_ints}")
                
                self.path = path_ints
                self.path_index = 0
                if len(self.path) > 1:
                    self.current_goal = self.path[1]
                    rospy.loginfo(f"Đã tìm thấy đường đi: {self.path}. Đích đến đầu tiên: {self.current_goal}")
                else:
                    rospy.logwarn("Path too short")
            except Exception as e:
                rospy.logerr(f"Path planning failed: {e}")
                # Fallback to simple path
                self.path = node_list
                self.path_index = 0
                self.current_goal = node_list[1] if len(node_list) > 1 else None
    
    def lidar_callback(self, msg):
        """LIDAR data callback"""
        with self.scan_lock:
            self.current_scan = msg
    
    def image_callback(self, msg):
        """Camera image callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.image_lock:
                self.current_image = cv_image
        except Exception as e:
            rospy.logwarn(f"Failed to convert image: {e}")
    
    def get_front_distance(self):
        """Get distance in front of robot from LIDAR"""
        with self.scan_lock:
            if self.current_scan is None:
                return float('inf')
            
            # Get readings in front (center ±30 degrees)
            ranges = np.array(self.current_scan.ranges)
            angles = np.arange(len(ranges)) * self.current_scan.angle_increment + self.current_scan.angle_min
            
            # Find front-facing readings
            front_mask = np.abs(angles) < np.pi/6  # ±30 degrees
            front_ranges = ranges[front_mask]
            
            # Filter out invalid readings
            valid_ranges = front_ranges[(front_ranges > self.current_scan.range_min) & 
                                      (front_ranges < self.current_scan.range_max) & 
                                      np.isfinite(front_ranges)]
            
            return np.min(valid_ranges) if len(valid_ranges) > 0 else float('inf')
    
    def detect_obstacles(self):
        """Detect obstacles and decide action"""
        front_distance = self.get_front_distance()
        
        if front_distance < self.obstacle_threshold:
            # Obstacle detected, find best direction
            with self.scan_lock:
                if self.current_scan is None:
                    return 'stop'
                
                ranges = np.array(self.current_scan.ranges)
                angles = np.arange(len(ranges)) * self.current_scan.angle_increment + self.current_scan.angle_min
                
                # Check left side (-90 to -30 degrees)
                left_mask = (angles >= -np.pi/2) & (angles <= -np.pi/6)
                left_ranges = ranges[left_mask]
                left_valid = left_ranges[(left_ranges > self.current_scan.range_min) & 
                                       (left_ranges < self.current_scan.range_max) & 
                                       np.isfinite(left_ranges)]
                left_distance = np.min(left_valid) if len(left_valid) > 0 else 0
                
                # Check right side (30 to 90 degrees)
                right_mask = (angles >= np.pi/6) & (angles <= np.pi/2)
                right_ranges = ranges[right_mask]
                right_valid = right_ranges[(right_ranges > self.current_scan.range_min) & 
                                         (right_ranges < self.current_scan.range_max) & 
                                         np.isfinite(right_ranges)]
                right_distance = np.min(right_valid) if len(right_valid) > 0 else 0
                
                # Decide direction
                if left_distance > right_distance:
                    return 'turn_left'
                elif right_distance > left_distance:
                    return 'turn_right'
                else:
                    return 'backup'
        
        return 'forward'
    
    def execute_action(self, action):
        """Execute robot action"""
        if action == 'forward':
            self.robot.forward(self.base_speed)
        elif action == 'turn_left':
            self.robot.left(self.turn_speed)
        elif action == 'turn_right':
            self.robot.right(self.turn_speed)
        elif action == 'backup':
            self.robot.backward(self.base_speed)
        elif action == 'stop':
            self.robot.stop()
    
    def run(self):
        """Main control loop"""
        rospy.loginfo("Bắt đầu vòng lặp. Đợi 3 giây...")
        time.sleep(3)
        rospy.loginfo("Hành trình bắt đầu!")
        
        rate = rospy.Rate(10)  # 10 Hz
        
        image_warning_time = time.time()
        
        try:
            while not rospy.is_shutdown():
                # Check for image data periodically
                if self.current_image is None and time.time() - image_warning_time > 5:
                    rospy.logwarn("Đang chờ dữ liệu hình ảnh từ topic camera...")
                    image_warning_time = time.time()
                
                # Main navigation logic
                action = self.detect_obstacles()
                self.execute_action(action)
                
                # Log current status
                front_dist = self.get_front_distance()
                if front_dist != float('inf'):
                    rospy.loginfo_throttle(5, f"Front distance: {front_dist:.2f}m, Action: {action}")
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Dừng chương trình do người dùng.")
        except Exception as e:
            rospy.logerr(f"Lỗi trong vòng lặp chính: {e}")
            traceback.print_exc()
        finally:
            self.robot.stop()
            rospy.loginfo("Robot đã dừng.")

def main():
    try:
        controller = JetBotEventDrivenController()
        controller.run()
    except Exception as e:
        rospy.logerr(f"Failed to start controller: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    main()