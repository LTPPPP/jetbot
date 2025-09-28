#!/usr/bin/env python3
# filepath: /home/ltpppp/Documents/HACKATHON2025/code/jetbot/simple_rule_based_robot.py

import requests
import time
import rospy
from jetbot import Robot
from enum import Enum
import networkx as nx
import threading
import sys
import select
import tty
import termios
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math

class Direction(Enum):
    NORTH, EAST, SOUTH, WEST = 'N', 'E', 'S', 'W'

class APIRobotController:
    def __init__(self, test_mode=False, test_type="square"):
        rospy.loginfo("Khởi tạo API Robot Controller...")
        self.test_mode = test_mode
        self.test_type = test_type
        self.setup_api_config()
        self.setup_robot_parameters()
        self.initialize_hardware()
        self.setup_camera_and_line_following()
        
        # Thêm flag để dừng robot
        self.should_stop = False
        self.input_thread = None
        
        if self.test_mode:
            if self.test_type == "square":
                rospy.loginfo("CHẠY Ở CHẾ ĐỘ TEST - DI THEO HÌNH VUÔNG")
                self.create_square_path()
            elif self.test_type == "turn180":
                rospy.loginfo("CHẠY Ở CHẾ ĐỘ TEST - QUAY 180 ĐỘ")
                self.create_turn180_path()
            else:
                rospy.logerr(f"Không hỗ trợ test_type: {self.test_type}")
                self.test_mode = False
        
        if not self.test_mode:
            # Lấy dữ liệu bản đồ từ API và tính đường đi ngắn nhất
            self.map_data = self.get_data()
            self.shortest_path_labels = self.problem_A()
            self.parse_map_data()
        
        rospy.loginfo("Khởi tạo hoàn tất!")

    def setup_api_config(self):
        """Cấu hình API"""
        self.base_url = "https://hackathon2025-dev.fpt.edu.vn/"
        self.get_map_endpoint = self.base_url + "api/maps/get_active_map/"
        self.token = "c0435b497712ae6704789b28b710dc88"

    def setup_robot_parameters(self):
        """Cấu hình thông số robot"""
        self.BASE_SPEED = 0.16
        self.TURN_SPEED = 0.2
        self.TURN_DURATION_90_DEG = 0.8
        self.STRAIGHT_DURATION = 6.0  # Chạy thẳng 7 giây
        
        # Hướng hiện tại của robot
        self.current_direction = Direction.EAST  # Mặc định hướng Đông

    def initialize_hardware(self):
        """Khởi tạo phần cứng robot"""
        try:
            self.robot = Robot()
            rospy.loginfo("Khởi tạo phần cứng JetBot thành công.")
        except Exception as e:
            rospy.logwarn(f"Không tìm thấy phần cứng JetBot, sử dụng Mock. Lỗi: {e}")
            from unittest.mock import Mock
            self.robot = Mock()

    def get_data(self):
        """Gọi API để lấy dữ liệu bản đồ"""
        try:
            rospy.loginfo("Đang gọi API để lấy dữ liệu bản đồ...")
            response = requests.get(
                self.get_map_endpoint, 
                params={"token": self.token, "map_type": "map_z"},
                timeout=10
            )
            
            if response.status_code == 200:
                data = response.json()
                rospy.loginfo("Lấy dữ liệu bản đồ thành công!")
                return data
            else:
                rospy.logerr(f"Lỗi API: {response.status_code}")
                return None
                
        except Exception as e:
            rospy.logerr(f"Lỗi khi gọi API: {e}")
            return None

    def make_graph(self, edges):
        """Tạo graph từ danh sách edges"""
        G = nx.DiGraph()
        for e in edges:
            G.add_edge(e["source"], e["target"], label=e["label"])
        return G

    def find_path(self, G, start, end):
        """Tìm đường đi ngắn nhất và trả về danh sách label của các edge"""
        path = nx.shortest_path(G, source=start["id"], target=end["id"])
        edge_labels = [G[u][v]["label"] for u, v in zip(path[:-1], path[1:])]
        return edge_labels

    def problem_A(self):
        """Lấy đường đi ngắn nhất từ Start đến End"""
        if not self.map_data:
            rospy.logerr("Không có dữ liệu bản đồ!")
            return []
            
        edges = self.map_data["edges"]
        nodes = self.map_data["nodes"]
        
        start = next((item for item in nodes if item["type"] == "Start"), None)
        end = next((item for item in nodes if item["type"] == "End"), None)
        
        if not start or not end:
            rospy.logerr("Không tìm thấy start hoặc end node!")
            return []

        G = self.make_graph(edges)
        path_labels = self.find_path(G, start, end)
        
        rospy.loginfo(f"Đường đi ngắn nhất: {path_labels}")
        return path_labels

    def create_square_path(self):
        """Tạo đường đi hình vuông cho chế độ test"""
        rospy.loginfo("Tạo đường đi hình vuông...")
        
        # Tạo 4 bước để đi hình vuông: Thẳng -> Phải -> Thẳng -> Phải -> Thẳng -> Phải -> Thẳng -> Phải
        square_commands = []
        
        for step in range(4):  # 4 cạnh của hình vuông
            square_commands.append({
                'step': step + 1,
                'label': 'SQUARE',
                'action': 'square_move',
                'from_direction': 'E',
                'to_direction': 'E'
            })
        
        self.navigation_commands = square_commands
        rospy.loginfo(f"Tạo được {len(square_commands)} lệnh cho hình vuông")

    def create_turn180_path(self):
        """Tạo đường đi test quay 180 độ"""
        rospy.loginfo("Tạo đường đi test quay 180 độ...")
        
        # Tạo 3 bước: Thẳng -> Quay 180° -> Thẳng -> Quay 180° -> Thẳng
        turn180_commands = []
        
        for step in range(3):  # 3 đoạn thẳng với 2 lần quay 180
            turn180_commands.append({
                'step': step + 1,
                'label': 'TURN180',
                'action': 'turn180_move',
                'from_direction': 'E',
                'to_direction': 'E'
            })
        
        self.navigation_commands = turn180_commands
        rospy.loginfo(f"Tạo được {len(turn180_commands)} lệnh cho test quay 180°")

    def parse_map_data(self):
        """Phân tích dữ liệu bản đồ từ API"""
        if not self.map_data or not self.shortest_path_labels:
            rospy.logerr("Không có dữ liệu để phân tích!")
            return
            
        # Tạo danh sách lệnh điều hướng từ path labels
        self.navigation_commands = self.create_navigation_commands_from_labels()

    def create_navigation_commands_from_labels(self):
        """Tạo lệnh điều hướng từ danh sách labels"""
        commands = []
        current_direction = self.current_direction
        
        for i, label in enumerate(self.shortest_path_labels):
            # Chuyển đổi label thành Direction enum
            try:
                target_direction = Direction(label)
            except ValueError:
                rospy.logwarn(f"Label không hợp lệ: {label}")
                continue
            
            # Tính toán hành động cần thực hiện
            action = self.calculate_turn_action(current_direction, target_direction)
            
            commands.append({
                'step': i + 1,
                'label': label,
                'action': action,
                'from_direction': current_direction.value,
                'to_direction': target_direction.value
            })
            
            # Cập nhật hướng hiện tại cho bước tiếp theo
            current_direction = target_direction
        
        rospy.loginfo(f"Tạo được {len(commands)} lệnh điều hướng từ labels")
        for cmd in commands:
            rospy.loginfo(f"Bước {cmd['step']}: {cmd['from_direction']} -> {cmd['to_direction']} ({cmd['action']})")
        
        return commands

    def calculate_turn_action(self, current_direction, target_direction):
        """Tính toán hành động quay cần thực hiện"""
        directions_order = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]
        
        current_idx = directions_order.index(current_direction)
        target_idx = directions_order.index(target_direction)
        
        diff = (target_idx - current_idx) % 4
        
        if diff == 0:
            return 'straight'
        elif diff == 1:
            return 'right'
        elif diff == 2:
            return 'back'
        elif diff == 3:
            return 'left'

    def keyboard_listener(self):
        """Lắng nghe phím từ bàn phím"""
        try:
            # Lưu lại cài đặt terminal hiện tại
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            
            rospy.loginfo("Nhấn 'q' hoặc 'c' để dừng robot...")
            
            while not self.should_stop and not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    if key in ['q', 'c']:
                        rospy.loginfo(f"Nhận được lệnh dừng từ phím '{key}'")
                        self.should_stop = True
                        break
                        
        except Exception as e:
            rospy.logwarn(f"Lỗi keyboard listener: {e}")
        finally:
            # Khôi phục cài đặt terminal
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                pass

    def start_keyboard_listener(self):
        """Khởi tạo thread lắng nghe bàn phím"""
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

    def execute_navigation(self):
        """Thực hiện điều hướng theo rule-based"""
        rospy.loginfo("Bắt đầu thực hiện điều hướng...")
        
        if not self.navigation_commands:
            rospy.logerr("Không có lệnh điều hướng để thực hiện!")
            return
        
        if self.test_mode:
            if self.test_type == "square":
                self.execute_square_navigation()
            elif self.test_type == "turn180":
                self.execute_turn180_navigation()
        else:
            self.execute_normal_navigation()

    def execute_square_navigation(self):
        """Thực hiện điều hướng hình vuông"""
        rospy.loginfo("Bắt đầu đi theo hình vuông...")
        
        for i, command in enumerate(self.navigation_commands):
            if self.should_stop:
                rospy.loginfo("Nhận được lệnh dừng, ngừng điều hướng.")
                break
                
            rospy.loginfo(f"Cạnh {command['step']}/4 của hình vuông")
            
            # Chạy thẳng 7 giây
            rospy.loginfo("Chạy thẳng 7 giây...")
            self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
            
            duration_remaining = self.STRAIGHT_DURATION
            while duration_remaining > 0 and not self.should_stop:
                sleep_time = min(0.5, duration_remaining)
                time.sleep(sleep_time)
                duration_remaining -= sleep_time
            
            self.robot.stop()
            rospy.loginfo("Dừng lại sau khi chạy thẳng 7 giây")
            time.sleep(1)  # Dừng lại 1 giây
            
            if self.should_stop:
                rospy.loginfo("Dừng robot theo yêu cầu người dùng.")
                break
            
            # Quay phải 90 độ (trừ lần cuối)
            if i < len(self.navigation_commands) - 1:  # Không quay ở bước cuối
                rospy.loginfo("Quay phải 90 độ để tiếp tục hình vuông")
                self.turn_robot('right')
            else:
                rospy.loginfo("Hoàn thành hình vuông!")
            
            if self.should_stop:
                break
                
            rospy.loginfo(f"Hoàn thành cạnh {command['step']}")
            time.sleep(1)
        
        if not self.should_stop:
            rospy.loginfo("Đã hoàn thành đi hình vuông!")
        else:
            rospy.loginfo("Hình vuông đã bị dừng bởi người dùng.")

    def execute_turn180_navigation(self):
        """Thực hiện điều hướng test quay 180 độ"""
        rospy.loginfo("Bắt đầu test quay 180 độ...")
        
        for i, command in enumerate(self.navigation_commands):
            if self.should_stop:
                rospy.loginfo("Nhận được lệnh dừng, ngừng điều hướng.")
                break
                
            rospy.loginfo(f"Đoạn {command['step']}/3 trong test quay 180°")
            
            # Chạy thẳng 7 giây
            rospy.loginfo("Chạy thẳng 7 giây...")
            self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
            
            duration_remaining = self.STRAIGHT_DURATION
            while duration_remaining > 0 and not self.should_stop:
                sleep_time = min(0.5, duration_remaining)
                time.sleep(sleep_time)
                duration_remaining -= sleep_time
            
            self.robot.stop()
            rospy.loginfo("Dừng lại sau khi chạy thẳng 7 giây")
            time.sleep(1)  # Dừng lại 1 giây
            
            if self.should_stop:
                rospy.loginfo("Dừng robot theo yêu cầu người dùng.")
                break
            
            # Quay 180 độ (trừ lần cuối)
            if i < len(self.navigation_commands) - 1:  # Không quay ở bước cuối
                rospy.loginfo("Quay 180 độ và tiếp tục")
                self.turn_robot('back')  # 'back' = quay 180°
            else:
                rospy.loginfo("Hoàn thành test quay 180°!")
            
            if self.should_stop:
                break
                
            rospy.loginfo(f"Hoàn thành đoạn {command['step']}")
            time.sleep(1)
        
        if not self.should_stop:
            rospy.loginfo("Đã hoàn thành test quay 180°!")
        else:
            rospy.loginfo("Test quay 180° đã bị dừng bởi người dùng.")

    def execute_normal_navigation(self):
        """Thực hiện điều hướng bình thường theo API"""
        for i, command in enumerate(self.navigation_commands):
            # Kiểm tra xem có lệnh dừng không
            if self.should_stop:
                rospy.loginfo("Nhận được lệnh dừng, ngừng điều hướng.")
                break
                
            rospy.loginfo(f"Bước {command['step']}/{len(self.navigation_commands)}: "
                         f"Hướng {command['from_direction']} -> {command['to_direction']} - "
                         f"Hành động: {command['action']}")
            
            # Chạy thẳng 7 giây trước (với kiểm tra dừng)
            rospy.loginfo("Chạy thẳng 7 giây...")
            self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
            
            # Chạy với kiểm tra dừng mỗi 0.5 giây
            duration_remaining = self.STRAIGHT_DURATION
            while duration_remaining > 0 and not self.should_stop:
                sleep_time = min(0.5, duration_remaining)
                time.sleep(sleep_time)
                duration_remaining -= sleep_time
            
            self.robot.stop()
            rospy.loginfo("Dừng lại sau khi chạy thẳng 7 giây")
            time.sleep(1)  # Dừng lại 1 giây
            
            if self.should_stop:
                rospy.loginfo("Dừng robot theo yêu cầu người dùng.")
                break
            
            # Sau đó mới thực hiện hành động quay (luôn quay phải)
            rospy.loginfo("Quay phải sau khi chạy thẳng")
            self.turn_robot('right')
            
            if self.should_stop:
                rospy.loginfo("Dừng robot theo yêu cầu người dùng.")
                break
            
            rospy.loginfo(f"Hoàn thành bước {command['step']}")
            time.sleep(1)  # Nghỉ 1 giây trước khi tiếp tục
        
        if not self.should_stop:
            rospy.loginfo("Hoàn thành điều hướng!")
        else:
            rospy.loginfo("Điều hướng đã bị dừng bởi người dùng.")

    def turn_robot(self, action):
        """Thực hiện quay robot"""
        if action == 'straight' or self.should_stop:
            return
            
        rospy.loginfo(f"Thực hiện quay: {action}")
        
        if action == 'right':
            self.robot.set_motors(self.TURN_SPEED, -self.TURN_SPEED)
            duration = self.TURN_DURATION_90_DEG
        elif action == 'left':
            self.robot.set_motors(-self.TURN_SPEED, self.TURN_SPEED)
            duration = self.TURN_DURATION_90_DEG
        elif action == 'back':
            self.robot.set_motors(self.TURN_SPEED, -self.TURN_SPEED)
            duration = self.TURN_DURATION_90_DEG * 2  # Quay 180 độ
        
        # Quay với kiểm tra dừng
        duration_remaining = duration
        while duration_remaining > 0 and not self.should_stop:
            sleep_time = min(0.1, duration_remaining)
            time.sleep(sleep_time)
            duration_remaining -= sleep_time
        
        self.robot.stop()
        time.sleep(0.5)  # Nghỉ sau khi quay

    def run(self):
        """Chạy chương trình chính"""
        if not self.test_mode:
            if not self.map_data:
                rospy.logerr("Không có dữ liệu bản đồ, không thể tiếp tục!")
                return
                
            if not self.shortest_path_labels:
                rospy.logerr("Không có đường đi, không thể tiếp tục!")
                return
        
        # Bắt đầu lắng nghe bàn phím
        self.start_keyboard_listener()
        
        if self.test_mode:
            if self.test_type == "square":
                rospy.loginfo("CHẾ ĐỘ TEST: Robot sẽ đi theo hình vuông")
            elif self.test_type == "turn180":
                rospy.loginfo("CHẾ ĐỘ TEST: Robot sẽ test quay 180 độ")
        
        rospy.loginfo("Đợi 3 giây trước khi bắt đầu...")
        rospy.loginfo("Nhấn 'q' hoặc 'c' bất cứ lúc nào để dừng robot...")
        time.sleep(3)
        
        try:
            if not self.should_stop:
                self.execute_navigation()
                rospy.loginfo("Đã hoàn thành nhiệm vụ!")
        except KeyboardInterrupt:
            rospy.loginfo("Dừng chương trình theo yêu cầu người dùng")
            self.should_stop = True
        except Exception as e:
            rospy.logerr(f"Lỗi trong quá trình thực hiện: {e}")
        finally:
            self.robot.stop()
            rospy.loginfo("Đã dừng robot")

def main():
    rospy.init_node('api_robot_controller', anonymous=True)
    
    # Kiểm tra parameter test
    test_mode = rospy.get_param('~test', False)
    test_type = rospy.get_param('~test_type', 'square')
    
    try:
        controller = APIRobotController(test_mode=test_mode, test_type=test_type)
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node đã bị ngắt")
    except Exception as e:
        rospy.logerr(f"Lỗi không xác định: {e}")

if __name__ == '__main__':
    main()