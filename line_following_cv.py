#!/usr/bin/env python3
"""
JetBot Line Following với Computer Vision
Sử dụng OpenCV để detect đường màu đen và điều khiển robot

Cách chạy:
    python3            # Normalize center_x về [-1, 1] và convert to float
            normalized_x = float((center_x - width/2) / (width/2))
            confidence = float(min(area / 1000, 1.0))
            
            # Tính góc của đường bằng cách fit line
            angle = self._calculate_line_angle(largest_contour, roi_offset, width, height)
            
            return normalized_x, confidence, angle, binaryllowing_cv.py

Nhấn 'q' để thoát chương trình
"""

import cv2
import numpy as np
import time
import threading
import sys
import signal
import json
import os
import select
from jetbot import Camera, Robot

# Tối ưu hóa NumPy operations
np.seterr(invalid='ignore', over='ignore')  # Bỏ qua warning không cần thiết

# Performance monitoring (optional)
class PerformanceMonitor:
    def __init__(self):
        self.timings = {}
        self.frame_count = 0
        self.start_time = time.time()
    
    def start_timer(self, name):
        self.timings[f"{name}_start"] = time.perf_counter()
    
    def end_timer(self, name):
        if f"{name}_start" in self.timings:
            duration = time.perf_counter() - self.timings[f"{name}_start"]
            if name not in self.timings:
                self.timings[name] = []
            self.timings[name].append(duration)
            if len(self.timings[name]) > 30:  # Chỉ giữ 30 samples gần nhất
                self.timings[name].pop(0)
    
    def get_average(self, name):
        if name in self.timings and self.timings[name]:
            return sum(self.timings[name]) / len(self.timings[name])
        return 0
    
    def get_fps(self):
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        return self.frame_count / elapsed if elapsed > 0 else 0

class LineFollower:
    def draw_birdview_polygon(self, image):
        """Vẽ các điểm birdview (src points) lên ảnh gốc để debug"""
        img = image.copy()
        h, w = img.shape[:2]
        # Lấy các điểm từ config
        pts = [
            (int(self.src_bottom_left[0] * w), int(self.src_bottom_left[1] * h)),
            (int(self.src_bottom_right[0] * w), int(self.src_bottom_right[1] * h)),
            (int(self.src_top_right[0] * w), int(self.src_top_right[1] * h)),
            (int(self.src_top_left[0] * w), int(self.src_top_left[1] * h)),
        ]
        # Vẽ polygon
        cv2.polylines(img, [np.array(pts, np.int32).reshape((-1,1,2))], isClosed=True, color=(0,255,255), thickness=2)
        # Vẽ các điểm
        for i, p in enumerate(pts):
            cv2.circle(img, p, 6, (0,0,255), -1)
            cv2.putText(img, f"P{i+1}", (p[0]+5, p[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        return img
    def __init__(self, config_file="config.txt"):
        print("🤖 Khởi tạo JetBot Line Follower...")
        
        # Khởi tạo camera và robot
        self.camera = Camera()
        self.robot = Robot()
        
        self.config_file = config_file
        
        # Thông số mặc định (sẽ được ghi đè bởi config file nếu có)
        self.default_config = {
            # Thông số điều khiển
            "speed_gain": 0.15,             # Tốc độ cơ bản (0.0-1.0)
            "steering_gain": 0.5,           # Độ mạnh steering (0.0-2.0)
            "steering_kd": 0.05,            # Derivative gain (0.0-0.5)
            "steering_bias": 0.0,           # Bias trái/phải (-0.3-0.3)
            "angle_factor": 0.3,            # Factor cho angle steering (0.0-1.0)
            
            # Thông số computer vision
            "black_threshold": 60,          # Ngưỡng phát hiện đen (30-120)
            "min_area": 100,                # Diện tích tối thiểu (50-500)
            "confidence_threshold": 0.1,    # Ngưỡng tin cậy tối thiểu (0.0-1.0)
            
            # Thông số chế độ chạy
            "run": True,                     # True: chạy theo đường, False: chỉ align vào giữa
            "alignment_threshold": 0.1,      # Ngưỡng để coi là đã align (0.0-0.5)
            
            # Thông số Bird's Eye View
            "birdview_enabled": True,       # Bật/tắt chế độ bird's eye view
            "birdview_width": 160,          # Chiều rộng bird's eye view (tối ưu cho JetBot)
            "birdview_height": 240,         # Chiều cao bird's eye view (tối ưu cho JetBot)
            "birdview_interpolation": "linear",  # linear, nearest, cubic
            "src_image_scale": 0.75,        # Scale down input image để tăng tốc độ
            
            # Thông số perspective transform (tỷ lệ từ 0.0-1.0) - Mở rộng góc nhìn
            "src_bottom_left": [0.05, 0.95],   # Mở rộng từ 0.15 -> 0.05 (rộng hơn)
            "src_bottom_right": [0.95, 0.95],  # Mở rộng từ 0.85 -> 0.95 (rộng hơn)
            "src_top_right": [0.65, 0.45],     # Đẩy góc trên lên cao hơn
            "src_top_left": [0.35, 0.45],      # Đẩy góc trên lên cao hơn
            "dst_margin": 0.1,                 # Giảm margin từ 0.15 -> 0.1 để tận dụng space
            "enable_roi_optimization": True,  # Chỉ xử lý ROI quan trọng
        }
        
        # Load config từ file
        self.load_config()
        
        # Thông số Bird's Eye View - Tối ưu hóa
        self.perspective_matrix = None  # Ma trận perspective transform
        self.inverse_matrix = None      # Ma trận inverse transform
        self.interpolation_flag = cv2.INTER_LINEAR  # Interpolation method
        self.scaled_width = 640         # Kích thước ảnh sau scale
        self.scaled_height = 480        # Kích thước ảnh sau scale
        self._transform_cache = {}       # Cache cho các ma trận transform
        
        # Biến trạng thái
        self.running = False
        self.last_steering = 0.0
        self.current_frame = None
        self.debug_frame = None
        self.birdview_frame = None
        self.is_aligned = False             # Trạng thái đã align chưa

        # Ghi video
        self.recording_enabled = True  # Bật/tắt chức năng quay video
        self.video_writer = None
        self.video_filename = None
        self.video_fps = 20
        self.video_size = (640, 480)  # Sửa nếu cần
        
        # Performance monitoring
        self.perf_monitor = PerformanceMonitor() if hasattr(self, 'enable_performance_monitoring') else None
        
        # Phát hiện môi trường chạy
        self.display_enabled = self._check_display_available()
        
        # Khởi tạo perspective transform
        self._setup_perspective_transform()
        
        # Lưu ảnh debug mỗi 1s
        self.last_debug_save_time = 0
        self.debug_save_dir = "debug_images"
        if not os.path.exists(self.debug_save_dir):
            os.makedirs(self.debug_save_dir)
        print("✅ Khởi tạo thành công!")
        self.print_config()

    def _check_display_available(self):
        """Kiểm tra xem có thể hiển thị GUI không (phát hiện SSH session)"""
        try:
            # Kiểm tra biến môi trường DISPLAY
            if not os.environ.get('DISPLAY'):
                print("⚠️ Không tìm thấy DISPLAY - Chạy trong chế độ SSH (không hiển thị)")
                return False
            
            # Kiểm tra SSH session
            if os.environ.get('SSH_CLIENT') or os.environ.get('SSH_TTY'):
                print("⚠️ Phát hiện SSH session - Tắt hiển thị GUI")
                return False
            
            # Thử tạo window test
            try:
                cv2.namedWindow('test', cv2.WINDOW_AUTOSIZE)
                cv2.destroyWindow('test')
                print("✅ Display khả dụng - Sẽ hiển thị cửa sổ debug")
                return True
            except:
                print("⚠️ Không thể tạo window - Chạy trong chế độ headless")
                return False
                
        except Exception as e:
            print(f"⚠️ Lỗi khi kiểm tra display: {e} - Tắt hiển thị")
            return False

    def load_config(self):
        """Load thông số từ config file"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    
                # Merge với default config
                for key, value in self.default_config.items():
                    setattr(self, key, config.get(key, value))
                    
                print(f"✅ Đã load config từ {self.config_file}")
            else:
                # Sử dụng default config và tạo file mới
                for key, value in self.default_config.items():
                    setattr(self, key, value)
                    
                self.save_config()
                print(f"📝 Đã tạo config mặc định: {self.config_file}")
                
        except Exception as e:
            print(f"⚠️ Lỗi khi load config: {e}")
            print("🔄 Sử dụng thông số mặc định")
            
            # Fallback to default
            for key, value in self.default_config.items():
                setattr(self, key, value)

    def save_config(self):
        """Lưu thông số hiện tại vào config file"""
        try:
            config = {}
            for key in self.default_config.keys():
                config[key] = getattr(self, key)
                
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=4, ensure_ascii=False)
                
            print(f"💾 Đã lưu config vào {self.config_file}")
            
        except Exception as e:
            print(f"❌ Lỗi khi lưu config: {e}")
    
    def toggle_run_mode(self):
        """Toggle chế độ run và lưu vào config"""
        self.run = not self.run
        self.save_config()
        mode = "Follow Line" if self.run else "Alignment Only"
        print(f"🔄 Chuyển đổi chế độ: {mode}")
        print(f"   Run mode: {self.run}")
        
    def console_input_loop(self):
        """Xử lý input từ console trong chế độ headless"""
        while self.running:
            try:
                # Kiểm tra có input trong 1 giây
                if select.select([sys.stdin], [], [], 1.0)[0]:
                    line = sys.stdin.readline().strip()
                    if line.lower() == 'r':
                        self.toggle_run_mode()
                    elif line.lower() == 'q':
                        print("🛑 Người dùng yêu cầu thoát...")
                        self.stop()
                        break
            except:
                # Ignore input errors trong headless mode
                continue

    def print_config(self):
        """In ra thông số hiện tại"""
        print(f"📊 Thông số hiện tại (từ {self.config_file}):")
        print(f"   🎮 Điều khiển:")
        print(f"      Speed Gain: {self.speed_gain}")
        print(f"      Steering Gain: {self.steering_gain}")
        print(f"      Steering kD: {self.steering_kd}")
        print(f"      Steering Bias: {self.steering_bias}")
        print(f"      Angle Factor: {self.angle_factor}")
        print(f"   📷 Computer Vision:")
        print(f"      Black Threshold: {self.black_threshold}")
        print(f"      Min Area: {self.min_area}")
        print(f"      Confidence Threshold: {self.confidence_threshold}")
        print(f"   🎯 Chế độ chạy:")
        print(f"      Run Mode: {'Follow Line' if self.run else 'Alignment Only'}")
        print(f"      Alignment Threshold: {self.alignment_threshold}")
        print(f"   🦅 Bird's Eye View:")
        print(f"      Enabled: {'Yes' if self.birdview_enabled else 'No'}")
        print(f"      Size: {self.birdview_width}x{self.birdview_height}")
        print(f"   📺 Display:")
        print(f"      GUI Display: {'Enabled' if self.display_enabled else 'Disabled (SSH/Headless)'}")

    def _setup_perspective_transform(self):
        """Thiết lập ma trận perspective transform tối ưu cho bird's eye view"""
        # Kích thước camera JetBot với tối ưu scale
        base_width, base_height = 640, 480
        self.scaled_width = int(base_width * getattr(self, 'src_image_scale', 0.75))
        self.scaled_height = int(base_height * getattr(self, 'src_image_scale', 0.75))
        
        # Thiết lập interpolation method
        interp_method = getattr(self, 'birdview_interpolation', 'linear')
        interpolation_map = {
            'nearest': cv2.INTER_NEAREST,    # Nhanh nhất, chất lượng thấp
            'linear': cv2.INTER_LINEAR,      # Cân bằng tốc độ/chất lượng
            'cubic': cv2.INTER_CUBIC,        # Chậm hơn, chất lượng cao
            'area': cv2.INTER_AREA           # Tốt cho downsampling
        }
        self.interpolation_flag = interpolation_map.get(interp_method, cv2.INTER_LINEAR)
        
        # Định nghĩa 4 điểm trong hình ảnh gốc (scaled) từ config
        src_points = np.float32([
            [self.scaled_width * self.src_bottom_left[0], self.scaled_height * self.src_bottom_left[1]],    
            [self.scaled_width * self.src_bottom_right[0], self.scaled_height * self.src_bottom_right[1]],  
            [self.scaled_width * self.src_top_right[0], self.scaled_height * self.src_top_right[1]],        
            [self.scaled_width * self.src_top_left[0], self.scaled_height * self.src_top_left[1]]           
        ], dtype=np.float32)  # Explicit dtype for optimization
        
        # Định nghĩa 4 điểm trong bird's eye view từ config
        margin = self.dst_margin
        dst_points = np.float32([
            [self.birdview_width * margin, self.birdview_height],                    
            [self.birdview_width * (1-margin), self.birdview_height],               
            [self.birdview_width * (1-margin), 0],                                  
            [self.birdview_width * margin, 0]                                       
        ], dtype=np.float32)  # Explicit dtype for optimization
        
        # Tính ma trận perspective transform
        self.perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        self.inverse_matrix = cv2.getPerspectiveTransform(dst_points, src_points)
        
        # Cache key cho optimization
        cache_key = f"{self.scaled_width}x{self.scaled_height}_{self.birdview_width}x{self.birdview_height}"
        self._transform_cache[cache_key] = {
            'perspective': self.perspective_matrix.copy(),
            'inverse': self.inverse_matrix.copy()
        }
        
        print(f"📐 Perspective transform đã được tối ưu hóa:")
        print(f"   Input size: {base_width}x{base_height} -> {self.scaled_width}x{self.scaled_height} (scale: {getattr(self, 'src_image_scale', 0.75)})")
        print(f"   Interpolation: {interp_method} ({self.interpolation_flag})")
        print(f"   Source points: {src_points.astype(int).tolist()}")
        print(f"   Bird's eye size: {self.birdview_width}x{self.birdview_height}")
        print(f"   ROI optimization: {'Enabled' if getattr(self, 'enable_roi_optimization', True) else 'Disabled'}")

    def get_birdview(self, image):
        """Chuyển đổi hình ảnh sang bird's eye view với tối ưu hóa"""
        if not self.birdview_enabled or self.perspective_matrix is None:
            return image
        
        # Tối ưu: Scale down input image nếu cần
        if hasattr(self, 'src_image_scale') and self.src_image_scale != 1.0:
            scaled_image = cv2.resize(
                image, 
                (self.scaled_width, self.scaled_height),
                interpolation=self.interpolation_flag
            )
        else:
            scaled_image = image
            
        # Áp dụng perspective transform với interpolation tối ưu
        birdview = cv2.warpPerspective(
            scaled_image, 
            self.perspective_matrix, 
            (self.birdview_width, self.birdview_height),
            flags=self.interpolation_flag,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(255, 255, 255)  # White borders
        )
        return birdview

    def detect_black_line(self, image):
        """
        Phát hiện đường màu đen bằng computer vision với tối ưu hóa
        Returns: (center_x, confidence, angle, binary) hoặc (None, 0, 0, binary)
        """
        # Chuyển sang bird's eye view nếu được bật với tối ưu hóa
        if self.birdview_enabled:
            processed_image = self.get_birdview(image)
            self.birdview_frame = processed_image.copy()
        else:
            # Tối ưu: Scale down input image ngay cả khi không dùng birdview
            if hasattr(self, 'src_image_scale') and self.src_image_scale != 1.0:
                processed_image = cv2.resize(
                    image, 
                    (self.scaled_width, self.scaled_height),
                    interpolation=self.interpolation_flag
                )
            else:
                processed_image = image
            
        # Tối ưu: ROI-first processing để giảm tính toán
        height, width = processed_image.shape[:2]
        roi_enabled = getattr(self, 'enable_roi_optimization', True)
        
        if roi_enabled:
            # Chỉ xử lý ROI quan trọng (nửa dưới)
            roi_start = int(height * 0.6)  # Tối ưu hóa: 60% thay vì 50%
            roi_image = processed_image[roi_start:, :].copy()  # Copy để tránh memory issues
        else:
            roi_image = processed_image
            roi_start = 0
            
        # Chuyển sang grayscale với memory-efficient operation
        if len(roi_image.shape) == 3:
            # Tối ưu: sử dụng weighted grayscale cho accuracy cao hơn
            gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi_image
        
        # Blur với kernel size tối ưu cho JetBot
        kernel_size = 3 if min(gray.shape) < 200 else 5  # Adaptive kernel size
        blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
        
        # Threshold với adaptive method cho lighting conditions khác nhau
        if hasattr(self, 'adaptive_threshold') and getattr(self, 'adaptive_threshold', False):
            binary = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                cv2.THRESH_BINARY_INV, 11, 2
            )
        else:
            _, binary = cv2.threshold(blurred, self.black_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Tìm contours với tối ưu hóa
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None, 0, 0, binary
        
        # Tối ưu: Lọc contours theo diện tích trước khi xử lý
        valid_contours = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        
        if not valid_contours:
            return None, 0, 0, binary
        
        # Tìm contour lớn nhất trong các contour hợp lệ
        largest_contour = max(valid_contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Tính center và angle với vectorized operations
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"]) + roi_start
            
            # Normalize center_x về [-1, 1] với float conversion tối ưu
            # Đảo ngược dấu để sửa vấn đề steering ngược
            current_width = binary.shape[1]
            normalized_x = float(-(center_x - current_width/2) / (current_width/2))
            
            # Confidence calculation tối ưu hóa
            confidence = float(min(area / 1000, 1.0))
            
            # Tính góc của đường bằng cách fit line
            angle = self._calculate_line_angle(largest_contour, roi_start, current_width, height)
            
            return normalized_x, confidence, angle, binary
        
        return None, 0, 0, binary

    def _calculate_line_angle(self, contour, roi_offset, width, height):
        """Tính góc của đường từ contour với tối ưu hóa"""
        try:
            # Tối ưu: Kiểm tra kích thước contour trước khi xử lý
            if len(contour) < 5:  # Cần ít nhất 5 điểm cho fitLine
                return 0.0
            
            # Fit một đường thẳng qua contour với optimization
            [vx, vy, x, y] = cv2.fitLine(
                contour, 
                cv2.DIST_L2,    # Distance type - L2 is good balance
                0,              # Parameter (not used for L2)
                0.01,           # Radius accuracy
                0.01            # Angle accuracy
            )
            
            # Tính góc từ vector direction (vx, vy) - sửa lỗi steering ngược
            # Đảo ngược vx để sửa vấn đề quẹo ngược chiều
            angle_rad = np.arctan2(float(-vx), float(vy))  # Góc theo radian (đảo dấu vx)
            angle_deg = np.degrees(angle_rad)  # Chuyển sang độ
            
            # Normalize góc về [-90, 90] độ - optimized  
            # FIXED: Sau khi đảo ngược vx, giờ âm = rẽ trái, dương = rẽ phải (đúng)
            angle_deg = angle_deg % 360
            if angle_deg > 180:
                angle_deg -= 360
            angle_deg = np.clip(angle_deg, -90, 90)
                
            return float(angle_deg)
            
        except Exception as e:
            # Fallback gracefully
            return 0.0

    def create_debug_image(self, original, binary, center_x=None, confidence=0, steering=0, angle=0):
        """Tạo hình ảnh debug để hiển thị"""
        height, width = binary.shape
        
        # Tạo hình ảnh 3 channel
        debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        
        # Vẽ đường trung tâm (xanh lá)
        cv2.line(debug_img, (width//2, 0), (width//2, height), (0, 255, 0), 2)
        
        # Vẽ điểm center nếu có (đỏ)
        if center_x is not None:
            pixel_x = int((center_x * width/2) + width/2)
            cv2.circle(debug_img, (pixel_x, height//2), 8, (0, 0, 255), -1)
            
            # Vẽ đường steering (xanh dương)
            cv2.line(debug_img, (width//2, height//2), (pixel_x, height//2), (255, 0, 0), 3)
            
            # Vẽ vector hướng của đường dựa trên angle (màu tím)
            angle_rad = np.radians(angle)
            end_x = int(pixel_x + 30 * np.sin(angle_rad))
            end_y = int(height//2 - 30 * np.cos(angle_rad))
            cv2.arrowedLine(debug_img, (pixel_x, height//2), (end_x, end_y), (255, 0, 255), 2)
        
        # Thêm text thông tin
        cv2.putText(debug_img, f"Confidence: {confidence:.2f}", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_img, f"Center X: {center_x:.3f}" if center_x else "No Line", 
                   (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_img, f"Angle: {angle:.1f}°", (10, 65), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_img, f"Steering: {steering:.3f}", (10, 85), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Thêm thông tin về chế độ chạy
        mode_text = "RUN" if hasattr(self, 'run') and self.run else "ALIGN"
        mode_color = (0, 255, 0) if hasattr(self, 'run') and self.run else (255, 255, 0)
        cv2.putText(debug_img, f"Mode: {mode_text}", (10, 105), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1)
        
        cv2.putText(debug_img, "Press 'q' to quit", (10, height-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        return debug_img

    def create_combined_debug_image(self, original, debug_processed, birdview=None):
        """Tạo hình ảnh kết hợp giữa original, processed và bird's eye view, có vẽ polygon birdview lên ảnh gốc"""
        # Vẽ polygon lên ảnh gốc
        original_with_poly = self.draw_birdview_polygon(original)
        if birdview is None or not self.birdview_enabled:
            # Chỉ hiển thị original và processed
            combined = np.hstack([
                cv2.resize(original_with_poly, (300, 300)),
                cv2.resize(debug_processed, (300, 300))
            ])
        else:
            # Hiển thị cả 3: original+poly, bird's eye view, processed
            original_resized = cv2.resize(original_with_poly, (200, 200))
            birdview_resized = cv2.resize(birdview, (200, 200))
            processed_resized = cv2.resize(debug_processed, (200, 200))
            # Tạo layout 2x2 (chỉ dùng 3 ô)
            top_row = np.hstack([original_resized, birdview_resized])
            bottom_row = np.hstack([processed_resized, np.zeros((200, 200, 3), dtype=np.uint8)])
            combined = np.vstack([top_row, bottom_row])
            # Thêm label cho mỗi view
            cv2.putText(combined, "Original+Poly", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(combined, "Bird's Eye", (210, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(combined, "Processed", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return combined

    def camera_callback(self, change):
        """Callback được gọi khi có frame mới từ camera"""
        self.current_frame = change['new'].copy()

    def control_loop(self):
        """Vòng lặp điều khiển chính với performance monitoring"""
        print("🚀 Bắt đầu điều khiển robot...")
        if self.display_enabled:
            print("📹 Hiển thị cửa sổ camera - nhấn 'q' để thoát, 'r' để chuyển chế độ")
        else:
            print("🖥️ Chạy trong chế độ headless - chỉ có log trong terminal")
        
        frame_count = 0
        last_perf_report = time.time()
        
        # Khởi động ghi video nếu bật
        if self.recording_enabled and self.video_writer is None:
            timestr = time.strftime("%Y%m%d-%H%M%S")
            self.video_filename = f"jetbot_run_{timestr}.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(self.video_filename, fourcc, self.video_fps, self.video_size)
            print(f"🎥 Đang ghi video vào {self.video_filename}")
        
        while self.running:
            if self.current_frame is None:
                time.sleep(0.01)
                continue
                
            try:
                # Performance monitoring start
                if self.perf_monitor:
                    self.perf_monitor.start_timer('total_frame')
                    self.perf_monitor.start_timer('detect_line')
                
                # Phát hiện đường với bird's eye view tối ưu hóa
                center_x, confidence, angle, binary = self.detect_black_line(self.current_frame)
                
                if self.perf_monitor:
                    self.perf_monitor.end_timer('detect_line')
                    self.perf_monitor.start_timer('control_logic')
                
                # Điều khiển robot
                steering = 0.0
                if center_x is not None and confidence > self.confidence_threshold:
                    # Kiểm tra trạng thái alignment
                    is_well_aligned = abs(center_x) < self.alignment_threshold
                    
                    if self.run or not is_well_aligned:
                        # Chế độ run=True HOẶC chưa align đúng thì tiếp tục điều khiển
                        
                        # Enhanced PID controller sử dụng cả center_x và angle
                        # Steering dựa trên center position
                        position_steering = center_x * self.steering_gain
                        
                        # Steering dựa trên angle của đường (preview steering)
                        # Chỉ sử dụng angle steering khi ở chế độ run
                        angle_steering = 0.0
                        if self.run:
                            angle_steering = np.sin(np.radians(angle)) * self.angle_factor
                        
                        # Kết hợp cả hai
                        steering = (position_steering + angle_steering + 
                                   (center_x - self.last_steering) * self.steering_kd + 
                                   self.steering_bias)
                        
                        # Convert to float and limit steering
                        steering = float(max(min(steering, 1.0), -1.0))
                        self.last_steering = center_x
                        
                        # Điều khiển động cơ
                        left_speed = max(min(self.speed_gain - steering, 1.0), 0.0)
                        right_speed = max(min(self.speed_gain + steering, 1.0), 0.0)

                        print(f"Debug: steering={steering}, left_speed={left_speed}, right_speed={right_speed}")
                        
                        # Convert to float to avoid numpy array error
                        self.robot.left_motor.value = float(left_speed)
                        self.robot.right_motor.value = float(right_speed)
                        
                        # Cập nhật trạng thái alignment
                        if not self.is_aligned and is_well_aligned and not self.run:
                            self.is_aligned = True
                            print("✅ Robot đã align vào giữa đường - dừng lại")
                        
                        # In thông tin (mỗi 30 frame)
                        if int(time.time() * 30) % 30 == 0:
                            mode_str = "RUN" if self.run else "ALIGN"
                            align_str = "✅" if is_well_aligned else "❌"
                            print(f"🎯 [{mode_str}] Center: {center_x:.3f} {align_str}, Angle: {angle:.1f}°, "
                                  f"Confidence: {confidence:.3f}, Steering: {steering:.3f}, "
                                  f"Speed: L={left_speed:.3f} R={right_speed:.3f}")
                    else:
                        # Chế độ alignment và đã align xong - dừng robot
                        self.robot.stop()
                        if not self.is_aligned:
                            self.is_aligned = True
                            print("✅ Robot đã align vào giữa đường - dừng lại")
                        
                        # In thông báo alignment (ít hơn)
                        if int(time.time() * 5) % 30 == 0:
                            print(f"⏸️ [ALIGNED] Center: {center_x:.3f} ✅ - Robot đang dừng")
                else:
                    # Không phát hiện được đường - dừng
                    self.robot.stop()
                    self.is_aligned = False  # Reset trạng thái alignment
                    if int(time.time() * 10) % 30 == 0:  # In ít hơn khi không thấy đường
                        print("⚠️ Không phát hiện được đường - Robot dừng")
                
                # Tạo debug image chỉ khi cần hiển thị
                if self.display_enabled:
                    debug_processed = self.create_debug_image(
                        self.current_frame, binary, center_x, confidence, steering, angle)
                    
                    # Tạo combined debug image với bird's eye view
                    self.debug_frame = self.create_combined_debug_image(
                        self.current_frame, debug_processed, self.birdview_frame)
                    
                    # Lưu 3 ảnh debug mỗi 1s (original+poly, birdview, processed)
                    now = time.time()
                    if now - self.last_debug_save_time > 1.0:
                        try:
                            # original+poly
                            original_poly = self.draw_birdview_polygon(self.current_frame)
                            cv2.imwrite(os.path.join(self.debug_save_dir, f"original_poly_{int(now)}.jpg"), original_poly)
                            # birdview
                            if self.birdview_frame is not None:
                                cv2.imwrite(os.path.join(self.debug_save_dir, f"birdview_{int(now)}.jpg"), self.birdview_frame)
                            # processed
                            cv2.imwrite(os.path.join(self.debug_save_dir, f"processed_{int(now)}.jpg"), debug_processed)
                            self.last_debug_save_time = now
                        except Exception as e:
                            print(f"⚠️ Lỗi lưu ảnh debug: {e}")
                
                # Ghi video frame (chỉ ghi frame gốc, resize về video_size)
                if self.recording_enabled and self.video_writer is not None:
                    try:
                        frame_to_write = cv2.resize(self.current_frame, self.video_size)
                        self.video_writer.write(frame_to_write)
                    except Exception as e:
                        print(f"⚠️ Lỗi ghi video: {e}")
            except Exception as e:
                print(f"❌ Lỗi trong control loop: {e}")
                self.robot.stop()
                
            time.sleep(0.05)  # 20 FPS

    def display_loop(self):
        """Vòng lặp hiển thị hình ảnh"""
        if not self.display_enabled:
            # Chế độ headless - chỉ chờ tín hiệu dừng
            print("🖥️ Chạy trong chế độ headless - nhấn Ctrl+C để dừng")
            try:
                while self.running:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n🛑 Nhận tín hiệu Ctrl+C...")
                self.stop()
            return
        
        # Chế độ có GUI
        try:
            cv2.namedWindow('JetBot Line Following', cv2.WINDOW_AUTOSIZE)
            
            while self.running:
                if self.debug_frame is not None:
                    cv2.imshow('JetBot Line Following', self.debug_frame)
                    
                # Kiểm tra phím nhấn
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' hoặc ESC
                    print("🛑 Người dùng yêu cầu thoát...")
                    self.stop()
                    break
                elif key == ord('r'):  # 'r' để toggle run mode
                    self.toggle_run_mode()
                    
                time.sleep(0.03)  # ~30 FPS cho display
            
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"⚠️ Lỗi hiển thị: {e} - Chuyển sang chế độ headless")
            # Fallback to headless mode
            try:
                while self.running:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n🛑 Nhận tín hiệu Ctrl+C...")
                self.stop()

    def start(self):
        """Bắt đầu line following"""
        if self.running:
            print("⚠️ Robot đã đang chạy!")
            return
            
        self.running = True
        
        # Bắt đầu camera callback
        self.camera.observe(self.camera_callback, names='value')
        
        # Tạo và bắt đầu các thread
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        
        self.control_thread.start()
        self.display_thread.start()
        
        # Thêm console input thread cho chế độ headless
        if not self.display_enabled:
            self.console_thread = threading.Thread(target=self.console_input_loop, daemon=True)
            self.console_thread.start()
        
        print("✅ Line following đã bắt đầu!")
        if self.display_enabled:
            print("💡 Mẹo: Nhấn 'q' trong cửa sổ camera để dừng, 'r' để chuyển chế độ")
        else:
            print("💡 Mẹo: Nhấn Ctrl+C để dừng robot, hoặc gõ 'q' + Enter")
            print("💡 Trong chế độ SSH: Gõ 'r' + Enter để chuyển chế độ")
        print("📊 Theo dõi log để xem trạng thái robot")
        
        # Chờ display thread kết thúc (khi người dùng nhấn 'q')
        try:
            self.display_thread.join()
        except KeyboardInterrupt:
            print("\n🛑 Nhận tín hiệu Ctrl+C...")
            self.stop()

    def stop(self):
        """Dừng line following"""
        if not self.running:
            return
            
        print("🛑 Đang dừng robot...")
        self.running = False
        
        # Ngắt camera callback
        try:
            self.camera.unobserve(self.camera_callback, names='value')
        except:
            pass
        
        # Dừng robot
        self.robot.stop()
        
        # Chờ threads kết thúc
        time.sleep(0.2)
        
        print("✅ Robot đã dừng!")

    def cleanup(self):
        """Giải phóng tài nguyên"""
        self.stop()
        try:
            self.camera.stop()
            print("🔌 Camera đã được đóng")
        except:
            pass

def signal_handler(sig, frame):
    """Xử lý tín hiệu Ctrl+C"""
    print("\n🛑 Nhận tín hiệu dừng...")
    if 'follower' in globals():
        follower.cleanup()
    sys.exit(0)

def main():
    """Hàm main"""
    print("=" * 60)
    print("🤖 JETBOT LINE FOLLOWING với COMPUTER VISION")
    print("=" * 60)
    print("📋 Tính năng:")
    print("   • Sử dụng OpenCV để detect đường màu đen")
    print("   • Bird's Eye View transformation cho góc chính xác")
    print("   • Enhanced PID controller với angle prediction")
    print("   • Alignment mode: chỉ căn giữa không chạy theo đường")
    print("   • Hiển thị real-time debug window (3 views)")
    print("   • Load/Save thông số từ config.txt")
    print("   • Tự động dừng khi không thấy đường")
    print()
    print("🎮 Điều khiển:")
    print("   • Desktop: Nhấn 'q' trong cửa sổ camera để thoát")
    print("   • SSH: Nhấn Ctrl+C trong terminal để thoát")
    print("   • Chỉnh sửa config.txt để điều chỉnh thông số")
    print("   • Tự động phát hiện môi trường SSH/headless")
    print()
    print("⚙️ Config:")
    print("   • run=true: chạy theo đường liên tục")
    print("   • run=false: chỉ căn giữa đường rồi dừng")
    print("   • alignment_threshold: ngưỡng để coi là đã căn giữa")
    print()
    print("📋 Yêu cầu:")
    print("   • Đường màu đen trên nền sáng")
    print("   • Ánh sáng đều, không có bóng đổ")
    print("   • Robot có đủ không gian an toàn")
    print("=" * 60)
    
    # Đăng ký signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    global follower
    follower = None
    
    try:
        # Tạo và khởi động line follower
        follower = LineFollower()
        
        input("\n🚀 Nhấn Enter để bắt đầu (hoặc Ctrl+C để thoát)...")
        
        follower.start()
        
    except KeyboardInterrupt:
        print("\n🛑 Chương trình bị gián đoạn")
    except Exception as e:
        print(f"❌ Lỗi: {e}")
    finally:
        if follower:
            follower.cleanup()

if __name__ == "__main__":
    main()