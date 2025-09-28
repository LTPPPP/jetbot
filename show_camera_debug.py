#!/usr/bin/env python3
"""
JetBot Camera Debug - Hiển thị ảnh gốc và ảnh sau xử lý contour
Sử dụng để debug line following
"""

import cv2
import numpy as np
from jetbot import Robot, Camera
import time
import os

# --- CẤU HÌNH ---

# 1. Kích thước ảnh từ camera
WIDTH = 640
HEIGHT = 480

# 2. Vùng quan tâm (Region of Interest - ROI)
ROI_Y = int(HEIGHT * 0.6)  # Bắt đầu từ 60% chiều cao của ảnh
ROI_H = int(HEIGHT * 0.4)  # Lấy 40% chiều cao còn lại

# 3. Thông số xử lý ảnh cho đường đen
BLACK_THRESHOLD = 20        # Ngưỡng phát hiện đen (30-120)
MIN_AREA = 100             # Diện tích tối thiểu của contour

# 4. Tốc độ của Robot
BASE_SPEED = 0.15
STEERING_GAIN = 0.5

# --- KHỞI TẠO ---
robot = Robot()
camera = Camera.instance(width=WIDTH, height=HEIGHT)

# Kiểm tra môi trường hiển thị
display_enabled = True
try:
    if not os.environ.get('DISPLAY'):
        print("⚠️ Không tìm thấy DISPLAY - Chạy trong chế độ SSH (không hiển thị)")
        display_enabled = False
    else:
        # Thử tạo window test
        cv2.namedWindow('test', cv2.WINDOW_AUTOSIZE)
        cv2.destroyWindow('test')
        print("✅ Display khả dụng - Sẽ hiển thị cửa sổ debug")
except:
    print("⚠️ Không thể tạo window - Chạy trong chế độ headless")
    display_enabled = False

print("Khởi tạo hoàn tất. Robot sẵn sàng...")
if display_enabled:
    print("📺 Sẽ hiển thị 2 cửa sổ: 'Original' và 'Processed'")
    print("Nhấn 'q' để thoát")
else:
    print("🖥️ Chạy trong chế độ headless - chỉ có log")
    print("Nhấn Ctrl+C để thoát")

time.sleep(2)
print("Bắt đầu!")

def detect_black_line(image):
    """Phát hiện đường màu đen và trả về thông tin debug"""
    # Lấy ROI
    roi = image[ROI_Y : ROI_Y + ROI_H, :]
    
    # Chuyển sang grayscale
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # Blur để giảm noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Threshold để tạo ảnh binary (đen-trắng)
    _, binary = cv2.threshold(blurred, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    
    # Tìm contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Tạo ảnh debug để hiển thị
    debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    
    # Vẽ đường trung tâm (xanh lá)
    cv2.line(debug_img, (WIDTH//2, 0), (WIDTH//2, ROI_H), (0, 255, 0), 2)
    
    center_x = None
    confidence = 0
    
    if len(contours) > 0:
        # Lọc contours theo diện tích
        valid_contours = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]
        
        if valid_contours:
            # Tìm contour lớn nhất
            largest_contour = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            confidence = min(area / 1000, 1.0)
            
            # Vẽ contour lên ảnh debug (màu xanh dương)
            cv2.drawContours(debug_img, [largest_contour], -1, (255, 0, 0), 2)
            
            # Tính tâm của contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Vẽ điểm tâm (đỏ)
                cv2.circle(debug_img, (cx, cy), 8, (0, 0, 255), -1)
                
                # Vẽ đường từ tâm màn hình đến tâm đường (màu vàng)
                cv2.line(debug_img, (WIDTH//2, cy), (cx, cy), (0, 255, 255), 3)
                
                # Normalize center_x về [-1, 1]
                center_x = (cx - WIDTH/2) / (WIDTH/2)
    
    # Thêm text thông tin
    cv2.putText(debug_img, f"Confidence: {confidence:.2f}", (10, 25), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(debug_img, f"Center X: {center_x:.3f}" if center_x else "No Line", 
               (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(debug_img, f"Contours: {len(contours)}", (10, 75), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    return center_x, confidence, debug_img

# --- VÒNG LẶP CHÍNH ---
try:
    frame_count = 0
    
    while True:
        # Đọc ảnh từ camera
        image = camera.value
        
        # Xử lý phát hiện đường
        center_x, confidence, processed_img = detect_black_line(image)
        
        # Tạo ảnh gốc với ROI được đánh dấu
        original_display = image.copy()
        
        # Vẽ ROI lên ảnh gốc (màu xanh lá)
        cv2.rectangle(original_display, (0, ROI_Y), (WIDTH, ROI_Y + ROI_H), (0, 255, 0), 2)
        cv2.putText(original_display, "ROI", (10, ROI_Y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Thêm thông tin lên ảnh gốc
        cv2.putText(original_display, f"Frame: {frame_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(original_display, f"Size: {WIDTH}x{HEIGHT}", (10, 55), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Điều khiển robot (đơn giản)
        if center_x is not None and confidence > 0.1:
            # Tính toán steering
            steering = center_x * STEERING_GAIN
            left_speed = max(0, min(1, BASE_SPEED - steering))
            right_speed = max(0, min(1, BASE_SPEED + steering))
            
            robot.left_motor.value = left_speed
            robot.right_motor.value = right_speed
            
            # Hiển thị trạng thái trên ảnh gốc
            status = f"RUNNING - L:{left_speed:.2f} R:{right_speed:.2f}"
            color = (0, 255, 0)
        else:
            robot.stop()
            status = "STOPPED - No line detected"
            color = (0, 0, 255)
            
        cv2.putText(original_display, status, (10, HEIGHT - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Hiển thị ảnh nếu có display
        if display_enabled:
            cv2.imshow('Original Camera', original_display)
            cv2.imshow('Processed (ROI + Contours)', processed_img)
            
            # Kiểm tra phím nhấn
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' hoặc ESC
                print("🛑 Người dùng yêu cầu thoát...")
                break
        
        # In log mỗi 30 frame
        if frame_count % 30 == 0:
            if center_x is not None:
                print(f"Frame {frame_count}: Center={center_x:.3f}, Confidence={confidence:.3f}")
            else:
                print(f"Frame {frame_count}: No line detected")
        
        frame_count += 1
        time.sleep(0.033)  # ~30 FPS

except KeyboardInterrupt:
    print("\n🛑 Chương trình bị gián đoạn")
finally:
    # Dọn dẹp
    robot.stop()
    if display_enabled:
        cv2.destroyAllWindows()
    camera.stop()
    print("✅ Đã dừng và dọn dẹp")