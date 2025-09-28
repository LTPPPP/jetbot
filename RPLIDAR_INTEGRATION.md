# JetBot RPLIDAR Integration

## Tổng quan

Tôi đã tích hợp thành công RPLiDAR vào JetBot của bạn. Package này cung cấp giao diện Python để sử dụng RPLiDAR sensor với JetBot, bao gồm collision avoidance và visualization.

## Các file đã tạo

### 1. Core LIDAR Interface

- **`jetbot/lidar.py`**: Giao diện chính để tương tác với RPLiDAR
  - Đọc dữ liệu laser scan từ ROS topic `/scan`
  - Phát hiện obstacle tự động
  - Điều khiển motor RPLiDAR
  - Truy xuất khoảng cách theo góc cụ thể

### 2. Collision Avoidance

- **`jetbot_lidar_collision_avoidance.py`**: Hệ thống tránh vật cản dựa trên LIDAR
  - Phân tích dữ liệu LIDAR để quyết định hành động
  - Điều khiển robot tự động tránh vật cản
  - Có thể tùy chỉnh thông số như tốc độ, khoảng cách tối thiểu

### 3. Launch Files

- **`launch/jetbot_rplidar_a1.launch`**: Cấu hình cho RPLIDAR A1
- **`launch/jetbot_rplidar_a3.launch`**: Cấu hình cho RPLIDAR A3
- **`launch/jetbot_lidar_visualization.launch`**: Bao gồm RViz visualization

### 4. Visualization

- **`rviz/jetbot_lidar.rviz`**: Cấu hình RViz để hiển thị dữ liệu LIDAR

### 5. Setup và Test

- **`setup_jetbot_lidar.py`**: Script kiểm tra và hướng dẫn setup
- **Sẽ tạo `test_lidar.py`**: Script test cơ bản

## Cách sử dụng

### 1. Cài đặt RPLIDAR ROS Package

```bash
# Di chuyển đến workspace
cd ~/catkin_ws/src

# Clone rplidar_ros package
git clone https://github.com/Slamtec/rplidar_ros.git

# Build workspace
cd ~/catkin_ws
catkin_make

# Source the workspace
source ~/catkin_ws/devel/setup.bash
```

### 2. Kết nối Hardware

1. Kết nối RPLiDAR vào JetBot qua USB
2. Kiểm tra device path (thường là `/dev/ttyUSB0`)
3. Đảm bảo có quyền truy cập: `sudo chmod 666 /dev/ttyUSB0`

### 3. Chạy Setup Script

```bash
cd /home/ltpppp/Documents/HACKATHON2025/code/jetbot
python3 setup_jetbot_lidar.py
```

### 4. Launch RPLIDAR

```bash
# Start roscore
roscore

# Trong terminal khác, launch RPLIDAR
roslaunch jetbot jetbot_rplidar_a1.launch

# Hoặc với visualization
roslaunch jetbot jetbot_lidar_visualization.launch
```

### 5. Sử dụng trong Code

```python
from jetbot import Robot, get_lidar
import time

# Initialize
robot = Robot()
lidar = get_lidar()

# Start LIDAR motor
lidar.start_motor()

# Wait for data
time.sleep(2)

# Get distances
front_distance = lidar.get_front_distance()
left_distance = lidar.get_distance_at_angle(-45)
right_distance = lidar.get_distance_at_angle(45)

print(f"Front: {front_distance:.2f}m")
print(f"Left: {left_distance:.2f}m")
print(f"Right: {right_distance:.2f}m")

# Simple collision avoidance
if front_distance < 0.5:
    if left_distance > right_distance:
        robot.left(0.2)
    else:
        robot.right(0.2)
else:
    robot.forward(0.3)

time.sleep(1)
robot.stop()
```

### 6. Chạy Collision Avoidance

```bash
# Sau khi đã launch rplidar
python3 jetbot_lidar_collision_avoidance.py
```

## Tính năng chính

### Lidar Class

- **`get_front_distance(angle_range)`**: Lấy khoảng cách phía trước
- **`get_distance_at_angle(angle)`**: Lấy khoảng cách ở góc cụ thể
- **`obstacle_detected`**: Property tự động phát hiện obstacle
- **`start_motor()` / `stop_motor()`**: Điều khiển motor RPLiDAR

### LidarCollisionAvoidance Class

- Tự động phân tích dữ liệu LIDAR
- Quyết định hành động: tiến, quay trái, quay phải, dừng
- Tùy chỉnh được các thông số

## Cấu hình cho các model RPLiDAR khác nhau

### RPLIDAR A1 (115200 baud)

```xml
<param name="serial_baudrate" type="int" value="115200"/>
<param name="scan_frequency" type="double" value="5.5"/>
```

### RPLIDAR A3 (256000 baud)

```xml
<param name="serial_baudrate" type="int" value="256000"/>
<param name="scan_mode" type="string" value="Sensitivity"/>
```

### S-Series với TCP

```xml
<param name="channel_type" type="string" value="tcp"/>
<param name="tcp_ip" type="string" value="192.168.0.7"/>
<param name="tcp_port" type="int" value="20108"/>
```

## Troubleshooting

1. **Không tìm thấy `/dev/ttyUSB0`**:

   - Kiểm tra kết nối USB
   - Thử `/dev/ttyUSB1` hoặc `/dev/ttyACM0`

2. **Permission denied**:

   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # hoặc add user vào dialout group
   sudo usermod -a -G dialout $USER
   ```

3. **Không có dữ liệu LIDAR**:

   - Kiểm tra roscore đang chạy
   - Kiểm tra rplidar node đang publish `/scan` topic
   - Kiểm tra motor có quay không

4. **Build error**:
   ```bash
   cd ~/catkin_ws
   catkin_make clean
   catkin_make
   ```

Bây giờ JetBot của bạn đã sẵn sàng sử dụng RPLiDAR để navigation và obstacle avoidance!
