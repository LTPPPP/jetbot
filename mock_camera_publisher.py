#!/usr/bin/env python3
"""
Simple camera publisher for testing when no camera is available
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MockCameraPublisher:
    def __init__(self):
        rospy.init_node('mock_camera_publisher', anonymous=True)
        
        self.pub = rospy.Publisher('/csi_cam_0/image_raw', Image, queue_size=1)
        self.bridge = CvBridge()
        
        # Create a simple test image
        self.test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(self.test_image, "MOCK CAMERA", (200, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        rospy.loginfo("Mock camera publisher started")
    
    def run(self):
        rate = rospy.Rate(10)  # 10 FPS
        
        while not rospy.is_shutdown():
            # Add timestamp to image
            img = self.test_image.copy()
            timestamp = rospy.Time.now().to_sec()
            cv2.putText(img, f"Time: {timestamp:.1f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Convert and publish
            try:
                msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "camera_link"
                self.pub.publish(msg)
            except Exception as e:
                rospy.logerr(f"Failed to publish image: {e}")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        publisher = MockCameraPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass