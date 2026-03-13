#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from scara_brain.modules.cv_utils import imgmsg_to_cv2, get_most_circular_contour, draw_crosshair
import numpy as np

class EyesNode(Node):
    def __init__(self):
        super().__init__("eyes_node")
        self.get_logger().info(f"eyes_node Started")

        self.img_sub = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        
    def image_callback(self, msg: Image):
        frame = imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,
            param1=50,   # edge detection threshold
            param2=30,   # accumulator threshold (lower = more detections)
            minRadius=20,
            maxRadius=100
        )
        
        result_img = frame.copy()
        height, width = gray.shape
        draw_crosshair(result_img, width // 2, height // 2, color=(0, 0, 255))
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            cx, cy, r = circles[0][0]  # best circle
            cv2.circle(result_img, (cx, cy), r, (255, 0, 0), 2)
            draw_crosshair(result_img, cx, cy, size=5, color=(0, 255, 0))
        
        cv2.imshow('ef_camera', result_img)
        cv2.waitKey(1)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = EyesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()