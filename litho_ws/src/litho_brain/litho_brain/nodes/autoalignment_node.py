#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from litho_brain.utils.cv_utils import imgmsg_to_cv2
from litho_brain.constants import MARKER_WIDTH_METERS
from sensor_msgs.msg import Image
import cv2
from ament_index_python.packages import get_package_share_directory
from litho_brain.utils.cv_utils import draw_crosshair
import os
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3


# todo look into Moiré Fringe (acheives 10nm precision, tho quite slow)
# todo for ROI to work we need to have a proper matchign template with the acutal correct size the image is expected to see
class AutoAlignment(Node):
    def __init__(self):
        super().__init__("autoalignment_node")
        self.get_logger().info(f"autoalignment_node Started")
        
        self.declare_parameter("debug_autoalignment", True)
        self.debug_autoalignment: bool = self.get_parameter("debug_autoalignment").value # type: ignore
        
        self._correction_fac_pub = self.create_publisher(Vector3, 'autoalignment/correction_vec', 10)
        self._marker_visible_pub = self.create_publisher(Bool, 'autoalignment/is_marker_visible', 10)
        self._img_sub = self.create_subscription(Image, '/camera/image', self._img_callback, 10)
        
        template_path = os.path.join(get_package_share_directory('litho_brain'), 'assets', 'cross_template.png')
        self._cross_template: cv2.typing.MatLike = cv2.imread(template_path) # type: ignore
        
    def _img_callback(self, msg: Image):
        img = imgmsg_to_cv2(msg)
        
        result = cv2.matchTemplate(img, self._cross_template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        
        # top_left = (max_loc[0], max_loc[1])
        # bottom_right = (max_loc[0] + self._cross_template.shape[1], max_loc[1] + self._cross_template.shape[0])
        # roi_img = img[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
        
        roi_img = img
        top_left = (0, 0)
        bottom_right = (img.shape[1], img.shape[0])
        
        if self.debug_autoalignment:
            visual_img = img.copy()
        
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        
        _, bin = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        marker_cx = -1
        marker_cy = -1
        img_cx = img.shape[1]/2
        img_cy = img.shape[0]/2
        microns_per_px = -1
        if contours:
            largest_blob = max(contours, key=cv2.contourArea)
            _, _, w, h = cv2.boundingRect(largest_blob)
            
            microns_per_px = MARKER_WIDTH_METERS / w
            
            M = cv2.moments(largest_blob)
            if M['m00'] > 0:
                marker_cx = top_left[0] + M['m10'] / M['m00']
                marker_cy = top_left[1] + M['m01'] / M['m00']
            
            if self.debug_autoalignment:
                cv2.drawContours(visual_img, [largest_blob + top_left], -1, (0, 255, 0), 1)
                
        marker_visible = len(contours) > 0
        self._marker_visible_pub.publish(Bool(data=marker_visible))
        
        if marker_visible:
            alignment_x_meters = -(img_cx - marker_cx) * microns_per_px
            alignment_y_meters = -(img_cy - marker_cy) * microns_per_px
            
            # notice the convention flipping, x is the forward which here is the y
            align_vec_msg = Vector3()
            align_vec_msg.y = alignment_x_meters
            align_vec_msg.x = alignment_y_meters
            
            self._correction_fac_pub.publish(align_vec_msg)


        if self.debug_autoalignment:
            cv2.rectangle(visual_img, top_left, bottom_right, (0, 0, 255), thickness=1)
            draw_crosshair(visual_img, int(marker_cx), int(marker_cy), size=1, color=(0, 255, 0), thickness=1)
            draw_crosshair(visual_img, round(img_cx), round(img_cy), size=1, color=(0, 0, 255), thickness=1)
            
            cv2.imshow('Contour', visual_img)
            cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)
    node = AutoAlignment()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()