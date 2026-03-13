import cv2
import numpy as np
from sensor_msgs.msg import Image

def imgmsg_to_cv2(msg: Image):
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

def get_most_circular_contour(contours):
    best_contour = None
    best_circularity = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if perimeter == 0:
            continue
            
        circularity = 4 * np.pi * area / (perimeter ** 2)
        
        if circularity > best_circularity:
            best_circularity = circularity
            best_contour = contour
            
    return best_contour
    
def draw_crosshair(img, cx, cy, size=20, color=(0, 255, 0), thickness=2):
    cv2.line(img, (cx - size, cy), (cx + size, cy), color, thickness)
    cv2.line(img, (cx, cy - size), (cx, cy + size), color, thickness)