#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64
import cv2
from litho_brain.utils.cv_utils import imgmsg_to_cv2
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


class AutoFocusNode(Node):
    def __init__(self):
        super().__init__("auto_focus_node")
        self.get_logger().info(f"auto_focus_node Started")
        
        self.declare_parameter('debug_autofocus', True)
        self._debug_autofocus = self.get_parameter('debug_autofocus').value

        self._img_sub = self.create_subscription(Image, '/camera/image', self._img_callback, 10)
        self._sharpness_pub = self.create_publisher(Float64, 'autofocus/sharpness', 10)
        
        if self._debug_autofocus:
            # setup live plot
            self._z_pos = 0.0
            self._z_history = []
            self._sharpness_history = []
            
            self._joint_sub = self.create_subscription(
                JointState, '/joint_states', self._joint_cb, 10)
            
            plt.ion()
            self._fig, self._ax = plt.subplots()
            self._ax.set_xlabel('Z position (m)')
            self._ax.set_ylabel('Sharpness (Laplacian variance)')
            self._ax.set_title('Focus Curve')
            self._line, = self._ax.plot([], [], 'b.')
            
    def _joint_cb(self, msg: JointState):
        idx = msg.name.index('y_z_stage_joint')
        self._z_pos = msg.position[idx]
        
    def _img_callback(self, msg: Image):
        # Todo use FFT high freq energy appraoch instead as it gets us sinc interpolation for sub pixel optimization
        img = imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        sharpness = laplacian.var().item()
        
        self._sharpness_pub.publish(Float64(data=sharpness))

        
        if self._debug_autofocus:
            self._z_history.append(self._z_pos)
            self._sharpness_history.append(sharpness)
            
            self._line.set_data(self._z_history, self._sharpness_history)
            self._ax.relim()
            self._ax.autoscale_view()
            self._fig.canvas.flush_events()
            plt.pause(0.001)

            display = cv2.normalize(laplacian, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U) # type: ignore
            cv2.putText(display, f"Sharpness: {sharpness:.2f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow('autofocus_laplacian', display)
            cv2.imshow('autofocus_camera', img)
            cv2.waitKey(1)
            

def main(args=None):
    rclpy.init(args=args)
    node = AutoFocusNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()