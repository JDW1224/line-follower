#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.bridge = CvBridge()
        # Pipeline can be overridden via ROS parameter `pipeline` (string)
        default_nv_pipeline = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=(int)960, height=(int)480, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
        )
        # Example V4L2 pipeline (set via parameter):
        # "v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink"

        self.declare_parameter('pipeline', default_nv_pipeline)
        self.gst_str = self.get_parameter('pipeline').get_parameter_value().string_value

        self.get_logger().info('Opening CSI camera...')
        self.get_logger().info(self.gst_str)

        self.cap = None
        self.fail_streak = 0
        self.open_camera()

        # 30fps 타이머
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.frame_count = 0
    
    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            if self.open_camera():
                self.get_logger().info('Reopened camera after failure')
            else:
                self.get_logger().warn('Camera not available; retrying...')
            return

        ret_val, frame = self.cap.read()
        if ret_val:
            # ROS Image 메시지로 변환 및 발행
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.publisher.publish(msg)
            
            self.frame_count += 1
            # if self.frame_count % 90 == 0:
            #     self.get_logger().info(f'Published {self.frame_count} frames')
            self.fail_streak = 0
        else:
            self.fail_streak += 1
            if self.fail_streak % 30 == 0:
                self.get_logger().warn('Failed to read frame ({} consecutive)'.format(self.fail_streak))
            if self.fail_streak >= 60:
                self.get_logger().warn('Too many failures, reopening camera')
                self.open_camera()
    
    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()

    def open_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.gst_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Error: Unable to open camera (will retry)')
            return False
        self.get_logger().info('Camera opened successfully - Publishing to /camera/image_raw')
        self.fail_streak = 0
        return True

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()