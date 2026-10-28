#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_upright',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/lane_detector/debug_image', qos_profile_sensor_data)
        self.get_logger().info('LaneDetector node initialized, listening on /camera/image_upright')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Resize for performance (optional)
        frame = cv2.resize(cv_image, (640, 360))

        # Grayscale & blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edge detection
        edges = cv2.Canny(blur, 100, 200)

        # ROI masking (가로 35~75%, 세로 하단 20%)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (int(width * 0.35), height),            # 좌하 (35%, 100%)
            (int(width * 0.75), height),            # 우하 (75%, 100%)
            (int(width * 0.75), int(height * 0.8)),  # 우상 (75%, 80%)
            (int(width * 0.35), int(height * 0.8))   # 좌상 (35%, 80%)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Hough Line Transform
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=30,
            maxLineGap=20
        )

        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # ROI 영역 시각화 (파란색 테두리)
        cv2.polylines(frame, [polygon], True, (255, 0, 0), 2)

        # Overlay the detected lines
        result = cv2.addWeighted(frame, 0.8, line_image, 1, 0)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(result, 'bgr8')
            debug_msg.header = msg.header
            self.publisher.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Could not publish debug image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()