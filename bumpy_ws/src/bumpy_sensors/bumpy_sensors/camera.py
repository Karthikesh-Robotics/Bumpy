#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration  # Import Duration class

class LowLatencyCameraNode(Node):
    def __init__(self):
        super().__init__('low_latency_camera_node')
        
        # Correct QoS profile with Duration object
        self.qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=0)  # Proper Duration object
        )
        
        # ROS2 Publisher
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            qos_profile=self.qos_profile
        )
        
        # Camera Setup
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            raise RuntimeError("Camera initialization failed")
        
        # Low-latency configuration for SSH
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Pre-allocate frame buffer
        self.frame = np.empty((240, 320, 3), dtype=np.uint8)
        
        # Publishing timer (15Hz)
        self.timer = self.create_timer(0.066, self.publish_frame)

    def publish_frame(self):
        ret = self.cap.read(self.frame)
        if ret:
            try:
                ros_image = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_optical_frame"
                self.publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Publish error: {str(e)}", throttle_duration_sec=1)

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info("Camera resources released")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LowLatencyCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()