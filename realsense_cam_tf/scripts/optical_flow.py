#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class RealsenseOpticalFlow(Node):
    def __init__(self):
        super().__init__('realsense_optical_flow_node')
        custom_qos = QoSProfile(depth=10)
        custom_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.rgb_sub = self.create_subscription(
            Image,
            'camera/camera/color/image_raw',
            self.image_data,
            10
        )

        self.aligned_depth_sub = self.create_subscription(
            Image,
            'camera/camera/aligned_depth_to_color/image_raw',
            self.depth_data_callback,
            10
        )

        # Timer to periodically run the process loop
        self.freq = 20.0  # Hz
        self.bridge = CvBridge() 
        self.timer = self.create_timer(1.0 / self.freq, self.process_loop)

        # Variables to store data
        self.depth_value_ = 0
        self.i = 0

        self.cX = 0
        self.cY = 0

        self.image = None

    
    def image_data(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data)
        self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
        self.image = cv2.resize(self.image, (640,640))
        cv2.imwrite('new.jpg', self.image)

    def depth_data_callback(self, msg):
        if len(msg.data) == 0:
            self.get_logger().warn("Received empty depth image.")
            return

        # Get image dimensions
        width = msg.width
        height = msg.height

        # Calculate center pixel (cX, cY) --> a placeholder for now
        self.cX = width // 2
        self.cY = height // 2

        # Assuming depth image format is 16UC1 (16-bit unsigned integer, single channel)
        # Convert the flat array into a 2D numpy array
        depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)

        # Get the depth value at the center pixel
        self.depth_value_ = depth_image[self.cY, self.cX]

    def process_loop(self):
        # realsense as a pin-hold model is approximation is quite reasonable to do the focal length values as well as the cam size scale
        sizeCamX = 480
        sizeCamY = 848
        centerCamX = 240 
        centerCamY = 414
        focalX = 609.3551635742188
        focalY = 609.5684814453125

        if self.i < 1:
            self.get_logger().info("Processing loop executed")
            self.i += 1

        
        depth_mm = self.depth_value_/1000;
        x = depth_mm* (sizeCamX - self.cX - centerCamX)/focalX
        y = depth_mm* (sizeCamY - self.cY - centerCamY)/focalY
        z = depth_mm

        self.get_logger().info(f"x: {x}, y: {y}: z: {z}")

        if self.image is not None:
            cv2.imshow('frame', self.image)

def main(args=None):
    try:
        rclpy.init(args=args)
        
        node = RealsenseOpticalFlow()
        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
