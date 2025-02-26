#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import cv2
import os
import apriltag


class MarkerPoseEstimation(Node):
    def __init__(self):
        super().__init__("marker_pose_estimation")
        # qos_profile = QoSProfile()
        callback_group = ReentrantCallbackGroup()
        self.image_sub_ = self.create_subscription(Image, "/camera/color/image", callback=self.extract_image_data, qos_profile=10 ,callback_group=callback_group)
        self.image_processing_rate = 0.1
        self.timer_ = self.create_timer(self.image_processing_rate, self.process_image)
        # CV-ROS2 Bridge
        self.cv_bridge = CvBridge()
        # Image Parameters
        self.color_image = None

        self.detector_options = apriltag.DetectorOptions(
            families="tag36h11"
        )
        self.detector = apriltag.Detector(self.detector_options)

    def extract_image_data(self, data):
        try:
            self.color_image = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
    def detect_tags(self, image):
        if image is None:
            self.get_logger().warn(f"Image Data Stream Empty!")
            return
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        
        tag_ids = [tag.tag_id for tag in results]
        corner_points = [tag.corners for tag in results]     
        # Now zip the lists to match IDs with their corners
        for tag_id, corners in zip(tag_ids, corner_points):
            pt1 = tuple(corners[0])
            pt2 = tuple(corners[1])
            pt3 = tuple(corners[2])
            pt4 = tuple(corners[3])
            pts = np.array([pt1, pt2, pt3, pt4], dtype=np.int32)
            
            self.get_logger().info(f"Tag-{tag_id}:\n{pts}")



    def process_image(self):
        self.detect_tags(image=self.color_image)



def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseEstimation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"User Interrupted Program")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    
if __name__ == "__main__":
    main()