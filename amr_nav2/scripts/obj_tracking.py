#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

import numpy as np
import time
import cv2
from cv_bridge import CvBridge
import mediapipe as mp




class ObjTracking(Node):

    def __init__(self):
        super().__init__('obj_tracking')
        self.image_raw_sub = self.create_subscription(Image, "camera_1/image_raw", self.raw_rgbimage, 10)
        self.depth_image_raw_sub = self.create_subscription(Image, "depth_camera/depth/image_raw", self.depth_imagecb, 10)
        self.timer_period = 20 # time in ms
        self.timer = self.create_timer(self.timer_period/1000, self.image_process)

        self.bridge = CvBridge()
        self.br = tf2_ros.TransformBroadcaster(self)
        
        self.image_info = None
        self.image = None
        self.compImage = None

        self.depth = None

        self.mp_objectron = mp.solutions.objectron
        self.objectron = self.mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=2,
            min_detection_confidence=0.1,
            min_tracking_confidence=0.8,
            model_name='Cup'            
        )

        self.previous_landmarks = []



    def raw_rgbimage(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('frame.jpg', self.image)
    
    def depth_imagecb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)
        cv2.imwrite('depth_frame.jpg', self.depth)

    def image_process(self):
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 360 
        centerCamY = 640
        focalX = 231.15334173635904
        focalY = 231.15334173635904


        smoothing_factor = 0.85 # smoothing factor

        if self.image is not None:    
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            results = self.objectron.process(self.image)
            self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)

            if results.detected_objects:
                for detected_object in results.detected_objects:
                    print(f"{detected_object}")
                    bbox = detected_object.landmarks_2d

                    current_landmarks = []
                    c1 = ((int(bbox.landmark[0].x*(self.image.shape[1]))), (int(bbox.landmark[0].y*(self.image.shape[0]))))
                    
                    current_landmarks.append(c1)
                    
                    if self.previous_landmarks:
                        smoothed_landmarks = [(
                            int(smoothing_factor*self.previous_landmarks[0][0] + (1-smoothing_factor)*current_landmarks[0][0]),
                            int(smoothing_factor*self.previous_landmarks[0][1] + (1-smoothing_factor)*current_landmarks[0][1]))
                        ]
                    else:
                        smoothed_landmarks = current_landmarks


                    for landmark in smoothed_landmarks:
                        print(f"(x,y): {landmark}")
                        depth_mm = self.depth[int(landmark[1]), int(landmark[0])]
                        print(f"landmark depth(mm): {depth_mm}")
                        cv2.circle(self.image, landmark, 5, (255, 0, 0), -1)

                    self.previous_landmarks = smoothed_landmarks
                    


            # Display the image with bounding box (optional)
            cv2.imshow('Object Detection', self.image)
            key = cv2.waitKey(1)

            if key & 0xFF == ord('q'):
                self.timer.cancel()

            if self.timer.is_canceled():
                cv2.destroyAllWindows()
                rclpy.shutdown()




def main():
    try:
        rclpy.init()

        node = ObjTracking()

        rclpy.spin(node)
        node.destroy_node()

        rclpy.shutdown()
    except KeyboardInterrupt:
        pass



if __name__ == '__main__':
    main()