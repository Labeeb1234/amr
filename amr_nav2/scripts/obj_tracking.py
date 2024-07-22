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
        self.image_raw_sub = self.create_subscription(Image, "camera/color/image_raw", self.raw_rgbimage, 10)
        self.depth_image_raw_sub = self.create_subscription(Image, "camera/aligned_depth_to_color/image_raw", self.depth_imagecb, 10)
        self.timer_period = 20 # time in ms
        self.timer = self.create_timer(self.timer_period/1000, self.image_process)

        self.bridge = CvBridge()
        self.buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        
        self.image_info = None
        self.image = None
        self.compImage = None

        self.depth = None

        self.mp_objectron = mp.solutions.objectron
        self.objectron = self.mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=2,
            min_detection_confidence=0.5,
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
        centerCamX = 640
        centerCamY = 360
        focalX = 931.1829833984375

        focalY = 931.1829833984375


        # smoothing_factor = 0.7 # smoothing factor
        try:
            if self.image is not None:    
                self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
                results = self.objectron.process(self.image)
                self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)

                if results.detected_objects:
                    for detected_object in results.detected_objects:
                        print(f"{detected_object}")
                        bbox = detected_object.landmarks_2d

                        for i in range(0, 1):
                        # current_landmarks = []
                            c1 = ((int(bbox.landmark[i].x*(self.image.shape[1]))), (int(bbox.landmark[i].y*(self.image.shape[0]))))
                            
                            # current_landmarks.append(c1)
                            
                            # if self.previous_landmarks:
                            #     smoothed_landmarks = [(
                            #         int(smoothing_factor*self.previous_landmarks[0][0] + (1-smoothing_factor)*current_landmarks[0][0]),
                            #         int(smoothing_factor*self.previous_landmarks[0][1] + (1-smoothing_factor)*current_landmarks[0][1]))
                            #     ]
                            # else:
                            #     smoothed_landmarks = current_landmarks


                            # for landmark in smoothed_landmarks:
                            print(f"(x,y): {c1}")
                            depth_mm = self.depth[int(c1[1]), int(c1[0])]
                            print(f"landmark depth(mm): {depth_mm}")
                            cv2.circle(self.image, c1, 5, (255, 0, 0), -1)

                            # converting each landmark(object tracking point) into x,y,z coordinates wrt camera frame  --> for now its just one landmark
                            realsense_depth = depth_mm/1000
                            z = realsense_depth # ---> realsense depth or distance from rgb cam
                            x = realsense_depth*(sizeCamX-c1[0]-centerCamX)/focalX
                            y = realsense_depth*(sizeCamY-c1[1]-centerCamY)/focalY

                            print(f"x_point: {x}, y_point: {y}, z_point: {z}")

                            # broadcasting tf (cam to tracking point)
                            tfs = TransformStamped()
                            tfs.header.frame_id = 'Camera_1'
                            tfs.header.stamp = self.get_clock().now().to_msg()
                            tfs.child_frame_id = 'cup_tracker' + str(i)
                            tfs.transform.translation.x = x
                            tfs.transform.translation.y = y
                            tfs.transform.translation.z = z
                            self.br.sendTransform(tfs)

                            # getting pose wrt to base_link of bot
                            from_frame_rel = 'cup_tracker' + str(i)                                                                   
                            to_frame_rel = 'odom'  
                            try:
                                transformed_tr = self.buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())  
                                # self.get_logger().info(f'Successfully received data!')
                            except tf2_ros.TransformException as e:
                                self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
                                return

                            x_tr  = transformed_tr.transform.translation.x
                            y_tr = transformed_tr.transform.translation.y
                            z_tr = transformed_tr.transform.translation.z

                            tf_br = TransformStamped()
                            tf_br.header.frame_id = to_frame_rel
                            tf_br.header.stamp = self.get_clock().now().to_msg()
                            tf_br.child_frame_id = 'cup_tracker_transformed' + str(i)
                            tf_br.transform.translation.x = x_tr
                            tf_br.transform.translation.y = y_tr
                            tf_br.transform.translation.z = z_tr
                            self.br.sendTransform(tf_br)

                            # self.previous_landmarks = smoothed_landmarks


                # Display the image with bounding box (optional)
                cv2.imshow('Object Detection', self.image)
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    self.timer.cancel()

                if self.timer.is_canceled():
                    cv2.destroyAllWindows()
                    rclpy.shutdown()

        except IndexError:
            pass






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