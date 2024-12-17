import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Vector3
from nav_msgs.msg import Odometry

import os
import random
import sys

class ObstacleSpawner(Node):
    def __init__(self, num_boxes, client):
        super().__init__("obstacle_spawner_node")
        user_dir = os.path.expanduser("~")
        box_model_fp = os.path.join(user_dir, "labeeb/rl_ws/src/", "obs_spawner/models/", "Cardboard Box/model.sdf")

        with open(box_model_fp, 'r') as sdf_file:
            self.box_model_xml = sdf_file.read()

        self.spawner_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.destroyer_client = self.create_client(DeleteEntity, "/delete_entity")
        self.bot_pose_sub_ = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        
        if client == "spawn":
            if not self.spawner_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Spawn entity service not available")
                sys.exit(1)
        elif client == "delete":
            if not self.destroyer_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Delete entity service not available")
                sys.exit(1)

        self.req = SpawnEntity.Request()
        self.del_req = DeleteEntity.Request()

        self.pose = Pose()
        self.pose.position.x = 1.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.0

        self.box_num = num_boxes
        self.boxes = []
        self.spawned_positions = []

        self.bot_pose = Pose()


    def odom_callback(self, msg):
        self.bot_pose = msg.pose.pose

    def populate_boxes(self):
        boxes = [f"box_{i+1}" for i in range(self.box_num)]
        return boxes

    def is_position_near(self, pose1, pose2, threshold=1.0):
        # Check if two positions are too close to each other (within a tolerance limit)
        distance = ((pose1.position.x - pose2.position.x) ** 2 +
                    (pose1.position.y - pose2.position.y) ** 2 +
                    (pose1.position.z - pose2.position.z) ** 2) ** 0.5
        return distance < threshold
    
    def randomize_positions(self, low, high):
        # to get unique random positions
        while True:
            x = random.uniform(low, high)
            y = random.uniform(low, high)
            z = 0.0

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z

            if all(not self.is_position_near(pose, pos) for pos in self.spawned_positions) and (not self.is_position_near(pose, self.bot_pose)):
                self.spawned_positions.append(pose)
                return pose

    def spawner_request(self):
        self.boxes = self.populate_boxes()
        results = []
        for i in range(self.box_num):
            self.req.name = self.boxes[i]
            self.req.xml = self.box_model_xml
            self.req.initial_pose = self.randomize_positions(low=-2.0, high=2.0)
            self.req.reference_frame = "world"
            self.future = self.spawner_client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            results.append(self.future.result())
        return results

    def delete_request(self):
        self.boxes = self.populate_boxes()
        results = []
        for i in range(self.box_num):
            self.del_req.name = self.boxes[i]
            self.del_future = self.destroyer_client.call_async(self.del_req)
            rclpy.spin_until_future_complete(self,self.del_future)
            results.append(self.del_future.result())
        return results