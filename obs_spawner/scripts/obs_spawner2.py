#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from ros_ign_interfaces.srv import SpawnEntity, DeleteEntity
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

        self.spawner_client = self.create_client(SpawnEntity, "/world/empty/create")
        self.destroyer_client = self.create_client(DeleteEntity, "/world/empty/remove")
        self.bot_pose_sub_ = self.create_subscription(Odometry, "odometry/unfiltered", self.odom_callback, 10)
        
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

    def is_position_near(self, pose1, pose2, threshold=0.5):
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
            self.req.entity_factory.name = self.boxes[i]
            self.req.entity_factory.sdf = self.box_model_xml
            self.req.entity_factory.pose = self.randomize_positions(low=-2.0, high=2.0)
            self.req.entity_factory.relative_to = "world"
            self.future = self.spawner_client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            results.append(self.future.result())
        return results

    def delete_request(self):
        self.boxes = self.populate_boxes()
        results = []
        for i in range(self.box_num):
            self.del_req.entity.name = self.boxes[i]
            self.del_future = self.destroyer_client.call_async(self.del_req)
            rclpy.spin_until_future_complete(self,self.del_future)
            results.append(self.del_future.result())
        return results

    
def main(args=None):
    rclpy.init(args=args)
    client_type = str(sys.argv[1])
    node = ObstacleSpawner(num_boxes=6, client=client_type)

    try:
        if client_type == "spawn":
            ret_val = node.spawner_request()
            print(f"Spawned {len(ret_val)} boxes successfully!")
        elif client_type == "delete":
            del_ret_val = node.delete_request()
            print(f"Deleted {len(del_ret_val)} boxes successfully!")
    except KeyboardInterrupt:
        print(f"Programe interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()