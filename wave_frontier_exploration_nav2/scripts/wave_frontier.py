#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid, Odometry
from action_msgs.msg import GoalStatus
from nav2_msgs.msg import Costmap, CostmapMetaData
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

import numpy as np
import matplotlib.pyplot as plt
import threading

def map_to_world_frames(mx, my, resolution, map_offset, map_originX, map_originY):
    wx = map_originX + (mx+map_offset)*resolution
    wy = map_originY + (my+map_offset)*resolution
    return wx, wy

def world_to_map(wx, wy, map_originX, map_originY, resolution, map_height, map_width):
    if (wx < map_originX or wy < map_originY):
        raise Exception("World coordinates out of bounds")

    mx = int((wx - map_originX) / resolution)
    my = int((wy - map_originY) / resolution)
    
    if  (my > map_height or mx > map_width):
        raise Exception("Out of bounds")

    return (mx, my)


class WaveFrontier(Node):

    def __init__(self):
        super().__init__("wave_frontier")

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.map_sub_ = self.create_subscription(OccupancyGrid, "map", self.map_data, pose_qos)
        #self.map_updates_sub_  = self.create_subscription(OccupancyGrid, "map_updates", self.partial_map_data, pose_qos)
        self.costmap_ = Costmap()

        self.odom_sub_ = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        
        self.fig, self.ax = plt.subplots()
        self.image = None
    
    def map_data(self, map_msg):
        map_data = np.array(map_msg.data)
        map_load_time = map_msg.info.map_load_time

        cell_resolution = map_msg.info.resolution
        print(f"map res: {cell_resolution}")
        
        map_originX = map_msg.info.origin.position.x
        map_originY = map_msg.info.origin.position.y
        print(f"origin-x: {map_originX}, origin-y: {map_originY}")
        
        map_width = map_msg.info.width
        map_height = map_msg.info.height
        print(f"{map_width} x {map_height}")

        data = map_data.data
        data = np.reshape(data, (map_height, map_width))
        print(data)


        
    
    def odom_callback(self, msg):
        glb_pos = msg.pose.pose.position
        glb_rot = [
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(glb_rot)




def main():
    try:
        rclpy.init()
        node = WaveFrontier()

        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
    