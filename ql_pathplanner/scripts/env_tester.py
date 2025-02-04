#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState as GazeboModelState

import numpy as np
import math


class EnvTester(Node):
    def __init__(self):
        super().__init__("env_tester_node")
        self.callback_group = ReentrantCallbackGroup()
        self.odom_sub_ = self.create_subscription(
            Odometry, "odometry/unfiltered", callback=self.get_bot_pose, qos_profile=10, callback_group=self.callback_group
        )
        self.laser_sub_ = self.create_subscription(
            LaserScan, "scan", callback=self.get_scan_data, qos_profile=10, callback_group=self.callback_group
        )

        self.position = np.zeros(2)
        self.orientation = 0.0
        self.linear_vel = 0.0 # for diff bot (only x-dir vel (bot frame))
        self.angular_vel = 0.0 # bot yaw vel
        self.laser_data = np.zeros(180)

        self.episode_step = 0
        self.EPISODES = 5

        # Gazebo reset services
        self.reset_simulation_client = self.create_client(
            Empty,
            '/reset_simulation',
            callback_group=self.callback_group
        )
        self.pause_physics_client = self.create_client(
            Empty,
            '/pause_physics',
            callback_group=self.callback_group
        )
        self.unpause_physics_client = self.create_client(
            Empty,
            '/unpause_physics',
            callback_group=self.callback_group
        )
        # Wait for services to become available
        self._wait_for_services()


        self.timer_ = self.create_timer(0.1, self.tester_loop)

    def quaternion_to_euler(self, w, x, y, z):
        # Roll (x-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = max(min(t2, 1.0), -1.0)  # Clamp to avoid invalid input for asin
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw
    
    def _wait_for_services(self):
        services = [
            (self.reset_simulation_client, '/reset_simulation'),
            (self.pause_physics_client, '/pause_physics'),
            (self.unpause_physics_client, '/unpause_physics')
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{service_name} service not available, waiting...')
    
    def _reset_gazebo(self):
        """Reset Gazebo simulation."""
        # Pause physics
        self.pause_physics_client.call_async(Empty.Request())
        # Reset simulation
        future = self.reset_simulation_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        # Unpause physics
        self.unpause_physics_client.call_async(Empty.Request())
        # Wait for a short duration to let Gazebo stabilize
        self.create_timer(1.0, lambda: None).cancel()
    
    def get_bot_pose(self, msg):
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        _, _, yaw = self.quaternion_to_euler(
            msg.pose.pose.orientation.w, 
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z
        )
        self.orientation = yaw

        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

        # self.get_logger().info(f"postion: [{self.position}]")
        # self.get_logger().info(f"yaw: [{self.orientation}]")
        # self.get_logger().info(f"linear vel: [{self.linear_vel}]")
        # self.get_logger().info(f"yaw vel: [{self.angular_vel}]")

    def get_scan_data(self, msg):
        self.laser_data = np.array(msg.ranges)
    
    def tester_loop(self):
        self.episode_step += 1
        self.get_logger().info(f"postion: [{self.position[0]:.2f}, {self.position[1]:.2f}]")
        self.get_logger().info(f"yaw: [{self.orientation:.2f}]")
        self.get_logger().info(f"linear vel: [{self.linear_vel:.2f}]")
        self.get_logger().info(f"yaw vel: [{self.angular_vel:.2f}]")
        self.get_logger().info(f"step: {self.episode_step}")

        if self.episode_step >= 100:
            self._reset_gazebo()
            self.episode_step = 0
            # self.timer_.cancel()




def main(args=None):
    rclpy.init(args=args)
    node = EnvTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
