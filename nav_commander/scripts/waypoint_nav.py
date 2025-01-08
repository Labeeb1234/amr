#! /usr/bin/python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import time
import math

x, y, yaw = 0.0, 0.0, 0.0

def quaternion_to_euler(w, x, y, z):
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

def euler_to_quaternion(roll, pitch, yaw):
    # Compute half angles
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Compute quaternion
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z

def odometry_callback(msg):
    global x, y, yaw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    _, _, yaw = quaternion_to_euler(
        msg.pose.pose.orientation.w, 
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z
    )


def main(args=None):
    rclpy.init(args=args)
    nav_node = BasicNavigator()
    callback_group = ReentrantCallbackGroup()
    odom_sub_ = nav_node.create_subscription(Odometry, "odometry/filtered", callback=odometry_callback, qos_profile=10, callback_group=callback_group)

    starting_pose = PoseStamped()
    starting_pose.header.frame_id = "map"
    starting_pose.header.stamp = nav_node.get_clock().now().to_msg()
    starting_pose.pose.position.x = 0.0
    starting_pose.pose.position.y = 0.0
    q = starting_pose.pose.orientation 
    q.w, q.x, q.y, q.z = euler_to_quaternion(0.0, 0.0, 0.0)
    nav_node.setInitialPose(starting_pose)
    nav_node.waitUntilNav2Active()

    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = "map"
    goal_pose1.header.stamp = nav_node.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.0
    goal_pose1.pose.position.y = 1.0
    q = goal_pose1.pose.orientation 
    q.w, q.x, q.y, q.z = euler_to_quaternion(0.0, 0.0, 0.0)
    goal_poses.append(goal_pose1)
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = "map"
    goal_pose2.header.stamp = nav_node.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.0
    goal_pose2.pose.position.y = 0.0
    q = goal_pose2.pose.orientation 
    q.w, q.x, q.y, q.z = euler_to_quaternion(0.0, 0.0, 0.0)

    nav_start = nav_node.get_clock().now()
    nav_node.followWaypoints(goal_poses)
    count=0
    while not nav_node.isTaskComplete():
        count+=1
        feedback = nav_node.getFeedback()
        if feedback and count % 10 == 0:
            nav_node.get_logger().info(f"current_pose:[{x}:.2f, {y}:.2f, {yaw}:2f]")
            nav_node.get_logger().info(f"Estimated Time of arrival: {rclpy.duration.Duration.from_msg(feedback.estimated_time_remaining).nanoseconds*1e-9}:.2f seconds.")
            now = nav_node.get_clock().now()
            # timeout
            if now-nav_start > rclpy.duration.Duration(seconds=600.0):
                nav_node.cancelTask()
    
    result = nav_node.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    nav_node.lifecycleShutdown()

    exit(0)
            
if __name__ == '__main__':
    main()