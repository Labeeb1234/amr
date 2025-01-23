#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Float32

import argparse
import logging
import os
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from random import sample


class ReplayBuffer:
    def __init__(self, capacity):
        self.experience_buffer = deque(maxlen=capacity)

    def add_memory(self, transition):
        self.experience_buffer.append(transition)

    def experience_replay(self, batch_size):
        if len(self.experience_buffer) < batch_size:
            return sample(self.experience_buffer, len(self.experience_buffer))
        return sample(self.experience_buffer, batch_size)

    def __len__(self):
        return len(self.experience_buffer)
        
class QLearningNode(Node):
    def __init__(self):
        super().__init__("qlearning_node")
        self.callback_group = ReentrantCallbackGroup()
        self.odom_sub_ = self.create_subscription(Odometry, "odometry/unfiltered", callback=self.get_bot_pose, qos_profile=10, callback_group=self.callback_group)
        self.laser_sub_ = self.create_subscription(LaserScan, "scan", callback=self.get_scan_data, qos_profile=10, callback_group=self.callback_group)
        self.twist_pub_ = self.create_publisher(Twist, "cmd_vel", 10)
    
        # inference params
        self.inference_mode = False
        self.timer_period = 0.1 # 10Hz controller frequency

        if not self.inference_mode:
            self.timer_ = self.create_timer(self.timer_period, self.training_loop)
        else:
            self.inference_timer_ = self.create_timer(self.timer_period, self.inference_loop)

        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation') 

        # STATE and ACTION PARAMS
        self.initial_pose_received = False
        self.state = None # (dist_to_goal, orient_to_goal)
        self.action = 0
        self.DISTANCE_BINS = 64
        self.ORIENTATION_BINS = 64
        self.LASER_BINS = 32 
        self.ACTION_SPACE = 4

        # Random Exploration PARAMS
        self.epsilon = 0.5
        self.min_epsilon = 0.01
        self.epsilon_decay_rate = 0.99

        ## Experience Replay PARAMS
        self.replay_buffer = ReplayBuffer(capacity=10000)
        self.BATCH_SIZE = 64

        # Q-learning PARAMS
        self.EPISODES = 600
        self.episode_counter = 0
        self.steps_per_episode = 400
        self.steps_counter = 0
        self.LEARNING_RATE = 0.01
        self.DISCOUNT_FACTOR = 0.99
        self.episode_reward = 0.0
        self.avg_episode_reward = []
        self.episode_rewards = []

        # initializing q-table
        # self.start_q_table = np.random.uniform(low=-2, high=0, size=(self.DISTANCE_BINS, self.ORIENTATION_BINS, self.ACTION_SPACE))
        self.start_q_table = self.load_qtable(f"q_table_43") # best for straight line
        self.q_table = self.start_q_table
        self.verify_qtable()

        # goal params
        self.goal_position = [4.0, 0.0, 0.0]
        self.goal_tolerance = 0.3 # in [m]
        self.orientation_tolerance = 0.2 # in [rad]
        self.goal_reached = False

        # bot info/sensor feedback params
        self.current_position = None
        self.current_yaw = 0.0
        self.distance_to_goal = 0.0
        self.previous_distance = 0.0
        self.angle_diff = 0.0
        self.laser_data = None

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
    
    def dist_to_goal(self, goal_pose, current_pose):
        # using eucledian distance formulae
        dist_to_goal = np.sqrt((goal_pose[0]-current_pose.x)**2 + (goal_pose[1]-current_pose.y)**2)
        return dist_to_goal

    def angle_goal_diff(self, goal_pose, current_yaw, degrees=False):
        # angle in rads
        if not degrees:
            angle_diff = goal_pose[2]-current_yaw
        else:
            angle_diff = (goal_pose[2]-current_yaw)*(180.0/3.14)
        return angle_diff

    def get_discretize_state(self, distance_to_goal, angle_diff, dist_bins_space, orient_bins_space):
        dist_bin_edges = np.linspace(0, 10, dist_bins_space+1)
        dist_bins = np.digitize(distance_to_goal, dist_bin_edges)

        # Normalize angle to [-π, π] range and properly bin it
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        orient_bin_edges = np.linspace(-np.pi, np.pi, orient_bins_space+1)
        orient_bins = np.digitize(angle_diff, orient_bin_edges)

        dist_bins = min(dist_bins, dist_bins_space - 1)
        orient_bins = min(orient_bins, orient_bins_space - 1)

        return (dist_bins, orient_bins)

    def discretize_laser_data(laser_data, num_bins):
        laser_min = min(laser_data.ranges)  # Minimum range value
        laser_max = max(laser_data.ranges)  # Maximum range value
        laser_bin_edges = np.linspace(laser_min, laser_max, num_bins + 1)  # Create bin edges

        # Discretize laser data to corresponding bin indices
        laser_bins = [np.digitize(range_, laser_bin_edges) for range_ in laser_data.ranges]
        return laser_bins

    def choose_action(self, discrete_state):
        if np.random.random() > self.epsilon:
            self.get_logger().info(f"using the best action...")
            return np.argmax(self.q_table[discrete_state])
        else:
            self.get_logger().info(f"Exploring....")
            return np.random.randint(0, self.ACTION_SPACE)
    
    def compute_reward(self, action):
        # resetting the reward components (for each movement)
        distance_reward = 0.0
        alignment_reward = 0.0
        progress_reward = 0.0
        goal_achievment_reward = 0.0
        action_specific_reward = 0.0
        #####################################################
        # Distance-based reward with smoother scaling
        # using inverse square law for distance_factor for smoothing the increment as well as the learning process
        distance_factor = 1.0 / (1.0 + self.distance_to_goal**2)  # Rewards more as robot gets closer
        distance_reward = 15.0 * distance_factor
        
        # Alignment-Based reward with smoother gradient
        alignment_factor = np.exp(-2.0 * np.abs(self.angle_diff))  # Smoother decay
        alignment_reward = 8.0 * alignment_factor

        # Progress reward towards goal
        progress = self.previous_distance - self.distance_to_goal
        if progress > 0.0:
            # Reward increases for consistent forward progress
            progress_reward = 50.0 * progress
            # aligned progress bonus reward
            if abs(self.angle_diff) < 0.3:
                progress_reward += 20.0*progress
        else:
            # Small negative reward for moving away from goal
            progress_reward = -25.0 * abs(progress)

        # Goal achievement reward with distance-based bonus
        if self.distance_to_goal < self.goal_tolerance and abs(self.angle_diff) < self.orientation_tolerance:
            if abs(self.angle_diff) < 0.2:
                goal_achievment_reward = 100.0 + (50.0 * alignment_factor)  # Extra reward for aligned arrival
            else:
                goal_achievment_reward = 100.0
            # what to do after reaching the goal
            self.goal_reached = True
            self.reset_simulation()
            self.get_logger().info(f"goal reached!")
            if self.steps_counter > 0:
                self.episode_reward = self.episode_reward / self.steps_counter
            else:
                self.episode_reward = 0.0  
            self.avg_episode_reward.append(self.episode_reward)
            self.get_logger().info(f"average episode reward: {self.avg_episode_reward}")
            self.episode_reward = 0.0
            self.steps_counter = 0
            self.episode_counter += 1

        # Action-specific rewards with better behavior shaping
        if action == 0:  # Moving forward
            if abs(self.angle_diff) > 0.2:  
                action_specific_reward = -15.0 * abs(self.angle_diff)
            else:
                action_specific_reward = 10.0 * (1.0 - abs(self.angle_diff)) 
        elif action in [1, 2]:  # Turning actions
            if abs(self.angle_diff) > 0.2:
                # Reward turning when misaligned
                action_specific_reward = 15.0 * (1.0 - abs(self.angle_diff)/np.pi)
            else:
                # Penalize unnecessary turning when aligned
                action_specific_reward = -20.0
        elif action == 3:  # Stopping
                # Only reward stopping when very close to goal AND well-aligned
                if self.distance_to_goal < self.goal_tolerance and abs(self.angle_diff) < 0.2:
                    action_specific_reward = 20.0
                else:
                    # Much stronger penalty for unnecessary stopping
                    action_specific_reward = -(50.0 + (50.0 * (1.0 - distance_factor)))  # Penalty increases with distance from goal

        total_reward = (
            distance_reward+
            alignment_reward+
            progress_reward+
            goal_achievment_reward+
            action_specific_reward
        )

        self.previous_distance = self.distance_to_goal

        return total_reward

    def update_qtable(self, state, action, reward, new_state):
        next_best_action = np.argmax(self.q_table[new_state])
        td_target = reward + self.DISCOUNT_FACTOR*(self.q_table[new_state+(next_best_action,)])
        new_qvalue = (1-self.LEARNING_RATE)*self.q_table[state+(action,)] + self.LEARNING_RATE*td_target
        self.q_table[state+(action,)] = new_qvalue

    ###################### (custom training goals) ###########################
    def randomize_goal(self):
        # Randomly set x-coordinate between 0 and 5
        self.goal_position[0] = float(np.random.randint(2, 4))  # Random int x value in range [1, 3]
        self.goal_position[1] = 0.0  # Fixed y position
        self.goal_position[2] = 0.0  # Fixed yaw angle pose of the bot
        self.get_logger().info(f"[New goal: {self.goal_position}]")
    ###########################################################################

    def move_bot(self, action):
        twist_msg = Twist()
        if action == 0:
            twist_msg.linear.x = 0.5
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 0.0
            self.twist_pub_.publish(twist_msg)
        elif action == 1: # Turning Left
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 1.0
            self.twist_pub_.publish(twist_msg)
        elif action == 2: # Turning Right
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = -1.0
            self.twist_pub_.publish(twist_msg)
        elif action == 3:
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 0.0
            self.twist_pub_.publish(twist_msg)
    
    def get_scan_data(self, scan_data):
        self.laser_data = scan_data

    def get_bot_pose(self, odom_msg):
        self.current_position = odom_msg.pose.pose.position
        _, _, self.current_yaw = self.quaternion_to_euler(
            odom_msg.pose.pose.orientation.w, 
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z
        )

        if not self.initial_pose_received:
            self.get_logger().info(f"current_position: [{self.current_position.x:.2f},{self.current_position.y:.2f}], current_yaw: {self.current_yaw:.2f}")
            self.distance_to_goal = self.dist_to_goal(self.goal_position, self.current_position)
            self.angle_diff = self.angle_goal_diff(self.goal_position, self.current_yaw)
            self.state = self.get_discretize_state(self.distance_to_goal, self.angle_diff, self.DISTANCE_BINS, self.ORIENTATION_BINS)
            self.previous_distance = self.distance_to_goal
            self.initial_pose_received = True
            self.get_logger().info(f"Initial state set to: {self.state}")
    
    def reset_simulation(self):
        if not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Gazebo reset service not available.")
            return
        request = Empty.Request()
        self.reset_simulation_client.call_async(request)
        # Reset attributes for a new episode
        self.get_logger().info(f"Resetting environment for episode {self.episode_counter}")

    def training_loop(self):
        if not self.initial_pose_received:
            self.get_logger().info(f"Waiting for initial position of the robot....")
            return
        
        if self.episode_counter <= self.EPISODES:
            if self.episode_counter == 0 or self.steps_counter >= self.steps_per_episode:
                # Calculate average reward before reset
                if self.steps_counter > 0:  # Prevent division by zero
                    self.episode_rewards.append(self.episode_reward)
                    self.episode_reward = self.episode_reward/self.steps_counter
                    self.avg_episode_reward.append(self.episode_reward)
                    self.episode_reward = 0.0        
                self.reset_simulation()
                time.sleep(0.5)  # Add small delay to ensure reset completes
                self.steps_counter = 0
                self.episode_counter += 1
                self.epsilon = max(self.min_epsilon, self.epsilon*self.epsilon_decay_rate)
                self.get_logger().info(f"average episode reward: {self.avg_episode_reward}")
                if self.episode_counter % 10 == 0:
                    self.get_logger().info(f"{10} episodes rewards: {self.avg_episode_reward[-10:]}")
                    self.plot_metrics()
                    time.sleep(1.0)
                   
            if self.current_position:
                # main training logic here (fixed logic take action)
                self.distance_to_goal = self.dist_to_goal(self.goal_position, self.current_position)
                self.angle_diff = self.angle_goal_diff(self.goal_position, self.current_yaw)
                self.state = self.get_discretize_state(self.distance_to_goal, self.angle_diff, self.DISTANCE_BINS, self.ORIENTATION_BINS)

                # action step
                self.action = self.choose_action(self.state)
                self.move_bot(self.action)
                reward = self.compute_reward(action=self.action)
                # new state after taking action
                self.distance_to_goal = self.dist_to_goal(self.goal_position, self.current_position)
                self.angle_diff = self.angle_goal_diff(self.goal_position, self.current_yaw)
                new_state = self.get_discretize_state(self.distance_to_goal, self.angle_diff, self.DISTANCE_BINS, self.ORIENTATION_BINS)

                # experience update and replay and qtable update
                experience = (self.state, self.action, reward, new_state)
                self.replay_buffer.add_memory(transition=experience)
                if len(self.replay_buffer) >= self.BATCH_SIZE:
                    batch = self.replay_buffer.experience_replay(self.BATCH_SIZE)
                    self.get_logger().info("Experience Replay")
                    for state, action ,reward, new_state in batch:
                        self.update_qtable(state=state, action=action, reward=reward, new_state=new_state)
                else:
                    self.update_qtable(state=self.state, action=self.action, reward=reward, new_state=new_state)
                # some logs
                self.get_logger().info(f"new state: [{new_state}]")
                self.get_logger().info(f"new distance to goal: [{self.distance_to_goal:.2f}]")
                self.get_logger().info(f"new angular difference to goal: [{self.angle_diff:.2f}]")
                self.get_logger().info(f"Optimized Action: {self.action}")
                self.get_logger().info(f"reward: {reward:.2f}")
                self.get_logger().info(f"updated q_value: [{self.q_table[self.state]}")
                # cumulation of rewards each step
                self.episode_reward += reward
                self.get_logger().info(f"cumulative [{self.episode_counter}] episode reward: {self.episode_reward:.2f}")
                self.steps_counter += 1

            self.save_qtable(self.episode_counter, self.q_table)
            self.get_logger().info(f"episode: {self.episode_counter}, epsilon: {self.epsilon}")
            self.get_logger().info(f"steps: {self.steps_counter}")

        else:
            self.get_logger().info(f"Training Finished after {self.episode_counter} episodes..!")
            self.timer_.cancel()
    
    def save_qtable(self, episode, q_table):
        os.makedirs(name="qtables", exist_ok=True)
        filename = f"qtables/q_table_{episode}.npy"
        np.save(filename, q_table)

    def load_qtable(self, saved_q_table):
        self.start_q_table = np.load(f'q_tables/{saved_q_table}.npy')
        self.get_logger().info(f"Initial Q-Table:\n{self.start_q_table}")
        return self.start_q_table
    
    def verify_qtable(self):
        """Add this method to check Q-table"""
        if self.q_table is None:
            self.get_logger().error("Q-table is None!")
            return False
        # Check for invalid values
        if np.isnan(self.q_table).any():
            self.get_logger().error("Q-table contains NaN values!")
            return False
        if np.isinf(self.q_table).any():
            self.get_logger().error("Q-table contains infinite values!")
            return False
        # Log some statistics
        self.get_logger().info(f"Q-table shape: {self.q_table.shape}")
        self.get_logger().info(f"Q-table min value: {np.min(self.q_table)}")
        self.get_logger().info(f"Q-table max value: {np.max(self.q_table)}")
        self.get_logger().info(f"Q-table mean value: {np.mean(self.q_table)}")
        
        return True
        
    def plot_metrics(self):
        fig, ax = plt.subplots()
        if self.avg_episode_reward is None:
            self.get_logger("Waiting for episodic reward datas...")
            return
        
        episodes = list(range(1, len(self.avg_episode_reward) + 1))
        ax.plot(episodes, self.avg_episode_reward, label="Average Episode Reward", color="b")
        ax.set_xlabel("Episode")
        ax.set_ylabel("Average Episode Reward")
        ax.grid()
        ax.legend()
        plt.show()
        

    def inference_loop(self):
        if not self.inference_mode or self.q_table is None:
            self.get_logger().error("Inference mode not properly initialized or Q-table not loaded")
            return

        if self.current_position is None:
            self.get_logger().warn("Waiting for position data...")
            return
        
        # Update current measurements
        self.distance_to_goal = self.dist_to_goal(self.goal_position, self.current_position)
        self.angle_diff = self.angle_goal_diff(self.goal_position, self.current_yaw)
        # Get the current state
        state = self.get_discretize_state(self.distance_to_goal, self.angle_diff, self.DISTANCE_BINS, self.ORIENTATION_BINS)
        
        # Add debug logging
        self.get_logger().info(f"Current position: [{self.current_position.x:.2f}, {self.current_position.y:.2f}]")
        self.get_logger().info(f"Goal position: [{self.goal_position[0]:.2f}, {self.goal_position[1]:.2f}]")
        self.get_logger().info(f"Distance to goal: {self.distance_to_goal:.2f}")
        self.get_logger().info(f"Angle difference: {self.angle_diff:.2f}")
        
        # Choose the best action based on the Q-table
        q_values = self.q_table[state]
        best_action = np.argmax(q_values)
        
        # Log Q-values for debugging
        self.get_logger().info(f"Q-values for state {state}: {q_values}")
        
        # Move the robot based on the best action
        self.move_bot(best_action)
        
        # Only compute reward for debugging
        reward = self.compute_reward(action=best_action)
        
        # Check if goal is reached
        if self.distance_to_goal < self.goal_tolerance:
            self.get_logger().info("Goal reached!")
            # Optionally stop the robot
            self.move_bot(3)  # Stop action
        
        self.get_logger().info(f"Inference: state={state}, action={best_action}, reward={reward}")



def main(args=None):
    rclpy.init(args=args)
    node = QLearningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        print(f"{e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    
    exit(0)


if __name__ == '__main__':
    main()
