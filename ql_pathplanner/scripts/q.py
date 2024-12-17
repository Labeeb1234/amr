#! /usr/bin/python3

import ql_pathplanner.obs_randomizer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
import ql_pathplanner.obs_randomizer as obsp
import matplotlib.pyplot as plt

import time
import threading
import numpy as np
import os

# here the scan data is considered and encoded as the state of this Model,  scan gives the instanteous 2D data of the current setup of the environment 
class QLearningNode(Node):
    def __init__(self):
        super().__init__("qlearning_node")
        callback_grp = ReentrantCallbackGroup()
        self.lidar_sub_ = self.create_subscription(LaserScan, "scan", callback=self.laser_callback, qos_profile=10, callback_group=callback_grp)
        self.odom_sub_ = self.create_subscription(Odometry, "odometry/unfiltered", callback=self.odometry_callback, qos_profile=10, callback_group=callback_grp)
        self.twist_pub_  = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer_ = self.create_timer(0.1, self.training_loop, callback_group=callback_grp)

        # initial class globals
        self.state = 0 # laser scan data (discretized)
        self.action = 0
        self.current_pose = None
        self.current_yaw = 0.0
        self.discrete_laser_range = None

        # goal params
        self.goal_position = [2.0, 1.0, 0.0]
        self.goal_tolerance = 0.01
        self.goal_reached = False
        # Q-Learning Params
        self.DISCOUNT_FACTOR = 0.7
        self.EPISODES = 20000
        self.LEARNING_RATE = 0.01
        self.STATE_SPACE_SIZE = 9
        self.ACTION_SPACE_SIZE = 3
        # params for random exploration for action selection
        self.min_exploration_rate = 0.1
        self.exploration_decay = 0.999
        self.exploration_rate = 1.0 # epsilon
        # intializing Q-Table
        self.q_table = np.zeros((self.STATE_SPACE_SIZE, self.ACTION_SPACE_SIZE))

        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')
        self.episode_counter=0

        # reward tracking params
        self.epi_rewards = []
        self.epi_aggregate = {
            'epi': [],
            'avg': [],
            'min': [],
            'max': []
        }


    def discretize_lidar_data(self, laser_data):
        discrete_ranges = []
        for i in range(self.STATE_SPACE_SIZE):
            start_angle = i*20
            end_angle = (i+1)*20
            sector_range = [
                r for j,r in enumerate(laser_data.ranges)
                if start_angle <= np.degrees(laser_data.angle_min + j * laser_data.angle_increment) < end_angle 
            ]
            discrete_ranges.append(min(sector_range) if sector_range else laser_data.angle_max)
        return discrete_ranges
    
    def get_binary_state(self, discrete_ranges):
        binary_state = []
        for i, dist in enumerate(discrete_ranges):
            if dist < 1.0:  # Obstacle close
                binary_state.append('1')
            else:
                binary_state.append('0')
        return ''.join(binary_state)
    
    def get_state(self, discrete_ranges):
        binary_state = self.get_binary_state(discrete_ranges=discrete_ranges)
        return int(binary_state, 2) % self.STATE_SPACE_SIZE
    
    def calculate_reward(self, action):
        reward = 0
        if self.current_pose:
            self.get_logger().info(f"{self.current_pose.x}, {self.current_pose.y}, {self.current_yaw}")

            if self.discrete_laser_range:
                min_laser_range = min(self.discrete_laser_range)
                self.get_logger().info(f"min laser range: {min_laser_range}")
                if min_laser_range < 0.2:
                    # Higher penalty for closer obstacles
                    reward += -100 * (1 / min_laser_range)
                
                # Gradual reward for maintaining safe distance
                if min_laser_range > 0.2:
                    reward += 10 * min_laser_range
                
                if min_laser_range >= 0.25:
                    reward += -1.0
 
                reward += -1.0                
                # Action-specific rewards
                if action == 0:  # Move forward 
                    reward += 10
                elif action == 1:  # Turn left
                    reward += 1
                elif action == 2:  # Turn right
                    reward += 1
            
            self.get_logger().info(f"reward: {reward}")

        return reward

    def choose_action(self, state):
        """
        Epsilon-greedy action selection
        """
        if np.random.rand() < self.exploration_rate:
            return np.random.randint(self.ACTION_SPACE_SIZE)
        else:
            return np.argmax(self.q_table[state])

    
    def update_qtable(self, state, action, reward, new_state):
        next_best_action = np.argmax(self.q_table[new_state])
        td_target = reward + self.DISCOUNT_FACTOR*(self.q_table[new_state,next_best_action])
        new_qvalue = (1-self.LEARNING_RATE)*self.q_table[state,action] + self.LEARNING_RATE*(td_target)
        self.q_table[state,action] = new_qvalue

        # Decay exploration rate
        self.exploration_rate = max(
            self.min_exploration_rate, 
            self.exploration_rate * self.exploration_decay
        )

    def laser_callback(self, laser_msg):
        self.discrete_laser_range = self.discretize_lidar_data(laser_data=laser_msg)

    def odometry_callback(self, odom_msg):
        self.current_pose = odom_msg.pose.pose.position
        _, _, self.current_yaw = euler_from_quaternion([
            odom_msg.pose.pose.orientation.w, 
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z
        ])

    def manoeuvre_bot(self, action):
        twist_msg = Twist()        
        if action == 0:
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0
            self.twist_pub_.publish(twist_msg)
        elif action == 1:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 1.0
            self.twist_pub_.publish(twist_msg)
        elif action == 2:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -1.0
            self.twist_pub_.publish(twist_msg)
    
    def reset_simulation(self):
        if not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Gazebo reset service not available.")
            return
        request = Empty.Request()
        self.reset_simulation_client.call_async(request)
        self.get_logger().info(f"Resetting environment for episode {self.episode_counter}")


    def training_loop(self):
        if self.episode_counter <= self.EPISODES:
            if self.episode_counter == 0 or self.state == 0 or self.goal_reached:
                self.reset_simulation()

            if self.episode_counter % 300 == 0:
                self.get_logger().info(f"EPISODE_NUMBER: {self.episode_counter}")
                self.reset_simulation()

            if self.discrete_laser_range:
                new_state = self.get_state(discrete_ranges=self.discrete_laser_range)
                action = self.choose_action(self.state)
                self.manoeuvre_bot(action=action)
                reward = self.calculate_reward(action=action)
                
                self.update_qtable(state=self.state, action=action, reward=reward, new_state=new_state)
                self.get_logger().info(f"{self.q_table}\n")

                if self.episode_counter % 300 == 0:
                    self.epi_aggregate['epi'].append(self.episode_counter)
                    self.epi_rewards.append(reward)
                    

                self.state = new_state
                self.action = action


            self.episode_counter += 1
        else:
            self.save_qtable()
            self.plot_metrics()
            self.get_logger().info(f"stopping training simulation; completed {self.EPISODES} episodes of training...")
            self.timer_.cancel()

    def save_qtable(self):
        os.makedirs('q_tables', exist_ok=True)
        filename = f'q_tables/q_table_episode_{self.episode_counter}.npy'
        np.save(filename, self.q_table)
        self.get_logger().info(f"Q-table saved to {filename}")
    
    def load_qtable(self, filename):
        try:
            loaded_qtable = np.load(filename)
            self.q_table = loaded_qtable
            self.get_logger().info(f"Q-table loaded from {filename}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to load Q-table: {e}")
            return False
    
    def plot_metrics(self):
        if self.epi_aggregate['epi']:
            self.epi_aggregate['avg'] = sum(self.epi_rewards[300:])/len(self.epi_rewards[300:])
            self.epi_aggregate['min'] = min(self.epi_rewards[300:])
            self.epi_aggregate['max'] = max(self.epi_rewards[300:])

            plt.plot(self.epi_aggregate['epi'], self.epi_aggregate['avg'], label='avg')
            plt.plot(self.epi_aggregate['epi'], self.epi_aggregate['min'], label='min')
            plt.plot(self.epi_aggregate['epi'], self.epi_aggregate['max'], label='max')
            
            plt.legend(loc=4)
            plt.show()
    
    def inference_mode(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = QLearningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        node.save_qtable()
        print(f"Program interrupted by the user")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()