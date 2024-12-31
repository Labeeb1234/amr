#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Float32

import os
import math
import numpy as np


class QLearningNode(Node):
    def __init__(self):
        super().__init__("qlearning_node")
        self.callback_group = ReentrantCallbackGroup()
        self.odom_sub_ = self.create_subscription(Odometry, "odometry/unfiltered", callback=self.get_bot_pose, qos_profile=10, callback_group=self.callback_group)
        self.twist_pub_ = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.timer_period = 0.1 # 10Hz controller frequency
        self.timer_ = self.create_timer(self.timer_period, self.training_loop)
        # self.inference_timer_ = self.create_timer(self.timer_period, self.inference_loop)

        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation') 

        # STATE and ACTION PARAMS
        self.state = (0,0) # (dist_to_goal, orient_to_goal)
        self.action = 0
        self.DISTANCE_BINS = 32
        self.ORIENTATION_BINS = 32
        self.ACTION_SPACE = 4

        # Random Exploration PARAMS
        self.epsilon = 1.0
        self.min_epsilon = 0.1
        self.epsilon_decay_rate = 0.9

        # Q-learning PARAMS
        self.EPISODES = 600
        self.episode_counter = 0
        self.steps_per_episode = 300
        self.steps_counter = 0
        self.LEARNING_RATE = 0.01
        self.DISCOUNT_FACTOR = 0.95
        self.episode_reward = 0.0

        # initializing q-table
        self.q_table = np.random.uniform(low=-2, high=0, size=(self.DISTANCE_BINS, self.ORIENTATION_BINS, self.ACTION_SPACE))
        self.start_q_table = None

        # goal params
        self.goal_position = [3.0, 0.0, 0.0]
        self.goal_tolerance = 0.9
        self.goal_reached = False

        # bot info/sensor feedback params
        self.current_position = None
        self.current_yaw = 0.0
        self.distance_to_goal = 0.0
        self.previous_distance = 0.0
        self.angle_diff = 0.0

        # inference params
        self.inference_mode = False

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

    def get_discretize_state(self, dist_bins_space, orient_bins_space):
        dist_bin_edges = np.linspace(0, 10, dist_bins_space)
        dist_bins = np.digitize(self.distance_to_goal, dist_bin_edges)

        orient_bin_edges = np.linspace(-np.pi, np.pi, orient_bins_space)
        orient_bins = np.digitize(self.angle_diff, orient_bin_edges)

        dist_bins = min(dist_bins, dist_bins_space - 1)
        orient_bins = min(orient_bins, orient_bins_space - 1)

        return (dist_bins, orient_bins)
    
    def choose_action(self, discrete_state):
        if np.random.random() > self.epsilon:
            return np.argmax(self.q_table[discrete_state])
        else:
            return np.random.randint(0, self.ACTION_SPACE)
    
    def compute_reward(self, action):
        reward = 0.0

        if self.angle_diff < 0.1 and self.angle_diff > -0.1:
            reward += 2500.0
        else:
            reward += -2000.0

        if self.distance_to_goal < self.goal_tolerance:
            reward = 1000000
            self.goal_reached = True
        else:
            if self.distance_to_goal < self.previous_distance:
                reward += 2500.0
            elif self.distance_to_goal >= self.previous_distance:
                reward += -2000.0


        self.previous_distance = self.distance_to_goal

        if action == 1 or action == 2:  # Rotational actions (turning left or right)
            reward -= 100  # Small penalty for rotational movement to encourage forward motion

        # Penalize for stopping in an incorrect position
        if action == 3 and self.distance_to_goal >= self.goal_tolerance:
            reward = -5000  # Large penalty for stopping when not at the goal
        elif action == 3 and self.distance_to_goal < self.goal_tolerance:
            reward += 10000

        return reward

    def update_qtable(self, state, action, reward, new_state):
        next_best_action = np.argmax(self.q_table[new_state])
        td_target = reward + self.DISCOUNT_FACTOR*(self.q_table[new_state+(next_best_action,)])
        new_qvalue = (1-self.LEARNING_RATE)*self.q_table[state+(action,)] + self.LEARNING_RATE*td_target
        self.q_table[state+(action,)] = new_qvalue

        self.epsilon = max(self.min_epsilon, self.epsilon*self.epsilon_decay_rate)

    ###################### (custom training goals) ###########################
    def randomize_straight_goal(self):
        pass

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

        # self.twist_pub_.publish(twist_msg)

    def get_bot_pose(self, odom_msg):
        self.current_position = odom_msg.pose.pose.position
        _, _, self.current_yaw = self.quaternion_to_euler(
            odom_msg.pose.pose.orientation.w, 
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z
        )
        # self.get_logger().info(f"[x: {self.current_position.x:.2f}, y: {self.current_position.y:.2f}, yaw: {self.current_yaw:.2f}]")
    
    def reset_simulation(self):
        if not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Gazebo reset service not available.")
            return
        request = Empty.Request()
        self.reset_simulation_client.call_async(request)
        # Reset attributes for a new episode
        self.get_logger().info(f"Resetting environment for episode {self.episode_counter}")

    def training_loop(self):
        if self.episode_counter <= self.EPISODES:
            if self.episode_counter == 0 or self.goal_reached or self.steps_counter >= self.steps_per_episode:
                self.reset_simulation()
                self.steps_counter = 0
                self.episode_counter += 1
                 
                # Reset the episode reward accumulator
                self.episode_reward = 0.0
                
            if self.current_position:
                # main training logic here
                self.distance_to_goal = self.dist_to_goal(self.goal_position, self.current_position)
                self.angle_diff = self.angle_goal_diff(self.goal_position, self.current_yaw)
                new_state = self.get_discretize_state(self.DISTANCE_BINS, self.ORIENTATION_BINS)
                
                self.get_logger().info(f"distance to goal: [{self.distance_to_goal:.2f}]")
                self.get_logger().info(f"Angular Difference to goal: [{self.angle_diff:.2f}]")
                self.get_logger().info(f"state: {self.state}")
              
                self.action = self.choose_action(self.state)
                self.move_bot(self.action)
                reward = self.compute_reward(action=self.action)
                self.episode_reward += reward
                self.episode_reward = self.episode_reward/self.steps_per_episode
                self.update_qtable(state=self.state, action=self.action, reward=reward, new_state=new_state)

                # self.get_logger().info(f"QTable: {self.q_table}\n")
                self.get_logger().info(f"Optimized Action: {self.action}")
                self.get_logger().info(f"reward: {reward}")
                
                self.state = new_state
                self.steps_counter += 1

            self.save_qtable(self.episode_counter, self.q_table)
            self.get_logger().info(f"Average-{self.episode_counter}-episode reward: {self.episode_reward}")
            self.get_logger().info(f"episode: {self.episode_counter}")
            self.get_logger().info(f"steps: {self.steps_counter}")

        else:
            self.get_logger().info(f"Training Finished after {self.episode_counter} episodes..!")
            self.timer_.cancel()
    
    def save_qtable(self, episode, q_table):
        os.makedirs(name="q_tables", exist_ok=True)
        filename = f"q_tables/q_table_{episode}.npy"
        np.save(filename, q_table)

    def load_qtable(self, saved_q_table):
        self.start_q_table = np.load(f'qtables/{saved_q_table}.npy')
        self.get_logger().info(f"Initial Q-Table:\n{self.start_q_table}")
    
    def plot_metrics(self):
        pass



    def inference_loop(self):
        if self.inference_mode:
            # Get the current state
            state = self.get_discretize_state(self.DISTANCE_BINS, self.ORIENTATION_BINS)
            # Choose the best action based on the learned Q-table
            best_action = np.argmax(self.q_table[state])
            # Move the robot based on the best action
            self.move_bot(best_action)
            # Optionally, you can compute a reward here, though this is usually not necessary during inference
            reward = self.compute_reward(action=best_action)
            # Log the current state and action for debugging purposes
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