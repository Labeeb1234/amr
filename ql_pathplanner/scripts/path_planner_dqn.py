#! /usr/bin/python3

import rclpy
import rclpy.node as Node


import tensorflow as tf
from tensorflow.keras.callbacks import TensorBoard


import random
import numpy as np
from collections import deque


class DQNAgent:
    def __init__(self, replay_buffer_capacity):
        self.capacity = replay_buffer_capacity
        
        self.model = self.build_model()
        self.target_model = self.build_model()
        self.target_model_weights = self.target_model.set_weights(self.model.get_weights())

        self.exp_replay_buff = deque(maxlen=self.capacity)

    def update_replay_memory(self, transition):
        # transitions (s, a, r, s', done)
        self.exp_replay_buff.append(transition)

    def build_model(self):
        model = tf.keras.layers.Input(shape=env.STATE_SPACE_SIZE)

        model.add(tf.keras.layers.Dense(64, activation="relu"))
        model.add(tf.keras.layers.Dense(64, activation="relu"))

        model.add(tf.keras.layers.Dense(ACTION_SPACE_SIZE, activation="linear"))
        
        model.compile(loss="mse", optimizer=tf.keras.optimizers.Adam(learning_rate=LEARNING_RATE), metrics=['accuracy'])

        return model
    
    def get_qs(self, state):
        return self.model.predict(np.array(state))



def main(args=None):
    rclpy.init(args=args)

if __name__ == '__main__':
    main()
    

    





