# Q-Learning Based Path Planner
A simple standard Q-Learning Algorithm was implemented on a mobile robot(4WD & mecanum bot) for creating a path planner as well as a lower layer kinematic controller. The efforts and results of this implementation are given below.
For the initial implementation, the simulation was done using ROS2 and in Gazebo-Classic simulator.
## Q Based Goal Pose Controller




Note: Any improvement suggestions are welcome to make this better (First Dabble on robot learning)

- Limitation in using standard q-learning for path planning, but was able to make a controller that can move to straight_line poses ---> for general waypoints ig the discretization and complexity of the problem make the Q-learning algo sub par in a 3D-simulated environment.

# DQN Based Path Planner
## Ongoing
- Starting with the creation of a gym-based botenv
