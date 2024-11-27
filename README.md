# Automated Mobile Robot(4-W-Mecanum Drive)
## Kinematic Based Custom Mecanum Drive Controller
### using ros2 control framework
requirements 🧰
- ROS2 Humble/Iron
- ROS2 Control
- Gazebo-Classic/Ignition Gazebo (For initial testing)

Kinematic Model of 4-W-Mecanum Drive
[Pick here]
[equations here]

start with week-2 of this [course](https://archive.nptel.ac.in/courses/112/106/112106298/) to understand kinematic models of a system (same can be drived for other mechanical systems not just robots), wheel config and derivation of the above equation.
- Note: since the bot's motion is planar mostly, the kinematic model is accurate enough for modelling this system. For advanced cases I think we should also include the dynamic model along with the kinematic model(not implemented here) but do check this [course's](https://archive.nptel.ac.in/courses/112/106/112106298/) week-3.


Developed a kinematic based custom ros2 control controller for 4 wheel mecanum mobile robot.
For the instructions for installing ros2 control pkg [use this](https://control.ros.org/humble/doc/getting_started/getting_started.html)
- Note: ros2 control pkg for iron(EOL as of 2024) and humble are fine, the one for jazzy is [here](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)


