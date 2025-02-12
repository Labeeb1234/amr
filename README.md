# AMR Documentation

## CAD Model
- **4-Wheeled Mecanum Setup**:  
  - The AMR is equipped with a chassis featuring four Mecanum wheels that allow for omnidirectional movement. This setup enables the robot to move forward, backward, sideways, and rotate with precision.

---

## Simulation
- **Simulation Software**:  
  - The automated simulation was performed using **NVIDIA IsaacSim** to test the AMR's capabilities in a controlled virtual environment.
  - The simulation was done using ROS2-NAV2 stack using NavFn Global Planner and DWB local planner/controller for the path planning, the environment used was a standard warehouse environment found in the ISAAC_NUCLEUS_DIRECTORY 
  - **Demo**:
      <div align="center">
        <img src="https://github.com/user-attachments/assets/74128058-e543-4375-81f6-d30da3b26e14" alt="NAV2-IsaacSim Demo">
      </div>

---

## Hardware Setup

  <div align="center">
    <img src="https://github.com/user-attachments/assets/0ed413d9-5ac8-4bfc-a520-5cfcfab95e45" width="640" height="480" alt="AMR-HARDWARE">
  </div>


### Total System Weight
- **12kg** (including payload)

### Components:
- **Chassis**:
  - Pre-assembled aluminum frame providing a sturdy structure for the robot.

- **Mecanum Wheels**:
  - Four wheels arranged at 45-degree angles to enable omnidirectional movement, allowing the robot to move in any direction without changing orientation.

- **DC Motors**: 
  - Motors control the speed and direction of the Mecanum wheels. These motors may also have encoders for feedback to ensure accurate movement.

- **Buck Converter (22V -> 12V step-down)**:
  - A DC-DC buck converter that steps down the 22V from the LiPo battery to the required 12V for the onboard electronics.

- **22V LiPo Battery**:
  - The power source for the AMR, providing the necessary voltage to operate the system.

- **MPU6050/9050 IMU**:
  - Inertial Measurement Unit (IMU) that measures the robot's acceleration and rotational rates, crucial for maintaining orientation and stability.

- **OE-775 Hall-Effect Quadrature Encoders**:
  - Encoders provide precise feedback on the wheel rotations, enabling accurate motion control and odometry calculation.

- **ESP32-Wroom 32**:
  - A microcontroller compatible with the **MicroROS (ROS2)** framework, which handles the communication between the hardware and the higher-level software layers.

- **Raspberry Pi/Jetson** (currently not in use):
  - Initially planned as the computing platform, but replaced by a local host machine (Z-Book) for processing tasks.

- **RPLidar A2 M8**:
  - A lidar sensor for mapping and localization, essential for autonomous navigation.

---

## Lower Layer Setup  
*Important Considerations Before Starting*

### Embedded Software Setup
- **PlatformIO-C Framework**:
  - The development environment for the embedded part of the system (lower layer code). For installation and documentation, check the [PlatformIO docs](https://docs.platformio.org/en/latest/).
  - Install **micro_ros_platformIO** libraries for MicroROS integration.

- **Arduino C Framework**:
  - The **Arduino C Framework** is chosen over Lua for programming the ESP32 due to time constraints and the need to quickly develop functional motor control and sensor integration.

- **Micro-ROS Setup**:
  - The system uses **ROS2 Humble** with the **Micro-ROS** version. Installation instructions can be found in the [Micro-ROS tutorial](https://micro.ros.org/docs/tutorials/core/first_application_linux/).
  - PlatformIO-specific libraries for MicroROS must also be installed. Refer to this [Micro-ROS Setup Repo](https://github.com/micro-ROS/micro_ros_setup) for detailed instructions.

- **Note on `platformio.ini`**:
  - The `platformio.ini` configuration file is vital for specifying upload instructions (e.g., baud rate, upload port) and defining the communication protocol (e.g., serial, Wi-Fi). It also ensures that the correct custom libraries for lower-layer control are installed.

---

## Motor and Sensor Setup

### Sensor Calibration
- **IMU Calibration**:
  - Calibration process for the **MPU6050/9050 IMU** to ensure accurate acceleration and gyroscope readings.
  - [Link to calibration script](#)

- **Encoder Calibration**:
  - Calibration for the **OE-775 Hall-Effect Quadrature Encoders** to ensure accurate wheel rotation feedback.
  - [Link to calibration script](#)

- **Upload Instructions**:
  To upload the calibration scripts to the ESP32, navigate to the directory containing the appropriate calibration code and run the following command:
    ```bash
    pio run -e esp32 -t upload
    ```
  To monitor the serial data from the terminal, use this command:
    ```bash
    pio device monitor -e esp32 -b 115200
    ```

### Velocity PID Tuning and Main Lower Layer Setup
- **PID Controller**:
  - **PID tuning scripts** are used to optimize the velocity control of the robot.
  - The parameters for the PID controller are adjusted to fine-tune the movement accuracy and responsiveness of the AMR.
  - The default firmware package was taken and stripped down to remove unnecessary components, focusing on our specific PID-tuning needs.  
    - [Link to PID tuning script](#)
  - The PID tuning script implements control logic that communicates with the **ROS2** layer via **serial communication** (for now, as it has been properly tested). It includes rcl_c-based code that publishes odometry data from the encoders, IMU data (yaw rate from the gyro), and subscribes to the `cmd_vel` topic for robot frame velocities.
  - The **kinematic layer** is integrated into this codebase via the [kinematics library of the linorobot2 hardware](), the major bot configurations supported by this library are **diffrential drive(2WD,4WD)** and the **mecanum drive**(what we used here); the kinematics is based on the frame arrangement as shown below:
    
      <div align="center">
        <img src="https://github.com/user-attachments/assets/97281fa5-9d19-4549-b99b-0b16a3caefe6" alt="Frame Arrangement">
      </div>
      
      Make sure the hardware wheel configuration follows the same convention as in the picture above otherwise the motor signal signs may get inverted making the bot move in undesireable ways
  - To upload the PID tuning code, navigate to the PID tuning directory and run the following:
    ```bash
    pio run -e esp32 -t upload
    ```
  - A custom [config.h]() file is created in order to consolidate all the PINOUT parameters of all the connections to the esp32 board. The default configs we used for our system is shown below:
  ```cpp
  
  ```
- **Note**
  - The name of the main lower layer codebase is same as the pid tuning codebase (need to change it).

- **Note on `platformio.ini`**:
  - The **`platformio.ini`** file is customized for the ESP32 serial communication environment. For reference, [check out the `ini` file here](#).
  - More on this configuration file is given [here](https://docs.platformio.org/en/latest/projectconf/index.html)


---

### Additional Notes:

- **Linorobot2 Hardware Repository**:
  - The [LinoRobot2 Hardware repo](https://github.com/linorobot/linorobot2_hardware/tree/humble) has been modified to meet the specific requirements of this project.
  - This repository serves as a template for various robotics projects as it integrates common components in an optimized and generalized manner.
  - A particularly useful feature is the [**encoder library**](https://github.com/linorobot/linorobot2_hardware/tree/humble/firmware/lib/encoder), which is one of the best in the open-source community and is optimized in assembly language for performance.

---

### Upper Layer ROS2 Stack Setup and Process

- **Navigation Stack Setup**:  
  As mentioned in the previous section, we utilized the template repository from **LinoRobot2** to set up the navigation stack for the hardware. This provided a solid foundation for our system's navigation capabilities.

- **Support Packages**:  
  The rest of the supporting packages were custom built, including:
  - The **bringup package** 
  - The **bot model description package** (model based on CAD)  
  [Source code for these packages here]()

- **RPLidar Integration**:  
  Unlike the encoders and IMU, the **RPLidar A2-M8** was directly integrated into the upper layer using [SLAMTech's RPLidar ROS2 package](https://github.com/Slamtec/rplidar_ros/tree/ros2).  
  [launch files here]()
  **Note on Lidar**- The [laser_filter](https://github.com/ros-perception/laser_filters) is an open-source package consisting different kinds of lidar data filter which helps in filtering out undesireable laser scan data, this packge is quite useful for many purposes that uses laser data so instead of wasting time reinventing the wheel please check this package out.

- **IMU Data Handling**:  
  To process the IMU data from the base IMU topic, which publishes at 10Hz, I developed a helper node called **imu_handler**. This node extracts the yaw rate and integrates it to publish the yaw data at the same rate as the base IMU publisher. This was done because we weren’t using a magnetometer, so only the gyroscope's yaw rate was available in the base IMU topic. This made the sensor fusion output significantly better than when we were using just yaw_rate from gyroscope.
  [Source code here](), [Integrated launch file here]()

- **Localization with EKF**:  
  Due to the unreliability of pure dead reckoning-based localization, we used the [robot_localization](https://github.com/cra-ros-pkg/robot_localization) package to fuse the odometry and IMU data using an Extended Kalman Filter (EKF). The integration involved configuring the necessary parameters for sensor fusion, such as `(x, y, x_vel, y_vel, yaw_vel)`. We mostly used the default parameters with adjusted publishing frequencies.

- **Navigation Setup**:  
  For navigation, we set up the **ROS2-NAV2 stack** to automate path planning. We tested and tuned the hardware using the default local and global planners (dwb-navfn).
   
  [Navigation parameters here]()


- **Note**
  - Packages like plotjuggler can be used to data visualization on-line and [rqt_reconfiguration_tool](https://github.com/ros-visualization/rqt_reconfigure) for dynamic tuning of the nav2 params(may not work for all the nav2 parameters)
  - The rqt_reconfigure packages comes with the full desktop installation of the ROS2 packages (no separate installation required else use apt for the pkg installation).
  - use the following commands to install plotjuggler easily:
    ```bash
    sudo apt install ros-humble-plotjuggler
    ```
  - (Observed in humble and Iron distros): sometime(only SOMETIMES) during the plotjuggler node launch you may encounter a plotjuggler plugin error, if this happens use either of the command given below:
    ```bash
    sudo apt install ros-humble-plotjuggler-ros
    ```
    ```bash
    sudo apt install ros-humble-plotjuggler-*
    ```

---

# ============================================================================================================



















