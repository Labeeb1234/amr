# AMR Documentation

## CAD Model
- **4-Wheeled Mecanum Setup**:  
  - A chassis equipped with 4 Mecanum wheels allowing for omni-directional movement (forward, backward, sideways, and rotational movements).

---

## Simulation
- **Simulation Software**:  
  - The automated simulation was done on NVIDIA IsaacSim
  -  **add the GIF demo here**
<div align="center">
  <img src="" alt="NAV2-IsaacSim Demo">
</div>
---

## Hardware Setup

### Total System Weight
- **12kg** (including payload)

### Components:
- **Chassis**:
  - Pre-assembled aluminum frame.
  
- **Mecanum Wheels**:
  - Four wheels enabling omnidirectional movement, typically mounted at 45-degree angles for versatile maneuverability.

- **DC Motors**: 
  - Motors controlling the speed and direction of the Mecanum wheels, possibly with integrated encoders for feedback.

- **Buck Converter (22v -> 12v step down)**:
  - Used to convert the input 22V LiPo battery to the 12V required for other components.

- **22V LiPo Battery**:
  - Powers the system, providing necessary energy for operation.

- **MPU6050/9050 IMU**:
  - Inertial Measurement Unit (IMU) for measuring acceleration and rotational rates.

- **OE-775 Hall-Effect Quadrature Encoders**:
  - Provide feedback on wheel rotation to allow for precise motion control.

- **ESP32-Wroom 32**:
  - A microcontroller compatible with MicroROS (ROS2) Framework, enabling communication with the upper layer of the system.
  
- **Raspberry Pi/Jetson** (currently not in use):
  - Originally intended as the computing platform, but switched to a local host machine (Z-Book).

- **RPLidar A2 M8**:
  - A lidar sensor used for mapping and localization.

---

## Lower Layer Setup  
(IMPORTANT STUFF BEFORE STARTING)

### Embedded Software Setup
- **PlatformIO-C Framework**:
  - Software development environment used for embedded part (lower layer code) of the AMR. [Link for installation and documentation](https://docs.platformio.org/en/latest/)
  - Install the **micro_ros_platformIO** libraries for micro_ros integration.
  
- **Arduino C Framework**:
  - Chosen over the standard Lua script for ESP32. This is used to program the ESP32 for motor control and sensor integration.  
  *(Reason: Short timeframe to get the system fully functional.)*

- **Micro-ROS Setup**:
  - ROS2 Humble-based micro_ros version was installed. [Instructions/Documentation for the same here](https://micro.ros.org/docs/tutorials/core/first_application_linux/) (do the source installation).
  - Also, install PlatformIO-specific libraries for micro_ros. [Check this repo for those specific instructions](https://github.com/micro-ROS/micro_ros_setup).
 
- **Note**:  
  The `platformio.ini` file is crucial for configuring upload instructions (such as the baudrate for serial communication, upload port, or specifying the communication protocol required, like serial or Wi-Fi). It also installs the necessary custom libraries for lower layer control or pre-compiled libraries.

---

## Motor and Sensor Setup

### Sensor Calibration
- **IMU Calibration**: 
  - Calibration process for the MPU6050/9050 to ensure accurate readings. [script here](#)
  
- **Encoder Calibration**: 
  - Calibration process for the OE-775 Hall-Effect Quadrature Encoders. [script here](#)

- **Note** 
  For uploading the script use this cmd while inside the directory containing the correct calibration code
    ```bash
    pio run -e esp32 -t upload
    ```
  For viewing the serial monitor data on the terminal run the command below
  ```bash
  pio  device monitor -e esp32 -b 115200
  ```

### Velocity PID Tuning
- **PID Controller**: 
  - Scripts used for PID tuning to optimize velocity control.
  - Tuning parameters are adjusted to improve movement accuracy and response.
  - The default firmware package was taken and all unnecessary components of the code were removed and rewritten to suit our PID-tuning needs. [script here](#)
  - The PID tuning script itself has the main lower layer control code which connects with the ROS2 layer via serial (for now, as it was properly tested); it was a bad naming convention which needs to be fixed in the future.
  - The PID tuning script has the rcl_c based code running with publishers for the processed odometry data from the encoders, processed IMU data (yaw rate from gyro only), and a subscription on the `cmd_vel` topic to get the bot frame velocities from the ROS2 layer (for Nav2, teleop, etc.).
  - For uploading use the command below
  - **Note**:
    The default `ini` file was used for our purpose with a custom environment for the ESP32 serial mode that we are using --> Check out the ini file [here](#).
    

- **Note**:
  The [Linorobot2 Hardware](https://github.com/linorobot/linorobot2_hardware/tree/humble) repo was modified to meet our requirements for this project; link to the same given here. This repo can even be used as a template for almost any robotics project as it has all the common component's integration into the embedded system, generalized as possible. Also, an important note: the encoder library in this repo is one of the best I have seen in open-source, with optimization done in assembly language.

---
