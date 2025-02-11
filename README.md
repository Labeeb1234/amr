# AMR Documentation

## CAD Model
- **4-Wheeled Mecanum Setup**:  
  - The AMR is equipped with a chassis featuring four Mecanum wheels that allow for omnidirectional movement. This setup enables the robot to move forward, backward, sideways, and rotate with precision.

---

## Simulation
- **Simulation Software**:  
  - The automated simulation was performed using **NVIDIA IsaacSim** to test the AMR's capabilities in a controlled virtual environment.
  
  - **Demo**:
    - *Add the GIF demo here to showcase the simulation*:
      <div align="center">
        <img src="" alt="NAV2-IsaacSim Demo">
      </div>

---

## Hardware Setup

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

### Velocity PID Tuning
- **PID Controller**:
  - **PID tuning scripts** are used to optimize the velocity control of the robot.
  - The parameters for the PID controller are adjusted to fine-tune the movement accuracy and responsiveness of the AMR.
  - The default firmware package was taken and stripped down to remove unnecessary components, focusing on our specific PID-tuning needs.  
    - [Link to PID tuning script](#)
  - The PID tuning script implements control logic that communicates with the **ROS2** layer via **serial communication** (for now, as it has been properly tested). It includes rcl_c-based code that publishes odometry data from the encoders, IMU data (yaw rate from the gyro), and subscribes to the `cmd_vel` topic for robot frame velocities.
  - To upload the PID tuning code, navigate to the PID tuning directory and run the following:
    ```bash
    pio run
    ```

- **Note on `platformio.ini`**:
  - The **`platformio.ini`** file is customized for the ESP32 serial communication environment. For reference, [download the `ini` file here](#).

---

### Additional Notes:

- **Linorobot2 Hardware Repository**:
  - The [Linorobot2 Hardware repo](https://github.com/linorobot/linorobot2_hardware/tree/humble) has been modified to meet the specific requirements of this project.
  - This repository serves as a template for various robotics projects as it integrates common components in an optimized and generalized manner.
  - A particularly useful feature is the [**encoder library**](https://github.com/linorobot/linorobot2_hardware/tree/humble/firmware/lib/encoder), which is one of the best in the open-source community and is optimized in assembly language for performance.

---
