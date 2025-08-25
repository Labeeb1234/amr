# AMR Documentation

## CAD Model

- **4-Wheeled Mecanum Setup**:  
  - The AMR is equipped with a chassis featuring four Mecanum wheels that allow for omnidirectional movement. This setup enables the robot to move forward, backward, sideways, and rotate with precision.
  - **The model**:
      <div align="center">
        <img src="https://github.com/user-attachments/assets/8dc21edd-334a-4aed-a621-587c5db2ec8e" alt="CAD-Model">
      </div>
---

## Simulation

- **Simulation Software**:  
  - The automated simulation was performed using **NVIDIA IsaacSim** to test the AMR's capabilities in a controlled virtual environment.
  - The simulation was done using ROS2-NAV2 stack using NavFn Global Planner and DWB local planner/controller for the path planning, the environment used was a standard warehouse environment found in the ISAAC_NUCLEUS_DIRECTORY (cloud asset)
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
  - Calibration process for the **MPU6050/9050 IMU** to ensure accurate acceleration and gyroscope readings. Note the current MPU9250 sensor driver code only includes calibration of gyro not accelerometer.
  - IMU: Accelerometer Scale: +-2g, Gyroscope Scale: +-250degree/s.
  - [Link to calibration script](https://github.com/Labeeb1234/amr/tree/main/AMR_Hardware/test_sensors)

- **Encoder Calibration**:
  - Calibration for the **OE-775 Hall-Effect Quadrature Encoders** to ensure accurate wheel rotation feedback.
  - [Link to calibration script](#https://github.com/Labeeb1234/amr/tree/main/AMR_Hardware/calibration)
  - [For testing the encoder counts and rpm of the motor after calibration use this script](https://github.com/Labeeb1234/amr/tree/main/AMR_Hardware/test_motors)

- **Magnetometer/Compass Setup**
  - For the magnetometer sensor we are using the inbuilt AK8963 driver in the MPU9250. But there is a catch the current linorobot2 codebase does not initialize this magnetometer it just publishing the yaw values of odom as a fake mag. (as of 2025) in the default codebase.
  - The MPU9250_MAG Scale: +-4800uT (at 14 bit resolution).
  - To be honest there is a possibility we may be missing something for setting this up, but the current work around we did was to write a driver code (for MPU9250_MAG) in the **default_mag.h** header file to access the magnetometer with the i2c_bypass which is already enabled in the **MP9250.cpp** lib compiled and present in the lino codebase. The modified mag driver header file is [here](#https://github.com/Labeeb1234/amr/tree/main/AMR_Hardware/firware_template/lib/imu/default_mag.h).
  - The calibration of magnetometer instructions are given in the main linorobot2 hardware repo under the "rolling" branch link ---> IMPORTANT STUFF: [here](https://github.com/linorobot/linorobot2_hardware/tree/rolling). As mentioned in their documentation if magentomter is enabled the firmware code published the magnetometer and imu data in two separate topics. The datas of both will be fused using Madgwick Filter (note to self: read about this in details), the ros2 pkg for madgwick filter is [here](https://github.com/CCNYRoboticsLab/imu_tools) install the pkg using the appropriate distro (as of 2025 available for Jazzy and Kilted too).

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
  - **PID tuning scripts** are used to optimize the velocity control of the PGDC-Motors used for the actuation.
  - Both the target motor velocity and feedback are in RPM units not in PWM units (PWM [0,255] ---> RPM using the motor libs).
  - The parameters for the PID controller are adjusted to fine-tune the movement accuracy and responsiveness of the AMR.
  - The PID tuning script implements control logic that communicates with the **ROS2** layer via **serial communication** (for now, as it has been properly tested). It includes rcl_c-based code that publishes odometry data from the encoders which is required to extract the motor velocity feedback for the PID tuning process and subscribes to the command topic for robot frame velocities for given cmd signals instead of direct motor signals (conveinence).
  - The feedback motor RPM from the encoder (after post-processing) is sent into a low-pass filter(emea) to cut-off high frequency encoder noise and then the filtered feedback is sent to input feed for error calculation and PID computation. Observed a better less noisy response from the system by doing this.
    
      <div align="center">
        <img src="" alt="PID response before low-pass filter">
      </div>
      <div align="center">
        <img src="" alt="PID response after low-pass filter">
      </div>
      
  - The default firmware package was taken and stripped down to remove unnecessary components, focusing on our specific PID-tuning needs.
    - [Link to PID tuning script](https://github.com/Labeeb1234/amr/tree/main/AMR_Hardware/pid_tuning)

- **Bot Kinematics**:
  - The **kinematic layer** is integrated into this codebase via the [kinematics library of the linorobot2 hardware](https://github.com/Labeeb1234/amr/tree/main/AMR_Hardware/firmware_template/lib/kinematics), the major bot configurations supported by this library are **diffrential drive(2WD,4WD)** and the **mecanum drive**(what we used here); the kinematics is based on the frame arrangement as shown below:
    
      <div align="center">
        <img src="https://github.com/user-attachments/assets/97281fa5-9d19-4549-b99b-0b16a3caefe6" alt="Wheel Configuration">
      </div>
      
      Make sure the hardware wheel configuration follows the same convention as in the picture above otherwise the motor signal signs may get inverted making the bot move in undesireable ways

- To upload the PID tuning code, navigate to the PID tuning directory and run the following:
    ```bash
    pio run -e esp32 -t upload
    ```
- A custom [config.h](https://github.com/Labeeb1234/amr/blob/main/AMR_Hardware/config/custom/esp32_config.h) file is created in order to consolidate all the PINOUT parameters of all the connections to the esp32 board as well some other important global paramters for running the firmware. The default configs we used for the "serial esp32-configuration" of our system is shown below:
  ```cpp
  #ifndef ESP32_CONFIG_H
  #define ESP32_CONFIG_H
  
  #define LED_PIN 2 //used for debugging status (using external LED)

  //uncomment the base you're building
  //#define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
  // #define LINO_BASE SKID_STEER            // 4WD robot
  #define LINO_BASE MECANUM                // Mecanum drive robot
  #define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
  //uncomment the IMU you're using
  #define USE_MPU9250_IMU
  #define USE_MPU9250MAG_MAG // for accessing magnetometer data from the MPU9250 
  
  // imu covariances
  #define ACCEL_COV { 0.01, 0.01, 0.01 } // 0.01
  #define GYRO_COV { 0.001, 0.001, 0.001 }
  #define ORI_COV { 0.01, 0.01, 0.01 }
  #define MAG_COV { 1e-12, 1e-12, 1e-12 }
  // encoder's initial covariance
  #define POSE_COV { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 } // 0.001
  #define TWIST_COV { 0.0001, 0.0001, 0.001, 0.003, 0.003, 0.003 } // 0.003

  /*
  ROBOT ORIENTATION
           FRONT
      MOTOR1  MOTOR2  (2WD/ACKERMANN)
      MOTOR3  MOTOR4  (4WD/MECANUM)
           BACK
  */
  
  /*
  ROBOT ORIENTATION
           FRONT
           MOTOR1
      MOTOR2  MOTOR4  (4W-90DEG-OMNI......yet to add!)
           MOTOR3  
           BACK
  */

  //define your robot/DC-motor(in this case) specs here
  #define MOTOR_MAX_RPM 256                   // motor's max RPM
  #define MAX_RPM_RATIO 0.85                // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
  // from fully charged battery supplying voltage
  #define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
  #define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
  #define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)

  // calibrated values
  #define COUNTS_PER_REV1 343.0                 // wheel1 encoder's no of ticks per rev
  #define COUNTS_PER_REV2 339.0                 // wheel2 encoder's no of ticks per rev
  #define COUNTS_PER_REV3 347.0                // wheel3 encoder's no of ticks per rev
  #define COUNTS_PER_REV4 347.0                // wheel4 encoder's no of ticks per rev
  
  #define WHEEL_DIAMETER 0.152                    // wheel's diameter in meters
  #define WHEELS_DISTANCE_DIFF 1.12           // distance between left and right wheels

  // MCU-concerning params (as of now fixed to default values --> impacts the control algorithm rate)
  #define PWM_BITS 10                         // PWM Resolution of the microcontroller
  #define PWM_FREQUENCY 20000                 // PWM Frequency(Hz)

  // INVERT ENCODER COUNTS
  #define MOTOR1_ENCODER_INV true 
  #define MOTOR2_ENCODER_INV true
  #define MOTOR3_ENCODER_INV false
  #define MOTOR4_ENCODER_INV true

  // INVERT MOTOR DIRECTIONS
  #define MOTOR1_INV true
  #define MOTOR2_INV true
  #define MOTOR3_INV true
  #define MOTOR4_INV true
  
  // ENCODER PINS
  #define MOTOR1_ENCODER_A  35 // 36  
  #define MOTOR1_ENCODER_B  34   //39
  
  #define MOTOR2_ENCODER_A  36 //35 
  #define MOTOR2_ENCODER_B  39 //34 
  
  #define MOTOR3_ENCODER_A  4  //32  
  #define MOTOR3_ENCODER_B  15 //27  
  
  #define MOTOR4_ENCODER_A  17  //26 
  #define MOTOR4_ENCODER_B  16  //25  

  // MOTOR PINS
  #ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
    #define MOTOR1_PWM  25  //19     
    #define MOTOR1_IN_A  26  //5   
    #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder (use only for XOR type motor driver logic---> PWM-PWM motor drivers)
   
    #define MOTOR2_PWM  32 //18   
    #define MOTOR2_IN_A 33 //15  
    #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  
    #define MOTOR3_PWM  27 //16    
    #define MOTOR3_IN_A 14 //13    
    #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  
    #define MOTOR4_PWM  12 // 17  
    #define MOTOR4_IN_A 13 // 12  
    #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  
    #define PWM_MAX pow(2, PWM_BITS) - 1
    #define PWM_MIN -PWM_MAX
  #endif


  #define DEVICE_HOSTNAME "esp32"
  #define APP_NAME "amr_ust_lower"
  #define BAUDRATE 115200
  #define SDA_PIN 21 // specify I2C pins
  #define SCL_PIN 22
  #define NODE_NAME "mecanum_node"
  #define CONTROL_TIMER 20
  
  
  #define BOARD_INIT {\ 
      Wire.begin(SDA_PIN, SCL_PIN); \ 
      Wire.setClock(400000); \ 
  }


  #endif

  ```

- **Note on `platformio.ini`**:
  - The **`platformio.ini`** file is customized for the ESP32 serial communication environment. For reference, [check out the `ini` file here](https://github.com/Labeeb1234/amr/blob/main/AMR_Hardware/firmware_template/platformio.ini), the environment we have used is under **<em>env.esp32</em>**.
  - More on this configuration file is given [here](https://docs.platformio.org/en/latest/projectconf/index.html)

- **Main Firmware Code**
  - The main firmware code to run the functionalities of the bot (lower layer) and interfacing with ROS2 is given [here](https://github.com/Labeeb1234/amr/tree/main/AMR_Hardware/amr_ust_lower) . This code is for the serial based communication transport.
  - For the wifi/UDP based communication the code is [here](https://github.com/Labeeb1234/amr/tree/main/AMR_Hardware/amr_wifi_lower) , its not complete still working on it for testing. 
---

### Additional Notes:

- **Linorobot2 Hardware Repository**:
  - The [LinoRobot2 Hardware repo](https://github.com/linorobot/linorobot2_hardware/tree/humble) has been modified to meet the specific requirements of this project.
  - This repository serves as a template for various robotics projects as it integrates common components in an optimized and generalized manner.
  - A particularly useful feature is the [**encoder library**](https://github.com/linorobot/linorobot2_hardware/tree/humble/firmware/lib/encoder), which is one of the best encoder library in the open-source community and is optimized using assembly language for performance.
  - Part of the upper layer and lower layer (especially the microROS agent launching, realsense d435i and RPLidar-A2M8) was run on an offboard laptop placed and connected on top of the AMR chassis; we plan on replacing it with an offboard computer like NVIDIA Jetson or a Raspi 4B 8GB RAM model (one available to us as of now).

- **Raspi 4B Setup and config**
  - As mentioned in the previous section we planned on implmenting the navigation stack on an offboard computer for a clean setup. So we moved the stack onto a Raspberry Pi-4 Model B 8GB RAM. The setup process roughly given below
    <div align="center">
      <img src="https://github.com/user-attachments/assets/c78c4fd5-e2c2-4f12-b267-fd57a89dbd35" width=640 height=640 alt="Raspi-Setup">
    </div>

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
### DEMOS

- The first test done on the system, to ensure connections between the lower layer and ROS layer are good enough, was a teleop test (shown below)
-  **teleop video**

- Next Demo on teleop-based SLAM
-  **Teleop-SLAM**

Next Demo on Automated-SLAM
- **NEED Tuning on hardware**
- simulation results using the wavefrontier methods were quite good but the hardware params for the same methods need some tuning, although an automated mapless navigation showed some good results for the same demo (not too reliable though)

- Next Demo Automated Map Based Navigation
**Map Based Automated Navigation video** (tuned for holonomic motion)



















