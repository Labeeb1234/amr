#include <Arduino.h>
#include <string.h>
#include <i2cdetect.h>
#include <Wire.h>

#include "wifis.h"
#include "syslog.h"
#include "ota.h"

#include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"  
#include "pwm.h"
#include "imu.h"
#include "mag.h"
#include "odometry.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/logging_macros.h>
#include "rmw/rmw.h"
#include "rmw/qos_policy_kind.h"
#include "rmw/qos_profiles.h"
#include "rmw/qos_string_conversions.h"
#include <rosidl_runtime_c/string_functions.h> 

// ROS mgs libs
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <nav_msgs/msg/odometry.h>
// #include <pid_msg/msg/pid.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>

// watch dog timers for resetting esp32
#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif
#ifdef USE_WIFI_TRANSPORT
// remove wifi initialization code from wifi transport
static inline void set_microros_net_transports(IPAddress agent_ip, uint16_t agent_port)
{
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}
#endif


//*********************************************** Define the required Macros here *********************************************
#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

#ifndef NODE_NAME
#define NODE_NAME "mecanum_node"
#endif

#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20 // in [ms]
#endif 

//*********************************************** define macros within this comment block ****************************************

// PID Gains 
float Kp = 4.0;
float Ki = 0.0001;
float Kd = 0.5;


const int total_motors = 4;
uint16_t previous_odom_time = 0;
uint16_t timer_period_ms = 0;
uint16_t time_offset = 0;
uint16_t prev_cmd_time = 0;

// ***********************************************
// some ros2 globals
rcl_subscription_t twist_subscriber;

rcl_publisher_t joint_state_publisher;
rcl_publisher_t pid_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_publisher_t odom_publisher;

rcl_timer_t timer;
geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__Vector3 pid_msg;
sensor_msgs__msg__JointState joint_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
nav_msgs__msg__Odometry odom_msg;


// microROS entities setup globals
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// suppoter class objects
Odometry odometry;
IMU imu;
Mag mag;

// **************************************************

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

// an array of "total_motors" number of PID class instances (0 indexing starts for motor-1 and ends at 3 for motor-4)
PID motor1_pid(PWM_MIN, PWM_MAX, Kp, Ki, Kd);
PID motor2_pid(PWM_MIN, PWM_MAX, Kp, Ki, Kd);
PID motor3_pid(PWM_MIN, PWM_MAX, Kp, Ki, Kd);
PID motor4_pid(PWM_MIN, PWM_MAX, Kp, Ki, Kd);

// float current_rpms[total_motors] = {0.0, 0.0, 0.0, 0.0};

// Kinematics class object
Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    WHEELS_DISTANCE_DIFF
);

// enumeration of the microROS agent state
enum states{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ########################################################
// LED functions (for debugging important)
void setLED(int value){
    #ifdef LED_PIN
        digitalWrite(LED_PIN, value);
    #endif    
}
int getLED(){
    #ifdef LED_PIN
        return digitalRead(LED_PIN);
    #else
        return 0;
    #endif
}
void initLED(){
    #ifdef LED_PIN
        pinMode(LED_PIN, OUTPUT);
    #endif
}
void flashLED(int n_times){
    for(int i=0; i<n_times; i++){
        setLED(HIGH);
        delay(500);
        setLED(LOW);
        delay(500);
    }
    delay(2000);
}
// ########################################################
// rclc return value checker for entity creation and destruction/for error handling
void rclErrorLoop(){
    while(true){
        flashLED(10); // flash 10 times
    }
}

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif
// ################################################################################################
// a helper function for periodic execution
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int16_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)
// #######################################
void full_stop(){
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
     
    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();  
}

// subscription callback function
void twist_callback(const void * msgin){
    // Cast received message to used type
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    setLED(LOW);
    prev_cmd_time = millis();
}


void move_cmd(){
    // if timeout for receving twist cmds is passed brake the bot
    uint16_t current_cmd_time = millis();
    if(current_cmd_time-prev_cmd_time >= 200){
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        setLED(HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z
    );

    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1,
        current_rpm2,
        current_rpm3,
        current_rpm4
    );

    // odom update
    uint16_t now = millis();
    float vel_dt = (now - previous_odom_time) / 1000.0;
    previous_odom_time = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z
    );

}

bool syncTime(){
    uint16_t timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
    timespec tp;
    tp.tv_sec = time_ns / 1000000000;
    tp.tv_nsec = time_ns % 1000000000;
    clock_settime(CLOCK_REALTIME, &tp);
#else
    uint16_t ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - millis();
#endif
    return true;
    }
    return false;
}

struct timespec getTime(){
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    uint16_t now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void publish_feedback_data(){
    odom_msg = odometry.getData();
    imu_msg = imu.getData();
    mag_msg = mag.getData();
#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif

    struct timespec time_stamp = getTime();
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
#ifndef USE_FAKE_MAG
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
#endif

}

void control_loop(rcl_timer_t * timer, int64_t last_call_time){
    uint16_t last_timer_call = (uint16_t) last_call_time;
    RCLC_UNUSED(last_timer_call);
    if (timer != NULL){
       move_cmd();
       publish_feedback_data();
    }
}

// microROS entity management
bool create_entities(){
    // syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    // QoS specifications    
    // creating the twist cmd subscription (default values)
    // RELIABILITY: RELIABLE
    // HISTORY: KEEP_LAST
    // DEPTH: 10
    // DURABILITY: VOLATILE 
    // LIFESPAN: 
    // LIVELINESS: 
    // LEASE DURATION: 
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default; 
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos.depth = 10;
    custom_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    custom_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;

    rwm_qos_profile_t sensor_qos = rwm_qos_profile_default;
    sensor_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sensor_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    sensor_qos.depth = 5;
    sensor_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    // publishers
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"
    ));
#ifndef USE_FAKE_MAG
    RCCHECK(rclc_publisher_init_default(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "imu/mag"
    ))
#endif

    // creating twist msg subscription
    RCCHECK(rclc_subscription_init(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel", &custom_qos
    ));

    // creating timer function to run the control and (feedback) functions
    // create timer for actuating the motors at 50 Hz (1000/20)
    uint16_t control_timeout = CONTROL_TIMER;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        control_loop
    ));
    
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twist_callback,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    setLED(HIGH);
    syncTime();

    return true;
}


bool destroy_entities(){
    // syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
#ifndef USE_FAKE_MAG
    RCSOFTCHECK(rcl_publisher_fini(&mag_publisher, &node));
#endif

    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));

    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node))
    RCSOFTCHECK(rclc_support_fini(&support));

    flashLED(1);

    return true;
}

void setup(){
    Serial.begin(BAUDRATE);
    //************************* initializing the I2C coms for getting IMU data
#ifdef BOARD_INIT // board specific setup, must include Wire.begin
    BOARD_INIT
#else
    Wire.begin(SDA_PIN, SCL_PIN);
#endif
    // ***********************************
    initLED();  
    // initPwm();

#ifdef WDT_TIMEOUT
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
#endif

    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();
    
    set_microros_net_transports(AGENT_IP, AGENT_PORT);
    flashLED(2);

    previous_odom_time = millis();
 
}


void loop() {
    uint16_t currentMillis = millis();

    switch (state){
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            //syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT){
                destroy_entities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED){
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
            //syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            full_stop();
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }

#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif

}
