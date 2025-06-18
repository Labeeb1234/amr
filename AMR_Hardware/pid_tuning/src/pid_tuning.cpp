#include <Arduino.h>
#include <string.h>
#include <i2cdetect.h>
#include <Wire.h>

#include "led.h"
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

// watch dog timers for resetting esp32
#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif


#define BAUDRATE 115200
#define CONTROL_TIMER 20 //ms


// PID tuning params
static volatile int32_t loop_start = -1; 

float Kp = 5.0;
float Ki = 0.05;
float Kd = 2.0;

int total_motors = 4;


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

// setting up kinematics of the system
Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    WHEELS_DISTANCE_DIFF
);

// a helper function for periodic execution
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int32_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// ************************************* FILTERS IF ANY ***********************************
float prev_motor_rpm1=0.0, pprev_motor_rpm1=0.0, prev_motor_rpm2=0.0, pprev_motor_rpm2=0.0;
float prev_motor_rpm3=0.0, pprev_motor_rpm3=0.0, prev_motor_rpm4=0.0, pprev_motor_rpm4=0.0;
float alpha = 0.2;
float deadband_threshold = 5.0;

float emea_filter(float& curr_val, float& prev_val){
    curr_val = alpha*curr_val + (1-alpha)*prev_val;
    prev_val = curr_val;
    return curr_val;
}
float running_avg(float& curr_val, float& prev_val, float& prev_prev_val){
  curr_val = (curr_val + prev_val + prev_prev_val)/3;
  prev_val = curr_val;
  prev_prev_val = prev_val;
  return curr_val;
}
//***********************************************************************

// pid runner logic
void pid_loop(){
    static unsigned long last_time = 0;
    unsigned long now = uxr_millis();
    // Serial.print(">uxr_millis: ");
    // Serial.print(now);
    Serial.print(">delta:");
    Serial.println(now - last_time);
    last_time = now;

    // runing tuning loop at 20ms -- 50Hz same as the main bot loop code for microROS interfacing and data transmission
    // float max_rpm = kinematics.getMaxRPM();
    Kinematics::rpm desired_motor_vel = kinematics.getRPM(0.5, 0.0, 0.0); // + forward motion
    // get RPM from the processed encoder feedback
    float current_rpm1 = motor1_encoder.getRPM();
    current_rpm1 = emea_filter(current_rpm1, prev_motor_rpm1);
    // current_rpm1 = running_avg(current_rpm1, prev_motor_rpm1, pprev_motor_rpm1);
    
    float current_rpm2 = motor2_encoder.getRPM();
    current_rpm2 = emea_filter(current_rpm2, prev_motor_rpm2);
    // current_rpm2 = running_avg(current_rpm2, prev_motor_rpm2, pprev_motor_rpm2);
    
    float current_rpm3 = motor3_encoder.getRPM();
    current_rpm3 = emea_filter(current_rpm3, prev_motor_rpm3);
    // current_rpm3 = running_avg(current_rpm3, prev_motor_rpm3, pprev_motor_rpm3);
    
    float current_rpm4 = motor4_encoder.getRPM();
    current_rpm4 = emea_filter(current_rpm4, prev_motor_rpm4);
    // current_rpm4 = running_avg(current_rpm4, prev_motor_rpm4, pprev_motor_rpm4);

    // applying deadband to the motor rpms (for no-load condition it helps (claims to help))
    // if (abs(desired_motor_vel.motor1 - current_rpm1) < deadband_threshold)
    //     current_rpm1 = desired_motor_vel.motor1;

    // if (abs(desired_motor_vel.motor2 - current_rpm2) < deadband_threshold)
    //     current_rpm2 = desired_motor_vel.motor2;

    // if (abs(desired_motor_vel.motor3 - current_rpm3) < deadband_threshold)
    //     current_rpm3 = desired_motor_vel.motor3;

    // if (abs(desired_motor_vel.motor4 - current_rpm4) < deadband_threshold)
    //     current_rpm4 = desired_motor_vel.motor4;

    // set motor speed based on above bot frame vel
    motor1_controller.spin(motor1_pid.compute(desired_motor_vel.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(desired_motor_vel.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(desired_motor_vel.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(desired_motor_vel.motor4, current_rpm4));


    // // for plots
    Serial.print(">Desired RPM:");
    Serial.println(desired_motor_vel.motor1);
    
    // Serial.print(">Filtered Motor-1 RPM:");
    // Serial.println(current_rpm1);
    Serial.print(">Filtered Motor-2 RPM:");
    Serial.println(current_rpm2);
    // Serial.print(">Filtered Motor-3 RPM:");
    // Serial.println(current_rpm3);
    // Serial.print(">Filtered Motor-4 RPM:");
    // Serial.println(current_rpm4);
    
}

void setup(){
    Serial.begin(BAUDRATE);
    initLed();

// #ifdef WDT_TIMEOUT
//     esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
//     esp_task_wdt_add(NULL); //add current thread to WDT watch
// #endif

    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();

    
    setLed(1);
    delay(1000);
    setLed(0);

    // loop_start = millis();
}

void loop(){

    // if(millis()-loop_start > 30000){
    //     Kp++;
    //     motor1_pid.updateConstants(Kp, Ki, Kd);
    //     motor2_pid.updateConstants(Kp, Ki, Kd);
    //     motor3_pid.updateConstants(Kp, Ki, Kd);
    //     motor4_pid.updateConstants(Kp, Ki, Kd);
    //     Serial.print(">Kp:");
    //     Serial.println(Kp);
        
    //     delay(5000);
    //     esp_restart();
    // }

    EXECUTE_EVERY_N_MS(CONTROL_TIMER, pid_loop());

// #ifdef WDT_TIMEOUT
//     esp_task_wdt_reset();
// #endif

}


