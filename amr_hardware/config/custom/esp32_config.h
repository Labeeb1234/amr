#ifndef ESP32_CONFIG_H
#define ESP32_CONFIG_H

#define LED_PIN 2 //used for debugging status (using external LED)

//uncomment the base you're building
//#define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER            // 4WD robot
#define LINO_BASE MECANUM                // Mecanum drive robot
#define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
//uncomment the IMU you're using
// #define USE_MPU6050_IMU
#define USE_MPU9250_IMU

// imu covariances
#define ACCEL_COV { 0.01, 0.01, 0.01 }
#define GYRO_COV { 0.001, 0.001, 0.001 }
#define ORI_COV { 0.01, 0.01, 0.01 }
#define MAG_COV { 1e-12, 1e-12, 1e-12 }
// encoder's initial covariance
#define POSE_COV { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }
#define TWIST_COV { 0.001, 0.001, 0.001, 0.003, 0.003, 0.003 }

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
    MOTOR2  MOTOR3  (4W-90DEG-OMNI......yet to add!)
         MOTOR4  
         BACK
*/

//define your robot' specs here
#define MOTOR_MAX_RPM 256                   // motor's max RPM
#define MAX_RPM_RATIO 0.85                // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
// #define COUNTS_PER_REV1 268.8                 // wheel1 encoder's no of ticks per rev
// #define COUNTS_PER_REV2 268.8                 // wheel2 encoder's no of ticks per rev
// #define COUNTS_PER_REV3 268.8                // wheel3 encoder's no of ticks per rev
// #define COUNTS_PER_REV4 268.8                 // wheel4 encoder's no of ticks per rev
#define COUNTS_PER_REV1 340 //134.4              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 340 //134.4                 // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 340   // 134.4                // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 340 // 134.4 
#define WHEEL_DIAMETER 0.152                    // wheel's diameter in meters
#define WHEELS_DISTANCE_DIFF 1.12           // distance between left and right wheels
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

// for wireless microROS communication
// #define AGENT_IP {192,168,136,77} // eg IP of the desktop computer
// #define AGENT_PORT 8888

// Enable WiFi with null terminated list of multiple APs SSID and password
#define WIFI_AP_LIST {{"inflab", "infinity@123"}, {NULL}}
#define WIFI_MONITOR 2 // min. period to send wifi signal strength to syslog
#define USE_ARDUINO_OTA
#define USE_SYSLOG
#define SYSLOG_SERVER {192, 168, 1, 77}  // eg IP of the desktop computer
#define SYSLOG_PORT 514

#define DEVICE_HOSTNAME "esp32"
#define APP_NAME "amr_ust_lower"
// #define BAUDRATE 921600
#define BAUDRATE 115200
// #define BAUDRATE 9600
#define SDA_PIN 21 // specify I2C pins
#define SCL_PIN 22
#define NODE_NAME "mecanum_node"
// #define TOPIC_PREFIX "esp32/"
#define CONTROL_TIMER 20
// #define BATTERY_TIMER 2000

// battery voltage ADC pin
//#define BATTERY_PIN 33
// 3.3V ref, 12 bits ADC, 33k + 10k voltage divider
// #define USE_ADC_LUT
// #ifdef USE_ADC_LUT
// const int16_t ADC_LUT[4096] = { /* insert adc_calibrate data here */ };
// #define BATTERY_ADJUST(v) (ADC_LUT[v] * (3.3 / 4096 * (33 + 10) / 10 * 1.0))
// #else
// #define BATTERY_ADJUST(v) ((v) * (3.3 / 4096 * (33 + 10) / 10))
// #endif
// // #define USE_INA219
// #define BATTERY_DIP 0.98  // battery voltage drop alert
// // #define BATTERY_CAP 2.0  // battery capacity Ah
// // #define BATTERY_MIN 9.0  // battery minimal voltage
// // #define BATTERY_MAX 12.6 // battery maximum voltage
// // #define TRIG_PIN 31 // ultrasonic sensor HC-SR04
// // #define ECHO_PIN 32
// #define USE_SHORT_BRAKE // for shorter stopping distance
// #define WDT_TIMEOUT 60 // Sec
#define BOARD_INIT {\ 
    Wire.begin(SDA_PIN, SCL_PIN); \ 
    Wire.setClock(400000); \ 
}
// #define BOARD_INIT_LATE {}
// #define BOARD_LOOP {}

#ifdef USE_SYSLOG
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    syslog(LOG_ERR, "%s RCCHECK failed %d", __FUNCTION__, temp_rc); \
    return false; }}
#else
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    flashLED(3); \
    return false; }} // do not block
#endif

#endif
