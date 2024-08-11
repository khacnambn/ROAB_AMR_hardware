// This is ESP32 S3 config 
#ifndef ESP32S3_CONFIG_H
#define ESP32S3_CONFIG_H

#define LED_PIN 39 // used for debugging status

// uncomment the base you're building
//  #define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
//  #define LINO_BASE SKID_STEER            // 4WD robot
#define LINO_BASE MECANUM // Mecanum drive robot

// uncomment the motor driver you're using
#define USE_GENERIC_2_IN_MOTOR_DRIVER // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
// #define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
// #define USE_BTS7960_MOTOR_DRIVER        // BTS7970 Motor Driver using A4950 (<40V) module or DRV8833 (<10V)
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors

// uncomment the IMU you're using
#define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU
// #define USE_QMI8658_IMU
// #define USE_HMC5883L_MAG
// #define USE_AK8963_MAG
// #define USE_AK8975_MAG
// #define USE_AK09918_MAG
// #define USE_QMC5883L_MAG
// #define MAG_BIAS { 0, 0, 0 }

#define K_P 1.4   // P constant
#define K_I 0.1   // I constant
#define K_D 0.025 // D constant

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

// define your robot' specs here
#define MOTOR_MAX_RPM 170               // motor's max RPM
#define MAX_RPM_RATIO 0.85              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 12      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12 // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 450             // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 450             // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 450             // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 450             // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.0796           // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.25575        // distance between left and right wheels
#define PWM_BITS 10                     // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000             // PWM Frequency

// INVERT ENCODER COUNTS
#define MOTOR4_ENCODER_INV false
#define MOTOR3_ENCODER_INV true
#define MOTOR1_ENCODER_INV true
#define MOTOR2_ENCODER_INV false

// INVERT MOTOR DIRECTIONS
#define MOTOR4_INV false
#define MOTOR3_INV false
#define MOTOR1_INV true
#define MOTOR2_INV true

// ENCODER PINS
#define MOTOR4_ENCODER_A 47
#define MOTOR4_ENCODER_B 48

#define MOTOR3_ENCODER_A 36
#define MOTOR3_ENCODER_B 37

#define MOTOR1_ENCODER_A 38
#define MOTOR1_ENCODER_B 40

#define MOTOR2_ENCODER_A 41
#define MOTOR2_ENCODER_B 42


// MOTOR PINS
#define MOTOR4_PWM 14
#define MOTOR4_IN_A 13
#define MOTOR4_IN_B 12

#define MOTOR3_PWM 9
#define MOTOR3_IN_A 11
#define MOTOR3_IN_B 10

#define MOTOR1_PWM 18
#define MOTOR1_IN_A 17
#define MOTOR1_IN_B 16

#define MOTOR2_PWM 6
#define MOTOR2_IN_A 15
#define MOTOR2_IN_B 7


#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

// #define AGENT_IP     \
//   {                  \
//     192, 168, 1, 100 \
//   } // eg IP of the desktop computer
// #define AGENT_PORT 8888
// #define WIFI_SSID "WIFI_SSID"
// #define WIFI_PASSWORD "WIFI_PASSWORD"
// // Enable WiFi with null terminated list of multiple APs SSID and password
// // #define WIFI_AP_LIST {{"WIFI_SSID", "WIFI_PASSWORD"}, {NULL}}
// #define WIFI_MONITOR 2 // min. period to send wifi signal strength to syslog
// // #define USE_ARDUINO_OTA
// // #define USE_SYSLOG
// #define SYSLOG_SERVER \
//   {                   \
//     192, 168, 1, 100  \
//   } // eg IP of the desktop computer
// #define SYSLOG_PORT 514
// #define DEVICE_HOSTNAME "esp32s3"
// #define APP_NAME "hardware"



// #define USE_LIDAR_UDP
// #define LIDAR_RXD 2
// // #define LIDAR_PWM 15
// #define LIDAR_SERIAL 1 // uart number
// #define LIDAR_BAUDRATE 230400
// #define LIDAR_SERVER \
//   {                  \
//     192, 168, 1, 100 \
//   } // eg IP of the desktop computer
// #define LIDAR_PORT 8889
// #define BAUDRATE 921600


#define SDA_PIN 4 // specify I2C pins
#define SCL_PIN 5
#define NODE_NAME "esp32s3"
// #define TOPIC_PREFIX "esp32s3/"

// battery voltage ADC pin
#define BATTERY_PIN 1
// 3.3V ref, 12 bits ADC, 33k + 10k voltage divider
// #define USE_ADC_LUT
#ifdef USE_ADC_LUT
const int16_t ADC_LUT[4096] = {/* insert adc_calibrate data here */};
#define BATTERY_ADJUST(v) (ADC_LUT[v] * (3.3 / 4096 * (33 + 10) / 10 * 1.0))
#else
#define BATTERY_ADJUST(v) ((v) * (3.3 / 4096 * (33 + 10) / 10))
#endif
// #define USE_INA219
// #define TRIG_PIN 31 // ultrasonic sensor HC-SR04
// #define ECHO_PIN 32
#define USE_SHORT_BRAKE // for shorter stopping distance
// #define WDT_TIMEOUT 60 // Sec
#define BOARD_INIT                \
  {                               \
    Wire.begin(SDA_PIN, SCL_PIN); \
    Wire.setClock(400000);        \
  }
// #define BOARD_INIT_LATE {}
// #define BOARD_LOOP {}

#ifdef USE_SYSLOG
#define RCCHECK(fn)                                                   \
  {                                                                   \
    rcl_ret_t temp_rc = fn;                                           \
    if ((temp_rc != RCL_RET_OK))                                      \
    {                                                                 \
      syslog(LOG_ERR, "%s RCCHECK failed %d", __FUNCTION__, temp_rc); \
    }                                                                 \
  }
#else
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      flashLED(3);               \
    }                            \
  } // do not block
#endif

#endif
