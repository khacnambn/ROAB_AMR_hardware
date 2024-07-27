#include <Arduino.h>
#include <stdio.h>

// #include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#define MOTOR_MAX_RPM 150               // motor's max RPM
#define MAX_RPM_RATIO 0.85              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 12      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12 // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 450             // wheel1 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.0560           // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.224        // distance between left and right wheels
#define PWM_BITS 10                     // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000             // PWM Frequency

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

#define MOTOR1_ENCODER_A 1
#define MOTOR1_ENCODER_B 2

#define MOTOR1_PWM 39
#define MOTOR1_IN_A 37
#define MOTOR1_IN_B 35
#define LED_PIN LED_BUILTIN

#define K_P 0 // P constant
#define K_I 0 // I constant
#define K_D 0 // D constant

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

unsigned long prev_cmd_time = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  // giả mô phỏng
  motor1_controller.brake();
}

void loop()
{
  moveBase();
  delay(20); // Control loop at 50 Hz
}

void moveBase()
{
  // Example command for testing PID
  float desired_rpm = 100.0; // Example desired RPM

  // Get the current RPM from encoder
  float current_rpm = motor1_encoder.getRPM();

  // Calculate the PWM using PID controller
  float pwm_value = motor1_pid.compute(desired_rpm, current_rpm);

  // Apply the PWM to the motor
  motor1_controller.spin(pwm_value);

  // Output the values for debugging
  Serial.print(">Desired RPM: ");
  Serial.print(desired_rpm);
  Serial.print(">Current RPM: ");
  Serial.print(current_rpm);
  Serial.print("PWM value: ");
  Serial.println(pwm_value);
}

void rclErrorLoop()
{
  while (true)
  {
    flashLED(2);
  }
}

void flashLED(int n_times)
{
  for (int i = 0; i < n_times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  delay(1000);
}
