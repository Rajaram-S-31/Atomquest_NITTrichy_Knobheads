// Libraries
#include <Wire.h>
#include <AS5600.h>
#include <SimpleFOC.h> // Make sure to install SimpleFOC library

// Create AS5600 magnetic encoder object
AS5600 encoder;

// Motor driver pins (modify as per your setup)
const int PWM_UH = 3;  // High-side PWM for phase U
const int PWM_UL = 9;  // Low-side PWM for phase U
const int PWM_VH = 5;  // High-side PWM for phase V
const int PWM_VL = 10; // Low-side PWM for phase V
const int PWM_WH = 6;  // High-side PWM for phase W
const int PWM_WL = 11; // Low-side PWM for phase W

// BLDC motor and driver setup
BLDCMotor motor = BLDCMotor(7); // Number of pole pairs
BLDCDriver6PWM driver = BLDCDriver6PWM(PWM_UH, PWM_UL, PWM_VH, PWM_VL, PWM_WH, PWM_WL);

// PID controller gains
float P = 2.0;  // Proportional gain
float I = 0.5;  // Integral gain
float D = 0.1;  // Derivative gain

// Target position (in radians)
float target_position = 0.0;

void setup() {
  // Initialize serial monitor
  Serial.begin(9600);

  // Initialize AS5600 encoder
  Wire.begin();
  encoder.begin();

  // Initialize motor driver
  driver.voltage_power_supply = 12; // Supply voltage in volts
  driver.init();

  // Initialize motor
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::angle;
  motor.PID_velocity.P = P;
  motor.PID_velocity.I = I;
  motor.PID_velocity.D = D;
  motor.voltage_limit = 6;    // Voltage limit for control
  motor.velocity_limit = 50; // Max velocity limit in rad/s

  motor.init();
  motor.initFOC();

  Serial.println("Motor and encoder initialized.");
}

void loop() {
  // Manually read raw angle via I2C
  Wire.beginTransmission(0x36); // AS5600 I2C address
  Wire.write(0x0E); // Address of the raw angle (MSB)
  Wire.endTransmission(false); // Request repeated start
  Wire.requestFrom(0x36, 2); // Read 2 bytes (MSB + LSB)

  int rawAngle = 0;
  if (Wire.available() == 2) {
    rawAngle = (Wire.read() << 8) | Wire.read(); // Combine MSB and LSB
  }

  // Convert raw angle to radians (assuming 12-bit resolution, 0-4095)
  float current_position = (rawAngle / 4096.0) * TWO_PI;

  // Update motor position control
  motor.loopFOC();
  motor.move(target_position - current_position);

  // Debugging: Print current position
  Serial.print("Current position: ");
  Serial.println(current_position);

  delay(10); // Short delay for loop stability
}

// Function to update target position from serial input (optional)
void serialEvent() {
  if (Serial.available()) {
    target_position = Serial.parseFloat();
    Serial.print("New target position: ");
    Serial.println(target_position);
  }
}
