// Libraries
#include <Wire.h>
#include <SimpleFOC.h> // Make sure to install SimpleFOC library

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(11, 3, 10,9, 6, 5);

// PID controller gains
float P = 5;  // Proportional gain
float I = 2;  // Integral gain
float D = 0.1;  // Derivative gain

// Target position (in radians)
float target_position = 0.0;

// Haptic feedback parameters
int num_detents = 6;      // Number of detents per rotation
float detent_strength = 10; // Strength of detent feedback

void setup() {
  // Initialize serial monitor
  Serial.begin(9600);

  // Initialize AS5600 encoder
  Wire.begin();
  Wire.setClock(400000);
  // initialise magnetic sensor hardware
  sensor.init();


  // Debug: Check if encoder is working


  // Initialize motor driver
  driver.voltage_power_supply = 5; // Supply voltage in volts
  if (!driver.init()) {
    Serial.println("Error: Driver initialization failed!");
    while (1); // Halt execution
  }

  // Initialize motor
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::angle;
  motor.PID_velocity.P = P;
  motor.PID_velocity.I = I;
  motor.PID_velocity.D = D;
  motor.P_angle.P = 20;
  // default 20
  motor.velocity_limit = 4;
  // default voltage_power_supply
  motor.voltage_limit = 5;

  motor.linkSensor(&sensor);
  motor.useMonitoring(Serial);
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  // align sensor and start FOC
  
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }
  Serial.println("Motor ready.");
  delay(100);
  Serial.println("Motor and encoder initialized.");
  Serial.println("Send 'd:<num_detents>' to set detents, 's:<strength>' to set detent strength.");
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

  // Calculate detent position
  float detent_interval = TWO_PI / num_detents;
  float nearest_detent = round(current_position / detent_interval) * detent_interval;
  float detent_error = nearest_detent - current_position;

  // Apply haptic feedback as a virtual spring
  float haptic_torque = detent_strength * detent_error;

  // Debugging: Print encoder and detent data
  Serial.print("Current position: ");
  Serial.print(current_position);
  Serial.print(" | Nearest detent: ");
  Serial.print(nearest_detent);
  Serial.print(" | Detent error: ");
  Serial.print(detent_error);
  Serial.print(" | Haptic torque: ");
  Serial.println(haptic_torque);

  // Update motor position control with haptic feedback
  motor.loopFOC();
  motor.move(target_position - current_position + haptic_torque);

  delay(10); // Short delay for loop stability
}

// Function to update target position and haptic parameters from serial input
void serialEvent() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    if (input.startsWith("d:")) {
      num_detents = input.substring(2).toInt();
      Serial.print("Number of detents updated to: ");
      Serial.println(num_detents);
    } else if (input.startsWith("s:")) {
      detent_strength = input.substring(2).toFloat();
      Serial.print("Detent strength updated to: ");
      Serial.println(detent_strength);
    } else {
      target_position = input.toFloat();
      Serial.print("New target position: ");
      Serial.println(target_position);
    }
  }
}