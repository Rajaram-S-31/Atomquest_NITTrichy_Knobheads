#include <SimpleFOC.h>
#include <Wire.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(11, 3, 10,9, 6, 5);



float P = 5;
float I = 2.5;
float D = 0.001;
float target_angle=0;
int num_det = 4;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  //encoder stuff
  Serial.begin(115200);

  // configure i2C
  Wire.setClock(400000);
  // initialise magnetic sensor hardware
  sensor.init();

  driver.voltage_power_supply = 5; // Supply voltage in volts
  if (!driver.init()) {
    Serial.println("Error: Driver initialization failed!");
    while (1); // Halt if driver initialization fails
  }
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::angle; // Control type

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
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(0x36); // AS5600 I2C address
  Wire.write(0x0E); // Address of the raw angle (MSB)
  Wire.endTransmission(false); // Request repeated start
  Wire.requestFrom(0x36, 2); // Read 2 bytes (MSB + LSB)

  int rawAngle = 0;
  //if (Wire.available() == 2) {
    //rawAngle = (Wire.read() << 8) | Wire.read(); // Combine MSB and LSB
  //}
  // Convert raw angle to radians (assuming 12-bit resolution, 0-4095)
  sensor.update();
  
  // display the angle and the angular velocity to the terminal
  rawAngle = sensor.getAngle();
  float current_position = (rawAngle / 4096.0) * TWO_PI;

  float detent_interval = TWO_PI / num_det;
  float nearest_det = ceil(current_position / detent_interval) * detent_interval;
  if(nearest_det==6.28){
  nearest_det=0;
target_angle = nearest_det;
}
  else{
 target_angle = nearest_det;
}
  

    // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move(target_angle);
}