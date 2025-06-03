#include "Motor.hpp"              // File for motor.setPWM() function
#include "ThreeLidar.hpp"         // File for Lidar distances
#include "DualEncoder.hpp"        // File tracking dual encoder counts
#include "EncoderOdometry.hpp"    // File for position and orientation
#include "PIDController.hpp"      // File for adjusting PID Control
#include <MPU6050_light.h>        // MPU
#include <Wire.h>                 // Talks to wire.begin() AKA I2C
#include "parameters.hpp"

MPU6050 mpu(Wire);

// Command sequence for chained movements
const char commandSequence[] = "fffff"; // Example sequence: forward, forward, right, backward, left, forward

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PIDControl ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
float kp = 9.5;  // Proportional gain
float kd = 26;   // Derivative gain
float ki = 2.5;  // Integral gain
float tolerance = 0.2; // Tolerance for PID settling

////////////////////////////////////////////////////////////////////////////////

bool isMoving = false;
int commandIndex = 0;

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INITIALISE ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
mtrn3100::Motor motor(MOT1PWM, MOT1DIR, MOT2PWM, MOT2DIR);
mtrn3100::ThreeLidar lidar(LIDAR1_PIN, LIDAR2_PIN, LIDAR3_PIN);
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(radiusLength, axleLength);
mtrn3100::PIDController leftPID(kp, ki, kd, tolerance);
mtrn3100::PIDController rightPID(kp, ki, kd, tolerance);
mtrn3100::PIDController centeringPID(kp, ki, kd, tolerance);

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN CODE /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  lidar.init();
  delay(1000);
}

void loop() {
  if (!isMoving) {
    if (commandIndex < sizeof(commandSequence) - 1) {
      char command = commandSequence[commandIndex];
      executeCommand(command);
      isMoving = true;
      commandIndex++;
    } else {
      // All commands executed
      while (1); // Stop the robot
    }
  }

  if (isMoving) {
    moveRobot();
  }
}

void executeCommand(char command) {
  switch (command) {
    case 'f':
      moveForward();
      break;
    case 'b':
      moveBackward();
      break;
    case 'r':
      turnRight();
      break;
    case 'l':
      turnLeft();
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

void moveForward() {
  encoder.l_count = 0;
  encoder.r_count = 0;
  leftPID.zeroAndSetTarget(encoder.getLeftRotation(), FORWARD_DISTANCE); 
  rightPID.zeroAndSetTarget(-encoder.getRightRotation(), FORWARD_DISTANCE); 
}

void moveBackward() {
  encoder.l_count = 0;
  encoder.r_count = 0;
  leftPID.zeroAndSetTarget(encoder.getLeftRotation(), -FORWARD_DISTANCE); 
  rightPID.zeroAndSetTarget(-encoder.getRightRotation(), -FORWARD_DISTANCE); 
}

void turnRight() {
  encoder.l_count = 0;
  encoder.r_count = 0;
  leftPID.zeroAndSetTarget(encoder.getLeftRotation(), THETA_90DEG); 
  rightPID.zeroAndSetTarget(-encoder.getRightRotation(), -THETA_90DEG); 
}

void turnLeft() {
  encoder.l_count = 0;
  encoder.r_count = 0;
  leftPID.zeroAndSetTarget(encoder.getLeftRotation(), -THETA_90DEG); 
  rightPID.zeroAndSetTarget(-encoder.getRightRotation(), THETA_90DEG); 
}

void moveRobot() {
  // Update encoder readings
 
     delay(5);                       
  float leftCounts = encoder.getLeftRotation();
  float rightCounts = -encoder.getRightRotation();  
  encoder_odometry.update(leftCounts, rightCounts);

  // Compute control outputs from PID controllers
  float leftControl = leftPID.compute(leftCounts);
  float rightControl = rightPID.compute(rightCounts);

  float controlDifference = leftCounts - rightCounts;
    float adjustedLeftControl = leftControl - controlDifference;
    float adjustedRightControl = rightControl + controlDifference;
    // Set motor speeds
    motor.setPWMLeft(adjustedLeftControl);
    motor.setPWMRight(adjustedRightControl);

  // Check if the robot has reached the desired distance using isSettled method
  if (leftPID.isSettled() && rightPID.isSettled()) {
    motor.setPWMLeft(0);
    motor.setPWMRight(0);
    Serial.println("Target distance reached.");
    isMoving = false;
    delay(200); // Short delay before the next command
  }
}

