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
const char commandSequence[] = "ffff"; // Example sequence: forward, forward, right, backward, left, forward

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PIDControl ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
float kp_encoder = 9.5;  // Proportional gain
float kd_encoder = 26;   // Derivative gain
float ki_encoder = 2.5;  // Integral gain
float tolerance = 0.135; // Tolerance for PID 
// 0.135 is good for turning, 0.2 is good for driving straight


float kp_lidar = 0;
float kd_lidar = 0;
float ki_lidar = 0;


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
mtrn3100::PIDController leftPID(kp_encoder, ki_encoder, kd_encoder, tolerance);
mtrn3100::PIDController rightPID(kp_encoder, ki_encoder, kd_encoder, tolerance);
mtrn3100::PIDController centeringPID(kp_lidar, ki_lidar, kd_lidar, tolerance);

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

  // Previous before trying to implement lidar
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


// void lidarCorrection() {

//   int lidarDistanceLeft = lidar.readLeftSensor();
//   int lidarDistanceForward = lidar.readForwardSensor();
//   int lidarDistanceRight = lidar.readRightSensor();

//   if (lidarDistanceLeft <= 100 && lidarDistanceRight <=100) {
//     int centeringError = lidarDistanceLeft- lidarDistanceRight;
//   }



// }







// MICHAEL LIDAR ATTEMPT 
// very messy rn

// void moveRobot() {
//   // Update encoder readings
//   delay(5);                       
//   float leftCounts = encoder.getLeftRotation();
//   float rightCounts = -encoder.getRightRotation();  
//   encoder_odometry.update(leftCounts, rightCounts);

//   // Compute control outputs from PID controllers
//   float leftControl = leftPID.compute(leftCounts);
//   float rightControl = rightPID.compute(rightCounts);

//   // LIDAR   
//   // Read LiDAR data for wall following
//   int distance1 = lidar.readLeftSensor();
//   int distance3 = lidar.readSensor3();
  
//   // Wall following logic
//   float distWall = 70; // Desired distance from the walls (in mm)
//   float wallFollowAdjustmentLeft = 0;
//   float wallFollowAdjustmentRight = 0;

//   float test = 0.25;

//   // Adjust motor speeds based on LiDAR readings
//   if (distance1 < distWall) {
//     wallFollowAdjustmentLeft = test * (distWall - distance1);
//     Serial.print("wallFollowAdjustmentLeft: ");
//     Serial.println(wallFollowAdjustmentLeft);
//   }

//   if (distance3 < distWall) {
//     wallFollowAdjustmentRight = test * (distWall - distance3);
//     Serial.print("wallFollowAdjustmentRight: ");
//     Serial.println(wallFollowAdjustmentRight);
//   }


//   // Adjust speeds to synchronize wheel rates
//   float controlDifference = leftCounts - rightCounts;
//   // float adjustedLeftControl = leftControl - controlDifference + wallFollowAdjustmentLeft;
//   // float adjustedRightControl = rightControl + controlDifference + wallFollowAdjustmentRight;

//   float adjustedLeftControl = leftControl;
//   float adjustedRightControl = rightControl;
//   Serial.print("adjustedLeftControl: ");
//   Serial.println(adjustedLeftControl);
//   Serial.print("adjustedRightControl: ");
//   Serial.println(adjustedRightControl);

//   // Set motor speeds
//   motor.setPWMLeft(adjustedLeftControl);
//   motor.setPWMRight(adjustedRightControl);

//   // Check if the robot has reached the desired distance using isSettled method
//   if (leftPID.isSettled() && rightPID.isSettled()) {
//     motor.setPWMLeft(0);
//     motor.setPWMRight(0);
//     Serial.println("Target distance reached.");
//     isMoving = false;
//     delay(200); // Short delay before the next command
//   }
// }