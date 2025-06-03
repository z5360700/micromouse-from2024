#include <U8g2lib.h>
#include "Motor.hpp"              // File for motor.setPWM() function
#include "ThreeLidar.hpp"         // File for Lidar distances
#include "DualEncoder.hpp"        // File tracking dual encoder counts
#include "EncoderOdometry.hpp"    // File for position and orientation
#include "PIDController.hpp"      // File for adjusting PID Control
#include <MPU6050_light.h>        // MPU
#include <Wire.h>                 // Talks to wire.begin() AKA I2C
#include "parameters.hpp"

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INITIALISE ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
mtrn3100::Motor motor(MOT1PWM, MOT1DIR, MOT2PWM, MOT2DIR);
mtrn3100::ThreeLidar lidar(LIDAR1_PIN, LIDAR2_PIN, LIDAR3_PIN);
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(radiusLength, axleLength);
mtrn3100::PIDController leftPID(kp_encoder, ki_encoder, kd_encoder, tolerance);
mtrn3100::PIDController rightPID(kp_encoder, ki_encoder, kd_encoder, tolerance);
mtrn3100::PIDController RotateLeftPID(rkp_encoder, rki_encoder, rkd_encoder, rtolerance);
mtrn3100::PIDController RotateRightPID(rkp_encoder, rki_encoder, rkd_encoder, rtolerance);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // Using hardware I2C

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN CODE /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  lidar.init();

  u8g2.begin();
  u8g2.clearBuffer(); // Clear the internal memory
  u8g2.setFont(font); // Choose a suitable font
  u8g2.setCursor(setCursorFirst, setCursorSecond);
  u8g2.print("Staring!!!"); // Write a string to the display
  u8g2.sendBuffer(); // Transfer internal memory to the display
  delay(1000);
}

char command; // Declare command as a global variable
bool after = 0 ;

void loop() {
  if (!isMoving) {
    // Read sensor data
    displayMessage("scanning lidar");
    delay(500);

    int distanceRight = lidar.readRightSensor();
    int distanceForward = lidar.readForwardSensor();
    delay(1000);

    // if (distanceForward < 120) {
    //   command = 'l';  // Obstacle detected in front, turn left (to avoid the obstacle)
    // } else if (distanceRight < 120) {
    //   command = 'f';  // Wall detected on the right, move forward
    // } else {
    //   command = 'r';  // No wall on the right, turn right to follow the wall
    // }



    // if (distanceRight > 120) {
    //   command = 'r';
    // }
    // else if ((distanceRight < 120) && (distanceForward > 150)) {
    //   command = 'f';
    // }
    // else if (distanceRight > 150) {
    //   command = 'r';
    // }
    // else if ((distanceRight < 100) && (distanceForward < 100)) {
    //   command = 'l';
    // }


    // if (distanceForward < 120 && distanceRight < 120) {
    //   command = 'l';  // Obstacle detected in front, turn right
    // } else if (distanceRight < 120) {
    //   command = 'f';  // Wall detected on the right, move forward
    // } else {
    //   command = 'r';  // No wall on the right, turn right to follow the wall
    // }

// if (distanceForward > 120) {
//   command = 'f';  // Path ahead is clear, move forward
// } else if (distanceRight > 120) {
//   command = 'r';  // No wall on the right, turn right to follow the wall
// } else if ((distanceRight < 120) && (distanceForward < 120)) {
//   command = 'l';  // Obstacle detected both on the right and in front, turn left
// }



if (after == 1){
  command = 'f';
  after = 0;
} else if (distanceRight > 200) {
  command = 'r';  // No wall on the right, turn right to follow the wall
  after = 1;
} else if (distanceForward > 200) {
  command = 'f';  // Path ahead is clear, move forward
} else if ((distanceRight < 120) && (distanceForward < 120)) {
  command = 'l';  // Obstacle detected both on the right and in front, turn left
}


    // Execute the determined command
    executeCommand(command);
    isMoving = true;
  }

  if (isMoving) {
    // Continue the current movement based on the command
    if (command == 'f') {
      moveRobotForward();
    } else if (command == 'r' || command == 'l') {
      moveRobotRotate();
    } else if (command == 's') {
      moveRobotForwardSmall();
    }
  }
}


void executeCommand(char command) {
  switch (command) {
    case 'f':
      displayMessage("Going Forward");
      moveForward();
      break;
    case 'r':
      displayMessage("Turning Right");
      turnRight();
      break;
    case 'l':
      displayMessage("Turning Left");
      turnLeft();
      break;
    case 's':
      displayMessage("Small increment");
      moveForwardSmall();
      break;
    default:
      displayMessage("Invalid Command");
      Serial.println("Invalid Command");
      break;
  }
}

void displayMessage(const char* message) {
  u8g2.clearBuffer(); // Clear the internal memory
  u8g2.setCursor(setCursorFirst, setCursorSecond); // Set the cursor to the start position
  u8g2.print(message); // Print the message
  u8g2.sendBuffer(); // Transfer internal memory to the display
}

void moveForward() {
  encoder.l_count = 0;
  encoder.r_count = 0;
  leftPID.zeroAndSetTarget(encoder.getLeftRotation(), FORWARD_DISTANCE); 
  rightPID.zeroAndSetTarget(encoder.getRightRotation(), FORWARD_DISTANCE); 
}

void turnRight() {
  encoder.l_count = 0;
  encoder.r_count = 0;
  RotateLeftPID.zeroAndSetTarget(encoder.getLeftRotation(), THETA_90DEG); 
  RotateRightPID.zeroAndSetTarget(-encoder.getRightRotation(), -THETA_90DEG); 
}

void turnLeft() {
  encoder.l_count = 0;
  encoder.r_count = 0;
  RotateLeftPID.zeroAndSetTarget(-encoder.getLeftRotation(), -THETA_90DEG); 
  RotateRightPID.zeroAndSetTarget(encoder.getRightRotation(), THETA_90DEG); 
}

void moveForwardSmall() {
  encoder.l_count = 0;
  encoder.r_count = 0;
  leftPIDSmall.zeroAndSetTarget(encoder.getLeftRotation(), FORWARD_DISTANCE_SMALL); 
  rightPIDSmall.zeroAndSetTarget(encoder.getRightRotation(), FORWARD_DISTANCE_SMALL); 
}

void moveRobotForward() {
  // Reset iterationCount for each new 'f' command
  static int iterationCount = 0;
  iterationCount = 0;

  while (true) {
    int distanceLeft = lidar.readLeftSensor();
    int distanceForward = lidar.readForwardSensor();
    int distanceRight = lidar.readRightSensor();

    float leftCounts = encoder.getLeftRotation();
    float rightCounts = encoder.getRightRotation();  
    encoder_odometry.update(leftCounts, rightCounts);

    // Compute control outputs from PID controllers
    float leftControl = leftPID.compute(leftCounts);
    float rightControl = rightPID.compute(rightCounts);

    // Wall following logic
    float wallFollowAdjustmentLeft = 0;
    float wallFollowAdjustmentRight = 0;

    // Adjust motor speeds based on LiDAR readings
    if (distanceLeft < distWall) {
      wallFollowAdjustmentLeft = test * (distWall - distanceLeft);
    }

    if (distanceRight < distWall) {
      wallFollowAdjustmentRight = test * (distWall - distanceRight);
    }

    float controlDifference = leftCounts - rightCounts;

    // Introduce a scaling factor for the control outputs over 20 iterations
    if (iterationCount < 20) {
      float scalingFactor = 0.3 + 0.7 * iterationCount / 19; // Scale from 30% to 100%
      leftControl *= scalingFactor;
      rightControl *= scalingFactor;
      iterationCount++;
    }

    float adjustedLeftControl = leftControl - controlDifference + wallFollowAdjustmentLeft;
    float adjustedRightControl = rightControl + controlDifference + wallFollowAdjustmentRight;

    motor.setPWMLeft(adjustedLeftControl);
    motor.setPWMRight(adjustedRightControl);

    if (distanceForward < lidarfrontstopping) {
      motor.setPWMLeft(0);
      motor.setPWMRight(0);
      Serial.println("---FRONT LIDAR: Target has front block---");
    
      u8g2.clearBuffer(); // Clear the internal memory
      u8g2.setCursor(setCursorFirst, setCursorSecond); 
      u8g2.print("!!! FRONT BLOCK"); // Print the message
      u8g2.sendBuffer(); // Transfer internal memory to the display

      isMoving = false;
      delay(settletimeforwardblock);
      break;
    }

    // Check if the robot has reached the desired distance using isSettled method
    if (leftPID.isSettled() && rightPID.isSettled()) {
      motor.setPWMLeft(0);
      motor.setPWMRight(0);
      Serial.println("Target distance reached: FORWARDS.");
      isMoving = false;
      delay(settletimeforwardloop); // Short delay before the next command
      break;
    }

    delay(5);                       
  }
}

void moveRobotBackward() {
  int distanceLeft = lidar.readLeftSensor();
  int distanceRight = lidar.readRightSensor();

  float leftCounts = encoder.getLeftRotation();
  float rightCounts = encoder.getRightRotation();  
  encoder_odometry.update(leftCounts, rightCounts);

  // Compute control outputs from PID controllers
  float leftControl = leftPID.compute(leftCounts);
  float rightControl = rightPID.compute(rightCounts);

  // Wall following logic
  float wallFollowAdjustmentLeft = 0;
  float wallFollowAdjustmentRight = 0;

  test = 2;

  //Adjust motor speeds based on LiDAR readings
  if (distanceLeft < distWall) {
    wallFollowAdjustmentLeft = test * (distWall - distanceLeft);
  }

  if (distanceRight < distWall) {
    wallFollowAdjustmentRight = test * (distWall - distanceRight);
  }

  // switched controldifference
  float controlDifference = leftCounts - rightCounts;
  float adjustedLeftControl = leftControl + controlDifference - wallFollowAdjustmentLeft;
  float adjustedRightControl = rightControl - controlDifference - wallFollowAdjustmentRight;

  motor.setPWMLeft(adjustedLeftControl);
  motor.setPWMRight(adjustedRightControl);

  // Check if the robot has reached the desired distance using isSettled method
  if (leftPID.isSettled() && rightPID.isSettled()) {
    motor.setPWMLeft(0);
    motor.setPWMRight(0);
    Serial.println("Target distance reached: BACKWARD.");
    isMoving = false;
    delay(settletime); // Short delay before the next command
  }

  delay(5);                       
}

void moveRobotRotate() {
  float leftCounts1 = encoder.getLeftRotation();
  float rightCounts1 = encoder.getRightRotation();  
  encoder_odometry.update(leftCounts1, rightCounts1);

  // Compute control outputs from PID controllers
  float leftControl1 = RotateLeftPID.compute(leftCounts1);
  float rightControl1 = RotateRightPID.compute(rightCounts1);

  float controlDifference1 = abs(leftCounts1) - abs(rightCounts1);
  float adjustedLeftControl1 = leftControl1 + abs(controlDifference1);
  float adjustedRightControl1 = rightControl1 + abs(controlDifference1);
  
  //Set motor speeds
  motor.setPWMLeft(adjustedLeftControl1);
  motor.setPWMRight(adjustedRightControl1);

  // Check if the robot has reached the desired distance using isSettled method
  if (RotateLeftPID.isSettled() && RotateLeftPID.isSettled()) {
    motor.setPWMLeft(0);
    motor.setPWMRight(0);
    Serial.println("Target distance reached: ROTATION.");
    isMoving = false;
    delay(settletime); // Short delay before the next command
  }

  delay(5);                       
}

void moveRobotForwardSmall() {
  while (true) {
    int distanceLeft = lidar.readLeftSensor();
    int distanceForward = lidar.readForwardSensor();
    int distanceRight = lidar.readRightSensor();

    float leftCounts = encoder.getLeftRotation();
    float rightCounts = encoder.getRightRotation();  
    encoder_odometry.update(leftCounts, rightCounts);

    // Compute control outputs from PID controllers
    float leftControl = leftPIDSmall.compute(leftCounts);
    float rightControl = rightPIDSmall.compute(rightCounts);

    // Wall following logic
    float wallFollowAdjustmentLeft = 0;
    float wallFollowAdjustmentRight = 0;

    // Adjust motor speeds based on LiDAR readings
    if (distanceLeft < 50) {
      wallFollowAdjustmentLeft = test * (distWall - distanceLeft);
    }

    if (distanceRight < 50) {
      wallFollowAdjustmentRight = test * (distWall - distanceRight);
    }

    float controlDifference = leftCounts - rightCounts;

    float adjustedLeftControl = leftControl - controlDifference + wallFollowAdjustmentLeft;
    float adjustedRightControl = rightControl + controlDifference + wallFollowAdjustmentRight;



    motor.setPWMLeft(45 + wallFollowAdjustmentLeft);
    motor.setPWMRight(45.35 + wallFollowAdjustmentRight);

    if (distanceForward < lidarfrontstopping) {
      motor.setPWMLeft(0);
      motor.setPWMRight(0);
      Serial.println("---FRONT LIDAR: Target distance reached---");
    
      u8g2.clearBuffer(); // Clear the internal memory
      u8g2.setCursor(setCursorFirst, setCursorSecond); // Set the cursor to the start position
      u8g2.print("!!! FRONT BLOCK"); // Print the message
      u8g2.sendBuffer(); // Transfer internal memory to the display

      isMoving = false;
      delay(settletimeforwardblock);
      break;
    }

    // Check if the robot has reached the desired distance using isSettled method
    if (leftPIDSmall.isSettled() && rightPIDSmall.isSettled()) {
      motor.setPWMLeft(0);
      motor.setPWMRight(0);
      Serial.println("Target distance reached: FORWARDS.");
      isMoving = false;
      delay(settletimeforwardloop); // Short delay before the next command
      break;
    }

    delay(5);                       
  }
}