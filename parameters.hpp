// Motor Pins
#define MOT1PWM 9       // PIN 9 is a PWM pin
#define MOT1DIR 10  
#define MOT2PWM 11      // PIN 11 is a pwm pin
#define MOT2DIR 12

// Enable Pins
#define EN_1_A 2        // These are the pins for the PCB encoder
#define EN_1_B 7        // These are the pins for the PCB encoder
#define EN_2_A 3        // These are the pins for the PCB encoder
#define EN_2_B 8        // These are the pins for the PCB encoder

// Lidar Pins
#define LIDAR1_PIN A0   // TOF1GP0
#define LIDAR2_PIN A1   // TOF2GP0
#define LIDAR3_PIN A2   // TOF3GP0

// Odometry Controls
#define radiusLength 16  
#define axleLength 98.5  
#define encoderCountsPerRev 700

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PIDControl ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// FASTER MOVE FORWARD
float kp_encoder = 7;  // Proportional gain
float ki_encoder = 14;  // Integral gain
float kd_encoder = 7;   // Derivative gain
float tolerance = 2; // Tolerance for PID 

// float kp_encoder = 3;  // Proportional gain
// float ki_encoder = 1;  // Integral gain
// float kd_encoder = 0.3;   // Derivative gain
// float tolerance = 0.3; // Tolerance for PID 

float rkp_encoder = 6;  // Proportional gain
float rki_encoder = 18;  // Integral gain
float rkd_encoder = 14;   // Derivative gain
float rtolerance = 0.23; // Tolerance for PID 

int settletimeforwardblock = 700;
int settletimeforwardloop = 700;
int settletime = 700;

// Miscellaneous
bool isMoving = false;          // if robot is moving
int commandIndex = 0;           // length loop of input keys

int lidarfrontstopping = 110;
float distWall = 77;            // Desired distance from the walls (in mm)
float test = 4;                 // kp multiplier for the size wall input LIDAR

int setCursorFirst = 10;        // OLED
int setCursorSecond = 7;        // OLED 
auto font = u8g2_font_6x12_mf;  // OLED

// Formulas for calculations
float wheelDiameter = radiusLength * 2; 
float wheelCircumference = PI * wheelDiameter;

// How far we want the robot to travel
float desiredDistance = 250;
const float THETA_90DEG = (PI * axleLength) / (radiusLength * 4);
const float FORWARD_DISTANCE = ((desiredDistance / wheelCircumference) * 2 * PI) + 0.8;

// SMALLER INCREMENTS
float desiredDistanceSmall = 80;
const float FORWARD_DISTANCE_SMALL = ((desiredDistanceSmall / wheelCircumference) * 2 * PI) ;

/*
// ORIGINAL!!!!
float kp_encoder = 3;  // Proportional gain
float ki_encoder = 1;  // Integral gain
float kd_encoder = 0.3;   // Derivative gain
float tolerance = 0.3; // Tolerance for PID 

float rkp_encoder = 9.5;  // Proportional gain
float rki_encoder = 2.5;  // Integral gain
float rkd_encoder = 26;   // Derivative gain
float rtolerance = 0.149; // Tolerance for PID 
*/

// float skp_encoder = 2;  // Proportional gain
// float ski_encoder = 10;  // Integral gain
// float skd_encoder = 0.1;   // Derivative gain
// float stolerance = 0.264; // Tolerance for PID 

float skp_encoder = 2;  // Proportional gain
float ski_encoder = 15;  // Integral gain
float skd_encoder = 8;   // Derivative gain
float stolerance = 0.30; // Tolerance for PID 
mtrn3100::PIDController leftPIDSmall(skp_encoder, ski_encoder, skd_encoder, stolerance);
mtrn3100::PIDController rightPIDSmall(skp_encoder, ski_encoder, skd_encoder, tolerance);
