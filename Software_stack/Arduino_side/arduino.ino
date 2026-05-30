// ===================== INCLUDES =====================
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

// Motor direction correction: set to +1 or -1 so that
// a positive speed_req_* means "forward" in robot frame.
const int LEFT_MOTOR_DIR  = 1;   // change to -1 if needed
const int RIGHT_MOTOR_DIR = 1;   // change to -1 if needed

// ===================== USER CONFIGURABLE CONSTANTS =====================
const float WHEEL_RADIUS_M = 0.055;     // wheel radius in meters
const long  TICKS_PER_REV  = 375.5;     // encoder counts per wheel revolution
const float WHEEL_BASE_M   = 0.395;     // distance between wheel centers (meters)
                                                                                                                                                           


const unsigned long ODOM_UPDATE_MS  = 20;   // odometry update period (50 Hz)
const unsigned long PRINT_UPDATE_MS = 100;  // print pose every 100 ms
unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT_MS = 500; // Stop after 500ms without commands
   
// Max PWM for motors
const int MAX_PWM = 255;

// ===================== PIN DEFINITIONS =====================
// ---- Encoders ----
const uint8_t LEFT_ENC_A_PIN  = 2;    // interrupt pin
const uint8_t LEFT_ENC_B_PIN  = 4;
const uint8_t RIGHT_ENC_A_PIN = 3;    // interrupt pin
const uint8_t RIGHT_ENC_B_PIN = 5;

// ---- L298N pins ----
// Left motor (connected to OUT1/OUT2 on L298N)
// ENA = PWM enable for left motor
const uint8_t L_ENA  = 9;   // PWM pin
const uint8_t L_IN1  = 7;
const uint8_t L_IN2  = 8;

// Right motor (connected to OUT3/OUT4 on L298N)
// ENB = PWM enable for right motor
const uint8_t R_ENB  = 10;   // PWM pin
const uint8_t R_IN3  = 11;
const uint8_t R_IN4  = 12;

// ===================== ODOMETRY STATE =====================
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

volatile float xPose = 0.0f;  // [m]
volatile float yPose = 0.0f;  // [m]
volatile float theta = 0.0f;  // [rad]

float distancePerTick = 0.0099;

long lastLeftTicks  = 0;
long lastRightTicks = 0;

unsigned long lastOdomUpdate  = 0;
unsigned long lastPrintUpdate = 0;

// ===================== CMD_VEL VARIABLES =====================
double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_req_right = 0;                   //Desired speed for right wheel in m/s

// ===================== ROS SERIAL =====================
ros::NodeHandle nh;
geometry_msgs::Vector3Stamped pose_msg;
// Topic name is "speed" as requested
ros::Publisher pose_pub("speed", &pose_msg);

// ===================== CMD_VEL CALLBACK =====================
//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  lastCmdTime = millis();
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(WHEEL_BASE_M/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(WHEEL_BASE_M/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)

// ===================== ENCODER ISRs =====================
// Left encoder A channel ISR
void leftEncoderA_ISR() {
  bool A = digitalRead(LEFT_ENC_A_PIN);
  bool B = digitalRead(LEFT_ENC_B_PIN);

  if (A == B) {
    leftTicks++;   // forward
  } else {
    leftTicks--;   // reverse
  }
}

// Right encoder A channel ISR
void rightEncoderA_ISR() {
  bool A = digitalRead(RIGHT_ENC_A_PIN);
  bool B = digitalRead(RIGHT_ENC_B_PIN);

  if (A == B) {
    rightTicks++;  // forward
  } else {
    rightTicks--;  // reverse
  }
}

// ===================== MOTOR CONTROL (L298N) =====================
// pwmVal in range [-MAX_PWM, +MAX_PWM]
void setLeftMotor(int pwmVal) {
  pwmVal = constrain(pwmVal, -MAX_PWM, MAX_PWM);

  if (pwmVal > 0) {
    // forward
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_ENA, pwmVal);
  } else if (pwmVal < 0) {
    // reverse
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_ENA, -pwmVal);
  } else {
    // stop (coast)
    analogWrite(L_ENA, 0);
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
  }
}

void setRightMotor(int pwmVal) {
  pwmVal = constrain(pwmVal, -MAX_PWM, MAX_PWM);

  if (pwmVal > 0) {
    // forward
    digitalWrite(R_IN3, HIGH);
    digitalWrite(R_IN4, LOW);
    analogWrite(R_ENB, pwmVal);
  } else if (pwmVal < 0) {
    // reverse
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, HIGH);
    analogWrite(R_ENB, -pwmVal);
  } else {
    // stop (coast)
    analogWrite(R_ENB, 0);
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, LOW);
  }
}

// Optional helper to set both motors
void setMotors(int leftPWM, int rightPWM) {
  setLeftMotor(leftPWM);
  setRightMotor(rightPWM);
}
// Add after the motor control functions
void executeMotorCommands() {
  // Timeout safety
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    setLeftMotor(0);
    setRightMotor(0);
    return;
  }

  const float MAX_SPEED = 0.4f;  // m/s (your max linear wheel speed)

  // Convert requested wheel speeds to PWM
  float leftNorm  = speed_req_left  / MAX_SPEED;
  float rightNorm = speed_req_right / MAX_SPEED;

  // Clamp to [-1, 1]
  if (leftNorm >  1.0f) leftNorm  =  1.0f;
  if (leftNorm < -1.0f) leftNorm  = -1.0f;
  if (rightNorm >  1.0f) rightNorm =  1.0f;
  if (rightNorm < -1.0f) rightNorm = -1.0f;

  int leftPWM  = (int)(leftNorm  * MAX_PWM * LEFT_MOTOR_DIR);
  int rightPWM = (int)(rightNorm * MAX_PWM * RIGHT_MOTOR_DIR);

  setLeftMotor(leftPWM);
  setRightMotor(rightPWM);
}
// ===================== ODOMETRY UPDATE =====================
void updateOdometry() {
  // Snapshot encoder ticks
  noInterrupts();
  long currentLeftTicks  = leftTicks;
  long currentRightTicks = rightTicks;
  interrupts();

  long dLeftTicks  = currentLeftTicks  - lastLeftTicks;
  long dRightTicks = currentRightTicks - lastRightTicks;

  lastLeftTicks  = currentLeftTicks;
  lastRightTicks = currentRightTicks;

  float dLeft  = dLeftTicks  * distancePerTick;        // [m]
  float dRight = dRightTicks * distancePerTick;        // [m]

  float dCenter = 0.5f * (dLeft + dRight);             // [m]
  float dTheta  = (dRight - dLeft) / WHEEL_BASE_M;     // [rad]

  float dx, dy;

  // Check if robot is rotating in place (wheels moving in opposite directions)
  if ((dLeftTicks < 0 && dRightTicks > 0) || (dLeftTicks > 0 && dRightTicks < 0)) {
    // IN-PLACE ROTATION: Wheels moving in opposite directions
    // Robot pivots around its center point
    // No linear displacement, only angular change
    dx = 0.0f;
    dy = 0.0f;
    
  } else {
    // LINEAR MOTION: Both wheels moving in same direction (original logic)
    float thetaMid = theta + 0.5f * dTheta;
    dx = dCenter * cos(thetaMid);
    dy = dCenter * sin(thetaMid);
  }

  noInterrupts();
  xPose += dx;
  yPose += dy;
  theta += dTheta;

  // Normalize theta to [-pi, pi]
  if (theta > 3.14159265f) {
    theta -= 2.0f * 3.14159265f;
  } else if (theta < -3.14159265f) {
    theta += 2.0f * 3.14159265f;
  }
  interrupts();
}
// ===================== ROS PUBLISH FUNCTION =====================
void publishPose() {
  // Copy pose atomically
  noInterrupts();
  float x  = xPose;
  float y  = yPose;
  float th = theta;
  interrupts();

  pose_msg.header.stamp = nh.now();  // ros::Time from rosserial
  // We publish x and y (plus theta in z for convenience)
  pose_msg.vector.x = x;
  pose_msg.vector.y = y;
  pose_msg.vector.z = th;

  pose_pub.publish(&pose_msg);
}

// ===================== STEPPER MOTOR SECTION =====================
// ---- NEMA 17 Stepper Configuration ----
const uint8_t STEPPER_STEP_PIN = 22;    // Step pulse pin
const uint8_t STEPPER_DIR_PIN  = 24;    // Direction pin
const uint8_t STEPPER_ENABLE_PIN = 26;  // Enable pin (optional, set LOW to enable)

// Stepper parameters
const int STEPS_PER_REV = 200;           // NEMA 17 typical (1.8° per step)
const int MICROSTEPS = 16;               // Set according to your driver (A4988/DRV8825)
const long TOTAL_STEPS = STEPS_PER_REV * MICROSTEPS;

// Rod length: 1.5m, slow movement
const float ROD_LENGTH_M = 1.5f;
const float LEAD_SCREW_PITCH_MM = 8.0f;  // Adjust based on your lead screw (e.g., 8mm/rev)
const long STEPS_FOR_FULL_TRAVEL = (long)((ROD_LENGTH_M * 1000.0f / LEAD_SCREW_PITCH_MM) * TOTAL_STEPS);

// Speed control (microseconds between steps - higher = slower)
const unsigned int STEP_DELAY_US = 800;  // Adjust for desired speed (500-2000 us typical)

// Position tracking
bool plateAtBottom = true;               // Start position
bool stepperMoving = false;
unsigned long stepperStartTime = 0;

// ---- Stepper Control Callback ----
void handleStepperControl(const std_msgs::Bool& msg) {
  if (msg.data && !stepperMoving) {
    stepperMoving = true;
    stepperStartTime = millis();
    
    // Determine direction: if at bottom, go up; if at top, go down
    bool goingUp = plateAtBottom;
    digitalWrite(STEPPER_DIR_PIN, goingUp ? HIGH : LOW);
    digitalWrite(STEPPER_ENABLE_PIN, LOW);  // Enable stepper
    
    // Move the stepper
    for (long i = 0; i < STEPS_FOR_FULL_TRAVEL; i++) {
      digitalWrite(STEPPER_STEP_PIN, HIGH);
      delayMicroseconds(STEP_DELAY_US);
      digitalWrite(STEPPER_STEP_PIN, LOW);
      delayMicroseconds(STEP_DELAY_US);
      
      // Allow ROS to process messages periodically
      if (i % 100 == 0) {
        nh.spinOnce();
      }
    }
    
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);  // Disable stepper (optional, saves power)
    
    // Toggle position
    plateAtBottom = !plateAtBottom;
    stepperMoving = false;
  }
}

ros::Subscriber<std_msgs::Bool> stepper_sub("stepper_control", handleStepperControl);

// ===================== SETUP =====================
void setup() {
  Serial.begin(57600);
  delay(1000);

  // Encoder pins
  pinMode(LEFT_ENC_A_PIN,  INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PIN,  INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN),  leftEncoderA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncoderA_ISR, CHANGE);

  // Motor pins
  pinMode(L_ENA, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);

  pinMode(R_ENB, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);

  // Ensure motors off initially
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);

  // Precompute distance per tick
  distancePerTick = (2.0f * 3.14159265f * WHEEL_RADIUS_M) / (float)TICKS_PER_REV;

  // ---- Stepper motor pins ----
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  
  digitalWrite(STEPPER_STEP_PIN, LOW);
  digitalWrite(STEPPER_DIR_PIN, LOW);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);  // Disabled initially

  // ROS node handle init
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(pose_pub);
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.subscribe(stepper_sub);                // Subscribe to stepper control topic
}

// ===================== LOOP =====================
void loop() {
  unsigned long now = millis();
    // ---- Execute motor commands ----
  executeMotorCommands(); 
  // ---- Update odometry periodically ----
  if (now - lastOdomUpdate >= ODOM_UPDATE_MS) {
    lastOdomUpdate = now;
    updateOdometry();
  }

  // ---- Print pose periodically (debug) ----
  if (now - lastPrintUpdate >= PRINT_UPDATE_MS) {
    lastPrintUpdate = now;

    noInterrupts();
    float x     = xPose;
    float y     = yPose;
    float th    = theta;
    long lTicks = leftTicks;
    long rTicks = rightTicks;
    interrupts();

    // Also publish pose to ROS at the same period
    publishPose();
  }


  nh.spinOnce();
}