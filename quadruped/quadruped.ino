#line 1 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ============================================================
// PCA9685 / SERVO SETTINGS
// ============================================================
#define SERVO_FREQ 50
#define SERVO_MIN 500
#define SERVO_MAX 2400
#define MIN_ANGLE -90
#define MAX_ANGLE 90

// ============================================================
// SERVO NAMES & CHANNELS
// ============================================================
const char* servo[4][3] = {
  {"FL_1", "FL_2", "FL_3"},
  {"FR_1", "FR_2", "FR_3"},
  {"RR_1", "RR_2", "RR_3"},
  {"RL_1", "RL_2", "RL_3"}
};

const int CHANNEL[4][3] = {
  {0, 4, 8},       // FL
  {1, 5, 9},       // FR
  {2, 6, 10},      // RR
  {3, 7, 11}       // RL
};

// ============================================================
// HOME POSITION (STANDING / PARALLEL)
// ============================================================
const int HOME_ANGLE[4][3] = {
  {70, 100, 0},     // FL
  {110, 100, 0},     // FR
  {70, 100, 0},     // RR
  {110, 100, 0}      // RL
};

// ============================================================
// MECHANICAL LIMITS
// ============================================================
const int MIN_LIMIT[4][3] = {
  {10, 90, 0},     // FL
  {90, 90, 0},     // FR
  {10, 90, 0},     // RR
  {90, 90, 0}      // RL
};

const int MAX_LIMIT[4][3] = {
  {90, 170, 180},  // FL
  {170, 170, 180}, // FR
  {90, 170, 180},  // RR
  {170, 170, 180}  // RL
};

// ============================================================
// SERVO DIRECTIONS
// ============================================================
const int SERVO_DIRECTION[4][3] = {
  {-1, 1, 1},      // FL
  { 1, 1, 1},      // FR
  {-1, 1, 1},      // RR
  { 1, 1, 1}       // RL
};

int current_angle[4][3] = {
  {90, 90, 0},     // FL
  {90, 90, 0},     // FR
  {90, 90, 0},     // RR
  {90, 90, 0}      // RL
};

// ============================================================
// GAIT SETTINGS (4-BEAT CREEP / CRAWL)
// ============================================================
const unsigned long GAIT_UPDATE_MS = 20;

const float STEP_FREQUENCY = 1.0f; 

const float HIP_SWING = 30.0f;
const float KNEE_LIFT = 30.0f;
const float ANKLE_SWING = 15.0f;

const int KNEE_LIFT_DIRECTION[4] = { 1, 1, 1, 1 };

const float PI_F = 3.14159265358979323846f;

const float LEG_PHASE[4] = {
  0.0f,       // FL
  0.50f,      // FR
  0.25f,      // RR
  0.75f       // RL
};

// ============================================================
// MOVEMENT STATE
// ============================================================
enum MoveState { IDLE, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT };
MoveState moveState = IDLE;
bool moving = false;
float gaitTime = 0.0f;
unsigned long lastGaitMillis = 0;

// ============================================================
// HARDWARE COMMUNICATION
// ============================================================
#line 114 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
uint16_t angleToPulse(int angle);
#line 120 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void writeJoint(int leg, int joint);
#line 124 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void setJointAbsolute(int leg, int joint, int desiredAngle);
#line 131 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void moveJoint(int leg, int joint, int angle);
#line 139 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void moveToHome();
#line 146 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void initializeHome();
#line 155 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void moveAllJoints(int angle);
#line 165 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void startMove(MoveState state);
#line 180 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void stopMove();
#line 192 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void updateGait();
#line 253 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void setup();
#line 274 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
void loop();
#line 114 "C:\\Users\\JIGAR\\OneDrive\\Documents\\@Robotics_WS\\Arduino Projects\\spider_bot\\spider_bot.ino"
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  int pulse_us = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  return (uint32_t)pulse_us * 4096 / 20000;
}

void writeJoint(int leg, int joint) {
  pwm.setPWM(CHANNEL[leg][joint], 0, angleToPulse(current_angle[leg][joint]));
}

void setJointAbsolute(int leg, int joint, int desiredAngle) {
  desiredAngle = constrain(desiredAngle, MIN_LIMIT[leg][joint], MAX_LIMIT[leg][joint]);
  if (desiredAngle == current_angle[leg][joint]) return;
  current_angle[leg][joint] = desiredAngle;
  writeJoint(leg, joint);
}

void moveJoint(int leg, int joint, int angle) {
  int newAngle = current_angle[leg][joint] + SERVO_DIRECTION[leg][joint] * angle;
  newAngle = constrain(newAngle, MIN_LIMIT[leg][joint], MAX_LIMIT[leg][joint]);
  if (newAngle == current_angle[leg][joint]) return;
  current_angle[leg][joint] = newAngle;
  writeJoint(leg, joint);
}

void moveToHome() {
  moving = false; moveState = IDLE;
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++)
      setJointAbsolute(leg, joint, HOME_ANGLE[leg][joint]);
}

void initializeHome() {
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      current_angle[leg][joint] = HOME_ANGLE[leg][joint];
      writeJoint(leg, joint);
    }
  }
}

void moveAllJoints(int angle) {
  moving = false; moveState = IDLE;
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++)
      setJointAbsolute(leg, joint, HOME_ANGLE[leg][joint] + SERVO_DIRECTION[leg][joint] * angle);
}

// ============================================================
// MOVEMENT TRIGGERS
// ============================================================
void startMove(MoveState state) {
  if (moveState == state) return; // Prevent resetting gaitTime if already moving in this direction
  
  moveState = state;
  moving = true;
  gaitTime = 0.0f; // Start fresh
  lastGaitMillis = millis();
  
  Serial.print("START: ");
  if(state == FORWARD) Serial.println("FORWARD");
  if(state == BACKWARD) Serial.println("BACKWARD");
  if(state == TURN_LEFT) Serial.println("TURN LEFT");
  if(state == TURN_RIGHT) Serial.println("TURN RIGHT");
}

void stopMove() {
  if (!moving) return; // Prevent duplicate stops
  
  moving = false;
  moveState = IDLE;
  Serial.println("STOP");
  moveToHome(); // Stand still
}

// ============================================================
// CRAWL / CREEP GAIT ENGINE
// ============================================================
void updateGait() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastGaitMillis;
  if (elapsed < GAIT_UPDATE_MS) return;
  lastGaitMillis = now;

  float dt = elapsed / 1000.0f;
  
  gaitTime += dt * STEP_FREQUENCY;
  if (gaitTime >= 1.0f) gaitTime -= 1.0f;

  for (int leg = 0; leg < 4; leg++) {
    float localTime = gaitTime + LEG_PHASE[leg];
    if (localTime >= 1.0f) localTime -= 1.0f;

    float liftAmount = 0.0f;
    float hipPosition = 0.0f;

    if (localTime < 0.25f) {
      float swingTime = localTime * 4.0f;
      liftAmount = sin(swingTime * PI_F);
      hipPosition = -cos(swingTime * PI_F);
    } else {
      float stanceTime = (localTime - 0.25f) / 0.75f;
      liftAmount = 0.0f;
      hipPosition = 1.0f - (2.0f * stanceTime);
    }

    float movementDirection = 1.0f;
    bool isRear = (leg == 2 || leg == 3);

    if (moveState == FORWARD) {
      movementDirection = isRear ? -1.0f : 1.0f;
    } 
    else if (moveState == BACKWARD) {
      movementDirection = isRear ? 1.0f : -1.0f;
    } 
    else if (moveState == TURN_LEFT) {
      bool isLeft = (leg == 0 || leg == 3);
      movementDirection = isLeft ? -1.0f : 1.0f;
    } 
    else if (moveState == TURN_RIGHT) {
      bool isLeft = (leg == 0 || leg == 3);
      movementDirection = isLeft ? 1.0f : -1.0f;
    }

    float logicalHipOffset = movementDirection * HIP_SWING * hipPosition;

    float hipTarget = HOME_ANGLE[leg][0] + (SERVO_DIRECTION[leg][0] * logicalHipOffset);
    float kneeTarget = HOME_ANGLE[leg][1] + (KNEE_LIFT * liftAmount * KNEE_LIFT_DIRECTION[leg]);
    float ankleTarget = HOME_ANGLE[leg][2] + (ANKLE_SWING * liftAmount);

    setJointAbsolute(leg, 0, (int)round(hipTarget));
    setJointAbsolute(leg, 1, (int)round(kneeTarget));
    setJointAbsolute(leg, 2, (int)round(ankleTarget));
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize Dabble with a custom Bluetooth name
  Dabble.begin("SpiderBot"); 

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(500);

  initializeHome();
  lastGaitMillis = millis();
  
  Serial.println("SpiderBot Ready. Connect via Dabble App.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // 1. Process incoming Bluetooth Data
  Dabble.processInput();

  // 2. Read Gamepad inputs to determine the desired state
  MoveState desiredState = IDLE;

  if (GamePad.isUpPressed()) {
    desiredState = FORWARD;
  } else if (GamePad.isDownPressed()) {
    desiredState = BACKWARD;
  } else if (GamePad.isLeftPressed()) {
    desiredState = TURN_LEFT;
  } else if (GamePad.isRightPressed()) {
    desiredState = TURN_RIGHT;
  }

  // 3. Apply the movement logic based on gamepad input
  if (desiredState != IDLE) {
    if (moveState != desiredState) {
      startMove(desiredState);
    }
  } else {
    // If no directional buttons are held down, stop walking
    if (moving) {
      stopMove();
    }
    
    // Allow special functions when standing still
    if (GamePad.isTrianglePressed()) {
      moveToHome();
    }
  }

  // Generate Gait Cycle
  if (moving) {
    updateGait();
  }
}

