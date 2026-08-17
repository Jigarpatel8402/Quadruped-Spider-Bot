#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);


#define SERVO_FREQ 50       // MG90S: 50 Hz

#define SERVO_MIN 500
#define SERVO_MAX 2400

// FL = Front Left
// FR = Front Right
// RR = Rear Right
// RL = Rear Left

// 1 = Hip
// 2 = Knee
// 3 = Ankle

// FL_1 = CH0
// FR_1 = CH1
// RR_1 = CH2
// RL_1 = CH3

// FL_2 = CH4
// FR_2 = CH5
// RR_2 = CH6
// RL_2 = CH7

// FL_3 = CH8
// FR_3 = CH9
// RR_3 = CH10
// RL_3 = CH11

#define MIN_ANGLE -90
#define MAX_ANGLE 90


const char* servo[4][3] = {
  {"FL_1", "FL_2", "FL_3"},
  {"FR_1", "FR_2", "FR_3"},
  {"RR_1", "RR_2", "RR_3"},
  {"RL_1", "RL_2", "RL_3"}
};

const int CHANNEL[4][3] = {
  {0, 4, 8},      // FL
  {1, 5, 9},      // FR
  {2, 6, 10},     // RR
  {3, 7, 11}      // RL
};

const int HOME_ANGLE[4][3] = {
  {90, 90, 0},     // FL
  {90, 90, 0},     // FR
  {90, 90, 0},     // RR
  {90, 90, 0}      // RL
};

const int MIN_LIMIT[4][3] = {
  {10, 90, 0},     // FL
  {90, 90, 0},     // FR
  {10, 90, 0},     // RR
  {90, 90, 0}      // RL
};

const int MAX_LIMIT[4][3] = {
  {90, 120, 15},   // FL
  {170, 120, 15},  // FR
  {90, 120, 15},   // RR
  {170, 120, 15}   // RL
};

// +1 = normal
// -1 = reversed
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


uint16_t angleToPulse(int angle)
{
  angle = constrain(angle, 0, 180);

  int pulse_us = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);

  uint16_t pulse = (uint32_t)pulse_us * 4096 / 20000;

  return pulse;
}

void moveJoint(int leg, int joint, int angle)
{
  current_angle[leg][joint] += SERVO_DIRECTION[leg][joint] * angle;
  current_angle[leg][joint] = constrain(
    current_angle[leg][joint], 
    MIN_LIMIT[leg][joint], 
    MAX_LIMIT[leg][joint]
  );

  uint16_t pulse = angleToPulse(current_angle[leg][joint]);
  
  int channel = CHANNEL[leg][joint];

  pwm.setPWM(channel, 0, pulse);
  
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(current_angle[leg][joint]);
  Serial.println(" deg");
}

void moveToHome()
{
  for(int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
    {
      int delta = (HOME_ANGLE[i][j] - current_angle[i][j]) * SERVO_DIRECTION[i][j];
      moveJoint(i, j, delta);
    }
}

void moveAllJoints(int angle)
{
  for(int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
    {
      moveJoint(i, j, angle);
    }
}

void printHelp()
{
  Serial.println();
  Serial.println("======================================");
  Serial.println(" Quadruped Servo Controller");
  Serial.println("======================================");
  Serial.println();
  Serial.println("Individual servo:");
  Serial.println("  FL_1 90");
  Serial.println("  FR_2 80");
  Serial.println("  RR_3 120");
  Serial.println("  RL_1 100");
  Serial.println();
  Serial.println("All servos:");
  Serial.println("  ALL 90");
  Serial.println();
  Serial.println("Home position:");
  Serial.println("  HOME");
  Serial.println();
  Serial.println("Show help:");
  Serial.println("  HELP");
  Serial.println();
}

void processCommand(String command)
{
  command.trim();

  if (command.length() == 0)
    return;
  
  command.toUpperCase();

  if (command == "HOME")
  {
    Serial.println("Home");
    moveToHome();
    Serial.println();
    return;
  }
  else if (command == "HELP")
  {
    printHelp();
    return;
  }

  if (command.startsWith("ALL "))
  {
    int angle = command.substring(4).toInt();

    if (angle < MIN_ANGLE || angle > MAX_ANGLE)
    {
      Serial.println("[ERROR] Angle must be between -90 and 90");
      Serial.println();
      return;
    }

    Serial.print("All ");
    Serial.println(angle);

    moveAllJoints(angle);
    
    Serial.println();
    
    return;
  }

  // INDIVIDUAL SERVO
  int spaceIndex = command.indexOf(' ');

  if (spaceIndex == -1)
  {
    Serial.println("[ERROR] Invalid command");
    Serial.println("Example: FL_1 90");
    Serial.println();
    return;
  }

  String jointName = command.substring(0, spaceIndex);
  int angle = command.substring(spaceIndex + 1).toInt();

  int underscoreIndex = jointName.indexOf('_');

  if (underscoreIndex == -1)
  {
    Serial.println("[ERROR] Invalid servo name");
    Serial.println("Example: FL_1 90");
    Serial.println();
    return;
  }

  String legName = jointName.substring(0, underscoreIndex);
  int joint = jointName.substring(underscoreIndex + 1).toInt() - 1;

  int leg;
  if (legName == "FL")
    leg = 0;
  else if (legName == "FR")
    leg = 1;
  else if (legName == "RR")
    leg = 2;
  else if (legName == "RL")
    leg = 3;
  else 
  {
    Serial.println("[ERROR] Unknown leg name");
    Serial.println("Use FL, FR, RR or RL.");
    Serial.println();
    return;
  }

  if (joint < 0 && joint > 2)
  {
    Serial.println("[ERROR] Invalid joint number");
    Serial.println("Use 1 = Hip, 2 = Knee, 3 = Ankle.");
    Serial.println();
    return;
  }

  if (angle < MIN_ANGLE || angle > MAX_ANGLE)
  {
    Serial.println("[ERROR] Angle must be between -90 and 90.");
    Serial.println();
    return;
  }
  
  Serial.print(legName);
  Serial.print("[");
  Serial.print(joint);
  Serial.print("] = ");
  Serial.print(angle);
  Serial.println(" deg");
  
  moveJoint(leg, joint, angle);
  
  Serial.println();
}


void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" Quadruped Servo Controller");
  Serial.println("======================================");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(500);

  moveToHome();

  Serial.println("Enter commands...");
  Serial.println();
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');

    processCommand(command);
  }
}
