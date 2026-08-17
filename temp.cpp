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


const char* servo[12] = {
  "FL_1",
  "FR_1",
  "RR_1",
  "RL_1",

  "FL_2",
  "FR_2",
  "RR_2",
  "RL_2",

  "FL_3",
  "FR_3",
  "RR_3",
  "RL_3"
};

const char* FL[3] = {"FL_1", "FL_2", "FL_3"};
const char* FR[3] = {"FR_1", "FR_2", "FR_3"};
const char* RR[3] = {"RR_1", "RR_2", "RR_3"};
const char* RL[3] = {"RL_1", "RL_2", "RL_3"};

const int HOME_ANGLE[12] = {
  90, 90, 90, 90,    // Hip
  90, 90, 90, 90,    // Knee
  0, 0, 0, 0         // Ankle
};

const int MIN_LIMIT[12] = {
  10, 90, 10, 90,     // Hip
  90, 90, 90, 90,     // Knee
   0,  0,  0,  0      // Ankle
};

const int MAX_LIMIT[12] = {
  90, 170, 90, 170,   // Hip
 120, 120, 120, 120,  // Knee
  15,  15,  15,  15   // Ankle
};
  
// +1 = normal
// -1 = reversed
const int SERVO_DIRECTION[12] = {
  -1, 1, -1, 1,       // Hip
  1, 1, 1, 1,         // Knee
  1, 1, 1, 1          //Ankle
};

int current_angle[12] = {
  90, 90, 90, 90,    // Hip
  90, 90, 90, 90,    // Knee
  0, 0, 0, 0         // Ankle
};


uint16_t angleToPulse(int angle)
{
  angle = constrain(angle, 0, 180);

  int pulse_us = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);

  uint16_t pulse = (uint32_t)pulse_us * 4096 / 20000;

  return pulse;
}

void moveJoint(int channel, int angle)
{
  current_angle[channel] += SERVO_DIRECTION[channel] * angle;
  current_angle[channel] = constrain(
    current_angle[channel], 
    MIN_LIMIT[channel], 
    MAX_LIMIT[channel]
  );

  uint16_t pulse = angleToPulse(current_angle[channel]);
  
  pwm.setPWM(channel, 0, pulse);
  
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(current_angle[channel]);
  Serial.println(" deg");
}

void moveToHome()
{
  for(int i = 0; i < 12; i++)
  {
    int delta = (HOME_ANGLE[i] - current_angle[i]) * SERVO_DIRECTION[i];
    moveJoint(i, delta);
  }
}

void moveAllJoints(int angle)
{
  for(int i = 0; i < 12; i++)
  {
    moveJoint(i, angle);
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

int getServoChannel(String name)
{
  name.trim();
  name.toUpperCase();

  for (int i = 0; i < 12; i++)
  {
    if (name == servo[i])
    {
      return i;
    }
  }

  return -1;
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

  int channel = getServoChannel(jointName);

  if (channel == -1)
  {
    Serial.println("[ERROR] Unknown servo name");
    Serial.println("Use HELP to see valid servo names.");
    Serial.println();
    return;
  }

  if (angle < MIN_ANGLE || angle > MAX_ANGLE)
  {
    Serial.println("[ERROR] Angle must be between -90 and 90.");
    Serial.println();
    return;
  }
  
  Serial.print(jointName);
  Serial.print(" = ");
  Serial.print(angle);
  Serial.println(" deg");
  
  moveJoint(channel, angle);
  
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
