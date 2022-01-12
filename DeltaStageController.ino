// -------- OpenFlexure Delta Stage Controller --------
// Author: Shivun Chinniah
// Date: 12/01/2021
// Version: 1.0

// Includes the Arduino Stepper Library
#include <AccelStepper.h>

// Defines the number of steps per rotation
const long stepsPerRevolution = 2038;

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper A = AccelStepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);
AccelStepper B = AccelStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper C = AccelStepper(AccelStepper::FULL4WIRE, 6, 7, 12, 13);

enum class DIR
{
  UP,
  DOWN,
  LEFT,
  RIGHT
};

void setup()
{
  // Nothing to do (Stepper Library sets pins as outputs)
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Wits OC LAB, OpenFlexure Delta Controller v1.0");
  Serial.println();

  // Configure Steppers
  A.setMaxSpeed(200.0);
  A.setAcceleration(600.0);

  B.setMaxSpeed(200.0);
  B.setAcceleration(600.0);

  C.setMaxSpeed(200.0);
  C.setAcceleration(600.0);
}


void loop()
{

  char incomingChar = 0; // for incoming serial data

  if (Serial.available() > 0)
  {
    incomingChar = Serial.read();
    String code;
    readCode(code);
    switch (incomingChar)
    {
    case 'S':
      handleSetting(code);
      break;
    case 'M':
      handleMotorCommand(code);
      break;
    case 'Q':
      handleQuery(code);
      break;
    case '\n':
      break;
    default:
      break;
    }
  }

  A.run();
  B.run();
  C.run();
}

void readCode(String &buffer)
{
  char temp = Serial.read();
  while (Serial.available() > 0 && !isWhitespace(temp))
  {
    buffer += temp;
    temp = Serial.read();
  }
}

