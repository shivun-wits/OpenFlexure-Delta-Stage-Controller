// -------- OpenFlexure Delta Stage Controller --------
// Author: Shivun Chinniah
// Date: 12/01/2021
// Version: 1.0

// Includes the Arduino Stepper Library
#include <AccelStepper.h>

// Defines the number of steps per rotation
#define STEPS_PER_ROT 2048
// const long stepsPerRevolution = 2038;

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper A = AccelStepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);
AccelStepper B = AccelStepper(AccelStepper::FULL4WIRE, 2, 4, 3, 5);
AccelStepper C = AccelStepper(AccelStepper::FULL4WIRE, 6, 12, 7, 13);

MultiStepper steppers;

// Defines the maximum stable speed for the 28BYJ-48 Motor
const float MAX_SPEED = 200.0;

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
      handleMovementCommand(code);
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


struct Vector3f {
  float x;
  float y;
  float z;

  Vector3f(){
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
};

// -------- Settings --------
#define S_MAX_SPEED 1
#define S_ACCELERATION 2
#define S_CALIBRATE 3
#define S_DIRECTION_VECTOR 4

float maxSpeed = 200.0;
float acceleration = 600.0;
long A_steps = 0;
long B_steps = 0;
long C_steps = 0;

Vector3f direction;




// -------- Movement Commands --------
#define M_POSITION 1
#define M_VECTOR_TRANSLATE 3

// Determine the cartesian position given the steps of the motors
/*
    OpenFlexure
    
    C
    |  \
    |      \
    |         \
    |            \  A
    |            /          
    |         /
    |      /
    |  /
    B


    ^ y
    |
    |
    |
    ------->x    z up through screen

*/

#define MM_PER_ROT 0.78
#define SIN_30 0.5
#define COS_30 0.866


Vector3f calcCartesianPosition(long a, long b, long c){
  Vector3f out;

  out.x = (stepsToRot(-a) - SIN_30 * stepsToRot(b + c)) * MM_PER_ROT;
  out.y = COS_30 * stepsToRot(c - b) * MM_PER_ROT;
  out.z = z_mm_per_rot((SIN_30 * stepsToRot(b+c)) - stepsToRot(a));

}

float z_mm_per_rot(float rot){
  return 70.0 - 10.43 * sqrt(1-0.5*rot);
}

float stepsToRot(long steps){
  return steps/STEPS_PER_ROT;
}



// -------- Query Commands --------
#define Q_POSITION 1
#define Q_TARGET_POSITION 2
#define Q_RAW_STEPS 3
#define Q_MAX_SPEED 4
#define Q_ACCELERATION 5

void unknownQuery()
{
  Serial.println("Error: Unknown Query");
}



void handleQueryCommand(const String &code)
{

  

  switch (code)
  {
  case "POS":
    /* code */
    break;
  
  default:
    break;
  }

}
