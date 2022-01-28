// -------- OpenFlexure Delta Stage Controller - Micro Metal Motors --------
// Author: Shivun Chinniah
// Date: 25/01/2021
// Version: 1.0

#include <PID_v1.h>
#include <Encoder.h>
#include "L298M.h"

#define MOTOR_A1 5
#define MOTOR_A2 6

#define MOTOR_A_ENC_0 3
#define MOTOR_A_ENC_90 4

Encoder encoderA(MOTOR_A_ENC_0, MOTOR_A_ENC_90);
L298M motorA(MOTOR_A1, MOTOR_A2);

double ATarget_PID, AFeedBack_PID, AOut_PID;
double Kp = 4, Kd = 0, Ki = 0;
double maxPWM_PID = 126;
bool enablePID = false;

PID A_PID(&AFeedBack_PID, &AOut_PID, &ATarget_PID, Kp, Kd, Ki, DIRECT);

void setup()
{

    // PID
    A_PID.SetOutputLimits(-maxPWM_PID, maxPWM_PID);
    A_PID.SetMode(MANUAL);
    A_PID.SetSampleTime(10);

    // Setup Motor Pins
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);

    // Serial
    Serial.begin(9600);
    OptionsMenu();
}

void loop()
{

    // Handle Serial Buffer
    while (Serial.available() > 0)
    {
        char input = Serial.read();
        handleSerialInput(input);
    }

    // PID
    if (enablePID)
    {
        AFeedBack_PID = rawToRot(encoderA.read());
        A_PID.Compute();
        motorA.setVector((int16_t)AOut_PID);
        motorA.run();
    }
}

// -------- Serial Interface --------
String linebuffer = "";
void handleSerialInput(char input)
{
    if (input == 13)
    {
        Serial.println();
        switch ((char)linebuffer[0])
        {
        case '1':
            stop();
            break;
        case '2':
            cw();
            break;
        case '3':
            ccw();
            break;
        case '4':
            readEncoders();
            break;
        case '5':
            resetEncoders();
            break;
        case '6':
            stopHold();
            break;
        case '7':
            break;
        case 'P':
            handlePIDCommand();
            break;

        default:
            Serial.println("Invalid Option");
            OptionsMenu();
            break;
        }

        linebuffer = "";
        Serial.print("> ");
    }
    else if (input != 0)
    {

        Serial.print(input);
        linebuffer += input;
    }
}

void OptionsMenu()
{
    Serial.println("OpenFlexure - MicroMetal Motor Driver V1");
    Serial.println();
    Serial.println("Enter a valid command:");
    Serial.println("1 - Stop");
    Serial.println("2 - Clock-wise");
    Serial.println("3 - Counter-clock-wise");
    Serial.println("4 - Read Encoder");
    Serial.println("5 - Reset Encoder");
    Serial.println("6 - Stop & Hold");
    Serial.println("P - PID Command");
}

// -------- PID Interface --------

void PID_menu()
{
    Serial.println("PID Commands: P{xxx}\n");
    Serial.println("Q        - Query info");
    Serial.println("P{float} - Change Kp");
    Serial.println("I{float} - Change Ki");
    Serial.println("D{float} - Change Kd");
    Serial.println("E        - Save to EEPROM");
    Serial.println("M{f,f,f} - Move Absolute");
    Serial.println("R{f,f,f} - Move Relative");
    Serial.println("S{int}   - Max PWM 0-255");
    Serial.println("C        - Calibrate");
    Serial.println("1        - Enable PID");
    Serial.println("0        - Disable PID");
    Serial.println("6{f}     - Set Point");
}

void PID_enable()
{
    enablePID = true;
    A_PID.SetMode(AUTOMATIC);
}

void PID_disable()
{
    enablePID = false;
    A_PID.SetMode(MANUAL);
}

void handlePIDCommand()
{
    String command;
    // Read First Character
    // char in = Serial.read();
    String buffer;
    char in = linebuffer[1];
    linebuffer = linebuffer.substring(1);
    switch (in)
    {
    case 'Q':
        queryPID();
        break;
    case 'P':
        readCode(buffer);
        setPIDConstant('P', buffer.toDouble());
        break;
    case 'I':
        readCode(buffer);
        setPIDConstant('I', buffer.toDouble());
        break;
    case 'D':
        readCode(buffer);
        setPIDConstant('D', buffer.toDouble());
        break;
    case '0':
        PID_disable();
        break;
    case '1':
        PID_enable();
        break;
    case '6':
        readCode(buffer);
        ATarget_PID = buffer.toDouble();
        Serial.print("Position Target --> ");
        Serial.println(ATarget_PID);
        break;
    case 'S':
        readCode(buffer);
        maxPWM_PID = buffer.toDouble();
        Serial.print("Max PWM --> ");
        Serial.println(maxPWM_PID);
        break;
    default:
        PID_menu();
        break;
    }
}

void queryPID()
{
    if (enablePID)
    {
        Serial.println("PID Enabled");
    }
    else
    {
        Serial.println("PID Disabled");
    }
    Serial.print("Position Target: ");
    Serial.println(ATarget_PID);
    Serial.print("Current Position: ");
    Serial.println(rawToRot(encoderA.read()));
    Serial.print("Current Control Output: ");
    Serial.println(AOut_PID);
    Serial.print("Kp: ");
    Serial.print(Kp);
    Serial.print(", Ki: ");
    Serial.print(Ki);
    Serial.print(", Kd: ");
    Serial.println(Kd);
    Serial.print("Max PWM: ");
    Serial.println(maxPWM_PID);
}

void setPIDConstant(char constant, double value)
{
    switch (constant)
    {
    case 'P':
        Kp = value;
        Serial.print("Kp --> ");
        Serial.println(Kp);
        break;
    case 'I':
        Ki = value;
        Serial.print("Ki --> ");
        Serial.println(Ki);
        break;
    case 'D':
        Kd = value;
        Serial.print("Kd --> ");
        Serial.println(Kd);
        break;
    default:
        break;
    }

    // Update PID controller
    A_PID.SetTunings(Kp, Ki, Kd);
}

// -------- Motor Functions --------
void stop()
{
    Serial.println("Stop");

    // stop PID as well
    PID_disable();

    motorA.setHold(false);
    motorA.stop();
}

void cw()
{
    Serial.println("CW");
    motorA.setVector(126);
    motorA.run();
}

void ccw()
{
    Serial.println("CCW");
    motorA.setVector(-126);
    motorA.run();
}

double rawToRot(long raw)
{
    return (double)raw / 645;
}

void readEncoders()
{
    Serial.println(micros());
    Serial.println("Motor A:");
    Serial.print("Rot:\t");
    Serial.print(rawToRot(encoderA.read()));
    Serial.print("\tRaw:\t");
    Serial.println(encoderA.read());
}

void resetEncoders()
{

    encoderA.write(0);
}

void stopHold()
{
    Serial.println("Stop Hold");
    PID_disable();
    motorA.setVector(0);
    motorA.setHold(true);
    motorA.stop();
}

// -------- MISC --------
void readCode(String &buffer)
{

    buffer = linebuffer.substring(1);
    // char temp = Serial.read();
    // // Read until next Character.
    // while (isWhitespace(temp) && Serial.available() > 0)
    // {
    //     temp = Serial.read();
    // }
    // while (Serial.available() > 0 && !isWhitespace(temp))
    // {
    //     buffer += temp;
    //     temp = Serial.read();
    // }
}
