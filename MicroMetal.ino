// -------- OpenFlexure Delta Stage Controller - Micro Metal Motors --------
// Author: Shivun Chinniah
// Date: 25/01/2021
// Version: 1.0

#include <PID_v1.h>
#include "EncoderPCI.h"
#include "L298M.h"

#define MOTOR_A1 3
#define MOTOR_A2 5

#define MOTOR_B1 6
#define MOTOR_B2 9

#define MOTOR_C1 10
#define MOTOR_C2 11

#define MOTOR_A_ENC_0 2
#define MOTOR_A_ENC_90 4

#define MOTOR_B_ENC_0 7
#define MOTOR_B_ENC_90 8

#define MOTOR_C_ENC_0 12
#define MOTOR_C_ENC_90 A0

EncoderPCI encoderA(MOTOR_A_ENC_0, MOTOR_A_ENC_90, 'A');
L298M motorA(MOTOR_A1, MOTOR_A2);

EncoderPCI encoderB(MOTOR_B_ENC_0, MOTOR_B_ENC_90, 'B');
L298M motorB(MOTOR_B1, MOTOR_B2);

EncoderPCI encoderC(MOTOR_C_ENC_0, MOTOR_C_ENC_90, 'C');
L298M motorC(MOTOR_C1, MOTOR_C2);

double ATarget_PID, AFeedBack_PID, AOut_PID;
double BTarget_PID, BFeedBack_PID, BOut_PID;
double CTarget_PID, CFeedBack_PID, COut_PID;

double AKp = 4000, AKd = 50, AKi = 1500;
double BKp = 4000, BKd = 50, BKi = 1500;
double CKp = 4000, CKd = 50, CKi = 1500;

double AmaxPWM_PID = 140;
double BmaxPWM_PID = 140;
double CmaxPWM_PID = 140;

bool enablePID = false;

PID A_PID(&AFeedBack_PID, &AOut_PID, &ATarget_PID, AKp, AKd, AKi, DIRECT);
PID B_PID(&BFeedBack_PID, &BOut_PID, &BTarget_PID, BKp, BKd, BKi, DIRECT);
PID C_PID(&CFeedBack_PID, &COut_PID, &CTarget_PID, CKp, CKd, CKi, DIRECT);

void setup()
{

    // PID
    A_PID.SetOutputLimits(-AmaxPWM_PID, AmaxPWM_PID);
    A_PID.SetMode(MANUAL);
    A_PID.SetSampleTime(10);

    B_PID.SetOutputLimits(-BmaxPWM_PID, BmaxPWM_PID);
    B_PID.SetMode(MANUAL);
    B_PID.SetSampleTime(10);

    C_PID.SetOutputLimits(-CmaxPWM_PID, CmaxPWM_PID);
    C_PID.SetMode(MANUAL);
    C_PID.SetSampleTime(10);

    // Setup Motor Pins
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);

    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);

    pinMode(MOTOR_C1, OUTPUT);
    pinMode(MOTOR_C2, OUTPUT);

    // Serial
    Serial.begin(19200);
    OptionsMenu();
    Serial.print("> ");
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

        BFeedBack_PID = rawToRot(encoderB.read());
        B_PID.Compute();
        motorB.setVector((int16_t)BOut_PID);
        motorB.run();

        CFeedBack_PID = rawToRot(encoderC.read());
        C_PID.Compute();
        motorC.setVector((int16_t)COut_PID);
        motorC.run();
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
    Serial.println("L        - Get Current Position");
    Serial.println("M{f,f,f} - Move Absolute");
    Serial.println("R{f,f,f} - Move Relative");
    Serial.println("S{c,int} - Max PWM 0-255");
    Serial.println("C        - Calibrate");
    Serial.println("1        - Enable PID");
    Serial.println("0        - Disable PID");
}

void PID_enable()
{
    enablePID = true;
    A_PID.SetMode(AUTOMATIC);
    B_PID.SetMode(AUTOMATIC);
    C_PID.SetMode(AUTOMATIC);

    A_PID.SetTunings(AKp, AKi, AKd);
    B_PID.SetTunings(BKp, BKi, BKd);
    C_PID.SetTunings(CKp, CKi, CKd);
}

void PID_disable()
{
    enablePID = false;
    A_PID.SetMode(MANUAL);
    B_PID.SetMode(MANUAL);
    C_PID.SetMode(MANUAL);
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
    case 'I':
    case 'D':
    {
        char motor = linebuffer[1];
        linebuffer = linebuffer.substring(1);
        readCode(buffer);
        setPIDconstant(in, buffer.toDouble(), motor);
        break;
    }
    case '0':
        PID_disable();
        break;
    case '1':
        PID_enable();
        break;
    case '6':
        readCode(buffer);
        ATarget_PID = buffer.toDouble();
        Serial.print("A Position Target --> ");
        Serial.println(ATarget_PID);
        break;
    case '7':
        readCode(buffer);
        BTarget_PID = buffer.toDouble();
        Serial.print("B Position Target --> ");
        Serial.println(BTarget_PID);
        break;
    case '8':
        readCode(buffer);
        CTarget_PID = buffer.toDouble();
        Serial.print("C Position Target --> ");
        Serial.println(CTarget_PID);
        break;

    case 'M':
        linebuffer = linebuffer.substring(1);
        parseAbsoluteMovement();
        break;

    case 'R':
        linebuffer = linebuffer.substring(1);
        parseRelativeMovement();
        break;

    default:
        PID_menu();
        break;
    }
}

double nextDouble(String &buffer)
{

    String number;
    unsigned int end = buffer.indexOf(',');

    // valid comma
    if (end > 0)
    {
        double out = buffer.substring(0, end).toDouble();
        buffer = buffer.substring(end + 1);
        return out;
    }
    else if (buffer.length() == 0)
    {
        // empty buffer
        return 0;
    }
    else
    {
        // last part
        double out = buffer.toDouble();
        buffer = "";
        return out;
    }
}

void parseRelativeMovement()
{
    MoveRelative(nextDouble(linebuffer), nextDouble(linebuffer), nextDouble(linebuffer));
}

void parseAbsoluteMovement()
{
    MoveAbsolute(nextDouble(linebuffer), nextDouble(linebuffer), nextDouble(linebuffer));
}

void MoveRelative(double c, double b, double a)
{
    ATarget_PID += a;
    BTarget_PID += b;
    CTarget_PID += c;
}

void MoveAbsolute(double c, double b, double a)
{
    ATarget_PID = a;
    BTarget_PID = b;
    CTarget_PID = c;
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

    Serial.print("Position Targets: ");
    printlnMatrix(ATarget_PID, BTarget_PID, CTarget_PID);

    Serial.print("Current Positions: ");
    printlnMatrix(rawToRot(encoderA.read()), rawToRot(encoderB.read()), rawToRot(encoderC.read()));

    Serial.print("Current Control Outputs: ");
    printlnMatrix(AOut_PID, BOut_PID, COut_PID);

    Serial.print("Kp: ");
    printlnMatrix(AKp, BKp, CKp);
    Serial.print("Ki: ");
    printlnMatrix(AKi, BKi, CKi);
    Serial.print("Kd: ");
    printlnMatrix(AKd, BKd, CKd);

    Serial.print("Max PWM: ");
    printlnMatrix(AmaxPWM_PID, BmaxPWM_PID, CmaxPWM_PID);
}

void setPIDconstant(char constant, double value, char channel)
{
    switch (channel)
    {
    case 'A':
        setAPIDConstant(constant, value);
        break;
    case 'B':
        setBPIDConstant(constant, value);
        break;
    case 'C':
        setCPIDConstant(constant, value);
        break;
    case '0':
        setAPIDConstant(constant, value);
        setBPIDConstant(constant, value);
        setCPIDConstant(constant, value);
        break;
    default:
        break;
    }
}

void setAPIDConstant(char constant, double value)
{
    switch (constant)
    {
    case 'P':
        AKp = value;
        Serial.print("A.Kp --> ");
        Serial.println(AKp);
        break;
    case 'I':
        AKi = value;
        Serial.print("A.Ki --> ");
        Serial.println(AKi);
        break;
    case 'D':
        AKd = value;
        Serial.print("A.Kd --> ");
        Serial.println(AKd);
        break;
    default:
        break;
    }

    // Update PID controller
    A_PID.SetTunings(AKp, AKi, AKd);
}

void setBPIDConstant(char constant, double value)
{
    switch (constant)
    {
    case 'P':
        BKp = value;
        Serial.print("B.Kp --> ");
        Serial.println(BKp);
        break;
    case 'I':
        BKi = value;
        Serial.print("B.Ki --> ");
        Serial.println(BKi);
        break;
    case 'D':
        BKd = value;
        Serial.print("B.Kd --> ");
        Serial.println(BKd);
        break;
    default:
        break;
    }

    // Update PID controller
    B_PID.SetTunings(BKp, BKi, BKd);
}

void setCPIDConstant(char constant, double value)
{
    switch (constant)
    {
    case 'P':
        CKp = value;
        Serial.print("C.Kp --> ");
        Serial.println(CKp);
        break;
    case 'I':
        CKi = value;
        Serial.print("C.Ki --> ");
        Serial.println(CKi);
        break;
    case 'D':
        CKd = value;
        Serial.print("C.Kd --> ");
        Serial.println(CKd);
        break;
    default:
        break;
    }

    // Update PID controller
    C_PID.SetTunings(CKp, CKi, CKd);
}

// -------- Motor Functions --------
void stop()
{
    Serial.println("Stop");

    // stop PID as well
    PID_disable();

    motorA.setHold(false);
    motorA.stop();

    motorB.setHold(false);
    motorB.stop();

    motorC.setHold(false);
    motorC.stop();
}

const int maximumpower = 130;

void cw()
{
    Serial.println("CW");
    motorA.setVector(maximumpower);
    motorA.run();
    motorB.setVector(maximumpower);
    motorB.run();
    motorC.setVector(maximumpower);
    motorC.run();
}

void ccw()
{
    Serial.println("CCW");
    motorA.setVector(-maximumpower);
    motorA.run();
    motorB.setVector(-maximumpower);
    motorB.run();
    motorC.setVector(-maximumpower);
    motorC.run();
}

double rawToRot(long raw)
{
    return (double)raw / 645;
}

void readEncoders()
{
    Serial.println(micros());
    Serial.println("Encoders:");
    Serial.print("Rot:\t");
    printlnMatrix(rawToRot(encoderA.read()), rawToRot(encoderB.read()), rawToRot(encoderC.read()));
    Serial.print("Raw:\t");
    printlnMatrix(encoderA.read(), encoderB.read(), encoderC.read());
}

void resetEncoders()
{
    encoderA.write(0);
    encoderB.write(0);
    encoderC.write(0);
}

void stopHold()
{
    Serial.println("Stop Hold");

    PID_disable();

    motorA.setVector(0);
    motorA.setHold(true);
    motorA.stop();

    motorB.setVector(0);
    motorB.setHold(true);
    motorB.stop();

    motorC.setVector(0);
    motorC.setHold(true);
    motorC.stop();
}

// -------- MISC --------
void readCode(String &buffer)
{
    buffer = linebuffer.substring(1);
}

template <typename T>
void printlnMatrix(T a, T b, T c)
{
    Serial.print("[");
    Serial.print(a);
    Serial.print(", ");
    Serial.print(b);
    Serial.print(", ");
    Serial.print(c);
    Serial.println("]");
}
