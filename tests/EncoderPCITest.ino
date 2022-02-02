#include "EncoderPCI.h"

// Motor A Encoder
#define MOTOR_A_ENC_0 3
#define MOTOR_A_ENC_90 4

// Motor B Encoder
#define MOTOR_B_ENC_0 7
#define MOTOR_B_ENC_90 8

EncoderPCI encoderA(MOTOR_A_ENC_0, MOTOR_A_ENC_90, 'A');
EncoderPCI encoderB(MOTOR_B_ENC_0, MOTOR_B_ENC_90, 'B');

void setup()
{
    Serial.begin(9600);
    Serial.println("EncoderPCI Tests");
    TestMenu();
}

void loop()
{
    // Handle Serial Buffer
    while (Serial.available() > 0)
    {
        char input = Serial.read();
        handleSerialInput(input);
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
            Serial.println("Encoder Values:");
            Serial.print("A: ");
            Serial.println(encoderA.read());
            Serial.print("B :");
            Serial.println(encoderB.read());
            break;
        case '2':
            Serial.println("Encoders Cleared...");
            encoderA.write(0);
            encoderB.write(0);
            break;
        case '3':
            Serial.println("Reversed A...");
            encoderA.setReversed(true);
            break;
        case '4':
            Serial.println("Reversed B...");
            encoderB.setReversed(true);
            break;
        case '5':
            Serial.println("Toggle...");
            
            break;

        default:
            Serial.println("Invalid Option");
            TestMenu();
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

void TestMenu()
{
    Serial.println("1 - Read Encoders");
    Serial.println("2 - Reset Encoders");
    Serial.println("3 - Reverse A");
    Serial.println("4 - Reverse B");
}