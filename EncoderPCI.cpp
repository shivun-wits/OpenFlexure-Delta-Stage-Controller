#include "EncoderPCI.h"
#include <PinChangeInterrupt.h>

EncoderPCI *EncoderPCI::ISR_A = nullptr;
EncoderPCI *EncoderPCI::ISR_B = nullptr;
EncoderPCI *EncoderPCI::ISR_C = nullptr;
EncoderPCI *EncoderPCI::ISR_D = nullptr;

EncoderPCI::EncoderPCI(uint8_t triggerPin, uint8_t directionPin, char channel)
{

    _triggerPin = triggerPin;
    _directionPin = directionPin;

    void (*ISRHandler)() = nullptr;
    void (*ISRHandler_90)() = nullptr;

    switch (channel)
    {
    case 'A':
    case 'a':
        ISRHandler = &ISRHandlerA;
        ISRHandler_90 = &ISRHandlerA_90;
        EncoderPCI::ISR_A = this;
        break;

    case 'B':
    case 'b':
        ISRHandler = &ISRHandlerB;
        ISRHandler_90 = &ISRHandlerB_90;
        EncoderPCI::ISR_B = this;
        break;

    case 'C':
    case 'c':
        ISRHandler = &ISRHandlerC;
        ISRHandler_90 = &ISRHandlerC_90;
        EncoderPCI::ISR_C = this;
        break;

    case 'D':
    case 'd':
        ISRHandler = &ISRHandlerD;
        ISRHandler_90 = &ISRHandlerD_90;
        EncoderPCI::ISR_D = this;
        break;

    default:
        break;
    }

    // Check if triggerPin is 2 or 3 -- > use interrupts
    if (triggerPin == 2 || triggerPin == 3)
    {
        attachInterrupt(digitalPinToInterrupt(_triggerPin), ISRHandler, CHANGE);
    }
    else
    {
        attachPCINT(digitalPinToPCINT(_triggerPin), ISRHandler, CHANGE);
    }

    // Check if directionPin is 2 or 3 -- > use interrupts
    if (directionPin == 2 || directionPin == 3)
    {
        attachInterrupt(digitalPinToInterrupt(_directionPin), ISRHandler_90, CHANGE);
    }
    else
    {
        attachPCINT(digitalPinToPCINT(_directionPin), ISRHandler_90, CHANGE);
    }
}

int32_t EncoderPCI::read()
{

    return _ticks;
}

void EncoderPCI::write(int32_t ticks)
{
    _ticks = ticks;
}

void EncoderPCI::_tick()
{

    bool trigger = digitalRead(_triggerPin);
    bool direction = digitalRead(_directionPin);

    // XOR to determine direction of rotation
    if (trigger ^ direction ^ _reversed)
    {
        // CW
        _ticks++;
    }
    else
    {
        // CCW
        _ticks--;
    }
}

void EncoderPCI::_tick_90()
{

    bool trigger = digitalRead(_triggerPin);
    bool direction = digitalRead(_directionPin);

    // XOR to determine direction of rotation
    if (!trigger ^ direction ^ _reversed)
    {
        // CW
        _ticks++;
    }
    else
    {
        // CCW
        _ticks--;
    }
}

bool EncoderPCI::isReversed()
{
    return _reversed;
}

void EncoderPCI::setReversed(bool reverse)
{

    _reversed = reverse;
}

void EncoderPCI::ISRHandlerA()
{

    EncoderPCI::ISR_A->_tick();
}

void EncoderPCI::ISRHandlerB()
{

    EncoderPCI::ISR_B->_tick();
}

void EncoderPCI::ISRHandlerC()
{

    EncoderPCI::ISR_C->_tick();
}

void EncoderPCI::ISRHandlerD()
{

    EncoderPCI::ISR_D->_tick();
}

void EncoderPCI::ISRHandlerA_90()
{

    EncoderPCI::ISR_A->_tick_90();
}

void EncoderPCI::ISRHandlerB_90()
{

    EncoderPCI::ISR_B->_tick_90();
}

void EncoderPCI::ISRHandlerC_90()
{

    EncoderPCI::ISR_C->_tick_90();
}

void EncoderPCI::ISRHandlerD_90()
{

    EncoderPCI::ISR_D->_tick_90();
}
