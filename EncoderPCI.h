#ifndef ENCODER_PCI_H
#define ENCODER_PCI_H

enum class EncoderISRChannel
{
    A,
    B,
    C,
    D
};

class EncoderPCI
{
public:
    EncoderPCI(uint8_t triggerPin, uint8_t directionPin, char channel);
    int32_t read();
    void write(int32_t ticks);
    bool isReversed();
    void setReversed(bool reverse);

    static EncoderPCI *ISR_A;
    static EncoderPCI *ISR_B;
    static EncoderPCI *ISR_C;
    static EncoderPCI *ISR_D;

private:
    int32_t _ticks;
    void _tick();
    void _tick_90();
    uint8_t _triggerPin;
    uint8_t _directionPin;
    bool _reversed;

    static void ISRHandlerA();
    static void ISRHandlerB();
    static void ISRHandlerC();
    static void ISRHandlerD();

    static void ISRHandlerA_90();
    static void ISRHandlerB_90();
    static void ISRHandlerC_90();
    static void ISRHandlerD_90();
};

#endif