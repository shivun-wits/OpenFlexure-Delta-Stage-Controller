#ifndef L298M_H
#define L298M_H

#include <stdint.h>

class L298M
{

private:
    uint8_t CW_pin;
    uint8_t CCW_pin;

    int16_t _vector; // -255 to 255
    bool _hold; // when _vector is 0 should position be held
    

public:
    L298M(uint8_t cw, uint8_t ccw);
    void stop();
    void run();
    void setVector(int16_t vector);
    int16_t getVector();
    void setHold(bool hold);
    bool getHold();
};

#endif
