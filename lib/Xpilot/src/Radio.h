#ifndef _RADIO_H
#define _RADIO_H

#include <Arduino.h>
#include "config.h"

typedef enum : uint8_t
{
    passthrough = 1U,
    rate = 2U,
    stabilize = 3U
} FlightMode;

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    FlightMode currentMode;
    FlightMode previousMode;
} Control;

class Radio
{
public:
    Radio(void);
    void init(void);
    void processInput(void);
    Control rx;
};

extern Radio radio;
#endif