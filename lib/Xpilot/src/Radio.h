#ifndef _RADIO_H
#define _RADIO_H

#include <Arduino.h>
#include "config.h"

typedef enum
{
    low = 1U,
    mid = 2U,
    high = 3U
} SwitchState;

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    SwitchState mode;
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