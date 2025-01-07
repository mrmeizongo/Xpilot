#ifndef _RADIO_H
#define _RADIO_H

#include <inttypes.h>
#include <PlaneConfig.h>

enum FlightMode : uint8_t
{
    PASSTHROUGH = 1U,
    RATE,
    STABILIZE
};

struct Control
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t rollPWM;
    uint16_t pitchPWM;
    uint16_t yawPWM;
    FlightMode currentMode;

    Control(void)
        : roll{0}, pitch{0}, yaw{0},
          rollPWM{INPUT_MID_PWM}, pitchPWM{INPUT_MID_PWM}, yawPWM{INPUT_MID_PWM},
          currentMode{RATE} {}

    Control(int16_t _roll, int16_t _pitch, int16_t _yaw, FlightMode _currentMode)
        : roll{_roll}, pitch{_pitch}, yaw{_yaw},
          rollPWM{INPUT_MID_PWM}, pitchPWM{INPUT_MID_PWM}, yawPWM{INPUT_MID_PWM},
          currentMode{_currentMode} {}
};

class Radio
{
public:
    Radio(void);
    void init(void);
    void processInput(void);

    int16_t getRxRoll(void) { return rx.roll; }
    int16_t getRxPitch(void) { return rx.pitch; }
    int16_t getRxYaw(void) { return rx.yaw; }

    int16_t getRxRollPWM(void) { return rx.rollPWM; }
    int16_t getRxPitchPWM(void) { return rx.pitchPWM; }
    int16_t getRxYawPWM(void) { return rx.yawPWM; }

    FlightMode getRxCurrentMode(void) { return rx.currentMode; }

private:
    Control rx;
};

extern Radio radio;
#endif