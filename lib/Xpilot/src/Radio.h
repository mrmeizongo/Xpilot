#ifndef _RADIO_H
#define _RADIO_H

#include <inttypes.h>
#include <PlaneConfig.h>

/*
 * Radio input values
 * Roll, Pitch, Yaw, Mode
 * Depending on transmitter settings, the pulse width can be between INPUT_MIN_PWM <-> INPUT_MAX_PWM
 * The pulse width is read in microseconds
 * When used with a 3 position switch, the mid point is considered 1, the low end is 0 and the high end is 2
 * Low Point: 0 - Passthrough, Mid Point: 1 - Rate, High Point: 2 - Stabilize
 */
enum FlightMode : uint8_t
{
    PASSTHROUGH = 1U,
    RATE,
    STABILIZE
};

/*
 * Control struct
 * Holds the radio input values
 * Roll, Pitch, Yaw, Mode
 * Roll, Pitch & Yaw PWM values
 */
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
        : Control(RATE) {}

    Control(FlightMode _currentMode)
        : roll{0}, pitch{0}, yaw{0},
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