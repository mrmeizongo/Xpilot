#ifndef _ACTUATORS_H
#define _ACTUATORS_H
#include <Servo.h>

class Actuators
{
public:
    enum class ControlSurface : uint8_t
    {
        AILERON1 = 0,
        AILERON2,
        ELEVATOR,
        RUDDER
    };

    Actuators(void);
    void init(void);
    void writeServo(ControlSurface, int16_t);

private:
    Servo aileron1Servo; // Aileron servo channel
    Servo aileron2Servo; // Aileron servo channel
    Servo elevatorServo; // Elevator servo channel
    Servo rudderServo;   // Rudder servo channel
};

extern Actuators actuators;

#endif //_ACTUATORS_H