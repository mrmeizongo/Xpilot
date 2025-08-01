#include "Mode.h"

// ISO C++ forbids in-class initialization of non-const static members
// We define them here instead
int16_t Mode::rollOut = 0;
int16_t Mode::pitchOut = 0;
int16_t Mode::yawOut = 0;
int16_t Mode::SRVout[Actuators::Channel::NUM_CHANNELS]{0, 0, 0, 0};
PIDF Mode::rollPIDF{ROLL_KP, ROLL_KI, ROLL_KD, ROLL_KF, ROLL_I_WINDUP_MAX};
PIDF Mode::pitchPIDF{PITCH_KP, PITCH_KI, PITCH_KD, PITCH_KF, PITCH_I_WINDUP_MAX};
PIDF Mode::yawPIDF{YAW_KP, YAW_KI, YAW_KD, YAW_KF, YAW_I_WINDUP_MAX};
#if defined(USE_FLAPERONS)
uint16_t Mode::flaperonOut = 0;
#endif

/*
 * Mixer for airplane type
 * Only tested with a full plane(traditional & V-tail) i.e. ailerons, elevator and rudder
 * Proceed with caution. Perform thorough pre-flight checks and reverse servo direction as needed.
 */
void Mode::planeMixer(const int16_t roll, const int16_t pitch, const int16_t yaw)
{
#if defined(FULL_PLANE_TRADITIONAL)
    SRVout[Actuators::Channel::CH1] = roll;
    SRVout[Actuators::Channel::CH2] = roll;
    SRVout[Actuators::Channel::CH3] = pitch;
    SRVout[Actuators::Channel::CH4] = yaw;
#elif defined(FULL_PLANE_V_TAIL)
    SRVout[Actuators::Channel::CH1] = roll;
    SRVout[Actuators::Channel::CH2] = roll;
    SRVout[Actuators::Channel::CH3] = pitch + yaw;
    SRVout[Actuators::Channel::CH4] = yaw - pitch;
#elif defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
    SRVout[Actuators::Channel::CH1] = 0;
    SRVout[Actuators::Channel::CH2] = 0;
    SRVout[Actuators::Channel::CH3] = pitch + yaw;
    SRVout[Actuators::Channel::CH4] = yaw - pitch;
#elif defined(FLYING_WING_W_RUDDER)
    SRVout[Actuators::Channel::CH1] = roll + pitch;
    SRVout[Actuators::Channel::CH2] = roll - pitch;
    SRVout[Actuators::Channel::CH3] = 0;
    SRVout[Actuators::Channel::CH4] = yaw;
#elif defined(FLYING_WING_NO_RUDDER)
    SRVout[Actuators::Channel::CH1] = roll + pitch;
    SRVout[Actuators::Channel::CH2] = roll - pitch;
    SRVout[Actuators::Channel::CH3] = 0;
    SRVout[Actuators::Channel::CH4] = 0;
#elif defined(RUDDER_ELEVATOR_ONLY_PLANE)
    SRVout[Actuators::Channel::CH1] = 0;
    SRVout[Actuators::Channel::CH2] = 0;
    SRVout[Actuators::Channel::CH3] = pitch;
    SRVout[Actuators::Channel::CH4] = yaw;
#elif defined(AILERON_ELEVATOR_ONLY)
    SRVout[Actuators::Channel::CH1] = roll;
    SRVout[Actuators::Channel::CH2] = roll;
    SRVout[Actuators::Channel::CH3] = pitch;
    SRVout[Actuators::Channel::CH4] = 0;
#else
#error No airplane type selected!
#endif
}

void Mode::rudderMixer(void)
{
#if defined(FULL_PLANE) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_W_RUDDER)
#if defined(REVERSE_RUDDER_MIX)
    yawOut = yawOut - (rollOut * RUDDER_MIXING);
#else
    yawOut = yawOut + (rollOut * RUDDER_MIXING);
#endif
#endif
}

#if defined(USE_FLAPERONS)
void Mode::setFlaperons(void)
{
    SRVout[Actuators::Channel::CH1] += flaperonOut;
    SRVout[Actuators::Channel::CH2] -= flaperonOut;
}
#endif

void Mode::setServoOut(void)
{
    SRVout[Actuators::Channel::CH1] = constrain(SRVout[Actuators::Channel::CH1], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::CH2] = constrain(SRVout[Actuators::Channel::CH2], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::CH3] = constrain(SRVout[Actuators::Channel::CH3], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::CH4] = constrain(SRVout[Actuators::Channel::CH4], SERVO_MIN_PWM, SERVO_MAX_PWM);
    actuators.setServoOut(SRVout);
}

void Mode::controlFailsafe(void)
{
    // Default failsafe implementation
    rollOut = 0;
    pitchOut = 0;
    yawOut = 0;
#if defined(USE_FLAPERONS)
    flaperonOut = 0;
#endif
}