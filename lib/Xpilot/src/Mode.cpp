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

/*
 * Mixer for airplane type
 * Only tested with a full plane(traditional & V-tail) i.e. ailerons, elevator and rudder
 * Proceed with caution. Perform thorough pre-flight checks and reverse servo direction as needed.
 */
void Mode::planeMixer(const int16_t roll, const int16_t pitch, const int16_t yaw)
{
#if defined(FULL_PLANE_TRADITIONAL)
    SRVout[Actuators::Channel::AILERON1] = roll;
    SRVout[Actuators::Channel::AILERON2] = roll;
    SRVout[Actuators::Channel::ELEVATOR] = pitch;
    SRVout[Actuators::Channel::RUDDER] = yaw;
#elif defined(FULL_PLANE_V_TAIL)
    SRVout[Actuators::Channel::AILERON1] = roll;
    SRVout[Actuators::Channel::AILERON2] = roll;
    SRVout[Actuators::Channel::ELEVATOR] = pitch + yaw;
    SRVout[Actuators::Channel::RUDDER] = yaw - pitch;
#elif defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
    SRVout[Actuators::Channel::AILERON1] = 0;
    SRVout[Actuators::Channel::AILERON2] = 0;
    SRVout[Actuators::Channel::ELEVATOR] = pitch + yaw;
    SRVout[Actuators::Channel::RUDDER] = yaw - pitch;
#elif defined(FLYING_WING_W_RUDDER)
    SRVout[Actuators::Channel::AILERON1] = roll + pitch;
    SRVout[Actuators::Channel::AILERON2] = roll - pitch;
    SRVout[Actuators::Channel::ELEVATOR] = 0;
    SRVout[Actuators::Channel::RUDDER] = yaw;
#elif defined(FLYING_WING_NO_RUDDER)
    SRVout[Actuators::Channel::AILERON1] = roll + pitch;
    SRVout[Actuators::Channel::AILERON2] = roll - pitch;
    SRVout[Actuators::Channel::ELEVATOR] = 0;
    SRVout[Actuators::Channel::RUDDER] = 0;
#elif defined(RUDDER_ELEVATOR_ONLY_PLANE)
    SRVout[Actuators::Channel::AILERON1] = 0;
    SRVout[Actuators::Channel::AILERON2] = 0;
    SRVout[Actuators::Channel::ELEVATOR] = pitch;
    SRVout[Actuators::Channel::RUDDER] = yaw;
#elif defined(AILERON_ELEVATOR_ONLY)
    SRVout[Actuators::Channel::AILERON1] = roll;
    SRVout[Actuators::Channel::AILERON2] = roll;
    SRVout[Actuators::Channel::ELEVATOR] = pitch;
    SRVout[Actuators::Channel::RUDDER] = 0;
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

void Mode::setServoOut(void)
{
    SRVout[Actuators::Channel::AILERON1] = constrain(SRVout[Actuators::Channel::AILERON1], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::AILERON2] = constrain(SRVout[Actuators::Channel::AILERON2], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::ELEVATOR] = constrain(SRVout[Actuators::Channel::ELEVATOR], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::RUDDER] = constrain(SRVout[Actuators::Channel::RUDDER], SERVO_MIN_PWM, SERVO_MAX_PWM);
    actuators.setServoOut(SRVout);
}

void Mode::controlFailsafe(void)
{
    // Default failsafe implementation
    rollOut = 0;
    pitchOut = 0;
    yawOut = 0;
}