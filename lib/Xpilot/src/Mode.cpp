#include "Mode.h"
#include "IMU.h"

// ISO C++ forbids in-class initialization of non-const static members
// We define them here instead
int16_t Mode::input_rpy[3]{0, 0, 0};
int16_t Mode::output_rpy[3]{0, 0, 0};
int16_t Mode::SRVout[Actuators::Channel::NUM_CHANNELS]{0, 0, 0, 0};
PIDF<int16_t> Mode::rollPIDF{ROLL_KP, ROLL_KI, ROLL_KD, ROLL_KF, ROLL_I_WINDUP_MAX, LPF_DT, LPF_FREQ};
PIDF<int16_t> Mode::pitchPIDF{PITCH_KP, PITCH_KI, PITCH_KD, PITCH_KF, PITCH_I_WINDUP_MAX, LPF_DT, LPF_FREQ};
PIDF<int16_t> Mode::yawPIDF{YAW_KP, YAW_KI, YAW_KD, YAW_KF, YAW_I_WINDUP_MAX, LPF_DT, LPF_FREQ};
uint8_t Mode::missedImuInstances = 0;
bool Mode::imuFault = false;
#if defined(USE_FLAPERONS)
uint16_t Mode::flaperonOut = 0;
#endif
// --------------------------------------------------------------------------------------

/*
 * Mixer for airplane type
 * Only tested with a full plane(traditional & V-tail) i.e. ailerons, elevator and rudder
 * Proceed with caution. Perform thorough pre-flight checks and reverse servo direction as needed.
 * Mixing is only performed for the 4 primary channels(aileron left, aileron right, elevator, rudder)
 */
void Mode::planeMixer(const int16_t roll, const int16_t pitch, const int16_t yaw)
{
#if defined(FULL_PLANE_TRADITIONAL) || defined(RUDDER_ELEVATOR_ONLY_PLANE) || defined(AILERON_ELEVATOR_ONLY)
    SRVout[Actuators::Channel::CH1] = roll;
    SRVout[Actuators::Channel::CH2] = roll;
    SRVout[Actuators::Channel::CH3] = pitch;
    SRVout[Actuators::Channel::CH4] = yaw;
#elif defined(FULL_PLANE_V_TAIL) || defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
    SRVout[Actuators::Channel::CH1] = roll;
    SRVout[Actuators::Channel::CH2] = roll;
    SRVout[Actuators::Channel::CH3] = pitch + yaw;
    SRVout[Actuators::Channel::CH4] = yaw - pitch;
#elif defined(FLYING_WING_W_RUDDER) || defined(FLYING_WING_NO_RUDDER)
    SRVout[Actuators::Channel::CH1] = roll + pitch;
    SRVout[Actuators::Channel::CH2] = roll - pitch;
    SRVout[Actuators::Channel::CH3] = pitch;
    SRVout[Actuators::Channel::CH4] = yaw;
#else
#error No airplane type selected!
#endif
}

void Mode::rudderMixer(void)
{
#if defined(FULL_PLANE) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_W_RUDDER)
#if defined(REVERSE_RUDDER_MIX)
    input_rpy[2] = input_rpy[2] - (input_rpy[0] * RUDDER_MIXING);
#else
    input_rpy[2] = input_rpy[2] + (input_rpy[0] * RUDDER_MIXING);
#endif
#endif
}

#if defined(USE_FLAPERONS)
void Mode::setFlaperons(void)
{
    SRVout[Actuators::Channel::CH1] -= flaperonOut;
    SRVout[Actuators::Channel::CH2] += flaperonOut;
}

void Mode::flaperonInput(void)
{
    flaperonOut = GETRAWINPUT(radio.getRxAux2PWM(), INPUT_MID_PWM, INPUT_MIN_PWM, 0, FLAPERON_MAX_RANGE);
}
#endif

bool Mode::imuDataHealthy(void)
{
    if (!imu.consumeNewData())
    {
        /*
         * This could be a sign of a faulty imu sensor
         * Threshold is 2 full loops gone without sensor values
         * It does not reset until a full system reboot is performed
         */
        if (++missedImuInstances == MISSED_IMU_VAL_THRESH * 2)
        {
            imuFault = true;
        }
        return false;
    }

    return true;
}

void Mode::servoOut(void *)
{
    SRVout[Actuators::Channel::CH1] = constrain(SRVout[Actuators::Channel::CH1], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::CH2] = constrain(SRVout[Actuators::Channel::CH2], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::CH3] = constrain(SRVout[Actuators::Channel::CH3], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::CH4] = constrain(SRVout[Actuators::Channel::CH4], SERVO_MIN_PWM, SERVO_MAX_PWM);
#if defined(USE_AUXOUT1)
    SRVout[Actuators::Channel::CH5] = constrain(SRVout[Actuators::Channel::CH5], SERVO_MIN_PWM, SERVO_MAX_PWM);
#endif
    actuators.writeServos(SRVout); // Write servo outputs to the actuators object
}

void Mode::resetControllers(void)
{
    rollPIDF.reset();
    pitchPIDF.reset();
    yawPIDF.reset();
}

void Mode::controlFailsafe(void)
{
    // Default failsafe implementation
    input_rpy[0] = 0;
    input_rpy[1] = 0;
    input_rpy[2] = 0;
#if defined(USE_FLAPERONS)
    flaperonOut = FLAPERON_MAX_RANGE; // set flaperons to landing position
#endif
}