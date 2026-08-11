#include "Mode.h"
#include "IMU.h"

// ISO C++ forbids in-class initialization of non-const static members
// We define them here instead
int16_t Mode::input_rpy[3]{0, 0, 0};
int16_t Mode::output_rpy[3]{0, 0, 0};
int16_t Mode::SRVout[Actuators::Channel::NUM_CHANNELS]{0, 0, 0, 0};
PIDF<int16_t> Mode::rollPIDF{ROLL_KP, ROLL_KI, ROLL_KD, ROLL_KF, ROLL_I_WINDUP_MAX, PROCESS_DT, AUTO_LPF_FREQ};
PIDF<int16_t> Mode::pitchPIDF{PITCH_KP, PITCH_KI, PITCH_KD, PITCH_KF, PITCH_I_WINDUP_MAX, PROCESS_DT, AUTO_LPF_FREQ};
PIDF<int16_t> Mode::yawPIDF{YAW_KP, YAW_KI, YAW_KD, YAW_KF, YAW_I_WINDUP_MAX, PROCESS_DT, AUTO_LPF_FREQ};
uint8_t Mode::missedImuInstances = 0;
bool Mode::imuFault = false;
AirplaneMixer Mode::airplaneMixer{};
#if defined(USE_FLAPERONS)
uint16_t Mode::flaperonOut = 0;
#endif
// --------------------------------------------------------------------------------------

Mode::Mode()
{
#if defined(FULL_TRADITIONAL_PLANE)
    airplaneMixer.setAirframeType(AirplaneMixer::AirframeType::CONVENTIONAL);
#elif defined(FULL_V_TAIL_PLANE)
    airplaneMixer.setAirframeType(AirplaneMixer::AirframeType::V_TAIL);
#elif defined(RUDDER_ELEVATOR_ONLY_PLANE)
    airplaneMixer.setAirframeType(AirplaneMixer::AirframeType::RUDDER_ELEVATOR);
#elif defined(AILERON_ELEVATOR_ONLY_PLANE)
    airplaneMixer.setAirframeType(AirplaneMixer::AirframeType::AILERON_ELEVATOR);
#elif defined(FLYING_WING_W_RUDDER_PLANE)
    airplaneMixer.setAirframeType(AirplaneMixer::AirframeType::FLYING_WING_RUDDER);
#elif defined(FLYING_WING_NO_RUDDER_PLANE)
    airplaneMixer.setAirframeType(AirplaneMixer::AirframeType::FLYING_WING_NO_RUDDER);
#else
#error No airplane type selected!
#endif
}

void Mode::rudderMixer(void)
{
#if defined(FULL_TRADITIONAL_PLANE) || defined(FULL_V_TAIL_PLANE) || defined(FLYING_WING_W_RUDDER_PLANE)
#if defined(REVERSE_RUDDER_MIX)
    output_rpy[2] = output_rpy[2] - (output_rpy[0] * RUDDER_MIXING);
#else
    output_rpy[2] = output_rpy[2] + (output_rpy[0] * RUDDER_MIXING);
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