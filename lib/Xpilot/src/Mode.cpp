#include "Mode.h"
#include "IMU.h"

// ISO C++ forbids in-class initialization of non-const static members
// We define them here instead
int16_t Mode::input_rpy[3]{0, 0, 0};
int16_t Mode::output_rpy[3]{0, 0, 0};
float Mode::imu_rpy[3]{0.f, 0.f, 0.f};
float Mode::imu_g[3]{0.f, 0.f, 0.f};
int16_t Mode::SRVout[Actuators::Channel::NUM_CHANNELS]{0, 0, 0, 0};
PIDF<int16_t> Mode::rollPIDF{ROLL_KP, ROLL_KI, ROLL_KD, ROLL_KF, ROLL_I_WINDUP_MAX, PROCESS_DT, AUTO_LPF_FREQ};
PIDF<int16_t> Mode::pitchPIDF{PITCH_KP, PITCH_KI, PITCH_KD, PITCH_KF, PITCH_I_WINDUP_MAX, PROCESS_DT, AUTO_LPF_FREQ};
PIDF<int16_t> Mode::yawPIDF{YAW_KP, YAW_KI, YAW_KD, YAW_KF, YAW_I_WINDUP_MAX, PROCESS_DT, AUTO_LPF_FREQ};
AirplaneMixer Mode::airplaneMixer{};
#if defined(USE_FLAPERONS)
uint16_t Mode::flaperonOut = 0;
#endif
// --------------------------------------------------------------------------------------

void Mode::init(void)
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

    imu.registerConsumer(updateAHRS, this);
}

void Mode::update(void)
{
#if defined(USE_FLAPERONS)
    flaperonInput();
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
void Mode::flaperonMixer(void)
{
    SRVout[Actuators::Channel::CH1] -= flaperonOut;
    SRVout[Actuators::Channel::CH2] += flaperonOut;
}
#endif

void Mode::updateInput(void *ctx)
{
    Mode **modePointer = static_cast<Mode **>(ctx);
    if (*modePointer != nullptr)
    {
        (*modePointer)->update();
    }
}

void Mode::runTask(void *ctx)
{
    Mode **modePointer = static_cast<Mode **>(ctx);
    if (*modePointer != nullptr)
    {
        (*modePointer)->run();
    }
}

void Mode::updateAHRS(float (&rpy)[3], float (&g)[3], void *ctx)
{
    (void)ctx; // Discard ctx since this is a static function that can be called directly
    imu_rpy[0] = rpy[0];
    imu_rpy[1] = rpy[1];
    imu_rpy[2] = rpy[2];

    imu_g[0] = g[0];
    imu_g[1] = g[1];
    imu_g[2] = g[2];
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