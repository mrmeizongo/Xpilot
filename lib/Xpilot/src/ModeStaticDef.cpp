#include "Mode.h"

/**
 * ISO C++ forbids in-class initialization of non-const static members
 * We define them here instead
 */

int32_t Mode::input_rpy[3]{0, 0, 0};
int16_t Mode::output_rpy[3]{0, 0, 0};

AirplaneMixer::Outputs Mode::mixerOutputs{0, 0, 0, 0};

int32_t Mode::imu_rpy[3]{0, 0, 0};
int32_t Mode::imu_g[3]{0, 0, 0};

int16_t Mode::SRVout[Actuators::Channel::CHANNEL_COUNT]{0, 0, 0, 0};

PIDF<int32_t, int16_t> Mode::rollPIDF;
PIDF<int32_t, int16_t> Mode::pitchPIDF;
PIDF<int32_t, int16_t> Mode::yawPIDF;

AirplaneMixer Mode::airplaneMixer;

SlewRateLimiter<int32_t, int16_t> Mode::rollSlew;
SlewRateLimiter<int32_t, int16_t> Mode::pitchSlew;
SlewRateLimiter<int32_t, int16_t> Mode::yawSlew;

#if defined(USE_FLAPERONS)
int16_t Mode::flaperonOut = 0;
#endif