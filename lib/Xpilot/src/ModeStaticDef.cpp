#include "Mode.h"

/**
 * ISO C++ forbids in-class initialization of non-const static members
 * We define them here instead
 */

int32_t Mode::input_trpy[4]{};
int16_t Mode::output_trpy[4]{};

AirplaneMixer::Outputs Mode::mixerOutputs{};

int32_t Mode::imu_rpy[3]{};
int32_t Mode::imu_g[3]{};

int16_t Mode::SRVout[Actuators::CHANNEL::CHANNEL_COUNT]{};

PIDF<int32_t, int16_t> Mode::rollPIDF{};
PIDF<int32_t, int16_t> Mode::pitchPIDF{};
PIDF<int32_t, int16_t> Mode::yawPIDF{};

AirplaneMixer Mode::airplaneMixer{};

SlewRateLimiter<int32_t> Mode::rollSlew{};
SlewRateLimiter<int32_t> Mode::pitchSlew{};
SlewRateLimiter<int32_t> Mode::yawSlew{};

int16_t Mode::flaperonInput{};