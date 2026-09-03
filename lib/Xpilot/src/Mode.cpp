#include "Mode.h"
#include "IMU.h"

void Mode::init(void)
{
    // Changing airframe type requires a reset to take effect
    airplaneMixer.setAirframeType(config().airframeType.type);

    rollSlew = SlewRateLimiter<int32_t>{config().processFilter.controlSlewRate, config().processFilter.processDT};
    pitchSlew = SlewRateLimiter<int32_t>{config().processFilter.controlSlewRate, config().processFilter.processDT};
    yawSlew = SlewRateLimiter<int32_t>{config().processFilter.controlSlewRate, config().processFilter.processDT};

    rollPIDF = PIDF<int32_t>{config().rollPIDF.Kp, config().rollPIDF.Ki, config().rollPIDF.Kd, config().rollPIDF.Kf,
                             config().rollPIDF.iWindUpMax, config().processFilter.processDT, config().processFilter.lowPassFilterFreq};

    pitchPIDF = PIDF<int32_t>{config().pitchPIDF.Kp, config().pitchPIDF.Ki, config().pitchPIDF.Kd, config().pitchPIDF.Kf,
                              config().pitchPIDF.iWindUpMax, config().processFilter.processDT, config().processFilter.lowPassFilterFreq};

    yawPIDF = PIDF<int32_t>{config().yawPIDF.Kp, config().yawPIDF.Ki, config().yawPIDF.Kd, config().yawPIDF.Kf,
                            config().yawPIDF.iWindUpMax, config().processFilter.processDT, config().processFilter.lowPassFilterFreq};

    imu.registerConsumer(updateAHRS);
    configManager.registerSubscriber(configSub, this);
}

void Mode::configSub(ConfigID id, void *ctx)
{
    (void)ctx;

    switch (id)
    {
    case ConfigID::AIRFRAME_TYPE:
        airplaneMixer.setAirframeType(config().airframeType.type);
        break;

    case ConfigID::FLIGHT_CONTROL_RES:
        airplaneMixer.setCommandLimit(config().flightConfig.controlResolution);
        break;

    case ConfigID::FILTER_SLEW_RATE:
        rollSlew.setRate(config().processFilter.controlSlewRate);
        pitchSlew.setRate(config().processFilter.controlSlewRate);
        yawSlew.setRate(config().processFilter.controlSlewRate);
        break;

    case ConfigID::PIDF_ROLL_KP:
        rollPIDF.setKp(config().rollPIDF.Kp);
        break;

    case ConfigID::PIDF_ROLL_KI:
        rollPIDF.setKi(config().rollPIDF.Ki);
        break;

    case ConfigID::PIDF_ROLL_KD:
        rollPIDF.setKd(config().rollPIDF.Kd);
        break;

    case ConfigID::PIDF_ROLL_KF:
        rollPIDF.setKf(config().rollPIDF.Kf);
        break;

    case ConfigID::PIDF_ROLL_I_WINDUP_MAX:
        rollPIDF.setIMax(config().rollPIDF.iWindUpMax);
        break;

    case ConfigID::PIDF_PITCH_KP:
        pitchPIDF.setKp(config().pitchPIDF.Kp);
        break;

    case ConfigID::PIDF_PITCH_KI:
        pitchPIDF.setKi(config().pitchPIDF.Ki);
        break;

    case ConfigID::PIDF_PITCH_KD:
        pitchPIDF.setKd(config().pitchPIDF.Kd);
        break;

    case ConfigID::PIDF_PITCH_KF:
        pitchPIDF.setKf(config().pitchPIDF.Kf);
        break;

    case ConfigID::PIDF_PITCH_I_WINDUP_MAX:
        pitchPIDF.setIMax(config().pitchPIDF.iWindUpMax);
        break;

    case ConfigID::PIDF_YAW_KP:
        yawPIDF.setKp(config().yawPIDF.Kp);
        break;

    case ConfigID::PIDF_YAW_KI:
        yawPIDF.setKi(config().yawPIDF.Ki);
        break;

    case ConfigID::PIDF_YAW_KD:
        yawPIDF.setKd(config().yawPIDF.Kd);
        break;

    case ConfigID::PIDF_YAW_KF:
        yawPIDF.setKf(config().yawPIDF.Kf);
        break;

    case ConfigID::PIDF_YAW_I_WINDUP_MAX:
        yawPIDF.setIMax(config().yawPIDF.iWindUpMax);
        break;

    default:
        break;
    }
}

void Mode::update(void)
{
#if defined(USE_FLAPERONS)
    int16_t flapPwm = radio.getPWM(Radio::CHANNEL::AUX2);
    flapPwm = constrain(flapPwm, RX_PWM_MIN, RX_PWM_TRIM);
    flaperonOut = static_cast<int16_t>((RX_PWM_TRIM - flapPwm) * config().flightConfig.flaperonScaleFactor);
#endif
}

void Mode::applyRudderMix(void)
{
    int16_t contribution = static_cast<int16_t>(input_rpy[0] * config().flightConfig.rudderMixScale);

    input_rpy[2] += config().flightConfig.reverseRudderMix ? -contribution : contribution;
}

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

void Mode::updateAHRS(float (&rpy)[3], float (&g)[3])
{
    imu_rpy[0] = rpy[0] * Control::RESOLUTION;
    imu_rpy[1] = rpy[1] * Control::RESOLUTION;
    imu_rpy[2] = rpy[2] * Control::RESOLUTION;

    imu_g[0] = g[0] * Control::RESOLUTION;
    imu_g[1] = g[1] * Control::RESOLUTION;
    imu_g[2] = g[2] * Control::RESOLUTION;
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
    flaperonOut = config().flightConfig.flaperonMax; // set flaperons to landing position
#endif
}