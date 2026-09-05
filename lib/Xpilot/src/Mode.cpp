#include "Mode.h"
#include "IMU.h"

void Mode::init(void)
{
    // Changing airframe type requires a reset to take effect
    airplaneMixer.setAirframeType(config().airframeConfig.type);

    rollSlew = SlewRateLimiter<int32_t, int16_t>{config().filterConfig.controlSlewRate, config().filterConfig.processDT};
    pitchSlew = SlewRateLimiter<int32_t, int16_t>{config().filterConfig.controlSlewRate, config().filterConfig.processDT};
    yawSlew = SlewRateLimiter<int32_t, int16_t>{config().filterConfig.controlSlewRate, config().filterConfig.processDT};

    rollPIDF = PIDF<int32_t, int16_t>{config().rPIDFConfig.Kp / Control::RESOLUTION,
                                      config().rPIDFConfig.Ki / Control::RESOLUTION,
                                      config().rPIDFConfig.Kd / Control::RESOLUTION,
                                      config().rPIDFConfig.Kf / Control::RESOLUTION,
                                      config().rPIDFConfig.iWindUpMax,
                                      config().filterConfig.processDT,
                                      config().filterConfig.lowPassFilterFreq};

    pitchPIDF = PIDF<int32_t, int16_t>{config().pPIDFConfig.Kp / Control::RESOLUTION,
                                       config().pPIDFConfig.Ki / Control::RESOLUTION,
                                       config().pPIDFConfig.Kd / Control::RESOLUTION,
                                       config().pPIDFConfig.Kf / Control::RESOLUTION,
                                       config().pPIDFConfig.iWindUpMax,
                                       config().filterConfig.processDT,
                                       config().filterConfig.lowPassFilterFreq};

    yawPIDF = PIDF<int32_t, int16_t>{config().yPIDFConfig.Kp / Control::RESOLUTION,
                                     config().yPIDFConfig.Ki / Control::RESOLUTION,
                                     config().yPIDFConfig.Kd / Control::RESOLUTION,
                                     config().yPIDFConfig.Kf / Control::RESOLUTION,
                                     config().yPIDFConfig.iWindUpMax,
                                     config().filterConfig.processDT,
                                     config().filterConfig.lowPassFilterFreq};

    imu.registerConsumer(updateAHRS);
    configManager.registerSubscriber(configSub, this);
}

void Mode::configSub(ConfigID id, void* ctx)
{
    (void)ctx;

    switch (id)
    {
        case ConfigID::AIRFRAME_TYPE:
            airplaneMixer.setAirframeType(config().airframeConfig.type);
            break;

        case ConfigID::FILTER_SLEW_RATE:
            rollSlew.setRate(config().filterConfig.controlSlewRate);
            pitchSlew.setRate(config().filterConfig.controlSlewRate);
            yawSlew.setRate(config().filterConfig.controlSlewRate);
            break;

        case ConfigID::PIDF_ROLL_KP:
            rollPIDF.setKp(config().rPIDFConfig.Kp);
            break;

        case ConfigID::PIDF_ROLL_KI:
            rollPIDF.setKi(config().rPIDFConfig.Ki);
            break;

        case ConfigID::PIDF_ROLL_KD:
            rollPIDF.setKd(config().rPIDFConfig.Kd);
            break;

        case ConfigID::PIDF_ROLL_KF:
            rollPIDF.setKf(config().rPIDFConfig.Kf);
            break;

        case ConfigID::PIDF_ROLL_I_WINDUP_MAX:
            rollPIDF.setIMax(config().rPIDFConfig.iWindUpMax);
            break;

        case ConfigID::PIDF_PITCH_KP:
            pitchPIDF.setKp(config().pPIDFConfig.Kp);
            break;

        case ConfigID::PIDF_PITCH_KI:
            pitchPIDF.setKi(config().pPIDFConfig.Ki);
            break;

        case ConfigID::PIDF_PITCH_KD:
            pitchPIDF.setKd(config().pPIDFConfig.Kd);
            break;

        case ConfigID::PIDF_PITCH_KF:
            pitchPIDF.setKf(config().pPIDFConfig.Kf);
            break;

        case ConfigID::PIDF_PITCH_I_WINDUP_MAX:
            pitchPIDF.setIMax(config().pPIDFConfig.iWindUpMax);
            break;

        case ConfigID::PIDF_YAW_KP:
            yawPIDF.setKp(config().yPIDFConfig.Kp);
            break;

        case ConfigID::PIDF_YAW_KI:
            yawPIDF.setKi(config().yPIDFConfig.Ki);
            break;

        case ConfigID::PIDF_YAW_KD:
            yawPIDF.setKd(config().yPIDFConfig.Kd);
            break;

        case ConfigID::PIDF_YAW_KF:
            yawPIDF.setKf(config().yPIDFConfig.Kf);
            break;

        case ConfigID::PIDF_YAW_I_WINDUP_MAX:
            yawPIDF.setIMax(config().yPIDFConfig.iWindUpMax);
            break;

        default:
            break;
    }
}

void Mode::update(void)
{
    if (radio.inFailsafe())
    {
        controlFailsafe();
        return;
    }

    input_rpy[0] = normalizeInput(radio.getPWM(Radio::CHANNEL::ROLL),
                                  config().rollRxConfig.min,
                                  config().rollRxConfig.trim,
                                  config().rollRxConfig.max,
                                  config().rollRxConfig.deadband);

    input_rpy[1] = normalizeInput(radio.getPWM(Radio::CHANNEL::PITCH),
                                  config().pitchRxConfig.min,
                                  config().pitchRxConfig.trim,
                                  config().pitchRxConfig.max,
                                  config().pitchRxConfig.deadband);

    input_rpy[2] = normalizeInput(radio.getPWM(Radio::CHANNEL::YAW),
                                  config().yawRxConfig.min,
                                  config().yawRxConfig.trim,
                                  config().yawRxConfig.max,
                                  config().yawRxConfig.deadband);
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

void Mode::updateInput(void* ctx)
{
    Mode** modePointer = static_cast<Mode**>(ctx);

    (*modePointer)->update();
}

void Mode::runTask(void* ctx)
{
    Mode** modePointer = static_cast<Mode**>(ctx);

    (*modePointer)->run();
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