#include "Mode.h"
#include "IMU.h"

void Mode::init(void)
{
    airplaneMixer.setAirframeType(config().airframeConfig.type);

    rollSlew = SlewRateLimiter<int32_t, int16_t>{config().controlConfig.controlSlewRate, config().controlConfig.dt};
    pitchSlew = SlewRateLimiter<int32_t, int16_t>{config().controlConfig.controlSlewRate, config().controlConfig.dt};
    yawSlew = SlewRateLimiter<int32_t, int16_t>{config().controlConfig.controlSlewRate, config().controlConfig.dt};

    rollPIDF = PIDF<int32_t, int16_t>{config().rPIDFConfig.Kp / config().controlConfig.controlResolution,
                                      config().rPIDFConfig.Ki / config().controlConfig.controlResolution,
                                      config().rPIDFConfig.Kd / config().controlConfig.controlResolution,
                                      config().rPIDFConfig.Kf / config().controlConfig.controlResolution,
                                      config().rPIDFConfig.iWindUpMax,
                                      config().controlConfig.dt,
                                      config().controlConfig.lowPassFilterFreq};

    pitchPIDF = PIDF<int32_t, int16_t>{config().pPIDFConfig.Kp / config().controlConfig.controlResolution,
                                       config().pPIDFConfig.Ki / config().controlConfig.controlResolution,
                                       config().pPIDFConfig.Kd / config().controlConfig.controlResolution,
                                       config().pPIDFConfig.Kf / config().controlConfig.controlResolution,
                                       config().pPIDFConfig.iWindUpMax,
                                       config().controlConfig.dt,
                                       config().controlConfig.lowPassFilterFreq};

    yawPIDF = PIDF<int32_t, int16_t>{config().yPIDFConfig.Kp / config().controlConfig.controlResolution,
                                     config().yPIDFConfig.Ki / config().controlConfig.controlResolution,
                                     config().yPIDFConfig.Kd / config().controlConfig.controlResolution,
                                     config().yPIDFConfig.Kf / config().controlConfig.controlResolution,
                                     config().yPIDFConfig.iWindUpMax,
                                     config().controlConfig.dt,
                                     config().controlConfig.lowPassFilterFreq};

    imu.registerConsumer(consumeAHRS);
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

        case ConfigID::CONTROL_SLEW_RATE:
            rollSlew.setRate(config().controlConfig.controlSlewRate);
            pitchSlew.setRate(config().controlConfig.controlSlewRate);
            yawSlew.setRate(config().controlConfig.controlSlewRate);
            break;

        case ConfigID::PIDF_ROLL_KP:
            rollPIDF.setKp(config().rPIDFConfig.Kp / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_ROLL_KI:
            rollPIDF.setKi(config().rPIDFConfig.Ki / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_ROLL_KD:
            rollPIDF.setKd(config().rPIDFConfig.Kd / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_ROLL_KF:
            rollPIDF.setKf(config().rPIDFConfig.Kf / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_ROLL_I_WINDUP_MAX:
            rollPIDF.setIMax(config().rPIDFConfig.iWindUpMax);
            break;

        case ConfigID::PIDF_PITCH_KP:
            pitchPIDF.setKp(config().pPIDFConfig.Kp / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_PITCH_KI:
            pitchPIDF.setKi(config().pPIDFConfig.Ki / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_PITCH_KD:
            pitchPIDF.setKd(config().pPIDFConfig.Kd / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_PITCH_KF:
            pitchPIDF.setKf(config().pPIDFConfig.Kf / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_PITCH_I_WINDUP_MAX:
            pitchPIDF.setIMax(config().pPIDFConfig.iWindUpMax);
            break;

        case ConfigID::PIDF_YAW_KP:
            yawPIDF.setKp(config().yPIDFConfig.Kp / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_YAW_KI:
            yawPIDF.setKi(config().yPIDFConfig.Ki / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_YAW_KD:
            yawPIDF.setKd(config().yPIDFConfig.Kd / config().controlConfig.controlResolution);
            break;

        case ConfigID::PIDF_YAW_KF:
            yawPIDF.setKf(config().yPIDFConfig.Kf / config().controlConfig.controlResolution);
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
        setFailsafeInputs();
        return;
    }

    input_rpy[0] = normalizeInput(radio.getPWM(Radio::CHANNEL::ROLL),
                                  config().rollRxConfig.min,
                                  config().rollRxConfig.trim,
                                  config().rollRxConfig.max,
                                  config().rollRxConfig.deadband,
                                  config().rollRxConfig.reverse);

    input_rpy[1] = normalizeInput(radio.getPWM(Radio::CHANNEL::PITCH),
                                  config().pitchRxConfig.min,
                                  config().pitchRxConfig.trim,
                                  config().pitchRxConfig.max,
                                  config().pitchRxConfig.deadband,
                                  config().pitchRxConfig.reverse);

    input_rpy[2] = normalizeInput(radio.getPWM(Radio::CHANNEL::YAW),
                                  config().yawRxConfig.min,
                                  config().yawRxConfig.trim,
                                  config().yawRxConfig.max,
                                  config().yawRxConfig.deadband,
                                  config().yawRxConfig.reverse);

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

void Mode::runTask(void* ctx)
{
    Mode** modePointer = static_cast<Mode**>(ctx);

    (*modePointer)->run();
}

void Mode::updateInput(void* ctx)
{
    Mode** modePointer = static_cast<Mode**>(ctx);

    (*modePointer)->update();
}

void Mode::processOutput(void* ctx)
{
    (void)ctx;

    output_rpy[0] =
        constrain(output_rpy[0], -config().controlConfig.controlResolution, config().controlConfig.controlResolution);
    output_rpy[1] =
        constrain(output_rpy[1], -config().controlConfig.controlResolution, config().controlConfig.controlResolution);
    output_rpy[2] =
        constrain(output_rpy[2], -config().controlConfig.controlResolution, config().controlConfig.controlResolution);

    if (config().rollSrvConfig.reverse)
        output_rpy[0] = -output_rpy[0];

    if (config().pitchSrvConfig.reverse)
        output_rpy[1] = -output_rpy[1];

    if (config().yawSrvConfig.reverse)
        output_rpy[2] = -output_rpy[2];

    mixerOutputs = airplaneMixer.mix(output_rpy[0], output_rpy[1], output_rpy[2]);

    SRVout[Actuators::Channel::CH1] =
        mapToSRV(mixerOutputs.leftAileron, config().rollSrvConfig.min, config().rollSrvConfig.max);

    SRVout[Actuators::Channel::CH2] =
        mapToSRV(mixerOutputs.rightAileron, config().rollSrvConfig.min, config().rollSrvConfig.max);

    SRVout[Actuators::Channel::CH3] =
        mapToSRV(mixerOutputs.elevator, config().pitchSrvConfig.min, config().pitchSrvConfig.max);

    SRVout[Actuators::Channel::CH4] = mapToSRV(mixerOutputs.rudder, config().yawSrvConfig.min, config().yawSrvConfig.max);

#if defined(USE_FLAPERONS)
    flaperonMixer();
#endif

    actuators.writeServos(SRVout);
}

void Mode::consumeAHRS(const float (&rpy)[3], const float (&g)[3])
{
    imu_rpy[0] = rpy[0] * config().controlConfig.controlResolution;
    imu_rpy[1] = rpy[1] * config().controlConfig.controlResolution;
    imu_rpy[2] = rpy[2] * config().controlConfig.controlResolution;

    imu_g[0] = g[0] * config().controlConfig.controlResolution;
    imu_g[1] = g[1] * config().controlConfig.controlResolution;
    imu_g[2] = g[2] * config().controlConfig.controlResolution;
}

void Mode::setFailsafeInputs(void)
{
    // Default failsafe implementation
    input_rpy[0] = 0;
    input_rpy[1] = 0;
    input_rpy[2] = 0;
#if defined(USE_FLAPERONS)
    flaperonOut = config().flightConfig.flaperonMax; // set flaperons to landing position
#endif
}