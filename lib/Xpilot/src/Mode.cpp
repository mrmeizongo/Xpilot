#include "Mode.h"
#include "IMU.h"

void Mode::init(void)
{
    airplaneMixer = AirplaneMixer(config().airframeConfig.type,
                                  config().controlConfig.controlResolution,
                                  config().rollSrvConfig.reverse,
                                  config().pitchSrvConfig.reverse,
                                  config().yawSrvConfig.reverse);

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

        case ConfigID::SRV_ROLL_REVERSE:
            airplaneMixer.setRollReverse(config().rollSrvConfig.reverse);
            break;

        case ConfigID::SRV_PITCH_REVERSE:
            airplaneMixer.setPitchReverse(config().pitchSrvConfig.reverse);
            break;

        case ConfigID::SRV_YAW_REVERSE:
            airplaneMixer.setYawReverse(config().yawSrvConfig.reverse);
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

    input_trpy[Radio::CHANNEL::THROTTLE] = normalizeInput(radio.getPWM(Radio::CHANNEL::THROTTLE),
                                                          config().throttleRxConfig.min,
                                                          config().throttleRxConfig.trim,
                                                          config().throttleRxConfig.max,
                                                          config().throttleRxConfig.deadband,
                                                          config().throttleRxConfig.reverse);

    input_trpy[Radio::CHANNEL::ROLL] = normalizeInput(radio.getPWM(Radio::CHANNEL::ROLL),
                                                      config().rollRxConfig.min,
                                                      config().rollRxConfig.trim,
                                                      config().rollRxConfig.max,
                                                      config().rollRxConfig.deadband,
                                                      config().rollRxConfig.reverse);

    input_trpy[Radio::CHANNEL::PITCH] = normalizeInput(radio.getPWM(Radio::CHANNEL::PITCH),
                                                       config().pitchRxConfig.min,
                                                       config().pitchRxConfig.trim,
                                                       config().pitchRxConfig.max,
                                                       config().pitchRxConfig.deadband,
                                                       config().pitchRxConfig.reverse);

    input_trpy[Radio::CHANNEL::YAW] = normalizeInput(radio.getPWM(Radio::CHANNEL::YAW),
                                                     config().yawRxConfig.min,
                                                     config().yawRxConfig.trim,
                                                     config().yawRxConfig.max,
                                                     config().yawRxConfig.deadband,
                                                     config().yawRxConfig.reverse);

#if defined(USE_FLAPERONS)
    flaperonInput = normalizeInput(radio.getPWM(Radio::CHANNEL::AUX2),
                                   config().rollRxConfig.min,
                                   config().rollRxConfig.trim,
                                   config().rollRxConfig.max,
                                   config().rollRxConfig.deadband,
                                   false);

    flaperonInput = static_cast<int16_t>(flaperonInput * config().flightConfig.flaperonScaleFactor);
#endif
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

    mixerOutputs = airplaneMixer.mix(output_trpy[Radio::CHANNEL::ROLL],
                                     output_trpy[Radio::CHANNEL::PITCH],
                                     output_trpy[Radio::CHANNEL::YAW],
                                     flaperonInput);

    SRVout[Actuators::Channel::CH1] =
        mapToSRV(output_trpy[Radio::CHANNEL::THROTTLE], config().throttleSrvConfig.min, config().throttleSrvConfig.max);

    SRVout[Actuators::Channel::CH2] =
        mapToSRV(mixerOutputs.leftAileron, config().rollSrvConfig.min, config().rollSrvConfig.max);

    SRVout[Actuators::Channel::CH3] =
        mapToSRV(mixerOutputs.rightAileron, config().rollSrvConfig.min, config().rollSrvConfig.max);

    SRVout[Actuators::Channel::CH4] =
        mapToSRV(mixerOutputs.elevator, config().pitchSrvConfig.min, config().pitchSrvConfig.max);

    SRVout[Actuators::Channel::CH5] = mapToSRV(mixerOutputs.rudder, config().yawSrvConfig.min, config().yawSrvConfig.max);

    actuators.writeServos(SRVout);
}

void Mode::consumeAHRS(const float (&rpy)[IMU::Axis::AXIS_COUNT], const float (&g)[IMU::Axis::AXIS_COUNT])
{
    imu_rpy[IMU::Axis::X_AXIS] = rpy[IMU::Axis::X_AXIS] * config().controlConfig.controlResolution;
    imu_rpy[IMU::Axis::Y_AXIS] = rpy[IMU::Axis::Y_AXIS] * config().controlConfig.controlResolution;
    imu_rpy[IMU::Axis::Z_AXIS] = rpy[IMU::Axis::Z_AXIS] * config().controlConfig.controlResolution;

    imu_g[IMU::Axis::X_AXIS] = g[IMU::Axis::X_AXIS] * config().controlConfig.controlResolution;
    imu_g[IMU::Axis::Y_AXIS] = g[IMU::Axis::Y_AXIS] * config().controlConfig.controlResolution;
    imu_g[IMU::Axis::Z_AXIS] = g[IMU::Axis::Z_AXIS] * config().controlConfig.controlResolution;
}

void Mode::setFailsafeInputs(void)
{
    // Default failsafe implementation
    input_trpy[Radio::CHANNEL::THROTTLE] = -config().controlConfig.controlResolution;
    input_trpy[Radio::CHANNEL::ROLL] = 0;
    input_trpy[Radio::CHANNEL::PITCH] = 0;
    input_trpy[Radio::CHANNEL::YAW] = 0;
#if defined(USE_FLAPERONS)
    flaperonInput = -config().flightConfig.flaperonMax; // set flaperons to landing position
#endif
}