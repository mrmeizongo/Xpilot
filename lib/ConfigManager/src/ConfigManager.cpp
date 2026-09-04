#include <Arduino.h>
#include "ConfigManager.h"

#include <EEPROM.h>
#include <string.h>

ConfigManager::ConfigManager()
    : _config{},
      _dirty(false)
{
}

void ConfigManager::init()
{
    if (!load())
    {
        loadDefaults();
        save();
    }
}

const Config &ConfigManager::config() const
{
    return _config;
}

bool ConfigManager::isDirty() const
{
    return _dirty;
}

void ConfigManager::loadDefaults()
{
    _config.airframeConfig.type = Config::AirframeType::CONVENTIONAL;

    _config.rollRxConfig.min = 1100;
    _config.rollRxConfig.trim = 1500;
    _config.rollRxConfig.max = 1900;
    _config.rollRxConfig.deadband = 8;

    _config.pitchRxConfig.min = 1100;
    _config.pitchRxConfig.trim = 1500;
    _config.pitchRxConfig.max = 1900;
    _config.pitchRxConfig.deadband = 8;

    _config.yawRxConfig.min = 1100;
    _config.yawRxConfig.trim = 1500;
    _config.yawRxConfig.max = 1900;
    _config.yawRxConfig.deadband = 8;

    _config.srvConfig.min = 1000;
    _config.srvConfig.trim = 1500;
    _config.srvConfig.max = 2000;

    _config.flightConfig.maxRollRateDegs = 60;
    _config.flightConfig.maxPitchRateDegs = 45;
    _config.flightConfig.maxYawRateDegs = 45;

    _config.flightConfig.maxRollAngleDegs = 75;
    _config.flightConfig.maxPitchAngleDegs = 60;

    _config.flightConfig.rollAngleKp = 1.f;
    _config.flightConfig.pitchAngleKp = 1.f;

    _config.flightConfig.flaperonScaleFactor = 1.f;
    _config.flightConfig.flaperonMax = 500;

    _config.flightConfig.reverseRudderMix = false;
    _config.flightConfig.rudderMixScale = 0.3f;

    _config.rPIDFConfig.Kp = 5.f;
    _config.rPIDFConfig.Ki = 2.1f;
    _config.rPIDFConfig.Kd = 0.f;
    _config.rPIDFConfig.Kf = 1.5f;
    _config.rPIDFConfig.iWindUpMax = 500.f;

    _config.pPIDFConfig.Kp = 7.f;
    _config.pPIDFConfig.Ki = 2.1f;
    _config.pPIDFConfig.Kd = 0.f;
    _config.pPIDFConfig.Kf = 1.5f;
    _config.pPIDFConfig.iWindUpMax = 500.f;

    _config.yPIDFConfig.Kp = 9.f;
    _config.yPIDFConfig.Ki = 0.f;
    _config.yPIDFConfig.Kd = 0.f;
    _config.yPIDFConfig.Kf = 1.5f;
    _config.yPIDFConfig.iWindUpMax = 500.f;

    _config.imuConfig.accBiasX = 0.f;
    _config.imuConfig.accBiasY = 0.f;
    _config.imuConfig.accBiasZ = 0.f;

    _config.imuConfig.gyroBiasX = 0.f;
    _config.imuConfig.gyroBiasY = 0.f;
    _config.imuConfig.gyroBiasZ = 0.f;

    _config.imuConfig.calibrated = false;

    _config.filterConfig.controlSlewRate = 2000;
    _config.filterConfig.lowPassFilterFreq = 10;
    _config.filterConfig.processDT = 0.004;

    _dirty = true;
}

bool ConfigManager::get(
    ConfigID id,
    ConfigValue &value,
    ConfigValueType &type) const
{
    value.raw = 0;

    switch (id)
    {
    case ConfigID::AIRFRAME_TYPE:
        type = ConfigValueType::UINT8;
        value.u8 = static_cast<uint8_t>(_config.airframeConfig.type);
        break;

    case ConfigID::RC_ROLL_MIN:
        type = ConfigValueType::INT16;
        value.i16 = _config.rollRxConfig.min;
        break;

    case ConfigID::RC_ROLL_TRIM:
        type = ConfigValueType::INT16;
        value.i16 = _config.rollRxConfig.trim;
        break;

    case ConfigID::RC_ROLL_MAX:
        type = ConfigValueType::INT16;
        value.i16 = _config.rollRxConfig.max;
        break;

    case ConfigID::RC_ROLL_DB:
        type = ConfigValueType::UINT8;
        value.u8 = _config.rollRxConfig.deadband;
        break;

    case ConfigID::RC_PITCH_MIN:
        type = ConfigValueType::INT16;
        value.i16 = _config.pitchRxConfig.min;
        break;

    case ConfigID::RC_PITCH_TRIM:
        type = ConfigValueType::INT16;
        value.i16 = _config.pitchRxConfig.trim;
        break;

    case ConfigID::RC_PITCH_MAX:
        type = ConfigValueType::INT16;
        value.i16 = _config.pitchRxConfig.max;
        break;

    case ConfigID::RC_PITCH_DB:
        type = ConfigValueType::UINT8;
        value.u8 = _config.pitchRxConfig.deadband;
        break;

    case ConfigID::RC_YAW_MIN:
        type = ConfigValueType::INT16;
        value.i16 = _config.yawRxConfig.min;
        break;

    case ConfigID::RC_YAW_TRIM:
        type = ConfigValueType::INT16;
        value.i16 = _config.yawRxConfig.trim;
        break;

    case ConfigID::RC_YAW_MAX:
        type = ConfigValueType::INT16;
        value.i16 = _config.yawRxConfig.max;
        break;

    case ConfigID::RC_YAW_DB:
        type = ConfigValueType::UINT8;
        value.u8 = _config.yawRxConfig.deadband;
        break;

    case ConfigID::SRV_MIN:
        type = ConfigValueType::INT16;
        value.i16 = _config.srvConfig.min;
        break;

    case ConfigID::SRV_TRIM:
        type = ConfigValueType::INT16;
        value.i16 = _config.srvConfig.trim;
        break;

    case ConfigID::SRV_MAX:
        type = ConfigValueType::INT16;
        value.i16 = _config.srvConfig.max;
        break;

    case ConfigID::FLIGHT_MAX_ROLL_RATE_DEGS:
        type = ConfigValueType::INT16;
        value.i16 = _config.flightConfig.maxRollRateDegs;
        break;

    case ConfigID::FLIGHT_MAX_PITCH_RATE_DEGS:
        type = ConfigValueType::INT16;
        value.i16 = _config.flightConfig.maxPitchRateDegs;
        break;

    case ConfigID::FLIGHT_MAX_YAW_RATE_DEGS:
        type = ConfigValueType::INT16;
        value.i16 = _config.flightConfig.maxYawRateDegs;
        break;

    case ConfigID::FLIGHT_MAX_ROLL_ANGLE_DEGS:
        type = ConfigValueType::INT16;
        value.i16 = _config.flightConfig.maxRollAngleDegs;
        break;

    case ConfigID::FLIGHT_MAX_PITCH_ANGLE_DEGS:
        type = ConfigValueType::INT16;
        value.i16 = _config.flightConfig.maxPitchAngleDegs;
        break;

    case ConfigID::FLIGHT_ROLL_ANGLE_KP:
        type = ConfigValueType::FLOAT;
        value.f = _config.flightConfig.rollAngleKp;
        break;

    case ConfigID::FLIGHT_PITCH_ANGLE_KP:
        type = ConfigValueType::FLOAT;
        value.f = _config.flightConfig.pitchAngleKp;
        break;

    case ConfigID::FLIGHT_FLAPERON_SCALE_FACTOR:
        type = ConfigValueType::FLOAT;
        value.f = _config.flightConfig.flaperonScaleFactor;
        break;

    case ConfigID::FLIGHT_MAX_FLAPERON:
        type = ConfigValueType::FLOAT;
        value.f = _config.flightConfig.flaperonMax;
        break;

    case ConfigID::FLIGHT_REVERSE_RUDDER_MIX:
        type = ConfigValueType::BOOL;
        value.u8 = _config.flightConfig.reverseRudderMix ? 1U : 0U;
        break;

    case ConfigID::FLIGHT_RUDDER_MIX_SCALE_FACTOR:
        type = ConfigValueType::FLOAT;
        value.f = _config.flightConfig.rudderMixScale;

    case ConfigID::PIDF_ROLL_KP:
        type = ConfigValueType::FLOAT;
        value.f = _config.rPIDFConfig.Kp;
        break;

    case ConfigID::PIDF_ROLL_KI:
        type = ConfigValueType::FLOAT;
        value.f = _config.rPIDFConfig.Ki;
        break;

    case ConfigID::PIDF_ROLL_KD:
        type = ConfigValueType::FLOAT;
        value.f = _config.rPIDFConfig.Kd;
        break;

    case ConfigID::PIDF_ROLL_KF:
        type = ConfigValueType::FLOAT;
        value.f = _config.rPIDFConfig.Kf;
        break;

    case ConfigID::PIDF_ROLL_I_WINDUP_MAX:
        type = ConfigValueType::FLOAT;
        value.f = _config.rPIDFConfig.iWindUpMax;
        break;

    case ConfigID::PIDF_PITCH_KP:
        type = ConfigValueType::FLOAT;
        value.f = _config.pPIDFConfig.Kp;
        break;

    case ConfigID::PIDF_PITCH_KI:
        type = ConfigValueType::FLOAT;
        value.f = _config.pPIDFConfig.Ki;
        break;

    case ConfigID::PIDF_PITCH_KD:
        type = ConfigValueType::FLOAT;
        value.f = _config.pPIDFConfig.Kd;
        break;

    case ConfigID::PIDF_PITCH_KF:
        type = ConfigValueType::FLOAT;
        value.f = _config.pPIDFConfig.Kf;
        break;

    case ConfigID::PIDF_PITCH_I_WINDUP_MAX:
        type = ConfigValueType::FLOAT;
        value.f = _config.pPIDFConfig.iWindUpMax;
        break;

    case ConfigID::PIDF_YAW_KP:
        type = ConfigValueType::FLOAT;
        value.f = _config.yPIDFConfig.Kp;
        break;

    case ConfigID::PIDF_YAW_KI:
        type = ConfigValueType::FLOAT;
        value.f = _config.yPIDFConfig.Ki;
        break;

    case ConfigID::PIDF_YAW_KD:
        type = ConfigValueType::FLOAT;
        value.f = _config.yPIDFConfig.Kd;
        break;

    case ConfigID::PIDF_YAW_KF:
        type = ConfigValueType::FLOAT;
        value.f = _config.yPIDFConfig.Kf;
        break;

    case ConfigID::PIDF_YAW_I_WINDUP_MAX:
        type = ConfigValueType::FLOAT;
        value.f = _config.yPIDFConfig.iWindUpMax;
        break;

    case ConfigID::IMU_ACC_BIAS_X:
        type = ConfigValueType::FLOAT;
        value.f = _config.imuConfig.accBiasX;
        break;

    case ConfigID::IMU_ACC_BIAS_Y:
        type = ConfigValueType::FLOAT;
        value.f = _config.imuConfig.accBiasY;
        break;

    case ConfigID::IMU_ACC_BIAS_Z:
        type = ConfigValueType::FLOAT;
        value.f = _config.imuConfig.accBiasZ;
        break;

    case ConfigID::IMU_GYRO_BIAS_X:
        type = ConfigValueType::FLOAT;
        value.f = _config.imuConfig.gyroBiasX;
        break;

    case ConfigID::IMU_GYRO_BIAS_Y:
        type = ConfigValueType::FLOAT;
        value.f = _config.imuConfig.gyroBiasY;
        break;

    case ConfigID::IMU_GYRO_BIAS_Z:
        type = ConfigValueType::FLOAT;
        value.f = _config.imuConfig.gyroBiasZ;
        break;

    case ConfigID::IMU_CALIBRATED:
        type = ConfigValueType::BOOL;
        value.u8 = _config.imuConfig.calibrated ? 1U : 0U;
        break;

    case ConfigID::FILTER_SLEW_RATE:
        type = ConfigValueType::INT16;
        value.i16 = _config.filterConfig.controlSlewRate;
        break;

    case ConfigID::FILTER_LPF_FREQ:
        type = ConfigValueType::INT16;
        value.i16 = _config.filterConfig.lowPassFilterFreq;
        break;

    case ConfigID::FILTER_PROCESS_DT:
        type = ConfigValueType::FLOAT;
        value.f = _config.filterConfig.processDT;
        break;

        // ------------------------------------------------------------

    default:
        return false;
    }

    return true;
}

bool ConfigManager::set(
    ConfigID id,
    const ConfigValue &value)
{
    if (!validate(id, value))
    {
        return false;
    }

    switch (id)
    {
    case ConfigID::AIRFRAME_TYPE:
        _config.airframeConfig.type = static_cast<Config::AirframeType>(value.u8);
        break;
    case ConfigID::RC_ROLL_MIN:
        _config.rollRxConfig.min = value.i16;
        break;

    case ConfigID::RC_ROLL_TRIM:
        _config.rollRxConfig.trim = value.i16;
        break;

    case ConfigID::RC_ROLL_MAX:
        _config.rollRxConfig.max = value.i16;
        break;

    case ConfigID::RC_ROLL_DB:
        _config.rollRxConfig.deadband = value.u8;
        break;

    case ConfigID::RC_PITCH_MIN:
        _config.pitchRxConfig.min = value.i16;
        break;

    case ConfigID::RC_PITCH_TRIM:
        _config.pitchRxConfig.trim = value.i16;
        break;

    case ConfigID::RC_PITCH_MAX:
        _config.pitchRxConfig.max = value.i16;
        break;

    case ConfigID::RC_PITCH_DB:
        _config.pitchRxConfig.deadband = value.u8;
        break;

    case ConfigID::RC_YAW_MIN:
        _config.yawRxConfig.min = value.i16;
        break;

    case ConfigID::RC_YAW_TRIM:
        _config.yawRxConfig.trim = value.i16;
        break;

    case ConfigID::RC_YAW_MAX:
        _config.yawRxConfig.max = value.i16;
        break;

    case ConfigID::RC_YAW_DB:
        _config.yawRxConfig.deadband = value.u8;
        break;

    case ConfigID::SRV_MIN:
        _config.srvConfig.min = value.i16;
        break;

    case ConfigID::SRV_TRIM:
        _config.srvConfig.trim = value.i16;
        break;

    case ConfigID::SRV_MAX:
        _config.srvConfig.max = value.i16;
        break;

    case ConfigID::FLIGHT_MAX_ROLL_RATE_DEGS:
        _config.flightConfig.maxRollRateDegs = value.i16;
        break;

    case ConfigID::FLIGHT_MAX_PITCH_RATE_DEGS:
        _config.flightConfig.maxPitchRateDegs = value.i16;
        break;

    case ConfigID::FLIGHT_MAX_YAW_RATE_DEGS:
        _config.flightConfig.maxYawRateDegs = value.i16;
        break;

    case ConfigID::FLIGHT_MAX_ROLL_ANGLE_DEGS:
        _config.flightConfig.maxRollAngleDegs = value.i16;
        break;

    case ConfigID::FLIGHT_MAX_PITCH_ANGLE_DEGS:
        _config.flightConfig.maxPitchRateDegs = value.i16;
        break;

    case ConfigID::FLIGHT_ROLL_ANGLE_KP:
        _config.flightConfig.rollAngleKp = value.f;
        break;

    case ConfigID::FLIGHT_PITCH_ANGLE_KP:
        _config.flightConfig.pitchAngleKp = value.f;
        break;

    case ConfigID::FLIGHT_FLAPERON_SCALE_FACTOR:
        _config.flightConfig.flaperonScaleFactor = value.f;
        _config.flightConfig.flaperonMax =
            static_cast<uint16_t>((_config.srvConfig.max - _config.srvConfig.trim) * _config.flightConfig.flaperonScaleFactor);
        break;

    case ConfigID::FLIGHT_REVERSE_RUDDER_MIX:
        _config.flightConfig.reverseRudderMix = (value.u8 != 0U);
        break;

    case ConfigID::FLIGHT_RUDDER_MIX_SCALE_FACTOR:
        _config.flightConfig.rudderMixScale = value.f;
        break;

    case ConfigID::PIDF_ROLL_KP:
        _config.rPIDFConfig.Kp = value.f;
        break;

    case ConfigID::PIDF_ROLL_KI:
        _config.rPIDFConfig.Ki = value.f;
        break;

    case ConfigID::PIDF_ROLL_KD:
        _config.rPIDFConfig.Kd = value.f;
        break;

    case ConfigID::PIDF_ROLL_KF:
        _config.rPIDFConfig.Kf = value.f;
        break;

    case ConfigID::PIDF_ROLL_I_WINDUP_MAX:
        _config.rPIDFConfig.iWindUpMax = value.f;
        break;

    case ConfigID::PIDF_PITCH_KP:
        _config.pPIDFConfig.Kp = value.f;
        break;

    case ConfigID::PIDF_PITCH_KI:
        _config.pPIDFConfig.Ki = value.f;
        break;

    case ConfigID::PIDF_PITCH_KD:
        _config.pPIDFConfig.Kd = value.f;
        break;

    case ConfigID::PIDF_PITCH_KF:
        _config.pPIDFConfig.Kf = value.f;
        break;

    case ConfigID::PIDF_PITCH_I_WINDUP_MAX:
        _config.pPIDFConfig.iWindUpMax = value.f;
        break;

    case ConfigID::PIDF_YAW_KP:
        _config.yPIDFConfig.Kp = value.f;
        break;

    case ConfigID::PIDF_YAW_KI:
        _config.yPIDFConfig.Ki = value.f;
        break;

    case ConfigID::PIDF_YAW_KD:
        _config.yPIDFConfig.Kd = value.f;
        break;

    case ConfigID::PIDF_YAW_KF:
        _config.yPIDFConfig.Kf = value.f;
        break;

    case ConfigID::PIDF_YAW_I_WINDUP_MAX:
        _config.yPIDFConfig.iWindUpMax = value.f;
        break;

    case ConfigID::FILTER_SLEW_RATE:
        _config.filterConfig.controlSlewRate = value.i16;
        break;

    case ConfigID::FILTER_LPF_FREQ:
        _config.filterConfig.lowPassFilterFreq = value.i16;
        break;

    case ConfigID::FILTER_PROCESS_DT:
        _config.filterConfig.processDT = value.f;
        break;

    default:
        return false;
    }

    _dirty = true;

    for (uint8_t i = 0; i < subscriberCount; i++)
    {
        subscribers[i].cb(id, subscribers[i].ctx);
    }

    return true;
}

bool ConfigManager::validate(
    ConfigID id,
    const ConfigValue &value) const
{
    switch (id)
    {
    case ConfigID::AIRFRAME_TYPE:
        return value.u8 < static_cast<uint8_t>(Config::AirframeType::COUNT);

    // RCConfi
    case ConfigID::RC_ROLL_MIN:
    case ConfigID::RC_ROLL_TRIM:
    case ConfigID::RC_ROLL_MAX:

    case ConfigID::RC_PITCH_MIN:
    case ConfigID::RC_PITCH_TRIM:
    case ConfigID::RC_PITCH_MAX:

    case ConfigID::RC_YAW_MIN:
    case ConfigID::RC_YAW_TRIM:
    case ConfigID::RC_YAW_MAX:

    case ConfigID::SRV_MIN:
    case ConfigID::SRV_TRIM:
    case ConfigID::SRV_MAX:

        return value.i16 >= 544 &&
               value.i16 <= 2400;

    case ConfigID::RC_ROLL_DB:
    case ConfigID::RC_PITCH_DB:
    case ConfigID::RC_YAW_DB:

        return value.u8 <= 250;

    case ConfigID::FLIGHT_MAX_ROLL_RATE_DEGS:
    case ConfigID::FLIGHT_MAX_PITCH_RATE_DEGS:
    case ConfigID::FLIGHT_MAX_YAW_RATE_DEGS:

    case ConfigID::FLIGHT_MAX_ROLL_ANGLE_DEGS:
    case ConfigID::FLIGHT_MAX_PITCH_ANGLE_DEGS:

        return value.i16 >= 0 &&
               value.i16 <= 360;

    case ConfigID::FLIGHT_FLAPERON_SCALE_FACTOR:

    case ConfigID::FLIGHT_RUDDER_MIX_SCALE_FACTOR:

        return value.f >= .01f &&
               value.f <= 1.f;

    case ConfigID::FILTER_LPF_FREQ:
        return value.i16 >= 1 &&
               value.i16 <= 1000;

    case ConfigID::PIDF_ROLL_KP:
    case ConfigID::PIDF_ROLL_KI:
    case ConfigID::PIDF_ROLL_KD:
    case ConfigID::PIDF_ROLL_KF:

    case ConfigID::PIDF_PITCH_KP:
    case ConfigID::PIDF_PITCH_KI:
    case ConfigID::PIDF_PITCH_KD:
    case ConfigID::PIDF_PITCH_KF:

    case ConfigID::PIDF_YAW_KP:
    case ConfigID::PIDF_YAW_KI:
    case ConfigID::PIDF_YAW_KD:
    case ConfigID::PIDF_YAW_KF:

    case ConfigID::FLIGHT_ROLL_ANGLE_KP:
    case ConfigID::FLIGHT_PITCH_ANGLE_KP:

        return value.f >= 0.f &&
               value.f <= static_cast<float>(Control::RESOLUTION);

    case ConfigID::PIDF_ROLL_I_WINDUP_MAX:
    case ConfigID::PIDF_PITCH_I_WINDUP_MAX:
    case ConfigID::PIDF_YAW_I_WINDUP_MAX:
        return value.f >= 0.f;

    case ConfigID::FLIGHT_REVERSE_RUDDER_MIX:

        return value.u8 <= 1U;

    case ConfigID::FILTER_SLEW_RATE:

        return value.i16 >= 1 &&
               value.i16 <= 10000;

    case ConfigID::FILTER_PROCESS_DT:

        return value.f > 0.f &&
               value.f <= 1.f;

    default:
        return false;
    }
}

bool ConfigManager::save()
{
    StoredConfig stored{};

    stored.magic = EEPROM_MAGIC;
    stored.version = EEPROM_VERSION;
    stored.config = _config;

    stored.checksum =
        calculateChecksum(
            reinterpret_cast<const uint8_t *>(&stored.config),
            sizeof(Config));

    EEPROM.put(
        EEPROM_ADDRESS,
        stored);

    _dirty = false;

    return true;
}

bool ConfigManager::load()
{
    StoredConfig stored{};

    EEPROM.get(
        EEPROM_ADDRESS,
        stored);

    if (stored.magic != EEPROM_MAGIC)
    {
        return false;
    }

    if (stored.version != EEPROM_VERSION)
    {
        return false;
    }

    const uint16_t checksum =
        calculateChecksum(
            reinterpret_cast<const uint8_t *>(&stored.config),
            sizeof(Config));

    if (checksum != stored.checksum)
    {
        return false;
    }

    _config = stored.config;

    _dirty = false;

    return true;
}

void ConfigManager::registerSubscriber(Callback cb, void *ctx)
{
    if (subscriberCount >= MAX_SUBSCRIBERS)
        return;

    subscribers[subscriberCount++] = {cb, ctx};
}

void ConfigManager::setIMUCalibration(const float (&accelBias)[3], const float (&gyroBias)[3])
{
    _config.imuConfig.accBiasX = accelBias[0];
    _config.imuConfig.accBiasY = accelBias[1];
    _config.imuConfig.accBiasZ = accelBias[2];

    _config.imuConfig.gyroBiasX = gyroBias[0];
    _config.imuConfig.gyroBiasY = gyroBias[1];
    _config.imuConfig.gyroBiasZ = gyroBias[2];

    _config.imuConfig.calibrated = true;

    _dirty = true;
}

uint16_t ConfigManager::calculateChecksum(
    const uint8_t *data,
    uint16_t length)
{
    uint16_t checksum = 0;

    for (uint16_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }

    return checksum;
}