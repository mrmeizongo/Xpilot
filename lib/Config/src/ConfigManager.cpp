#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include "ConfigManager.h"
#include "SysConfig.h"

constexpr uint16_t ConfigManager::EEPROM_MAGIC; // XP - XPilot firmware signature
constexpr uint8_t ConfigManager::EEPROM_VERSION;
constexpr uint16_t ConfigManager::EEPROM_ADDRESS;
constexpr int ConfigManager::EEPROM_MAGIC_ADDR;
constexpr int ConfigManager::EEPROM_VERSION_ADDR;
constexpr int ConfigManager::EEPROM_CONFIG_ADDR;
constexpr int ConfigManager::EEPROM_CHECKSUM_ADDR;
constexpr uint8_t ConfigManager::MAX_SUBSCRIBERS;

ConfigManager::ConfigManager()
    : _config{}
    , _dirty(false)
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

bool ConfigManager::save()
{
    const uint16_t checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&_config), sizeof(_config));

    // 4 EEPROM.puts are used to save on SRAM
    EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    EEPROM.put(EEPROM_VERSION_ADDR, EEPROM_VERSION);
    EEPROM.put(EEPROM_CONFIG_ADDR, _config);
    EEPROM.put(EEPROM_CHECKSUM_ADDR, checksum);

    _dirty = false;

    return true;
}

bool ConfigManager::load()
{
    uint16_t magic;
    uint8_t version;
    uint16_t storedChecksum;

    EEPROM.get(EEPROM_MAGIC_ADDR, magic);

    if (magic != EEPROM_MAGIC)
    {
        return false;
    }

    EEPROM.get(EEPROM_VERSION_ADDR, version);

    if (version != EEPROM_VERSION)
    {
        return false;
    }

    EEPROM.get(EEPROM_CHECKSUM_ADDR, storedChecksum);

    EEPROM.get(EEPROM_CONFIG_ADDR, _config);

    const uint16_t checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&_config), sizeof(_config));

    if (checksum != storedChecksum)
    {
        return false;
    }

    _dirty = false;

    return true;
}

const Config& ConfigManager::config() const { return _config; }

bool ConfigManager::isDirty() const { return _dirty; }

void ConfigManager::loadDefaults()
{
    _config.airframeConfig.type = Config::AirframeType::CONVENTIONAL;

    _config.throttleRxConfig.min = 1100;
    _config.throttleRxConfig.trim = 1500;
    _config.throttleRxConfig.max = 1900;
    _config.throttleRxConfig.deadband = 12;
    _config.throttleRxConfig.reverse = false;

    _config.rollRxConfig.min = 1100;
    _config.rollRxConfig.trim = 1500;
    _config.rollRxConfig.max = 1900;
    _config.rollRxConfig.deadband = 12;
    _config.rollRxConfig.reverse = false;

    _config.pitchRxConfig.min = 1100;
    _config.pitchRxConfig.trim = 1500;
    _config.pitchRxConfig.max = 1900;
    _config.pitchRxConfig.deadband = 12;
    _config.pitchRxConfig.reverse = false;

    _config.yawRxConfig.min = 1100;
    _config.yawRxConfig.trim = 1500;
    _config.yawRxConfig.max = 1900;
    _config.yawRxConfig.deadband = 12;
    _config.yawRxConfig.reverse = false;

    _config.aux1RxConfig.min = 1100;
    _config.aux1RxConfig.trim = 1500;
    _config.aux1RxConfig.max = 1900;
    _config.aux1RxConfig.deadband = 12;
    _config.aux1RxConfig.reverse = false;

    _config.aux2RxConfig.min = 1100;
    _config.aux2RxConfig.trim = 1500;
    _config.aux2RxConfig.max = 1900;
    _config.aux2RxConfig.deadband = 12;
    _config.aux2RxConfig.reverse = false;

    _config.throttleSrvConfig.min = 1000;
    _config.throttleSrvConfig.trim = 1500;
    _config.throttleSrvConfig.max = 2000;
    _config.throttleSrvConfig.reverse = false;

    _config.rollSrvConfig.min = 1000;
    _config.rollSrvConfig.trim = 1500;
    _config.rollSrvConfig.max = 2000;
    _config.rollSrvConfig.reverse = false;

    _config.pitchSrvConfig.min = 1000;
    _config.pitchSrvConfig.trim = 1500;
    _config.pitchSrvConfig.max = 2000;
    _config.pitchSrvConfig.reverse = false;

    _config.yawSrvConfig.min = 1000;
    _config.yawSrvConfig.trim = 1500;
    _config.yawSrvConfig.max = 2000;
    _config.yawSrvConfig.reverse = false;

    _config.flightConfig.maxRollRateDegs = 60;
    _config.flightConfig.maxPitchRateDegs = 45;
    _config.flightConfig.maxYawRateDegs = 45;

    _config.flightConfig.maxRollAngleDegs = 75;
    _config.flightConfig.maxPitchAngleDegs = 60;

    _config.flightConfig.rollAngleKp = 1.f;
    _config.flightConfig.pitchAngleKp = 1.f;

    _config.flightConfig.flaperonScaleFactor = 1.f;
    _config.flightConfig.flaperonMax = 400;

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

    _config.controlConfig.controlSlewRate = 2000;
    _config.controlConfig.lowPassFilterFreq = 10;
    _config.controlConfig.controlResolution = 1000;
    _config.controlConfig.dt = 1.f / CONTROL_LOOP_HZ;

    _dirty = true;
}

bool ConfigManager::get(ConfigID id, ConfigValue& value, ConfigValueType& type) const
{
    value.raw = 0;

    switch (id)
    {
        case ConfigID::AIRFRAME_TYPE:
            type = ConfigValueType::UINT8;
            value.u8 = static_cast<uint8_t>(_config.airframeConfig.type);
            break;

        case ConfigID::RC_THROTTLE_MIN:
            type = ConfigValueType::INT16;
            value.i16 = _config.throttleRxConfig.min;
            break;

        case ConfigID::RC_THROTTLE_TRIM:
            type = ConfigValueType::INT16;
            value.i16 = _config.throttleRxConfig.trim;
            break;

        case ConfigID::RC_THROTTLE_MAX:
            type = ConfigValueType::INT16;
            value.i16 = _config.throttleRxConfig.max;
            break;

        case ConfigID::RC_THROTTLE_DB:
            type = ConfigValueType::UINT8;
            value.u8 = _config.throttleRxConfig.deadband;
            break;

        case ConfigID::RC_THROTTLE_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.throttleRxConfig.reverse ? 1U : 0U;
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

        case ConfigID::RC_ROLL_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.rollRxConfig.reverse ? 1U : 0U;
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

        case ConfigID::RC_PITCH_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.pitchRxConfig.reverse ? 1U : 0U;
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

        case ConfigID::RC_YAW_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.aux1RxConfig.reverse ? 1U : 0U;
            break;

        case ConfigID::RC_AUX1_MIN:
            type = ConfigValueType::INT16;
            value.i16 = _config.aux1RxConfig.min;
            break;

        case ConfigID::RC_AUX1_TRIM:
            type = ConfigValueType::INT16;
            value.i16 = _config.aux1RxConfig.trim;
            break;

        case ConfigID::RC_AUX1_MAX:
            type = ConfigValueType::INT16;
            value.i16 = _config.aux1RxConfig.max;
            break;

        case ConfigID::RC_AUX1_DB:
            type = ConfigValueType::UINT8;
            value.u8 = _config.aux1RxConfig.deadband;
            break;

        case ConfigID::RC_AUX1_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.aux1RxConfig.reverse ? 1U : 0U;
            break;

        case ConfigID::RC_AUX2_MIN:
            type = ConfigValueType::INT16;
            value.i16 = _config.aux2RxConfig.min;
            break;

        case ConfigID::RC_AUX2_TRIM:
            type = ConfigValueType::INT16;
            value.i16 = _config.aux2RxConfig.trim;
            break;

        case ConfigID::RC_AUX2_MAX:
            type = ConfigValueType::INT16;
            value.i16 = _config.aux2RxConfig.max;
            break;

        case ConfigID::RC_AUX2_DB:
            type = ConfigValueType::UINT8;
            value.u8 = _config.aux2RxConfig.deadband;
            break;

        case ConfigID::RC_AUX2_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.aux2RxConfig.reverse ? 1U : 0U;

        case ConfigID::SRV_THROTTLE_MIN:
            type = ConfigValueType::INT16;
            value.i16 = _config.throttleSrvConfig.min;
            break;

        case ConfigID::SRV_THROTTLE_TRIM:
            type = ConfigValueType::INT16;
            value.i16 = _config.throttleSrvConfig.trim;
            break;

        case ConfigID::SRV_THROTTLE_MAX:
            type = ConfigValueType::INT16;
            value.i16 = _config.throttleSrvConfig.max;
            break;

        case ConfigID::SRV_THROTTLE_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.throttleSrvConfig.reverse ? 1U : 0U;
            break;

        case ConfigID::SRV_ROLL_MIN:
            type = ConfigValueType::INT16;
            value.i16 = _config.rollSrvConfig.min;
            break;

        case ConfigID::SRV_ROLL_TRIM:
            type = ConfigValueType::INT16;
            value.i16 = _config.rollSrvConfig.trim;
            break;

        case ConfigID::SRV_ROLL_MAX:
            type = ConfigValueType::INT16;
            value.i16 = _config.rollSrvConfig.max;
            break;

        case ConfigID::SRV_ROLL_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.rollSrvConfig.reverse ? 1U : 0U;
            break;

        case ConfigID::SRV_PITCH_MIN:
            type = ConfigValueType::INT16;
            value.i16 = _config.pitchSrvConfig.min;
            break;

        case ConfigID::SRV_PITCH_TRIM:
            type = ConfigValueType::INT16;
            value.i16 = _config.pitchSrvConfig.trim;
            break;

        case ConfigID::SRV_PITCH_MAX:
            type = ConfigValueType::INT16;
            value.i16 = _config.pitchSrvConfig.max;
            break;

        case ConfigID::SRV_PITCH_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.pitchSrvConfig.reverse ? 1U : 0U;
            break;

        case ConfigID::SRV_YAW_MIN:
            type = ConfigValueType::INT16;
            value.i16 = _config.yawSrvConfig.min;
            break;

        case ConfigID::SRV_YAW_TRIM:
            type = ConfigValueType::INT16;
            value.i16 = _config.yawSrvConfig.trim;
            break;

        case ConfigID::SRV_YAW_MAX:
            type = ConfigValueType::INT16;
            value.i16 = _config.yawSrvConfig.max;
            break;

        case ConfigID::SRV_YAW_REVERSE:
            type = ConfigValueType::BOOL;
            value.u8 = _config.yawSrvConfig.reverse ? 1U : 0U;
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

        case ConfigID::CONTROL_SLEW_RATE:
            type = ConfigValueType::INT16;
            value.u16 = _config.controlConfig.controlSlewRate;
            break;

        case ConfigID::CONTROL_LPF_FREQ:
            type = ConfigValueType::UINT16;
            value.u16 = _config.controlConfig.lowPassFilterFreq;
            break;

        case ConfigID::CONTROL_RESOLUTION:
            type = ConfigValueType::UINT16;
            value.u16 = _config.controlConfig.controlResolution;
            break;

        case ConfigID::CONTROL_DT:
            type = ConfigValueType::FLOAT;
            value.f = _config.controlConfig.dt;
            break;

            // ------------------------------------------------------------

        default:
            return false;
    }

    return true;
}

bool ConfigManager::set(ConfigID id, const ConfigValue& value)
{
    if (!validateSet(id, value))
    {
        return false;
    }

    switch (id)
    {
        case ConfigID::AIRFRAME_TYPE:
            _config.airframeConfig.type = static_cast<Config::AirframeType>(value.u8);
            break;

        case ConfigID::RC_THROTTLE_MIN:
            _config.throttleRxConfig.min = value.i16;
            break;

        case ConfigID::RC_THROTTLE_TRIM:
            _config.throttleRxConfig.trim = value.i16;
            break;

        case ConfigID::RC_THROTTLE_MAX:
            _config.throttleRxConfig.max = value.i16;
            break;

        case ConfigID::RC_THROTTLE_DB:
            _config.throttleRxConfig.deadband = value.u8;
            break;

        case ConfigID::RC_THROTTLE_REVERSE:
            _config.throttleRxConfig.reverse = (value.u8 != 0U);
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

        case ConfigID::RC_ROLL_REVERSE:
            _config.rollRxConfig.reverse = (value.u8 != 0U);
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

        case ConfigID::RC_PITCH_REVERSE:
            _config.pitchRxConfig.reverse = (value.u8 != 0U);
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

        case ConfigID::RC_YAW_REVERSE:
            _config.yawRxConfig.reverse = (value.u8 != 0U);
            break;

        case ConfigID::RC_AUX1_MIN:
            _config.aux1RxConfig.min = value.i16;
            break;

        case ConfigID::RC_AUX1_TRIM:
            _config.aux1RxConfig.trim = value.i16;
            break;

        case ConfigID::RC_AUX1_MAX:
            _config.aux1RxConfig.max = value.i16;
            break;

        case ConfigID::RC_AUX1_DB:
            _config.aux1RxConfig.deadband = value.u8;
            break;

        case ConfigID::RC_AUX1_REVERSE:
            _config.aux1RxConfig.reverse = (value.u8 != 0U);
            break;

        case ConfigID::RC_AUX2_MIN:
            _config.aux2RxConfig.min = value.i16;
            break;

        case ConfigID::RC_AUX2_TRIM:
            _config.aux2RxConfig.trim = value.i16;
            break;

        case ConfigID::RC_AUX2_MAX:
            _config.aux2RxConfig.max = value.i16;
            break;

        case ConfigID::RC_AUX2_DB:
            _config.aux2RxConfig.deadband = value.u8;
            break;

        case ConfigID::RC_AUX2_REVERSE:
            _config.aux2RxConfig.reverse = (value.u8 != 0U);
            break;

        case ConfigID::SRV_THROTTLE_MIN:
            _config.throttleSrvConfig.min = value.i16;
            break;

        case ConfigID::SRV_THROTTLE_TRIM:
            _config.throttleSrvConfig.trim = value.i16;
            break;

        case ConfigID::SRV_THROTTLE_MAX:
            _config.throttleSrvConfig.max = value.i16;
            break;

        case ConfigID::SRV_THROTTLE_REVERSE:
            _config.throttleSrvConfig.reverse = (value.u8 != 0U);
            break;

        case ConfigID::SRV_ROLL_MIN:
            _config.rollSrvConfig.min = value.i16;
            break;

        case ConfigID::SRV_ROLL_TRIM:
            _config.rollSrvConfig.trim = value.i16;
            break;

        case ConfigID::SRV_ROLL_MAX:
            _config.rollSrvConfig.max = value.i16;
            break;

        case ConfigID::SRV_ROLL_REVERSE:
            _config.rollSrvConfig.reverse = (value.u8 != 0U);
            break;

        case ConfigID::SRV_PITCH_MIN:
            _config.pitchSrvConfig.min = value.i16;
            break;

        case ConfigID::SRV_PITCH_TRIM:
            _config.pitchSrvConfig.trim = value.i16;
            break;

        case ConfigID::SRV_PITCH_MAX:
            _config.pitchSrvConfig.max = value.i16;
            break;

        case ConfigID::SRV_PITCH_REVERSE:
            _config.pitchSrvConfig.reverse = (value.u8 != 0U);
            break;

        case ConfigID::SRV_YAW_MIN:
            _config.yawSrvConfig.min = value.i16;
            break;

        case ConfigID::SRV_YAW_TRIM:
            _config.yawSrvConfig.trim = value.i16;
            break;

        case ConfigID::SRV_YAW_MAX:
            _config.yawSrvConfig.max = value.i16;
            break;

        case ConfigID::SRV_YAW_REVERSE:
            _config.yawSrvConfig.reverse = (value.u8 != 0U);
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
            _config.flightConfig.maxPitchAngleDegs = value.i16;
            break;

        case ConfigID::FLIGHT_ROLL_ANGLE_KP:
            _config.flightConfig.rollAngleKp = value.f;
            break;

        case ConfigID::FLIGHT_PITCH_ANGLE_KP:
            _config.flightConfig.pitchAngleKp = value.f;
            break;

        case ConfigID::FLIGHT_FLAPERON_SCALE_FACTOR:
            _config.flightConfig.flaperonScaleFactor = value.f;
            _config.flightConfig.flaperonMax = static_cast<int16_t>((_config.rollRxConfig.max - _config.rollRxConfig.trim) *
                                                                    _config.flightConfig.flaperonScaleFactor);
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

        case ConfigID::CONTROL_SLEW_RATE:
            _config.controlConfig.controlSlewRate = value.u16;
            break;

        case ConfigID::CONTROL_LPF_FREQ:
            _config.controlConfig.lowPassFilterFreq = value.u16;
            break;

        default:
            return false;
    }

    _dirty = true;

    if (_subscriber != nullptr)
        _subscriber(id);

    return true;
}

// Primary settable config determinant
// If configID is not included in cases, it is not directly settable via xp_serial.py
// If configID is included in cases, it validates range of provided value
bool ConfigManager::validateSet(ConfigID id, const ConfigValue& value) const
{
    switch (id)
    {
        case ConfigID::AIRFRAME_TYPE:

            return value.u8 < static_cast<uint8_t>(Config::AirframeType::COUNT);

        case ConfigID::RC_THROTTLE_MIN:
        case ConfigID::RC_THROTTLE_TRIM:
        case ConfigID::RC_THROTTLE_MAX:

        case ConfigID::RC_ROLL_MIN:
        case ConfigID::RC_ROLL_TRIM:
        case ConfigID::RC_ROLL_MAX:

        case ConfigID::RC_PITCH_MIN:
        case ConfigID::RC_PITCH_TRIM:
        case ConfigID::RC_PITCH_MAX:

        case ConfigID::RC_YAW_MIN:
        case ConfigID::RC_YAW_TRIM:
        case ConfigID::RC_YAW_MAX:

        case ConfigID::RC_AUX1_MIN:
        case ConfigID::RC_AUX1_TRIM:
        case ConfigID::RC_AUX1_MAX:

        case ConfigID::RC_AUX2_MIN:
        case ConfigID::RC_AUX2_TRIM:
        case ConfigID::RC_AUX2_MAX:

        case ConfigID::SRV_ROLL_MIN:
        case ConfigID::SRV_ROLL_TRIM:
        case ConfigID::SRV_ROLL_MAX:

        case ConfigID::SRV_PITCH_MIN:
        case ConfigID::SRV_PITCH_TRIM:
        case ConfigID::SRV_PITCH_MAX:

        case ConfigID::SRV_YAW_MIN:
        case ConfigID::SRV_YAW_TRIM:
        case ConfigID::SRV_YAW_MAX:

            return value.i16 >= 544 && value.i16 <= 2400;

        case ConfigID::RC_THROTTLE_DB:
        case ConfigID::RC_ROLL_DB:
        case ConfigID::RC_PITCH_DB:
        case ConfigID::RC_YAW_DB:
        case ConfigID::RC_AUX1_DB:
        case ConfigID::RC_AUX2_DB:

            return value.u8 < 256;

        case ConfigID::FLIGHT_REVERSE_RUDDER_MIX:

        case ConfigID::RC_THROTTLE_REVERSE:
        case ConfigID::RC_ROLL_REVERSE:
        case ConfigID::RC_PITCH_REVERSE:
        case ConfigID::RC_YAW_REVERSE:
        case ConfigID::RC_AUX1_REVERSE:
        case ConfigID::RC_AUX2_REVERSE:

        case ConfigID::SRV_THROTTLE_REVERSE:
        case ConfigID::SRV_ROLL_REVERSE:
        case ConfigID::SRV_PITCH_REVERSE:
        case ConfigID::SRV_YAW_REVERSE:

            return value.u8 <= 1U;

        case ConfigID::FLIGHT_MAX_ROLL_RATE_DEGS:
        case ConfigID::FLIGHT_MAX_PITCH_RATE_DEGS:
        case ConfigID::FLIGHT_MAX_YAW_RATE_DEGS:

        case ConfigID::FLIGHT_MAX_ROLL_ANGLE_DEGS:
        case ConfigID::FLIGHT_MAX_PITCH_ANGLE_DEGS:

            return value.i16 > 0;

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

        case ConfigID::PIDF_ROLL_I_WINDUP_MAX:
        case ConfigID::PIDF_PITCH_I_WINDUP_MAX:
        case ConfigID::PIDF_YAW_I_WINDUP_MAX:

        case ConfigID::FLIGHT_FLAPERON_SCALE_FACTOR:

        case ConfigID::FLIGHT_RUDDER_MIX_SCALE_FACTOR:

            return value.f >= 0.f && value.f < 1.f;

        case ConfigID::CONTROL_LPF_FREQ:
        case ConfigID::CONTROL_SLEW_RATE:

            return value.u16 > 0;

        default:
            return false;
    }
}

void ConfigManager::registerSubscriber(Subscriber sb) { _subscriber = sb; }

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

uint16_t ConfigManager::calculateChecksum(const uint8_t* data, uint16_t length)
{
    uint16_t checksum = 0;

    for (uint16_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }

    return checksum;
}