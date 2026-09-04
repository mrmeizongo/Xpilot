#ifndef _CONFIG_H
#define _CONFIG_H
#include <stdint.h>

namespace Control
{
    constexpr int16_t RESOLUTION = 1000;
}

// Parameters that can be changed at runtime.
struct Config
{
    enum class AirframeType : uint8_t
    {
        CONVENTIONAL = 0,
        V_TAIL,
        FLYING_WING_RUDDER,
        FLYING_WING_NO_RUDDER,
        RUDDER_ELEVATOR,
        AILERON_ELEVATOR,
        CUSTOM,

        COUNT
    };

    struct AirframeConfig
    {
        AirframeType type;
    };

    struct RxConfig
    {
        int16_t min;
        int16_t trim;
        int16_t max;

        uint8_t deadband;
    };

    struct SRVConfig
    {
        int16_t min;
        int16_t trim;
        int16_t max;
    };

    struct FlightConfig
    {
        int16_t maxRollRateDegs;
        int16_t maxPitchRateDegs;
        int16_t maxYawRateDegs;

        int16_t maxRollAngleDegs;
        int16_t maxPitchAngleDegs;

        float rollAngleKp;
        float pitchAngleKp;

        float flaperonScaleFactor;
        int16_t flaperonMax;

        bool reverseRudderMix;
        float rudderMixScale;
    };

    struct PIDFConfig
    {
        float Kp;
        float Ki;
        float Kd;
        float Kf;
        float iWindUpMax;
    };

    struct IMUConfig
    {
        float accBiasX;
        float accBiasY;
        float accBiasZ;

        float gyroBiasX;
        float gyroBiasY;
        float gyroBiasZ;

        bool calibrated;
    };

    struct FilterConfig
    {
        int16_t controlSlewRate;
        int16_t lowPassFilterFreq;
        float processDT;
    };

    AirframeConfig airframeConfig;

    RxConfig rollRxConfig;
    RxConfig pitchRxConfig;
    RxConfig yawRxConfig;

    SRVConfig srvConfig;

    FlightConfig flightConfig;

    PIDFConfig rPIDFConfig;
    PIDFConfig pPIDFConfig;
    PIDFConfig yPIDFConfig;

    IMUConfig imuConfig;

    FilterConfig filterConfig;
};

#endif //_CONFIG_G