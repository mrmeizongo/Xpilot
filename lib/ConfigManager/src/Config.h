#ifndef _CONFIG_H
#define _CONFIG_H
#include <stdint.h>

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

    struct RCConfig
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
        int16_t controlResolution;

        int16_t maxRollRateDegs;
        int16_t maxPitchRateDegs;
        int16_t maxYawRateDegs;

        int16_t maxRollAngleDegs;
        int16_t maxPitchAngleDegs;

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

    AirframeConfig airframeType;

    RCConfig rollRC;
    RCConfig pitchRC;
    RCConfig yawRC;

    SRVConfig srvConfig;

    FlightConfig flightConfig;

    PIDFConfig rollPIDF;
    PIDFConfig pitchPIDF;
    PIDFConfig yawPIDF;

    IMUConfig ahrsIMU;

    FilterConfig processFilter;
};

#endif //_CONFIG_G