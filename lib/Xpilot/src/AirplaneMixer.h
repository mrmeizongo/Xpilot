#ifndef _AIRPLANE_MIXER
#define _AIRPLANE_MIXER

#include <Arduino.h>
#include <Config.h>

class AirplaneMixer
{
public:
    /*
     * For common and differential moving flight control surfaces(v tail and flying wings),
     * elevator and rudder act as left and right surfaces, respectively
     */
    struct Outputs
    {
        int32_t leftAileron;
        int32_t rightAileron;
        int32_t elevator;
        int32_t rudder;
    };

    AirplaneMixer(AirplaneMixer &&) = default;
    AirplaneMixer &operator=(AirplaneMixer &&) = default;

    explicit AirplaneMixer(
        Config::AirframeType type = Config::AirframeType::CONVENTIONAL,
        int32_t commandLimit = Control::RESOLUTION)
        : _type(type),
          _commandLimit(commandLimit) {}

    Outputs mix(int32_t roll, int32_t pitch, int32_t yaw) const;

    void setAirframeType(Config::AirframeType type) { _type = type; }
    Config::AirframeType getAirframeType() const { return _type; }

    void setCommandLimit(int16_t limit) { _commandLimit = limit; }
    int16_t getCommandLimit() const { return _commandLimit; }

protected:
    /*
     * Custom mixer hook.
     */
    virtual void mixCustom(
        int32_t roll,
        int32_t pitch,
        int32_t yaw,
        Outputs &out) const
    {
        // Default custom behavior = conventional
        mixConventional(roll, pitch, yaw, out);
    }

private:
    Config::AirframeType _type;
    int32_t _commandLimit;

    void mixConventional(
        int32_t roll,
        int32_t pitch,
        int32_t yaw,
        Outputs &out) const;

    void mixVTail(
        int32_t roll,
        int32_t pitch,
        int32_t yaw,
        Outputs &out) const;

    void mixFlyingWingRudder(
        int32_t roll,
        int32_t pitch,
        int32_t yaw,
        Outputs &out) const;

    void mixFlyingWingNoRudder(
        int32_t roll,
        int32_t pitch,
        Outputs &out) const;

    void mixRudderElevator(
        int32_t pitch,
        int32_t yaw,
        Outputs &out) const;

    void mixAileronElevator(
        int32_t roll,
        int32_t pitch,
        Outputs &out) const;

    void mixDifferential(
        int32_t common,
        int32_t differential,
        int32_t &output1,
        int32_t &output2) const;

    void normalizePair(int32_t &a, int32_t &b) const;
};
#endif // _AIRPLANE_MIXER