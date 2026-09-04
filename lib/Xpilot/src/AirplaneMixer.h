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
        int16_t leftAileron;
        int16_t rightAileron;
        int16_t elevator;
        int16_t rudder;
    };

    AirplaneMixer(AirplaneMixer&&) = default;
    AirplaneMixer& operator=(AirplaneMixer&&) = default;

    explicit AirplaneMixer(Config::AirframeType type = Config::AirframeType::CONVENTIONAL)
        : _type(type)
        , _commandLimit(Control::RESOLUTION)
    {
    }

    Outputs mix(int16_t roll, int16_t pitch, int16_t yaw) const;

    void setAirframeType(Config::AirframeType type) { _type = type; }
    Config::AirframeType getAirframeType() const { return _type; }

    int16_t getCommandLimit() const { return _commandLimit; }

protected:
    /*
     * Custom mixer hook.
     */
    virtual void mixCustom(int32_t roll, int32_t pitch, int32_t yaw, Outputs& out) const
    {
        // Default custom behavior = conventional
        mixConventional(roll, pitch, yaw, out);
    }

private:
    Config::AirframeType _type;
    int16_t _commandLimit;

    void mixConventional(int16_t roll, int16_t pitch, int16_t yaw, Outputs& out) const;

    void mixVTail(int16_t roll, int16_t pitch, int16_t yaw, Outputs& out) const;

    void mixFlyingWingRudder(int16_t roll, int16_t pitch, int16_t yaw, Outputs& out) const;

    void mixFlyingWingNoRudder(int16_t roll, int16_t pitch, Outputs& out) const;

    void mixRudderElevator(int16_t pitch, int16_t yaw, Outputs& out) const;

    void mixAileronElevator(int16_t roll, int16_t pitch, Outputs& out) const;

    void mixDifferential(int16_t common, int16_t differential, int16_t& output1, int16_t& output2) const;

    void normalizePair(int32_t& a, int32_t& b) const;
};
#endif // _AIRPLANE_MIXER