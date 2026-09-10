#ifndef _AIRPLANE_MIXER
#define _AIRPLANE_MIXER

#include <Arduino.h>
#include "FlightConfigAccess.h"

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

    explicit AirplaneMixer(Config::AirframeType type = Config::AirframeType::CONVENTIONAL,
                           int16_t limit = 1000,
                           bool reverseRoll = false,
                           bool reversePitch = false,
                           bool reverseYaw = false)
        : _type{type}
        , _commandLimit{limit}
        , _reverseRollOutput{reverseRoll}
        , _reversePitchOutput{reversePitch}
        , _reverseYawOutput{reverseYaw}
    {
    }

    Outputs mix(int16_t roll, int16_t pitch, int16_t yaw, int16_t flaperon) const;

    int32_t mixRudderInput(const int32_t&, const int32_t&);

    Config::AirframeType getAirframeType() const { return _type; }
    void setAirframeType(Config::AirframeType type) { _type = type; }

    int16_t getCommandLimit() const { return _commandLimit; }
    void setCommandLimit(int16_t limit) { _commandLimit = limit; }

    bool getRollReverse() const { return _reverseRollOutput; }
    void setRollReverse(bool roll) { _reverseRollOutput = roll; }

    bool getPitchReverse() const { return _reversePitchOutput; }
    void setPitchReverse(bool pitch) { _reversePitchOutput = pitch; }

    bool getYawReverse() const { return _reverseYawOutput; }
    void setYawReverse(bool yaw) { _reverseYawOutput = yaw; }

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

    bool _reverseRollOutput;
    bool _reversePitchOutput;
    bool _reverseYawOutput;

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