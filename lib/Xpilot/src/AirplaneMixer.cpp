#include "AirplaneMixer.h"

// Main interface requested
AirplaneMixer::Outputs AirplaneMixer::mix(int16_t roll, int16_t pitch, int16_t yaw) const
{
    Outputs out{};

    switch (_type)
    {
    case AirframeType::CONVENTIONAL:
        mixConventional(roll, pitch, yaw, out);
        break;

    case AirframeType::V_TAIL:
        mixVTail(roll, pitch, yaw, out);
        break;

    case AirframeType::FLYING_WING_RUDDER:
        mixFlyingWingRudder(roll, pitch, yaw, out);
        break;

    case AirframeType::FLYING_WING_NO_RUDDER:
        mixFlyingWingNoRudder(roll, pitch, out);
        break;

    case AirframeType::RUDDER_ELEVATOR:
        mixRudderElevator(pitch, yaw, out);
        break;

    case AirframeType::AILERON_ELEVATOR:
        mixAileronElevator(roll, pitch, out);
        break;

    case AirframeType::CUSTOM:
        mixCustom(roll, pitch, yaw, out);
        break;
    }

    return out;
}

/*
 * Conventional aircraft
 *
 *             Roll
 *              |
 *       +------+------+
 *       |             |
 *    Left ail.     Right ail.
 *
 * Pitch -> Elevator
 * Yaw   -> Rudder
 */
void AirplaneMixer::mixConventional(
    int16_t roll,
    int16_t pitch,
    int16_t yaw,
    Outputs &out) const
{
    out.leftAileron = roll;
    out.rightAileron = -roll;

    out.elevator = pitch;
    out.rudder = yaw;
}

/*
 * V-tail:
 *
 * Elevator contribution moves both surfaces together.
 * Rudder contribution moves them differentially.
 *
 * Left  = pitch + yaw
 * Right = pitch - yaw
 */
void AirplaneMixer::mixVTail(
    int16_t roll,
    int16_t pitch,
    int16_t yaw,
    Outputs &out) const
{
    out.leftAileron = roll;
    out.rightAileron = roll;

    mixDifferential(
        yaw,
        pitch,
        out.rudder,
        out.elevator);
}

/*
 * Flying wings:
 *
 * Pitch moves both elevons together.
 * Roll moves them differentially.
 *
 * Left  = pitch + roll
 * Right = pitch - roll
 */
void AirplaneMixer::mixFlyingWingRudder(
    int16_t roll,
    int16_t pitch,
    int16_t yaw,
    Outputs &out) const
{
    out.rudder = yaw;

    mixDifferential(
        roll,
        pitch,
        out.leftAileron,
        out.rightAileron);
}

void AirplaneMixer::mixFlyingWingNoRudder(
    int16_t roll,
    int16_t pitch,
    Outputs &out) const
{
    mixDifferential(
        roll,
        pitch,
        out.leftAileron,
        out.rightAileron);
}

/*
 * Rudder/elevator aircraft:
 *
 * Typically used for simple trainers or aircraft with sufficient
 * dihedral to convert yaw into roll.
 */
void AirplaneMixer::mixRudderElevator(
    int16_t pitch,
    int16_t yaw,
    Outputs &out) const
{
    out.elevator = pitch;
    out.rudder = yaw;
}

void AirplaneMixer::mixAileronElevator(
    int16_t roll,
    int16_t pitch,
    Outputs &out) const
{
    out.leftAileron = roll;
    out.rightAileron = roll;
    out.elevator = pitch;
}

/*
 * Mix two axes while preserving their relationship if saturation occurs.
 *
 * Example:
 *
 * pitch = 800
 * yaw   = 600
 *
 * raw:
 *   left  = 1400
 *   right = 200
 *
 * Instead of simply clipping left to 1000, both outputs are scaled
 * proportionally:
 *
 *   left  = 1000
 *   right = 143
 *
 * This preserves the requested pitch/yaw ratio better than independent
 * clipping.
 */
void AirplaneMixer::mixDifferential(
    int16_t common,
    int16_t differential,
    int16_t &output1,
    int16_t &output2) const
{
    int16_t a = common + differential;
    int16_t b = common - differential;

    normalizePair(a, b);

    output1 = a;
    output2 = b;
}

void AirplaneMixer::normalizePair(int16_t &a, int16_t &b) const
{
    uint16_t maxMagnitude = abs(a);
    const uint16_t bMagnitude = abs(b);

    if (bMagnitude > maxMagnitude)
    {
        maxMagnitude = bMagnitude;
    }

    if (maxMagnitude <= _commandLimit)
    {
        return;
    }

    a = (a * _commandLimit) / maxMagnitude;
    b = (b * _commandLimit) / maxMagnitude;
}