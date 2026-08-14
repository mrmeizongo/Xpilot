#include "AirplaneMixer.h"

AirplaneMixer::Outputs AirplaneMixer::mix(int16_t roll, int16_t pitch, int16_t yaw) const
{
    Outputs out{};

    switch (_type)
    {
    default:
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
        out.elevator,
        out.rudder);
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
    int32_t left = common + differential;
    int32_t right = common - differential;

    normalizePair(left, right);

    output1 = left;
    output2 = right;
}

void AirplaneMixer::normalizePair(int32_t &a, int32_t &b) const
{
    int32_t maxMagnitude = abs(a);
    const int32_t bMagnitude = abs(b);

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