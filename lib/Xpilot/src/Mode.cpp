#include "Mode.h"

/*
 * Mixer for airplane type
 * Only tested with a full plane i.e. ailerons, elevator and rudder
 * Proceed with caution. Perform thorough pre-flight checks and reverse servo direction as needed.
 *
 * Default
 * For V-Tail - If servos are mounted on top, with face (sticker side) of servo facing the same direction
 *
 *                 Rudder Left
 *
 *                                ^
 *                 _       _      |
 *         <======|O|     |O|=====>
 *         |  Face| | Face| |
 *         v      |_|     |_|
 *
 *   Push surface out   Pull surface in
 *
 * For Flying Wings, reverse install elevon servos same as you would ailerons for planes
 */
void Mode::planeMixer(const int16_t roll, const int16_t pitch, const int16_t yaw)
{
#if defined(FULL_PLANE)
    SRVout[Actuators::Channel::AILERON1] = roll;
    SRVout[Actuators::Channel::AILERON2] = roll;
    SRVout[Actuators::Channel::ELEVATOR] = pitch;
    SRVout[Actuators::Channel::RUDDER] = yaw;
#elif defined(FULL_PLANE_V_TAIL)
    SRVout[Actuators::Channel::AILERON1] = roll;
    SRVout[Actuators::Channel::AILERON2] = roll;
    SRVout[Actuators::Channel::ELEVATOR] = yaw + pitch;
    SRVout[Actuators::Channel::RUDDER] = yaw - pitch;
#elif defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
    SRVout[Actuators::Channel::AILERON1] = 0;
    SRVout[Actuators::Channel::AILERON2] = 0;
    SRVout[Actuators::Channel::ELEVATOR] = yaw + pitch;
    SRVout[Actuators::Channel::RUDDER] = yaw - pitch;
#elif defined(FLYING_WING_W_RUDDER)
    SRVout[Actuators::Channel::AILERON1] = roll - pitch;
    SRVout[Actuators::Channel::AILERON2] = roll + pitch;
    SRVout[Actuators::Channel::ELEVATOR] = 0;
    SRVout[Actuators::Channel::RUDDER] = yaw;
#elif defined(FLYING_WING_NO_RUDDER)
    SRVout[Actuators::Channel::AILERON1] = roll - pitch;
    SRVout[Actuators::Channel::AILERON2] = roll + pitch;
    SRVout[Actuators::Channel::ELEVATOR] = 0;
    SRVout[Actuators::Channel::RUDDER] = 0;
#elif defined(RUDDER_ELEVATOR_ONLY_PLANE)
    SRVout[Actuators::Channel::AILERON1] = 0;
    SRVout[Actuators::Channel::AILERON2] = 0;
    SRVout[Actuators::Channel::ELEVATOR] = pitch;
    SRVout[Actuators::Channel::RUDDER] = yaw;
#else
#error No airplane type selected!
#endif
}

void Mode::rudderMixer(void)
{
#if defined(FULL_PLANE) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_W_RUDDER)
#if defined(REVERSE_RUDDER_MIX)
    yawOut = yawOut - (rollOut * RUDDER_MIXING);
#else
    yawOut = yawOut + (rollOut * RUDDER_MIXING);
#endif
#endif
}