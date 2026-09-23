// Started - 08/19/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
// Updated - 02/25/2025 by Jamal Meizongo
// This and other library code in this repository
// are partial releases and work is still in progress.
// Please keep this in mind as you use this piece of software.

/* ============================================
Flight stabilization software
    Copyright (C) 2024 Jamal Meizongo (mrmeizongo@outlook.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
===============================================
*/
#ifndef _RADIO_H
#define _RADIO_H

#include <stdint.h>
#include "SysConfig.h"
#include "FlightConfigAccess.h"

#define PIN_HIGH(pin) ((PIND & _BV(pin)) != 0)

constexpr uint16_t PWM_MIN_US = 600;            // Lowest valid pwm expected from transmitter
constexpr uint16_t PWM_TRIM_US = 1500;          // Trim pwm expected from transmitter
constexpr uint16_t PWM_MAX_US = 2400;           // Highest valid pwm expected from transmitter
constexpr uint32_t TIMEOUT_US = 110000;         // Rx timeout in micros; 5 missed PWM(22ms) frames triggers a failsafe
constexpr int16_t THREE_SW_POS_THRESHOLD = 136; // 3 position switch input separator

constexpr uint16_t THROTTLE_CUT_THRESHOLD = 1050;     // User selected normal throttle cut threshold (-125% throttle)
constexpr uint16_t THROTTLE_FAILSAFE_THRESHOLD = 950; // User selected failsafe throttle threshold (-150% throttle)
constexpr int16_t THROTTLE_SHUTOFF_VALUE = -1000;     // Normalized shut off value for throttle (-1000 : +1000)

inline int32_t
normalizeInput(int16_t rawVal, int16_t inputMin, int16_t inputTrim, int16_t inputMax, uint8_t deadband, bool reverse)
{
    const int32_t delta = static_cast<int32_t>(rawVal - inputTrim);

    if (abs(delta) <= deadband)
        return 0;

    int32_t output;

    if (delta > 0)
    {
        output = (delta * config().controlConfig.controlResolution) / (inputMax - inputTrim);
    }
    else
    {
        output = (delta * config().controlConfig.controlResolution) / (inputTrim - inputMin);
    }

    return reverse ? -output : output;
}

class Radio
{
public:
    enum class THROTTLE_STATE : uint8_t
    {
        NORMAL,
        CUT,
        FAILSAFE,
        SIGNAL_LOST
    };

    // 3-position switch
    enum class THREE_POS_SW : uint8_t
    {
        LOW_POS = 0U,
        MID_POS,
        HIGH_POS,
        UNDEFINED, // Undefined position, should not be used
    };

    // Do not change the order
    enum CHANNEL : uint8_t
    {
        THROTTLE = 0U,
        ROLL,
        PITCH,
        YAW,
        AUX1,
        AUX2,

        CHANNEL_COUNT
    };

    Radio(void);

    void init(void);

    void processInput(void);

    static void processInputTask(void* ctx) // Trampoline function for the scheduler to call the processInput function
    {
        static_cast<Radio*>(ctx)->processInput();
    }

    void setRawPWM(CHANNEL, const volatile uint16_t&, const volatile uint32_t&);

    bool getValidControlPWM(uint16_t*, uint8_t);

    uint16_t getPWM(CHANNEL);

    THREE_POS_SW
    getThreeSwitchPos(CHANNEL, uint16_t trim = PWM_TRIM_US, uint16_t threshold = THREE_SW_POS_THRESHOLD);

    uint32_t getSignalLossTimeUs(void) { return signalLossTimeUs; }

    bool inThrottleCut(void) { return txThrottleCut; }

    bool inFailsafe(void) const { return failSafe; }

private:
    uint16_t rawPWM[CHANNEL::CHANNEL_COUNT];
    uint16_t lastValidPWM[CHANNEL::CHANNEL_COUNT];

    uint32_t signalLossTimeUs;
    uint32_t lastRawPWMTimeUS[CHANNEL::CHANNEL_COUNT];

    bool failSafe;

    bool failSafeTimerStarted;

    bool txThrottleCut;

    void FailSafeDetector();

    enum CHANNELMASK : uint8_t
    {
        NONE = 0x0,
        REQ_THROTTLE = 1 << 0,
        REQ_ROLL = 1 << 1,
        REQ_PITCH = 1 << 2,
        REQ_YAW = 1 << 3
    };

    uint8_t requiredChannels();

    THROTTLE_STATE decodeThrottleState(uint32_t);
};

extern Radio radio;
#endif // _RADIO_H