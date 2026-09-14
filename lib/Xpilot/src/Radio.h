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

constexpr uint16_t RX_PWM_MIN_US = 600;           // Lowest valid pwm expected from transmitter
constexpr uint16_t RX_PWM_TRIM_US = 1500;         // Trim pwm expected from transmitter
constexpr uint16_t RX_PWM_MAX_US = 2400;          // Highest valid pwm expected from transmitter
constexpr uint32_t RX_TIMEOUT_US = 110000;        // Rx timeout in micros; 5 missed PWM(22ms) frames triggers a failsafe
constexpr int16_t RX_3_SW_POS_THRESHOLD = 133;    // 3 position switch input separator
constexpr uint16_t RX_THROTTLE_FAILSAFE_TOL = 52; // Differentiate between a commanded throttle cut and signal loss
constexpr int16_t THROTTLE_FAILSAFE_VALUE = -800; // Normalized failsafe value for throttle (-1000 : +1000)

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

inline void
capturePWMEdge(uint8_t pin, volatile uint32_t& riseTimeUs, volatile uint16_t& pulseUs, volatile uint32_t lastValid)
{
    const uint32_t now = micros();

    if (PIN_HIGH(pin))
    {
        riseTimeUs = now;
        return;
    }

    const uint32_t rawPulse = now - riseTimeUs;

    if (rawPulse >= RX_PWM_MIN_US && rawPulse <= RX_PWM_MAX_US)
    {
        pulseUs = static_cast<uint16_t>(rawPulse);
        lastValid = now;
    }
}

class Radio
{
public:
    enum CHANNEL : uint8_t
    {
        THROTTLE = 0U,
        ROLL,
        PITCH,
        YAW,
        AUX1,
#if defined(USE_AUX2IN)
        AUX2,
#endif
        CHANNEL_COUNT
    };

    // 3-position switch
    enum class THREE_POS_SW : uint8_t
    {
        UNDEFINED = 0U, // Undefined position, should not be used
        LOW_POS,
        MID_POS,
        HIGH_POS
    };

    Radio(void);

    void init(void);

    void processInput(void);

    static void processInputTask(void* ctx) // Trampoline function for the scheduler to call the processInput function
    {
        static_cast<Radio*>(ctx)->processInput();
    }

    void setPWM(CHANNEL ch, uint16_t rawPulse) { raw[ch] = rawPulse; }

    uint16_t getPWM(CHANNEL ch)
    {
        if (ch >= CHANNEL::CHANNEL_COUNT)
            return 0;

        if (failSafeTimerStarted)
            return RX_PWM_TRIM_US;

        return raw[ch];
    }

    THREE_POS_SW getThreeSwitchPos(CHANNEL ch)
    {
        if (ch >= CHANNEL::CHANNEL_COUNT)
            return THREE_POS_SW::UNDEFINED;

        if (raw[ch] < RX_PWM_TRIM_US - RX_3_SW_POS_THRESHOLD)
            return THREE_POS_SW::LOW_POS;

        if (raw[ch] > RX_PWM_TRIM_US + RX_3_SW_POS_THRESHOLD)
            return THREE_POS_SW::HIGH_POS;

        return THREE_POS_SW::MID_POS;
    }

    uint32_t getSignalLossTimeUs(void) { return signalLossTimeUs; }

    bool inFailsafe(void) const { return failSafe; }

private:
    uint16_t raw[CHANNEL::CHANNEL_COUNT];

    uint32_t signalLossTimeUs;

    bool failSafe;

    bool failSafeTimerStarted;

    void FailSafe();

    enum CHANNELMASK : uint8_t
    {
        NONE = 0x0,
        REQ_THROTTLE = 1 << 0,
        REQ_ROLL = 1 << 1,
        REQ_PITCH = 1 << 2,
        REQ_YAW = 1 << 3
    };

    uint8_t requiredChannels();
};

extern Radio radio;
#endif // _RADIO_H