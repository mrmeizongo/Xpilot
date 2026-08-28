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
#include "SystemConfig.h"

constexpr int16_t RX_PWM_MIN = 1000;           // Default owest pwm expected from transmitter
constexpr int16_t RX_PWM_MAX = 2000;           // Default highest pwm expected from transmitter
constexpr int16_t RX_PWM_TRIM = 1500;          // Mid pwm expected from transmitter
constexpr int16_t RX_FAILSAFE_TOLERANCE = 200; // Tolerance used for determining a failsafe condition
constexpr int16_t RX_TIMEOUT_MS = 500;         // Receiver timeout
constexpr int16_t RX_3_SW_POS_THRESHOLD = 276; // 3 position switch input separator

class Radio
{
public:
    enum CHANNELS : uint8_t
    {
        ROLL = 0U,
        PITCH,
        YAW,
        AUX1,
#if defined(USE_AUX2)
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
    static void processInputTask(void *ctx) // Trampoline function for the scheduler to call the processInput function
    {
        static_cast<Radio *>(ctx)->processInput();
    }

    int16_t getPWM(CHANNELS ch)
    {
        if (ch < CHANNELS::CHANNEL_COUNT)
        {
            if (failSafeTimerStarted)
                return RX_PWM_TRIM;

            return raw[ch];
        }

        return RX_PWM_TRIM;
    }

    void setPWM(int16_t &, uint32_t, Radio::CHANNELS);

    THREE_POS_SW getThreeSwitchPos(CHANNELS ch)
    {
        if (ch < CHANNELS::CHANNEL_COUNT)
        {
            int16_t pwm = raw[ch];
            if (pwm >= RX_PWM_MAX - RX_3_SW_POS_THRESHOLD)
                return THREE_POS_SW::HIGH_POS;
            else if (pwm <= RX_PWM_MIN + RX_3_SW_POS_THRESHOLD)
                return THREE_POS_SW::LOW_POS;
            else
                return THREE_POS_SW::MID_POS;
        }

        return THREE_POS_SW::UNDEFINED;
    }

    uint32_t getLastValidRxTimeMs(void) { return lastValidRxTimeMs; }

    bool inFailsafe(void) { return failSafe; }

private:
    int16_t raw[CHANNELS::CHANNEL_COUNT];
    uint32_t lastValidRxTimeMs;
    bool failSafe;
    bool failSafeTimerStarted;
    void FailSafe();
};

extern Radio radio;
#endif