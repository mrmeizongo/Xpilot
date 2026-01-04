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
#include <PlaneConfig.h>

// 3-position switch
enum class THREE_POS_SW : uint8_t
{
    UNDEFINED = 0U, // Undefined position, should not be used
    LOW_POS,
    MID_POS,
    HIGH_POS
};

/*
 * Control struct
 * Holds the radio input values
 * Roll, Pitch, Yaw, Mode
 * Roll, Pitch & Yaw PWM values
 */
struct Control
{
    uint16_t rollPWM;
    uint16_t pitchPWM;
    uint16_t yawPWM;
    uint16_t aux1PWM;
    THREE_POS_SW aux1SwitchPos;
#if defined(USE_FLAPERONS)
    uint16_t aux2PWM;
    THREE_POS_SW aux2SwitchPos;
#endif
#if defined(USE_AUX3)
    uint16_t aux3PWM;
    THREE_POS_SW aux3SwitchPos;
#endif

    Control(void)
    {
        rollPWM = INPUT_MID_PWM;
        pitchPWM = INPUT_MID_PWM;
        yawPWM = INPUT_MID_PWM;
        aux1PWM = INPUT_MID_PWM;
        aux1SwitchPos = THREE_POS_SW::UNDEFINED;
#if defined(USE_FLAPERONS)
        aux2PWM = INPUT_MID_PWM;
        aux2SwitchPos = THREE_POS_SW::UNDEFINED;
#endif
#if defined(USE_AUX3)
        aux3PWM = INPUT_MID_PWM;
        aux3SwitchPos = THREE_POS_SW::UNDEFINED;
#endif
    }
};

class Radio
{
public:
    Radio(void);
    void init(void);
    void processInput(void);
    bool inFailsafe(void) { return failSafe; }

    int16_t getRxRollPWM(void) { return currentRx.rollPWM; }
    int16_t getRxPitchPWM(void) { return currentRx.pitchPWM; }
    int16_t getRxYawPWM(void) { return currentRx.yawPWM; }
    int16_t getRxAux1PWM(void) { return currentRx.aux1PWM; }
    THREE_POS_SW getRxAux1Pos(void) { return currentRx.aux1SwitchPos; }
#if defined(USE_FLAPERONS)
    int16_t getRxAux2PWM(void) { return currentRx.aux2PWM; }
    THREE_POS_SW getRxAux2Pos(void) { return currentRx.aux2SwitchPos; }
#endif
#if defined(USE_AUX3)
    int16_t getRxAux3PWM(void) { return currentRx.aux3PWM; }
    THREE_POS_SW getRxAux3Pos(void) { return currentRx.aux3SwitchPos; }
#endif

private:
    Control currentRx;
    bool failSafe;
    void FailSafe();
};

extern Radio radio;
#endif