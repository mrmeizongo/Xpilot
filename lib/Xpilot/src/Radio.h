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
    LOW_POS = 0U,
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
    THREE_POS_SW modePos;

    Control(THREE_POS_SW _modePos)
        : rollPWM{INPUT_MID_PWM}, pitchPWM{INPUT_MID_PWM}, yawPWM{INPUT_MID_PWM},
          modePos{_modePos} {}
};

class Radio
{
public:
    Radio(void);
    void init(void);
    void processInput(void);
    bool inFailsafe(void) { return failsafe; }

    int16_t getRxRollPWM(void) { return rx.rollPWM; }
    int16_t getRxPitchPWM(void) { return rx.pitchPWM; }
    int16_t getRxYawPWM(void) { return rx.yawPWM; }
    THREE_POS_SW getRxModePos(void) { return rx.modePos; }

private:
    Control rx{THREE_POS_SW::MID_POS}; // RATE mode by default
    bool failsafe;
};

extern Radio radio;
#endif