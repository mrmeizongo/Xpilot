// 02/25/2025 by Jamal Meizongo (mrmeizongo@outlook.com)
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
#ifndef _MODE_H
#define _MODE_H

#include <Arduino.h>
#include "PlaneConfig.h"
#include "Radio.h"
#include "Actuators.h"
#include "PIDF.h"

// Helper define to transform radio values to mode dependent resolutions
#define SETINPUT(rawValue, deadBand, inLowRange, inMidRange, inHighRange, outLowRange, outHighRange) \
    (abs((rawValue) - (inMidRange)) <= (deadBand) ? 0 : map((rawValue), (inLowRange), (inHighRange), (outLowRange), (outHighRange)))

// Abstract flight mode class
class Mode
{
public:
    Mode() {};
    virtual const char *modeName4(void) const = 0;            // Returns string representation of the flight mode. 4 characters max
    virtual Control::MODEPOS modePos(void) const = 0;         // Return mode switch position for this mode
    virtual void enter(void) {}                               // Preliminary setup
    virtual void process(void) = 0;                           // Convert user input to mode specific targets, should be called first in the run function
    virtual void run(void) = 0;                               // High level processing specific to this mode
    virtual void exit(void) {}                                // Perform any clean up before switching to another mode
    virtual void controlFailsafe(void);                       // Placeholder for failsafe implementation. Default simply sets all outputs to neutral i.e. 0
    void setServoOut(void) { actuators.setServoOut(SRVout); } // Write servo outputs to the actuators object

protected:
    int16_t rollInput = 0;                                                // Roll output
    int16_t pitchInput = 0;                                               // Pitch output
    int16_t yawInput = 0;                                                 // Yaw output
    int16_t SRVout[Actuators::Channel::NUM_CHANNELS]{0, 0, 0, 0};         // Servo output array
    virtual void planeMixer(const int16_t, const int16_t, const int16_t); // Mixer for different airplane types
    virtual void yawController(void) {}                                   // Yaw control for for heading-hold-like functionality
    virtual void rudderMixer(void);                                       // Mix roll input with yaw input for rudder control(i.e. coordinated turns)
    PIDF rollPIDF{ROLL_KP, ROLL_KI, ROLL_KD, ROLL_KF, ROLL_I_WINDUP_MAX};
    PIDF pitchPIDF{PITCH_KP, PITCH_KI, PITCH_KD, PITCH_KF, PITCH_I_WINDUP_MAX};
    PIDF yawPIDF{YAW_KP, YAW_KI, YAW_KD, YAW_KF, YAW_I_WINDUP_MAX};
};

// Manual control of flight surfaces - USE WITH CAUTION. FOR ADVANCED FLYERS ONLY!
class PassthroughMode : public Mode
{
public:
    const char *modeName4(void) const override { return "PASS"; }
    Control::MODEPOS modePos(void) const override { return Control::MODEPOS::LOW_POS; }
    void process(void) override;
    void run(void) override;
};

// Gyro-based rate control
class RateMode : public Mode
{
public:
    const char *modeName4(void) const override { return "RATE"; }
    Control::MODEPOS modePos(void) const override { return Control::MODEPOS::MID_POS; }
    void enter(void) override;
    void process(void) override;
    void run(void) override;
    void yawController(void) override;
};

// Gyro-based rate control with wing leveling on stick release
class StabilizeMode : public Mode
{
public:
    const char *modeName4(void) const override { return "STAB"; }
    Control::MODEPOS modePos(void) const override { return Control::MODEPOS::HIGH_POS; }
    void enter(void) override;
    void process(void) override;
    void run(void) override;
    void yawController(void) override;
};

#endif // _MODE_H