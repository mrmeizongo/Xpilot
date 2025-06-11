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
#define SETINPUT(rawValue, deadBand, inLowRange, inMidRange, inHighRange, outLowRange, outMidRange, outHighRange) \
    (abs((rawValue) - (inMidRange)) <= (deadBand) ? (outMidRange) : map((rawValue), (inLowRange), (inHighRange), (outLowRange), (outHighRange)))

// Abstract flight mode class
class Mode
{
public:
    Mode() {};
    Mode(const Control::THREE_POS_SW modePos) { setModeSwitchPosition(modePos); } // Constructor with mode switch position;
    virtual const char *modeName4(void) const = 0;                                // Returns string representation of the flight mode. 4 characters max
    virtual void enter(void) {}                                                   // Preliminary setup on mode enter
    virtual void process(void) = 0;                                               // Convert user input to mode specific targets, should be called first in the run function
    virtual void run(void) = 0;                                                   // High level processing specific to this mode
    virtual void exit(void) {}                                                    // Perform any clean up before switching to another mode
    static void setServoOut(void) { actuators.setServoOut(SRVout); }              // Write servo outputs to the actuators object

    void setModeSwitchPosition(Control::THREE_POS_SW modePos) { modeSwitchPosition = modePos; } // Set the mode switch position. Should be called from main set up function for config
    Control::THREE_POS_SW getModeSwitchPosition(void) { return modeSwitchPosition; }            // Return mode switch position for this mode

protected:
    static int16_t rollOut;                                              // Roll output
    static int16_t pitchOut;                                             // Pitch output
    static int16_t yawOut;                                               // Yaw output
    static int16_t SRVout[Actuators::Channel::NUM_CHANNELS];             // Servo output array
    static void planeMixer(const int16_t, const int16_t, const int16_t); // Mixer for different airplane types
    static void rudderMixer(void);                                       // Mix roll input with yaw input for rudder control(i.e. coordinated turns)
    virtual void yawController(void) {}                                  // Yaw control for heading-hold-like functionality
    virtual void controlFailsafe(void) = 0;                              // Failsafe implementation

    Control::THREE_POS_SW modeSwitchPosition; // Mode's switch position

    // PID controllers
    static PIDF rollPIDF;
    static PIDF pitchPIDF;
    static PIDF yawPIDF;
};

// Manual control of flight surfaces - USE WITH CAUTION. FOR ADVANCED FLYERS ONLY!
class PassthroughMode : public Mode
{
public:
    const char *modeName4(void) const override { return "PASS"; }
    void process(void) override;
    void run(void) override;

protected:
    void controlFailsafe(void) override;
};

// Gyro-based rate control
class RateMode : public Mode
{
public:
    const char *modeName4(void) const override { return "RATE"; }
    void enter(void) override;
    void process(void) override;
    void run(void) override;
    void yawController(void) override;

protected:
    void controlFailsafe(void) override;
};

// Gyro-based rate control with wing leveling on stick release
class StabilizeMode : public Mode
{
public:
    const char *modeName4(void) const override { return "STAB"; }
    void enter(void) override;
    void process(void) override;
    void run(void) override;
    void yawController(void) override;

protected:
    void controlFailsafe(void) override;
};

#endif // _MODE_H