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
#include "PIDF.h"
#include "Radio.h"
#include "Actuators.h"

// Normalize input to a -1:1 range
#define NORM_INPUT(rawVal) \
    (2 * ((float)((rawVal) - (INPUT_MIN_PWM)) / (float)(INPUT_MAX_PWM - INPUT_MIN_PWM)) - 1)

#define FILTERED_NORM_INPUT(rawVal, deadBand) \
    (abs((rawVal) - (INPUT_MID_PWM)) <= (deadBand) ? 0 : (NORM_INPUT((rawVal))))

// Abstract flight mode class
class Mode
{
public:
    Mode() {}
    Mode(const THREE_POS_SW modePos) { setModeSwitchPosition(modePos); } // Constructor with mode switch position;
    virtual ~Mode() = default;                                           // Virtual destructor for proper cleanup of derived classes
    virtual const char *modeName4(void) const = 0;                       // Returns string representation of the flight mode. 4 characters max
    virtual void enter(void) {}                                          // Preliminary setup on mode enter
    virtual void process(void) = 0;                                      // Convert user input to mode specific targets, should be called first in the run function
    virtual void run(void) = 0;                                          // High level processing specific to this mode
    virtual void exit(void) {}                                           // Perform any clean up before switching to another mode
    virtual bool imuAssisted(void) const { return false; }               // Does this mode use the imu
    static void runTask(void *ctx)                                       // Trampoline function for the scheduler to call the run function
    {
        Mode **modePointer = static_cast<Mode **>(ctx);
        if (*modePointer != nullptr)
        {
            (*modePointer)->run();
        }
    }

    // Debug functions to get outputs for testing and tuning purposes.
    static int16_t getRollInput(void) { return input_rpy[0]; }
    static int16_t getPitchInput(void) { return input_rpy[1]; }
    static int16_t getYawInput(void) { return input_rpy[2]; }
    static int16_t getRollOutput(void) { return output_rpy[0]; }
    static int16_t getPitchOutput(void) { return output_rpy[1]; }
    static int16_t getYawOutput(void) { return output_rpy[2]; }
    static uint8_t getSkippedImuInstances(void) { return missedImuInstances; }
    static bool getFaultState(void) { return imuFault; }

#if defined(USE_FLAPERONS)
    static int16_t getFlaperon(void) { return flaperonOut; }
    static void flaperonInput(void);
#endif
    static void servoOut(void *); // Constrain and write servo outputs to the actuators object
    static bool imuDataHealthy(void);

    void setModeSwitchPosition(THREE_POS_SW modePos) { modeSwitchPosition = modePos; } // Set the mode switch position. Should be called from main set up function for config
    THREE_POS_SW getModeSwitchPosition(void) { return modeSwitchPosition; }            // Return mode switch position for this mode

protected:
    static int16_t input_rpy[3];                                         // Mode dependent transformed input roll, pitch and yaw
    static int16_t output_rpy[3];                                        // Mode dependent processed output for roll, pitch, yaw
    THREE_POS_SW modeSwitchPosition;                                     // Mode switch position for this mode
    static int16_t SRVout[Actuators::Channel::NUM_CHANNELS];             // Servo output array
    static uint8_t missedImuInstances;                                   // If the current mode goes 2 loops without ahrs sensor values, switch to passthrough mode
    static bool imuFault;                                                // if true, imu is in a faulted state
    static void planeMixer(const int16_t, const int16_t, const int16_t); // Mixer for different airplane types
    static void rudderMixer(void);                                       // Mix roll input with yaw input for rudder control(i.e. coordinated turns)
    static void resetControllers(void);                                  // Reset controllers when switching modes to prevent integral windup and derivative kick
    virtual void controlFailsafe(void);                                  // Failsafe implementation
#if defined(USE_FLAPERONS)
    static uint16_t flaperonOut;    // Flaperon position value, used in flaperon control
    static void setFlaperons(void); // Flaperon control, should be called in the run function of the flight mode
#endif

    // PID controllers
    static PIDF<int16_t> rollPIDF;
    static PIDF<int16_t> pitchPIDF;
    static PIDF<int16_t> yawPIDF;
};

// Manual control of flight surfaces - USE WITH CAUTION. FOR ADVANCED FLYERS ONLY!
class PassthroughMode : public Mode
{
public:
    const char *modeName4(void) const override { return "PASS"; }
    void process(void) override;
    void run(void) override;
};

// Gyro-based rate control
class RateMode : public Mode
{
public:
    const char *modeName4(void) const override { return "RATE"; }
    void enter(void) override;
    void process(void) override;
    void run(void) override;
    bool imuAssisted(void) const override { return true; }
};

// Gyro-based rate control with wing leveling on stick release
class StabilizeMode : public Mode
{
public:
    const char *modeName4(void) const override { return "STAB"; }
    void enter(void) override;
    void process(void) override;
    void run(void) override;
    bool imuAssisted(void) const override { return true; }

protected:
    void controlFailsafe(void) override;
};

#endif // _MODE_H