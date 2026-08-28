// 02/25/2025 by Jamal Meizongo (mrmeizongo@outlook.com)

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
#include "AirplaneMixer.h"
#include "SystemConfig.h"
#include "FlightConfigAccess.h"
#include "SlewRateLimiter.h"
#include "PIDF.h"
#include "Radio.h"
#include "Actuators.h"

inline float getNormalizedInput(
    int16_t rawVal,
    int16_t inputMin,
    int16_t inputTrim,
    int16_t inputMax,
    uint8_t deadband)
{
    const int16_t delta = rawVal - inputTrim;

    if (abs(delta) <= deadband)
        return 0.0f;

    if (delta > 0)
    {
        return static_cast<float>(delta) /
               static_cast<float>(inputMax - inputTrim);
    }

    return static_cast<float>(delta) /
           static_cast<float>(inputTrim - inputMin);
}

inline int16_t map(
    int16_t input,
    int16_t inputRes,
    int16_t outputMin,
    int16_t outputMax)
{
    // Guard against division by zero
    if (inputRes == 0)
        return outputMin;

    // Type promotion
    const int32_t x = input;
    const int32_t res = inputRes;
    const int32_t min = outputMin;
    const int32_t max = outputMax;
    const int32_t range = max - min;

    return static_cast<int16_t>(
        min + ((x + res) * range) / (2 * res));
}

// Abstract flight mode class
class Mode
{
public:
    Mode() {}
    Mode(const Radio::THREE_POS_SW modePos) { setModeSwitchPosition(modePos); } // Constructor with mode switch position;
    virtual ~Mode() = default;                                                  // Virtual destructor for proper cleanup of derived classes

    virtual const char *modeName4(void) const = 0; // Returns string representation of the flight mode. 4 characters max

    virtual void enter(void) {} // Preliminary setup on mode enter
    virtual void update(void);  // Convert user input to mode specific targets, should be called first in the run function
    virtual void run(void) = 0; // High level processing specific to this mode
    virtual void exit(void) {}  // Perform any clean up before switching to another mode

    virtual bool imuAssisted(void) const { return false; } // Does this mode use the imu

    void init(void);

    // Sceduler trampoline functions
    static void runTask(void *);
    static void updateInput(void *);

    static void updateAHRS(float (&)[3], float (&)[3], void *);

    static void configSub(ConfigID, void *);

    // Debug functions to get outputs for testing and tuning purposes.
    static int16_t getRollInput(void) { return input_rpy[0]; }
    static int16_t getPitchInput(void) { return input_rpy[1]; }
    static int16_t getYawInput(void) { return input_rpy[2]; }

    static int16_t getRollOutput(void) { return output_rpy[0]; }
    static int16_t getPitchOutput(void) { return output_rpy[1]; }
    static int16_t getYawOutput(void) { return output_rpy[2]; }

#if defined(USE_FLAPERONS)
    static int16_t getFlaperon(void) { return flaperonOut; }
    static void flaperonInput(void)
    {
        // flaperonOut = getRawInput(radio.getPWM(Radio::CHANNELS::AUX2), rollConfig.trim, rollConfig.min, 0, fConfig.flaperonMax);
        int16_t flapPwm = radio.getPWM(Radio::CHANNELS::AUX2);
        flapPwm = constrain(flapPwm, RX_PWM_MIN, RX_PWM_TRIM);
        flaperonOut = static_cast<int16_t>(
            (static_cast<int32_t>(RX_PWM_TRIM - flapPwm) * fConfig.flaperonMax) / 500);
    }
#endif

    void setModeSwitchPosition(Radio::THREE_POS_SW modePos) { modeSwitchPosition = modePos; } // Set the mode switch position. Should be called from main set up function for config
    Radio::THREE_POS_SW getModeSwitchPosition(void) { return modeSwitchPosition; }            // Return mode switch position for this mode

protected:
    static float imu_rpy[3]; // To hold imu rpy values
    static float imu_g[3];   // To hold imu g values

    static int16_t input_rpy[3];  // Input roll, pitch, and yaw
    static int16_t output_rpy[3]; // Output roll, pitch, and yaw

    static SlewRateLimiter<int16_t> rollSlew;
    static SlewRateLimiter<int16_t> pitchSlew;
    static SlewRateLimiter<int16_t> yawSlew;

    Radio::THREE_POS_SW modeSwitchPosition; // Mode switch position for this mode

    static int16_t SRVout[Actuators::Channel::CHANNEL_COUNT]; // Servo output array

    static void rudderMixer(void); // Mix roll input with yaw input for rudder control(i.e. coordinated turns)

    static void resetControllers(void); // Reset controllers when switching modes to prevent integral windup and derivative kick

    virtual void controlFailsafe(void); // Failsafe implementation

#if defined(USE_FLAPERONS)
    static int16_t flaperonOut;     // Flaperon position value, used in flaperon control
    static void flaperonMixer(void) // Flaperon control, should be called in the run function of the flight mode
    {
        SRVout[Actuators::Channel::CH1] -= flaperonOut;
        SRVout[Actuators::Channel::CH2] += flaperonOut;
    }
#endif

    static PIDF<int16_t> rollPIDF;
    static PIDF<int16_t> pitchPIDF;
    static PIDF<int16_t> yawPIDF;

    static Config::RCConfig rollConfig;
    static Config::RCConfig pitchConfig;
    static Config::RCConfig yawConfig;

    static Config::FlightConfig fConfig;

    static Config::SRVConfig srvConfig;

    static AirplaneMixer airplaneMixer;
};

// Manual control of flight surfaces - USE WITH CAUTION. FOR ADVANCED FLYERS ONLY!
class PassthroughMode : public Mode
{
public:
    const char *modeName4(void) const override { return "PASS"; }
    void enter(void) override;
    void update(void) override;
    void run(void) override;
};

// Gyro-based rate control
class RateMode : public Mode
{
public:
    const char *modeName4(void) const override { return "RATE"; }
    void enter(void) override;
    void update(void) override;
    void run(void) override;
    bool imuAssisted(void) const override { return true; }
};

// Gyro-based rate control with wing leveling on stick release
class StabilizeMode : public Mode
{
public:
    const char *modeName4(void) const override { return "STAB"; }
    void enter(void) override;
    void update(void) override;
    void run(void) override;
    bool imuAssisted(void) const override { return true; }
};

#endif // _MODE_H