// Started - 03/13/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
// Updated - 07/27/2026 by Jamal Meizongo

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

#ifndef _XPILOT_H
#define _XPILOT_H
#include "Mode.h"
#include "SerialConfigTask.h"

class Xpilot
{
public:
    Xpilot(void);
    Xpilot(const Xpilot&) = delete;            // Prevent this class from being copyable
    Xpilot& operator=(const Xpilot&) = delete; // Prevent this class from being assignable

    // Only functions called from the main setup and loop functions
    void setup(void);
    void loop(void);

    // Trampoline functions for the scheduler
    static void stateUpdateTask(void* ctx) { static_cast<Xpilot*>(ctx)->stateUpdate(); }
    static void runSerialConfigTask(void* ctx) { static_cast<Xpilot*>(ctx)->serialConfigTask.run(); }

    static void printSchedulerRateTask(void* ctx) { static_cast<Xpilot*>(ctx)->printSchedulerRate(); }
    static void printIOTask(void* ctx) { static_cast<Xpilot*>(ctx)->printIO(); }
    static void printIMUTaskStatTask(void* ctx) { static_cast<Xpilot*>(ctx)->printIMUTaskStats(); }
    static void printRadioTaskStatTask(void* ctx) { static_cast<Xpilot*>(ctx)->printRadioTaskStats(); }
    static void printStateUpdateTaskStatTask(void* ctx) { static_cast<Xpilot*>(ctx)->printStateUpdateTaskStats(); }
    static void printFlightModeUpdateTaskStatTask(void* ctx) { static_cast<Xpilot*>(ctx)->printFlightModeUpdateTaskStats(); }
    static void printFlightModeRunTaskStatTask(void* ctx) { static_cast<Xpilot*>(ctx)->printFlightModeRunTaskStats(); }
    static void printFlightModeOutputTaskStatTask(void* ctx) { static_cast<Xpilot*>(ctx)->printFlightModeOutputTaskStats(); }

    // Debug functions to get outputs for testing and tuning purposes.
    void printSchedulerRate(void);
    void printIO(void);

    void printIMUTaskStats(void);
    void printRadioTaskStats(void);
    void printStateUpdateTaskStats(void);
    void printFlightModeUpdateTaskStats(void);
    void printFlightModeRunTaskStats(void);
    void printFlightModeOutputTaskStats(void);

    static bool isArmed() { return armState == ArmState::ARMED || armState == ArmState::WAITING_FOR_DISARM_RELEASE; }

    const Mode* getCurrentFlightMode(void) const { return currentMode; }
    bool inFailsafe(void) const { return sysFailsafeActive; }

private:
    void sysInit(void); // Initialize system components

    enum class ArmState : uint8_t
    {
        ARMED,
        WAITING_FOR_DISARM_RELEASE,
        DISARMED,
        WAITING_FOR_ARM_RELEASE
    };

    static ArmState armState;

    static bool armStateTimerStarted;

    static uint32_t armStateStartTime;

    static bool sysFailsafeActive; // System failsafe active flag

    bool armDisarmInput(void);

    void updateFlightMode(void);

    void updateArmState(void);

    void stateUpdate(void);

    RateMode rateMode;
    StabilizeMode stabilizeMode;
    PassthroughMode passthroughMode;

    // This is the state of the flight stabilization system
    Mode* currentMode;

    // Task handlers for the scheduler to manage periodic tasks
    static uint8_t imuTaskId;
    static uint8_t radioTaskId;
    static uint8_t stateUpdateTaskId;
    static uint8_t flightModeUpdateTaskId;
    static uint8_t flightModeRunTaskId;
    static uint8_t flightModeOutputTaskId;
    static uint8_t ledNotifierTaskId;

    SerialConfigTask serialConfigTask;
};

extern Xpilot xpilot;
#endif // _XPILOT_H