// Started - 03/13/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
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

#ifndef _XPILOT_H
#define _XPILOT_H
#include <PlaneConfig.h>
#include "Scheduler.h"
#include "Mode.h"

class Xpilot
{
public:
    Xpilot(void) {}
    Xpilot(const Xpilot &) = delete;            // Prevent this class from being copyable
    Xpilot &operator=(const Xpilot &) = delete; // Prevent this class from being assignable
    // --------------------------------------------------------------------

    // Trampoline functions for the scheduler
    static void updateFlightModeTask(void *ctx) { static_cast<Xpilot *>(ctx)->updateFlightMode(); }
    static void printIMUTask(void *ctx) { static_cast<Xpilot *>(ctx)->printIMU(); }
    static void printIOTask(void *ctx) { static_cast<Xpilot *>(ctx)->printIO(); }
    static void printSchedulerRateTask(void *ctx) { static_cast<Xpilot *>(ctx)->printSchedulerRate(); }
    static void printIMUTaskStatTask(void *ctx) { static_cast<Xpilot *>(ctx)->printIMUTaskStats(); }
    static void printRadioTaskStatTask(void *ctx) { static_cast<Xpilot *>(ctx)->printRadioTaskStats(); }
    static void printFlightModeRunTaskStatTask(void *ctx) { static_cast<Xpilot *>(ctx)->printFlightModeRunTaskStats(); }
    static void printFlightModeUpdateTaskStatTask(void *ctx) { static_cast<Xpilot *>(ctx)->printFlightModeUpdateTaskStats(); }
    static void printServoTaskStatTask(void *ctx) { static_cast<Xpilot *>(ctx)->printActuatorTaskStats(); }
    // --------------------------------------------------------------------

    // Only functions called from the main setup and loop functions
    void setup(void);
    void loop(void);
    // --------------------------------------------------------------------

    // Debug functions to get outputs for testing and tuning purposes.
    void printIMU(void);
    void printIO(void);
    void printSchedulerRate(void);
    void printIMUTaskStats(void);
    void printRadioTaskStats(void);
    void printFlightModeRunTaskStats(void);
    void printFlightModeUpdateTaskStats(void);
    void printActuatorTaskStats(void);
    // --------------------------------------------------------------------

    Mode *getFlightMode(void) const { return currentMode; }
    bool inFailsafe(void) const { return failSafeActive; }

private:
    RateMode rateMode;
    StabilizeMode stabilizeMode;
    PassthroughMode passthroughMode;

    // This is the state of the flight stabilization system
    Mode *currentMode;
    Mode *previousMode;
    // --------------------------------------------------------------------

    Scheduler scheduler; // Scheduler object to manage periodic tasks
    // --------------------------------------------------------------------

    void sysInit(void); // Initialize system components
    // --------------------------------------------------------------------

    bool failSafeActive; // System failsafe active flag
    bool imuFaultActive; // Imu fault
    void updateFlightMode(void);
    // --------------------------------------------------------------------

    // Task handlers for the scheduler to manage periodic tasks
    static uint8_t imuTaskId;
    static uint8_t radioTaskId;
    static uint8_t flightModeUpdateTaskId;
    static uint8_t flightModeRunTaskId;
    static uint8_t actuatorTaskId;
    // --------------------------------------------------------------------
};

extern Xpilot xpilot;
#endif // _XPILOT_H