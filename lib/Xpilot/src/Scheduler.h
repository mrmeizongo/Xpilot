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
#ifndef _FLIGHT_SCHEDULER_H
#define _FLIGHT_SCHEDULER_H

#include <Arduino.h>

class Scheduler
{
public:
    using TaskCallback = void (*)(void *);

    /**
     * This is an arbitrary limit.
     * The more tasks you add, the more memory is used and the longer it takes to run all tasks.
     */
    static constexpr uint8_t MAX_TASKS = 8;
    static constexpr int8_t INVALID_TASK_ID = -1;

    struct TaskStats
    {
        uint32_t runCount;
        uint32_t missedPeriods;
        uint32_t overrunCount;
        uint32_t lastRuntimeUs;
        uint32_t maxRuntimeUs;
        uint32_t lastLoopRateUpdateUs;
        uint16_t loopRateHz;
        uint16_t loopCounter;
    };

    Scheduler(void);

    /**
     * Configures Timer2 to generate a 1 ms scheduler tick.
     *
     * Timer2 is reserved by the scheduler after this call.
     * This conflicts with Arduino tone() and any other library using Timer2.
     */
    void init(void);

    /**
     * @brief               Adds a periodic task.
     * @param callback      Function to execute.
     * @param context       Context pointer passed to the callback.
     * @param frequencyHz   Task frequency in Hertz.
     * @param startDelayMs  A value of 0 schedules the first run after one period.
     *
     * @return Task ID from 0 to MAX_TASKS - 1, or INVALID_TASK_ID on failure.
     */
    int8_t addTask(
        TaskCallback callback,
        void *context,
        uint16_t frequencyHz,
        uint16_t startDelayMs = 0);

    /**
     * Executes all tasks that are currently due.
     *
     * Call this continuously from Arduino loop().
     */
    void runTasks(void);

    bool isEnabled(int8_t taskId) const;

    bool getStats(int8_t taskId, TaskStats &stats) const;

    bool resetStats(int8_t taskId);

    /**
     * Returns milliseconds elapsed since begin().
     */
    static uint32_t ticks(void);

    /**
     * Called by the Timer2 compare-match ISR.
     *
     * Do not call this manually.
     */
    static void onTimerCompareISR();

private:
    struct Task
    {
        TaskCallback callback;
        void *context;

        uint32_t nextRunTick;
        uint16_t frequencyHz;
        uint32_t periodMs;

        bool occupied;
        bool enabled;

        TaskStats stats;
    };

    Task tasks_[MAX_TASKS];
    int8_t lastTask_;

    static volatile uint32_t tickCount;

    static bool deadlineReached(
        uint32_t currentTick,
        uint32_t deadlineTick);

    bool isValidTask(int8_t taskId) const;
};

#endif // _FLIGHT_SCHEDULER_H