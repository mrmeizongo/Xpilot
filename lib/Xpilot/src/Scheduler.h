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
#ifndef _SCHEDULER_H
#define _SCHEDULER_H

#include <Arduino.h>

class Scheduler
{
public:
    using TaskCallback = void (*)(void*);

    /**
     * This is an arbitrary limit.
     * The more tasks you add, the more memory is used and the longer it takes to run all tasks.
     */
    static constexpr uint8_t MAX_TASKS = 8;
    static constexpr int8_t INVALID_TASK_ID = -1;

    struct TaskStats
    {
        uint32_t runCount = 0;
        uint32_t missedPeriods = 0;
        uint32_t overrunCount = 0;
        uint32_t lastRuntimeUs = 0;
        uint32_t maxRuntimeUs = 0;
        uint32_t lastLoopRateUpdateUs = 0;
        uint16_t loopRateHz = 0;
        uint16_t loopCounter = 0;
    };

    Scheduler(void)
        : lastTask_{INVALID_TASK_ID}
    {
    }

    /**
     * Configures Timer2 to generate a 1 ms scheduler tick.
     *
     * Timer2 is reserved by the scheduler after this call.
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
    int8_t addTask(TaskCallback callback, void* context, uint16_t frequencyHz, uint16_t startDelayMs = 0);

    /**
     * Executes all tasks that are currently due.
     */
    void runTasks(void);

    bool isEnabled(int8_t taskId) const; // Returns true if task is enabled

    bool disableTask(int8_t taskId); // Returns true if operation is successful

    bool removeTask(int8_t taskId); // Returns true if operation is successful, does not compact/shift task array

    void removeAllTasks(void); // Clean slate

    bool getStats(int8_t taskId, TaskStats& stats) const; // Returns true if operation is successful

    bool resetStats(int8_t taskId); // Returns true if operation is successful

    /**
     * Returns milliseconds elapsed since init().
     */
    static uint32_t ticks(void);

    /**
     * Called by the Timer2 compare-match ISR.
     */
    static void onTimerCompareISR();

private:
    struct Task
    {
        TaskCallback callback = nullptr;
        void* context = nullptr;

        uint32_t nextRunTick = 0;
        uint16_t frequencyHz = 0;
        uint32_t periodMs = 0;

        bool occupied = false;
        bool enabled = false;

        TaskStats stats{};
    };

    Task tasks_[MAX_TASKS];
    int8_t lastTask_;

    static volatile uint32_t tickCount;

    static bool deadlineReached(uint32_t currentTick, uint32_t deadlineTick);

    bool isValidTask(int8_t taskId) const;
};

extern Scheduler scheduler;

#endif // _SCHEDULER_H