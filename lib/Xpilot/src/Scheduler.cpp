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

#include "Scheduler.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

volatile uint32_t Scheduler::tickCount = 0;

namespace
{
    constexpr uint32_t TIMER2_PRESCALER = 64UL;
    constexpr uint32_t SCHEDULER_TICK_HZ = 1000UL;

    constexpr uint32_t TIMER2_COMPARE_VALUE =
        (F_CPU / TIMER2_PRESCALER / SCHEDULER_TICK_HZ) - 1UL;

    static_assert(
        (TIMER2_COMPARE_VALUE >= 1UL) && (TIMER2_COMPARE_VALUE <= 255UL),
        "Timer2 compare value range 1 <= TIMER2_COMPARE_VALUE <= 255");
}

Scheduler::Scheduler()
{
    for (uint8_t i = 0; i < MAX_TASKS; ++i)
    {
        tasks_[i].callback = nullptr;
        tasks_[i].context = nullptr;
        tasks_[i].nextRunTick = 0;
        tasks_[i].frequencyHz = 0;
        tasks_[i].periodMs = 0;
        tasks_[i].occupied = false;
        tasks_[i].enabled = false;

        tasks_[i].stats.runCount = 0;
        tasks_[i].stats.missedPeriods = 0;
        tasks_[i].stats.overrunCount = 0;
        tasks_[i].stats.lastRuntimeUs = 0;
        tasks_[i].stats.maxRuntimeUs = 0;
        tasks_[i].stats.lastLoopRateUpdateUs = 0;
        tasks_[i].stats.loopRateHz = 0;
        tasks_[i].stats.loopCounter = 0;
    }
    numTasks_ = 0;
}

void Scheduler::init()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        tickCount = 0;

        // Stop Timer2 while configuring it.
        TCCR2A = 0;
        TCCR2B = 0;

        // Reset the counter.
        TCNT2 = 0;

        // Generate a compare match every 1 ms.
        OCR2A = static_cast<uint8_t>(TIMER2_COMPARE_VALUE);

        // CTC mode: clear timer when TCNT2 reaches OCR2A.
        TCCR2A |= _BV(WGM21);

        // Timer2 prescaler = 64.
        //
        // CS22 CS21 CS20
        //   1    0    0   => divide clock by 64
        TCCR2B |= _BV(CS22);

        // Clear any pending compare-match interrupt flag.
        TIFR2 = _BV(OCF2A);

        // Enable Timer2 compare-match A interrupt.
        TIMSK2 |= _BV(OCIE2A);
    }
}

int8_t Scheduler::addTask(
    TaskCallback callback,
    void *context,
    uint16_t frequencyHz,
    uint16_t startDelayMs)
{
    if (callback == nullptr || frequencyHz == 0 || frequencyHz > 1000)
    {
        return INVALID_TASK_ID;
    }

    const uint32_t periodMs =
        static_cast<uint32_t>(
            (1000UL + (frequencyHz / 3UL)) / frequencyHz);

    if (periodMs == 0)
    {
        return INVALID_TASK_ID;
    }

    for (uint8_t i = 0; i < MAX_TASKS; ++i)
    {
        Task &task = tasks_[i];

        if (!task.occupied)
        {
            task.callback = callback;
            task.context = context;
            task.frequencyHz = frequencyHz;
            task.periodMs = periodMs;
            task.occupied = true;
            task.enabled = true;

            task.stats.runCount = 0;
            task.stats.missedPeriods = 0;
            task.stats.overrunCount = 0;
            task.stats.lastRuntimeUs = 0;
            task.stats.maxRuntimeUs = 0;

            // startDelayMs == 0, means run after one period, otherwise run after startDelayMs
            // This allows system initialization to complete before the first task execution.
            const uint32_t initialDelayMs = (startDelayMs == 0)
                                                ? static_cast<uint32_t>(periodMs)
                                                : static_cast<uint32_t>(startDelayMs);

            task.nextRunTick = ticks() + initialDelayMs;
            numTasks_++;

            return static_cast<int8_t>(i);
        }
    }

    return INVALID_TASK_ID;
}

void Scheduler::runTasks()
{
    for (uint8_t i = 0; i < numTasks_; ++i)
    {
        Task &task = tasks_[i];

        if (!task.enabled ||
            task.callback == nullptr)
        {
            continue;
        }

        const uint32_t currentTick = ticks();

        // Skip tasks not due to run.
        if (!deadlineReached(currentTick, task.nextRunTick))
        {
            continue;
        }

        /*
         * Determine how many complete periods elapsed beyond the
         * scheduled deadline.
         *
         * The scheduler executes the task once and skips obsolete
         * invocations rather than repeatedly running the task to catch up.
         */
        const uint32_t latenessMs =
            currentTick - task.nextRunTick;

        const uint32_t missedPeriods =
            latenessMs / task.periodMs;

        /*
         * Advance from the previous scheduled deadline rather than from
         * the current time. This prevents normal execution jitter from
         * accumulating into long-term schedule drift.
         */
        task.nextRunTick +=
            (missedPeriods + 1UL) * task.periodMs;

        task.stats.missedPeriods += missedPeriods;

        const uint32_t startTimeUs = micros();

        task.callback(task.context);

        const uint32_t currentTimeUs = micros();

        const uint32_t taskPeriodTakenUs = currentTimeUs - startTimeUs;

        task.stats.runCount++;
        task.stats.loopCounter++;

        if ((currentTimeUs - task.stats.lastLoopRateUpdateUs) >= 1000000UL)
        {
            task.stats.loopRateHz = task.stats.loopCounter;
            task.stats.loopCounter = 0;
            task.stats.lastLoopRateUpdateUs = currentTimeUs;
        }
        task.stats.lastRuntimeUs = taskPeriodTakenUs;

        if (taskPeriodTakenUs > task.stats.maxRuntimeUs)
        {
            task.stats.maxRuntimeUs = taskPeriodTakenUs;
        }

        // Convert time task took to run to milliseconds and compare it to the alloted time
        const uint32_t availableTimeUs =
            static_cast<uint32_t>(task.periodMs) * 1000UL;

        if (taskPeriodTakenUs > availableTimeUs)
        {
            task.stats.overrunCount++;
        }
    }
}

bool Scheduler::isEnabled(int8_t taskId) const
{
    if (!isValidTask(taskId))
    {
        return false;
    }

    return tasks_[taskId].enabled;
}

bool Scheduler::getStats(
    int8_t taskId,
    TaskStats &stats) const
{
    if (!isValidTask(taskId))
    {
        return false;
    }

    stats = tasks_[taskId].stats;

    return true;
}

bool Scheduler::resetStats(int8_t taskId)
{
    if (!isValidTask(taskId))
    {
        return false;
    }

    TaskStats &stats = tasks_[taskId].stats;

    stats.runCount = 0;
    stats.missedPeriods = 0;
    stats.overrunCount = 0;
    stats.lastRuntimeUs = 0;
    stats.maxRuntimeUs = 0;

    return true;
}

uint32_t Scheduler::ticks()
{
    uint32_t tickSnapshot;

    /*
     * tickCount_ is 32 bits, but the ATmega328P is an 8-bit processor.
     * The atomic block prevents the ISR from modifying tickCount_ while
     * it is being copied.
     */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        tickSnapshot = tickCount;
    }

    return tickSnapshot;
}

void Scheduler::onTimerCompareISR()
{
    ++tickCount;
}

bool Scheduler::deadlineReached(
    uint32_t currentTick,
    uint32_t deadlineTick)
{
    /*
     * Signed subtraction allows correct comparisons across uint32_t
     * timer rollover, provided deadlines are less than 2^31 ms apart.
     */
    return static_cast<int32_t>(
               currentTick - deadlineTick) >= 0;
}

bool Scheduler::isValidTask(int8_t taskId) const
{
    if (taskId < 0 || taskId >= numTasks_)
    {
        return false;
    }

    return tasks_[taskId].enabled;
}

ISR(TIMER2_COMPA_vect)
{
    Scheduler::onTimerCompareISR();
}