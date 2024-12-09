// 03/13/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
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

#include <Arduino.h>
#include <PlaneConfig.h>
#include "Xpilot.h"
#include "ModeController.h"
#include "Radio.h"
#include "Actuators.h"
#include "IMU.h"

#define FIFTYHZ_LOOP 20U
#define ONEHZ_LOOP 1000U

// Timer variables
static unsigned long nowMs, outputLastMs = 0;
// -------------------------

// Debug helper functions
static void printDebug(void) __attribute__((unused));
static void printIO(void) __attribute__((unused));
static void printIMU(void) __attribute__((unused));
static void printLoopRate(void) __attribute__((unused));
// -------------------------

Xpilot::Xpilot(void)
{
}

void Xpilot::setup(void)
{
#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG) || defined(SELF_TEST_ACCEL_GYRO)
    Serial.begin(9600);
    while (!Serial)
        ; // Wait for Serial port to open
#endif

    imu.init();            // Initialize IMU
    radio.init();          // Initialize radio
    actuators.init();      // Initialize servos
    modeController.init(); // Initialize mode controller
}

/*
    Main Xpilot execution loop
    Read input, read imu data, process output to servos
*/
void Xpilot::loop(void)
{
    nowMs = millis();

    imu.processIMU();
    radio.processInput();

    // Process servo output at 50Hz
    if (nowMs - outputLastMs >= FIFTYHZ_LOOP)
    {
        modeController.processMode();
        actuators.writeServos();
        outputLastMs = nowMs;
    }

#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    printDebug();
#endif
}

// Debug helper functions
static void printDebug(void)
{
#if defined(IO_DEBUG)
    static unsigned long debugLastMs = 0;
    if (nowMs - debugLastMs >= ONEHZ_LOOP)
    {
        printIO();
        debugLastMs = nowMs;
    }
#endif

#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    printIMU();
#endif
#if defined(LOOP_DEBUG)
    printLoopRate();
#endif
}

static void printIO(void)
{
    Serial.print("\t\t");
    Serial.print("Flight Mode: ");
    Serial.println((uint8_t)radio.getRxCurrentMode());
    Serial.print("Input");
    Serial.print("\t\t\t\t");
    Serial.println("Output");

    Serial.print("Aileron 1: ");
    Serial.print(radio.getRxRoll());
    Serial.print("\t\t\t");
    Serial.print("Aileron 1: ");
    Serial.println(actuators.getServoOut(AILERON1));

    Serial.print("Aileron 2: ");
    Serial.print(radio.getRxRoll());
    Serial.print("\t\t\t");
    Serial.print("Aileron 2: ");
    Serial.println(actuators.getServoOut(AILERON2));

    Serial.print("Elevator: ");
    Serial.print(radio.getRxPitch());
    Serial.print("\t\t\t");
    Serial.print("Elevator: ");
    Serial.println(actuators.getServoOut(ELEVATOR));

    Serial.print("Rudder: ");
    Serial.print(radio.getRxYaw());
    Serial.print("\t\t\t");
    Serial.print("Rudder: ");
    Serial.println(actuators.getServoOut(RUDDER));
    Serial.println();
}

static void printIMU(void)
{
    Serial.print(">");
    Serial.print("Roll");
    Serial.print(imu.getRoll());
    Serial.print(", ");
    Serial.print("Pitch");
    Serial.print(imu.getPitch());
    Serial.print(", ");
    Serial.print("Yaw: ");
    Serial.print(imu.getYaw());
    Serial.println();
}

static void printLoopRate(void)
{
    unsigned long loopMillis = millis() - nowMs;
    loopMillis = loopMillis == 0 ? 1 : loopMillis; // To prevent division by 0
    Serial.print(">");
    Serial.print("Loop time");
    Serial.print(loopMillis);
    Serial.print(", ");
    Serial.print("Loop rate");
    Serial.print(1000 / loopMillis);
    Serial.println();
}
// ---------------------------