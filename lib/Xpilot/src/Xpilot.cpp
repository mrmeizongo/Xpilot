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
#include "Xpilot.h"
#include "ModeController.h"
#include "Radio.h"
#include "Actuators.h"
#include "IMU.h"
#include "config.h"

#define FIFTYHZ_LOOP 20U
#define ONEHZ_LOOP 1000U
#define I2C_CLOCK_400KHZ 400000U

// Timer variables
static unsigned long nowMs, outputLastMs = 0;
// -------------------------

// Debug helper functions
void printDebug(void);
void printIO(void);
// -------------------------

Xpilot::Xpilot(void)
{
}

void Xpilot::setup(void)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_400KHZ); // Setting I2C clock to 400Khz

#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG) || defined(SELF_TEST_ACCEL_GYRO)
    Serial.begin(9600);
    while (!Serial)
    {
        Serial.println("Serial port communication cannot be established.");
        delay(1000);
    }
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

    // Process servo output at 50Hz intervals
    if (nowMs - outputLastMs >= FIFTYHZ_LOOP)
    {
        modeController.processMode();
        actuators.writeServos();
        outputLastMs = nowMs;
    }

    printDebug();
}

// Debug helper functions
void printDebug(void)
{
#if defined(IO_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    static unsigned long debugLastMs = 0;
    if (nowMs - debugLastMs >= ONEHZ_LOOP)
    {
#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
        imu.printIMU();
#endif
#if defined(IO_DEBUG)
        printIO();
#endif
        debugLastMs = nowMs;
    }
#endif

#if defined(LOOP_DEBUG)
    unsigned long loopMillis = millis() - nowMs;
    loopMillis = loopMillis == 0 ? 1 : loopMillis; // To prevent division by 0
    Serial.print("Loop time");
    Serial.print("\t\t\t");
    Serial.println("Loop rate");
    Serial.print(loopMillis);
    Serial.print("ms");
    Serial.print("\t\t\t\t");
    Serial.print(1000 / loopMillis);
    Serial.println("Hz");
    Serial.println();
#endif
}

void printIO(void)
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
    Serial.println(actuators.getServoOut(Actuators::Control::AILERON1));

    Serial.print("Aileron 2: ");
    Serial.print(radio.getRxRoll());
    Serial.print("\t\t\t");
    Serial.print("Aileron 2: ");
    Serial.println(actuators.getServoOut(Actuators::Control::AILERON2));

    Serial.print("Elevator: ");
    Serial.print(radio.getRxPitch());
    Serial.print("\t\t\t");
    Serial.print("Elevator: ");
    Serial.println(actuators.getServoOut(Actuators::Control::ELEVATOR));

    Serial.print("Rudder: ");
    Serial.print(radio.getRxYaw());
    Serial.print("\t\t\t");
    Serial.print("Rudder: ");
    Serial.println(actuators.getServoOut(Actuators::Control::RUDDER));
    Serial.println();
}
// ---------------------------

Xpilot xpilot;