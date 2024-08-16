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

#include "Xpilot.h"
#include "config.h"
#include "ModeController.h"
#include "Radio.h"
#include "IMU.h"

#define TWOHUNDREDHZ_LOOP 5U
#define FIFTYHZ_LOOP 20U
#define ONEHZ_LOOP 1000U
#define IMU_WARMUP_LOOP 1000U
#define I2C_CLOCK_1MHZ 1000000U

// Timer variables
unsigned long nowMs, outputLastMs, imuLastMs = 0;
// -------------------------

Xpilot::Xpilot(void)
{
}

void Xpilot::setup(void)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_1MHZ); // Overclocking I2C to 1Mhz

#if defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(IMU_DEBUG) || defined(SELF_TEST_ACCEL_GYRO)
    Serial.begin(9600);
    while (!Serial)
    {
        Serial.println("Serial port communication cannot be established.");
        delay(1000);
    }
#endif

    imu.init();   // Initialize IMU
    radio.init(); // Initialize radio

    // Set up output servos
    aileron1Servo.attach(AILPIN1_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    aileron2Servo.attach(AILPIN2_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    elevatorServo.attach(ELEVPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    rudderServo.attach(RUDDPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);

    // Warm up the IMU before initial use
    for (uint16_t i = 0; i < IMU_WARMUP_LOOP; i++)
    {
        imu.processIMU();
    }
}

/*
    Main xpilot execution loop
    Read input, read imu data, process output to servos
*/
void Xpilot::loop(void)
{
    nowMs = millis();

    // Process IMU at 200Hz intervals
    if (nowMs - imuLastMs >= TWOHUNDREDHZ_LOOP)
    {
        imu.processIMU();
        imuLastMs = nowMs;
    }

    // Process radio input and servo output at 50Hz intervals
    if (nowMs - outputLastMs >= FIFTYHZ_LOOP)
    {
        radio.processInput();
        processOutput();
        outputLastMs = nowMs;
    }

#if defined(IO_DEBUG) || defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
    static unsigned long debugLastMs = 0;
    if (nowMs - debugLastMs >= ONEHZ_LOOP)
    {
#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
        imu.print_imu();
#endif
#if defined(IO_DEBUG)
        print_IO();
#endif
        debugLastMs = nowMs;
    }
#endif

#if defined(LOOP_DEBUG)
    unsigned long loopMillis = millis() - nowMs;
    Serial.print("Loop time: ");
    Serial.print(loopMillis);
    Serial.println("ms");
    Serial.print("Loop rate: ");
    loopMillis = loopMillis <= 0 ? 1 : loopMillis; // To prevent division by 0
    Serial.print(1000 / loopMillis);
    Serial.println("Hz");
#endif
}

void Xpilot::processOutput(void)
{
    modeController.process();

    aileron1_out = constrain(aileron1_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    aileron2_out = constrain(aileron2_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    elevator_out = constrain(elevator_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    rudder_out = constrain(rudder_out, SERVO_MIN_PWM, SERVO_MAX_PWM);

    aileron1Servo.writeMicroseconds(aileron1_out);
    aileron2Servo.writeMicroseconds(aileron2_out);
    elevatorServo.writeMicroseconds(elevator_out);
    rudderServo.writeMicroseconds(rudder_out);
}

#if defined(IO_DEBUG)
void Xpilot::print_IO(void)
{
    Serial.print("\t\t");
    Serial.print("Flight Mode: ");
    Serial.println((int)radio.rx.mode);
    Serial.print("Input");
    Serial.print("\t\t\t\t");
    Serial.println("Output");

    Serial.print("Aileron 1: ");
    Serial.print(radio.rx.roll);
    Serial.print("\t\t\t");
    Serial.print("Aileron 1: ");
    Serial.println(aileron1_out);

    Serial.print("Aileron 2: ");
    Serial.print(radio.rx.roll);
    Serial.print("\t\t\t");
    Serial.print("Aileron 2: ");
    Serial.println(aileron2_out);

    Serial.print("Elevator: ");
    Serial.print(radio.rx.pitch);
    Serial.print("\t\t\t");
    Serial.print("Elevator: ");
    Serial.println(elevator_out);

    Serial.print("Rudder: ");
    Serial.print(radio.rx.yaw);
    Serial.print("\t\t\t");
    Serial.print("Rudder: ");
    Serial.println(rudder_out);
    Serial.println();
}
#endif
// ---------------------------

Xpilot xpilot;