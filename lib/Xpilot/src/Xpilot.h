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

#pragma once

#include <MPU9250.h>
#include <Servo.h>
#include "config.h"

class Xpilot
{
public:
    Xpilot(void);

    // Prevent this class from being copied
    CLASS_NO_COPY(Xpilot);

    enum class FLIGHT_MODE : uint8_t
    {
        PASSTHROUGH = 1,
        FBW,
        STABILIZE
    };

    friend class Mode; // Flight mode controller

    void setup(void);
    void loop(void);

    void processIMU(void);
    void processInput(void);
    void processOutput(void);

    void get_imu_scaled(void);
    void mahoganyQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, double deltat);

    FLIGHT_MODE getCurrentMode() { return currentMode; }
    void setCurrentMode(FLIGHT_MODE _currentMode)
    {
#if DEBUG
        Serial.print("Flight mode: ");
        Serial.println((uint8_t)_currentMode);
#endif
        currentMode = _currentMode;
    }

#if IO_DEBUG
    void print_imu(void);
    void print_input(void);
    void print_output(void);
#endif

private:
    MPU9250 imu;

    /*
     * Aileron control variables
     */
    Servo aileronServo;             // Aileron servo channel
    uint8_t aileronInputPin;        // Input pin for aileron
    uint8_t aileronOutputPin;       // Output pin for aileron
    int16_t aileron_out = 0;        // Aileron servo output variable
    uint16_t aileronPulseWidth = 0; // Aileron values obtained from transmitter through interrupts

    /*
     *   Elevator control variables
     */
    Servo elevatorServo;             // Elevator servo channel
    uint8_t elevatorInputPin;        // Intput pin for elevator
    uint8_t elevatorOutputPin;       // Output pin for elevator
    int16_t elevator_out = 0;        // Elevator servo output variable
    uint16_t elevatorPulseWidth = 0; // Elevator values obtained from transmitter through interrupts

    /*
     *   Rudder control variables
     */
    Servo rudderServo;             // Rudder servo channel
    uint8_t rudderInputPin;        // Input pin for rudder
    uint8_t rudderOutputPin;       // Output pin for rudder
    int16_t rudder_out = 0;        // Rudder servo output variable
    uint16_t rudderPulseWidth = 0; // ElevatRudderor values obtained from transmitter through interrupts

    /*
     * Mode control variables
     */
    uint8_t modeInputPin;        // Input pin for mode
    uint16_t modePulseWidth = 0; // Mode values obtained from transmitter through interrupts

    FLIGHT_MODE currentMode = FLIGHT_MODE::PASSTHROUGH; // PASSTHROUGH is default mode

    // vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    // These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto 1.2
    double A_B[3]{539.75, 218.36, 834.53};
    double A_Ainv[3][3]{{0.51280, 0.00230, 0.00202},
                        {0.00230, 0.51348, -0.00126},
                        {0.00202, -0.00126, 0.50368}};

    // mag offsets and correction matrix
    double M_B[3]{18.15, 28.05, -36.09};
    double M_Ainv[3][3]{{0.68093, 0.00084, 0.00923},
                        {0.00084, 0.69281, 0.00103},
                        {0.00923, 0.00103, 0.64073}};

    double G_off[3] = {-299.7, 113.2, 202.4}; // raw offsets, determined for gyro at rest
    // ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // Raw data from IMU and scaled as vector
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    double Axyz[3];
    double Gxyz[3];
    double Mxyz[3];

    // Vector to hold quaternion values from AHRS algorithm
    double q[4] = {1.0, 0.0, 0.0, 0.0};

    // double ahrs_yaw;
    double ahrs_pitch, ahrs_roll, ahrs_yaw = 0; // Euler angle output
};

extern Xpilot xpilot;