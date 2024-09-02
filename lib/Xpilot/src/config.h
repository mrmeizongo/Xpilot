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
#ifndef _CONFIG_H
#define _CONFIG_H

/*
 * MANDATORY
 * Airplane type
 * Uncomment only one to select the type of airplane being flown
 * Directions are airplane-wise i.e. looking down the nose
 * FULL_PLANE: Has ailerons(1 or 2 channel), elevator and rudder.
 * FULL_PLANE_V_TAIL: Has ailerons(1 or 2 channel) and v tail deflectors. Left V-tail deflector goes to elevator ouput, right goes to rudder output.
 * FLYING_WING_W_RUDDER: Has 2 independent ailerons and a rudder. The aileron I/O is mixed with elevator I/O(elevon). Has rudder control
 * FLYING_WING_NO_RUDDER: Only has 2 independent ailerons. The aileron I/O is mixed with elevator I/O(elevon).
 * RUDDER_ELEVATOR_ONLY_PLANE: Only rudder and elevator. They go to their respective channels.
 * RUDDER_ELEVATOR_ONLY_V_TAIL: No ailerons. Left V-tail deflector goes to elevator ouput, right goes to rudder output.
 */
#define FULL_PLANE
// #define FULL_PLANE_V_TAIL
// #define RUDDER_ELEVATOR_ONLY_PLANE
// #define FLYING_WING_W_RUDDER
// #define FLYING_WING_NO_RUDDER
// #defined RUDDER_ELEVATOR_ONLY_V_TAIL

// ATmega328p pin definitions

/*
 * ISR vectors
 * All input pins use pin change interrupts
 * Depending on airplane type selected, input interrupt pins must be defined
 * Change these values to match your selected input pins
 * Best to keep these unchanged unless absolutely necessary
 * Changing any XXXXPIN_INT or XXXXPIN_INPUT value require modifications made to PinChangeInterrupt library
 *
 * | PCINT |  Uno/Nano/Mini  |
 * | ----- | --------------- |
 * |     0 |  8       (PB0)  |
 * |     1 |  9       (PB1)  |
 * |     2 | 10 SS    (PB2)  |
 * |     3 | 11 MISO  (PB3)  |
 * |     4 | 12 MOSI  (PB4)  |
 * |     5 | 13 SCK   (PB5)  |
 * |     6 |    XTAL1 (PB6)* |
 * |     7 |    XTAL2 (PB7)* |
 * | ----- | --------------- |
 * |     8 | A0       (PC0)  |
 * |     9 | A1       (PC1)  |
 * |    10 | A2       (PC2)  |
 * |    11 | A3       (PC3)  |
 * |    12 | A4 SDA   (PC4)  |
 * |    13 | A5 SDC   (PC5)  |
 * |    14 |    RST   (PC6)* |
 * |    15 |                 |
 * | ----- | --------------- |
 * |    16 |  0 RX    (PD0)  |
 * |    17 |  1 TX    (PD1)  |
 * |    18 |  2 INT0  (PD2)  |
 * |    19 |  3 INT1  (PD3)  |
 * |    20 |  4       (PD4)  |
 * |    21 |  5       (PD5)  |
 * |    22 |  6       (PD6)  |
 * |    23 |  7       (PD7)  |
 * | ----- | --------------- |
 */
#define AILPIN_INT 18
#define ELEVPIN_INT 19
#define RUDDPIN_INT 20
#define MODEPIN_INT 21

// Input pins
#define AILPIN_INPUT 2
#define ELEVPIN_INPUT 3
#define RUDDPIN_INPUT 4
#define MODEPIN_INPUT 5

// Output pins
#define AILPIN1_OUTPUT 8
#define AILPIN2_OUTPUT 9
#define ELEVPIN_OUTPUT 10
#define RUDDPIN_OUTPUT 11

// Max 12 servos
#define MAX_SERVO_CHANNELS 4
// ------------------------------------------------------------------------------------------------------

/*
 * Servo PWM range in micro seconds
 * Edit these to suit your servo PWN values
 * You can find these values in your servo data sheet
 * These values are conservative and should work for most servos
 */
#define SERVO_MIN_PWM 1000
#define SERVO_MID_PWM 1500
#define SERVO_MAX_PWM 2000

/*
 * Transmitter PWM range in micro seconds
 * All tx and rx are not designed the same
 * Default values obtained using SPEKTRUM SPMR6750 DX6 US VERSION and AR620 tx/rx combo
 * Refer to your tx/rx's manual to determine the incoming PWM range
 * If it is not specified, uncomment IO_DEBUG to view input/output data in serial monitor
 * Switch to PASSTHROUGH mode for this. Move stick through low and high points
 * The input range should be between the defined (-PASSTHROUGH_RES) to (PASSTHROUGH_RES)
 * The output range should be between the defined (SERVO_MIN_PWM) to (SERVO_MAX_PWM)
 * Adjust these values accordingly if there is a discrepancy
 */
#define INPUT_MIN_PWM 1100
#define INPUT_MID_PWM 1500
#define INPUT_MAX_PWM 1900
// ------------------------------------------------------------------------------------------------------

/*
 * Transmitter / Receiver values
 * Used to separate the 3 flight modes input threshold
 */
#define INPUT_THRESHOLD 200
// ------------------------------------------------------------------------------------------------------

// PID output limits
#define MAX_PID_OUTPUT 1000

// PID gain values

// Roll
#define ROLL_KP 14.2f
#define ROLL_KI 11.0f
#define ROLL_KD 0.02f
#define ROLL_I_WINDUP_MAX ((MAX_PID_OUTPUT) * 0.3f) // 10-30% of maximum control output is a good starting point
// Pitch
#define PITCH_KP 5.1f
#define PITCH_KI 1.3f
#define PITCH_KD 0.01f
#define PITCH_I_WINDUP_MAX ((MAX_PID_OUTPUT) * 0.3f) // 10-30% of maximum control output is a good starting point
// Yaw
#define YAW_KP 10.0f
#define YAW_KI 1.6f
#define YAW_KD 0.0f
#define YAW_I_WINDUP_MAX ((MAX_PID_OUTPUT) * 0.3f) // 10-30% of maximum control output is a good starting point
// ------------------------------------------------------------------------------------------------------

/*
 * Trim values to be used to correct gyro misalignment
 * Modify these values to suit your airplane if not calibrated properly
 */

#define IMU_ROLL_TRIM 0.f
#define IMU_PITCH_TRIM 0.f
#define IMU_YAW_TRIM 0.f
// ------------------------------------------------------------------------------------------------------

// To correct transmitter stick drift, be sure to modify these values
#define ROLL_INPUT_DEADBAND 20
#define PITCH_INPUT_DEADBAND 20
#define YAW_INPUT_DEADBAND 20
// ------------------------------------------------------------------------------------------------------

// Radio resolution values

// Stick resolution in passthrough mode
// This assumes a centered servo is exactly at the midpoint of MAX and MIN
// Change if not
#define PASSTHROUGH_RES ((SERVO_MAX_PWM) - (SERVO_MIN_PWM)) / 2

// Stick resolution in rate mode (degrees)
#define MAX_ROLL_RATE_DEGS 60
#define MAX_PITCH_RATE_DEGS 60
#define MAX_YAW_RATE_DEGS 50

// Max angles allowed in stabilize mode (angles)
#define MAX_ROLL_ANGLE_DEGS 45
#define MAX_PITCH_ANGLE_DEGS 30
// ------------------------------------------------------------------------------------------------------

// Uncomment or comment to set and unset respectively
// This depends on the mount direction of the MPU6050 on the circuit board
// Based on the aircraft coordinate system (Right-Hand, X-Forward, Z-Down)
// + on right roll, - on left roll
// - on pitch up, + on down pitch down
// + on right yaw, - on left yaw

// Uncomment to reverse stabilization output
#define REVERSE_ROLL_STABILIZE
// #define REVERSE_PITCH_STABILIZE
// #define REVERSE_YAW_STABILIZE

// Uncomment to reverse gyro output
#define REVERSE_ROLL_GYRO
#define REVERSE_PITCH_GYRO
#define REVERSE_YAW_GYRO

/*
 * Mix rudder to enable turn coordinations
 * Rudder mixing value is set in percentage ( value / 100)
 */
// #define RUDDER_MIX_IN_PASS
#define RUDDER_MIX_IN_RATE
#define RUDDER_MIX_IN_STABILIZE

// #define REVERSE_RUDDER_MIX   // Uncomment to reverse rudder mixing input if surfaces don't move in the expected direction
#define RUDDER_MIXING 0.15f // Amount of aileron input to mix with rudder to coordinate turns (10-20% is recommended)
// ------------------------------------------------------------------------------------------------------

/*
 * Uncomment to enable the respective debugging
 * It is wise to enable only one debug at a time to avoid chaos on serial bus
 * Use 9600 baudrate
 */
// #define LOOP_DEBUG
// #define IMU_DEBUG
// #define IO_DEBUG
// #define CALIBRATE_DEBUG
// #define CALIBRATE
// #define SELF_TEST_ACCEL_GYRO
// ------------------------------------------------------------------------------------------------------
#endif