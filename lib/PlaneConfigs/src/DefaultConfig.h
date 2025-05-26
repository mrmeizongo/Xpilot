// 11/19/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
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
#ifndef _DEFAULT_CONFIG_H
#define _DEFAULT_CONFIG_H

/*
 * MANDATORY
 * Airplane type
 * Uncomment only one to select the type of airplane being flown
 * Directions are airplane-wise i.e. looking down the nose
 * FULL_PLANE_TRADITIONAL: Has ailerons(1 or 2 channel), elevator and rudder.
 * FULL_PLANE_V_TAIL: Has ailerons(1 or 2 channel) and v tail deflectors. Left V-tail deflector goes to elevator ouput, right goes to rudder output.
 * FLYING_WING_W_RUDDER: Has 2 independent ailerons and a rudder. The aileron I/O is mixed with elevator I/O(elevon). Has rudder control
 * FLYING_WING_NO_RUDDER: Only has 2 independent ailerons. The aileron I/O is mixed with elevator I/O(elevon).
 * RUDDER_ELEVATOR_ONLY_PLANE: Only rudder and elevator. They go to their respective channels.
 * RUDDER_ELEVATOR_ONLY_V_TAIL: No ailerons. Left V-tail deflector goes to elevator ouput, right goes to rudder output.
 */
#define FULL_PLANE_TRADITIONAL
// #define FULL_PLANE_V_TAIL
// #define RUDDER_ELEVATOR_ONLY_PLANE
// #define FLYING_WING_W_RUDDER
// #define FLYING_WING_NO_RUDDER
// #defined RUDDER_ELEVATOR_ONLY_V_TAIL
// ------------------------------------------------------------------------------------------------------

// Servo config

/*
 * Servo PWM range in micro seconds
 * Refer to your servo datasheet to determine the control PWM range
 * These values are conservative and should work for most servos
 */
#define SERVO_MIN_PWM 1000
#define SERVO_MID_PWM 1500
#define SERVO_MAX_PWM 2000
// ------------------------------------------------------------------------------------------------------

// Transmitter config

/*
 * Transmitter PWM range in micro seconds
 * Refer to your transmitter manual to determine the incoming PWM range
 * These values are conservative and should work for most transmitters
 */
#define INPUT_MIN_PWM 1000
#define INPUT_MID_PWM 1500
#define INPUT_MAX_PWM 2000

/*
 * 3-position digital switch PWM separator
 * Used to separate the 3 flight modes input threshold
 */
#define INPUT_SEPARATOR 200

// To correct transmitter stick drift
#define ROLL_INPUT_DEADBAND 20
#define PITCH_INPUT_DEADBAND 20
#define YAW_INPUT_DEADBAND 20

// Stick resolution in passthrough mode (unitless)
#define MAX_PASS_THROUGH 1000

// Stick resolution in rate mode (degrees)
#define MAX_ROLL_RATE_DEGS 60
#define MAX_PITCH_RATE_DEGS 45
#define MAX_YAW_RATE_DEGS 30

// Max allowable angle in stabilize mode (angles)
#define MAX_ROLL_ANGLE_DEGS 60
#define MAX_PITCH_ANGLE_DEGS 45
// ------------------------------------------------------------------------------------------------------

// PID config

// PID output limits
#define MAX_PID_OUTPUT 1000

// PID gain values

// Roll
#define ROLL_KP 5.f
#define ROLL_KI 0.f
#define ROLL_KD 0.f
#define ROLL_KF 0.f
#define ROLL_I_WINDUP_MAX 300 // 10-30% of maximum control output is a good starting point

// Pitch
#define PITCH_KP 7.f
#define PITCH_KI 0.f
#define PITCH_KD 0.f
#define PITCH_KF 0.f
#define PITCH_I_WINDUP_MAX 300 // 10-30% of maximum control output is a good starting point

// Yaw
#define YAW_KP 9.f
#define YAW_KI 0.f
#define YAW_KD 0.f
#define YAW_KF 0.f
#define YAW_I_WINDUP_MAX 300 // 10-30% of maximum control output is a good starting point
// ------------------------------------------------------------------------------------------------------

// IMU config

/*
 * IMU ACCEL & GYRO biases obtained from running calibration function
 * Uncomment CALIBRATION_DEBUG to obtain these values
 */
#define ACC_X_BIAS 0.f
#define ACC_Y_BIAS 0.f
#define ACC_Z_BIAS 0.f
#define GYRO_X_BIAS 0.f
#define GYRO_Y_BIAS 0.f
#define GYRO_Z_BIAS 0.f

/*
 * Trim values to be used to correct gyro misalignment
 */

#define IMU_ROLL_TRIM 0.f
#define IMU_PITCH_TRIM 0.f
#define IMU_YAW_TRIM 0.f

// Uncomment or comment to set and unset respectively
// This depends on the mount direction of the MPU6050 on the circuit board
// Based on the aircraft coordinate system (Right-Hand, X-Forward, Z-Down)
// + on right roll(right wing down, left wing up), - on left roll(right wing up, left wing down)
// + on pitch up(nose up, tail down), - on down pitch down(nose down, tail up)
// + on right yaw(nose right, tail left), - on left yaw(nose left, tail right)

// Uncomment to reverse stabilization output
// #define REVERSE_ROLL_STABILIZE
// #define REVERSE_PITCH_STABILIZE
// #define REVERSE_YAW_STABILIZE    // Not required

// Uncomment to reverse gyro output
// #define REVERSE_ROLL_GYRO
// #define REVERSE_PITCH_GYRO
// #define REVERSE_YAW_GYRO
// ------------------------------------------------------------------------------------------------------

// Rudder config

/*
 * Mix rudder to enable automatic turn coordinations
 * Rudder mixing value is set in percentage ( value / 100)
 */
// #define RUDDER_MIX_IN_PASS
#define RUDDER_MIX_IN_RATE
#define RUDDER_MIX_IN_STABILIZE

// Uncomment to reverse rudder mixing input if rudder doesn't move in the expected direction
// #define REVERSE_RUDDER_MIX

// Amount of aileron input to be mixed with rudder to coordinate turns (1/4 - 1/3 of aileron input is recommended)
#define RUDDER_MIXING 0.3333f

// #define USE_HEADING_HOLD // Uncomment to enable heading-hold-like functionality when yaw centered
// ------------------------------------------------------------------------------------------------------

// Debug config

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
#endif // _DEFAULT_CONFIG_H