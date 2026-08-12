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
 * Uncomment only one to select the type of airplane being flown.
 * If flaps will be used, they should be connected directly to the receiver.
 *
 * FULL_TRADITIONAL_PLANE: Has ailerons(1 or 2 channel), elevator and rudder.
 * FULL_V_TAIL_PLANE: Has ailerons(1 or 2 channel) and v tail deflectors. Left V-tail deflector goes to elevator ouput, right goes to rudder output.
 * RUDDER_ELEVATOR_ONLY_PLANE: Only rudder and elevator. They go to their respective channels.
 * RUDDER_ELEVATOR_ONLY_V_TAIL_PLANE: No ailerons. Left V-tail deflector goes to elevator ouput, right goes to rudder output.
 * AILERON_ELEVATOR_ONLY_PLANE: Only ailerons and elevator. No aileron-elevator mixing. Ailerons go to aileron channels, elevator goes to elevator channel.
 * FLYING_WING_W_RUDDER_PLANE: Has 2 independent ailerons and a rudder. The aileron I/O is mixed with elevator I/O(elevon). Has rudder control
 * FLYING_WING_NO_RUDDER_PLANE: Only has 2 independent ailerons. The aileron I/O is mixed with elevator I/O(elevon).
 */
#define FULL_TRADITIONAL_PLANE
// #define FULL_V_TAIL_PLANE
// #define RUDDER_ELEVATOR_ONLY_PLANE
// #define AILERON_ELEVATOR_ONLY_PLANE
// #define FLYING_WING_W_RUDDER_PLANE
// #define FLYING_WING_NO_RUDDER_PLANE
// ------------------------------------------------------------------------------------------------------

// Servo config

/*
 * Servo PWM range in micro seconds
 * Refer to your servo datasheet to determine the control PWM range
 * The most popular standard servo testers use 800us - 2200us as the control range
 * Values below 544us and above 2400us are ignored
 */
#define SERVO_MIN_PWM 800
#define SERVO_MID_PWM 1500
#define SERVO_MAX_PWM 2200
// ------------------------------------------------------------------------------------------------------

// Transmitter config

/*
 * Transmitter PWM range in micro seconds
 * Refer to your transmitter manual to determine the outgoing PWM range
 * These values are conservative and should work for most transmitters
 */
#define INPUT_MIN_PWM 1000
#define INPUT_MID_PWM 1500
#define INPUT_MAX_PWM 2000

/*
 * 3-position digital switch PWM separator
 * Used to separate the 3 flight modes input threshold
 * ((INPUT_MAX_PWM - INPUT_MIN_PWM) / 3)
 */
#define INPUT_SEPARATOR 333

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

// Plane config

// Uncomment to use flaperons
// Flaperons are ailerons that can be used as flaps
// #define USE_FLAPERONS

#if defined(USE_FLAPERONS)
#define FLAPERON_PC 1.0f // Percentage of flap deflection compared to aileron deflection(0.0f - 1.0f)
#define FLAPERON_MAX_RANGE (SERVO_MAX_PWM - SERVO_MID_PWM) * (FLAPERON_PC)
#endif

#define PT_SLEW_RATE 1000
#define AUTO_LPF_FREQ 10  // Auto low-pass filter frequency in Hz
#define PROCESS_DT 0.004f // Auto low-pass filter delta time; 4ms

// Uncomment to enable auxiliary output channel 1
// #define USE_AUXOUT1

// Uncomment to use the second auxiliary switch.
// If using aux3 input, provide implementation for functionality.
// aux3 input is tied to pin D7
// #define USE_AUX3
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
#define ROLL_I_WINDUP_MAX 300.f // 10-30% of maximum control output is a good starting point

// Pitch
#define PITCH_KP 7.f
#define PITCH_KI 0.f
#define PITCH_KD 0.f
#define PITCH_KF 0.f
#define PITCH_I_WINDUP_MAX 300.f // 10-30% of maximum control output is a good starting point

// Yaw
#define YAW_KP 9.f
#define YAW_KI 0.f
#define YAW_KD 0.f
#define YAW_KF 0.f
#define YAW_I_WINDUP_MAX 300.f // 10-30% of maximum control output is a good starting point
// ------------------------------------------------------------------------------------------------------

// IMU config

// Uncomment or comment to set and unset respectively
// This depends on the mount direction of the MPU6050 on the circuit board
// Based on the aircraft coordinate system (Right-Hand, X-Forward, Z-Down)
// + on right roll, - on left roll
// + on pitch up, - on down pitch down
// + on right yaw, - on left yaw

// Uncomment to reverse stabilization output
// #define REVERSE_ROLL
// #define REVERSE_PITCH
// #define REVERSE_YAW    // Not required

// Uncomment to reverse gyro output
// #define REVERSE_X_GYRO
// #define REVERSE_Y_GYRO
// #define REVERSE_Z_GYRO

#define MISSED_IMU_VAL_THRESH 250

// #define CALIBRATE    // Uncomment to calibrate the IMU on startup. Recommended for first time use. Place airplane on level surface and keep it still during calibration. Comment and reflash when done.
// #define SELF_TEST_ACCEL_GYRO // Uncomment to perform a self-test calibration of the accel&gyro on startup.
// ------------------------------------------------------------------------------------------------------

// Rudder config

/*
 * Mix rudder to enable automatic turn coordinations
 * Rudder mixing value is set in percentage ( value / 100)
 */
#define RUDDER_MIX_IN_RATE
#define RUDDER_MIX_IN_STABILIZE

// Uncomment to reverse rudder mixing input if rudder doesn't move in the expected direction
// #define REVERSE_RUDDER_MIX

// Amount of aileron input to be mixed with rudder to coordinate turns (1/4 - 1/3 of aileron input is recommended)
#define RUDDER_MIXING 0.3333f
// ------------------------------------------------------------------------------------------------------

// Flight mode config

/*
 * Uncomment to switch to the respective mode when failsafe is active
 * This is useful for switching to a more stable mode when failsafe is detected
 * It is recommended to switch to stabilize mode when failsafe is active
 */
// #define FAILSAFE_TO_PASSTHROUGH
// #define FAILSAFE_TO_RATE
#define FAILSAFE_TO_STABILIZE

// Uncomment to set the default flight mode when the system is powered on
// #define DEFAULT_TO_PASSTHROUGH_MODE
#define DEFAULT_TO_RATE_MODE
// #define DEFAULT_TO_STABILIZE_MODE
// ------------------------------------------------------------------------------------------------------

// Task scheduler control config
#define CONTROL_LOOP_RATE_HZ 250                     // Control loop period in hz
#define IMU_UPDATE_RATE_HZ CONTROL_LOOP_RATE_HZ      // IMU update period in hz
#define FLIGHT_MODE_RUN_RATE_HZ CONTROL_LOOP_RATE_HZ // Flight mode run period in hz
#define FLIGHT_MODE_UPDATE_RATE_HZ 25                // Flight mode update period in hz
#define RADIO_INPUT_PROCESS_RATE_HZ 50               // Radio input period in hz
#define WRITE_SERVO_RATE_HZ 50                       // Write servo output period in hz
#define IMU_PRINT_RATE_HZ 4                          // IMU debug print period in hz
#define TASK_PRINT_RATE_HZ 1                         // Task rate debug print period in hz
#define IO_PRINT_RATE_HZ 1                           // IO debug print period in hz
// ------------------------------------------------------------------------------------------------------

// Debug config

/*
 * Uncomment to enable the respective debugging
 * CAUTION:
 * Only uncomment one debug option at a time
 */
// #define SCHEDULER_RATE_DEBUG
// #define IMU_DEBUG
// #define IO_DEBUG
// #define CALIBRATE_DEBUG
// #define CALIBRATE
// #define READ_CALIBRATION_FROM_EEPROM
// #define SELF_TEST_ACCEL_GYRO
// #define PRINT_IMU_TASK_STAT
// #define PRINT_RADIO_TASK_STAT
// #define PRINT_FM_RUN_TASK_STAT
// #define PRINT_FM_UPDATE_TASK_STAT
// #define PRINT_SERVO_TASK_STAT

#if defined(SCHEDULER_RATE_DEBUG) || defined(IMU_DEBUG) || defined(IO_DEBUG) || defined(CALIBRATE_DEBUG) || defined(CALIBRATE) || defined(READ_CALIBRATION_FROM_EEPROM) || defined(SELF_TEST_ACCEL_GYRO) || defined(PRINT_IMU_TASK_STAT) || defined(PRINT_RADIO_TASK_STAT) || defined(PRINT_FM_RUN_TASK_STAT) || defined(PRINT_FM_UPDATE_TASK_STAT) || defined(PRINT_SERVO_TASK_STAT)
#define DEBUG
#endif
// ------------------------------------------------------------------------------------------------------
#endif // _DEFAULT_CONFIG_H