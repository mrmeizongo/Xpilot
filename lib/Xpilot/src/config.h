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

// Input pins
#define AILPIN_INPUT 2
#define ELEVPIN_INPUT 3
#define RUDDPIN_INPUT 4
#define MODEPIN_INPUT 5
// ------------------------------------------------------------------------------------------------------

// Output pins
#define AILPIN1_OUTPUT 8
#define AILPIN2_OUTPUT 9
#define ELEVPIN_OUTPUT 10
#define RUDDPIN_OUTPUT 11
// ------------------------------------------------------------------------------------------------------

/*
 * ISR vectors
 * All input pins use pin change interrupts
 * Change these values to match your selected input pins
 * If you modify these values you will also have to modify the settings in the PinChangeInterrupt library
 * A cleaner config method will probaby be useful here
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
 * |     7 |    XTAL2 (PB7)* |h
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
// ------------------------------------------------------------------------------------------------------

/*
 * Servo PWM range
 * Edit this to suit your servo PWN values
 * You can find these values in the datasheet that comes with your servo
 * These values are conservative and should work for most servos
 */
#define SERVO_MIN_PWM 1000
#define SERVO_MID_PWM 1500
#define SERVO_MAX_PWM 2000
// ------------------------------------------------------------------------------------------------------

// PID gain values
// Roll
#define ROLL_KP 11.4f
#define ROLL_KI 3.9f
#define ROLL_KD 0.0f
// Pitch
#define PITCH_KP 9.0f
#define PITCH_KI 2.2f
#define PITCH_KD 0.0f
// Yaw
#define YAW_KP 10.4f
#define YAW_KI 0.0f
#define YAW_KD 0.0f
// ------------------------------------------------------------------------------------------------------

/*
 * Trim values to be used to correct gyro misalignment
 * You might want to modify these to suit your airplane
 */
#define IMU_ROLL_TRIM -1.51f
#define IMU_PITCH_TRIM 2.12f
#define IMU_YAW_TRIM 0.0f
// ------------------------------------------------------------------------------------------------------

// If your transmitter has drift, be sure to modify these values
#define ROLL_INPUT_DEADBAND 20
#define PITCH_INPUT_DEADBAND 20
#define YAW_INPUT_DEADBAND 20
// ------------------------------------------------------------------------------------------------------

// Radio resolution values
#define PASSTHROUGH_RES 1000

// Stick resolution (degrees)
#define MAX_ROLL_RATE_DEGS 60
#define MAX_PITCH_RATE_DEGS 60
#define MAX_YAW_RATE_DEGS 60

// Max angles allowed in stabilize mode (angles)
#define MAX_ROLL_ANGLE_DEGS 60
#define MAX_PITCH_ANGLE_DEGS 60

// PID output limits
#define MAX_PID_OUTPUT 1000
// ------------------------------------------------------------------------------------------------------

/*
 * Transmitter / Receiver values
 * Used to separate the 3 flight modes input threshold
 */
#define INPUT_THRESHOLD 200
// ------------------------------------------------------------------------------------------------------

// Comment or uncomment to reverse stabilization output direction
// Reverses stabilization output in stabilize mode
#define REVERSE_ROLL_STABILIZE
#define REVERSE_PITCH_STABILIZE
// #define REVERSE_YAW_STABILIZE

// Reverses gyro output in rate mode
#define REVERSE_ROLL_GYRO
// #define REVERSE_PITCH_GYRO
#define REVERSE_YAW_GYRO

// Mix rudder for turn coordinations
// Rudder mixing value is set in percentage ( value / 100)
// #define RUDDER_MIX_IN_PASS
#define RUDDER_MIXING 0.30f
// ------------------------------------------------------------------------------------------------------

/*
 * MANDATORY
 * Airplane type
 * Uncomment the type of airplane being flown
 * Only tested with a full house plane
 * Proceed with caution
 * FULL_PLANE: Has ailerons(1 or 2 channel), elevator and rudder.
 * FULL_PLANE_V_TAIL: Has ailerons(1 or 2 channel) and v tail deflectors. Left V tail deflector goes to elevator, right goes to rudder. Swap if not
 * RUDDER_ELEVATOR_ONLY: Only rudder and elevator. They go to their respective channels.
 * FLYING_WING_RUDDER: Has 2 ailerons and a rudder. The aileron output is mixed with elevator output(elevon). Has rudder control
 * FLYING_WING_NO_RUDDER: Only has 2 ailerons. The aileron output is mixed with elevator output(elevon).
 * RUDDER_ELEVATOR_ONLY_V_TAIL: No ailerons, left V tail deflector goes to elevator, right goes to rudder. Swap if not
 */
#define FULL_PLANE
// #define FULL_PLANE_V_TAIL
// #define RUDDER_ELEVATOR_ONLY
// #define FLYING_WING_RUDDER
// #define FLYING_WING_NO_RUDDER
// #defined RUDDER_ELEVATOR_ONLY_V_TAIL

#if defined(FULL_PLANE) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_RUDDER)
#if !defined(AILPIN_INT) || !defined(ELEVPIN_INT) || !defined(RUDDPIN_INT)
#error Aileron, Elevator and Rudder interrupt pins need to be defined!
#endif
#endif

#if defined(RUDDER_ELEVATOR_ONLY) || defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
#if !defined(ELEVPIN_INT) || !defined(RUDDPIN_INT)
#error Elevator and Rudder interrupt pins need to be defined!
#endif
#endif

#if defined(FLYING_WING_NO_RUDDER)
#if !defined(AILPIN_INT) || !defined(ELEVPIN_INT)
#error Aileron and Elevator interrupt pins need to be defined
#endif
#endif
// ------------------------------------------------------------------------------------------------------

/*
 * Uncomment to enable the respective debugging, zero otherwise
 * It is wise to enable only one debug at a time to avoid chaos on serial bus
 */
// #define LOOP_DEBUG
// #define IMU_DEBUG
// #define IO_DEBUG
// #define MIXING_DEBUG
// #define CALIBRATE_DEBUG
// #define CALIBRATE
// ------------------------------------------------------------------------------------------------------
#endif