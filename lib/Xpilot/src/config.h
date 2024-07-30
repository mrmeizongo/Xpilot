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
#define AILPIN_OUTPUT 9
#define ELEVPIN_OUTPUT 10
#define RUDDPIN_OUTPUT 11
// ------------------------------------------------------------------------------------------------------

/*
 * ISR vectors
 * All input pins use pin change interrupts
 * Change these values to match your selected input pins
 * If you modify these values you will also have to modify the settings in the PinChangeInterrupt library
 * A cleaner config method will probaby be useful here
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
#define ROLL_KP 12.0f
#define ROLL_KI 2.0f
#define ROLL_KD 0.2f
// Pitch
#define PITCH_KP 12.0f
#define PITCH_KI 2.0f
#define PITCH_KD 0.2f
// Yaw
#define YAW_KP 12.0f
#define YAW_KI 2.0f
#define YAW_KD 0.2f
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
// Stick resolution (degrees)
#define MAX_ROLL_RATE_DEGS 70
#define MAX_PITCH_RATE_DEGS 70
#define MAX_YAW_RATE_DEGS 70

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

// Set any of these to either 1 or 0 to reverse stabilization output direction
// Reverses stabilization output in stabilize mode
#define REVERSE_ROLL_STABILIZE 1
#define REVERSE_PITCH_STABILIZE 1
#define REVERSE_YAW_STABILIZE 0

// Reverses gyro output in rate mode
#define REVERSE_ROLL_GYRO 1
#define REVERSE_PITCH_GYRO 0
#define REVERSE_YAW_GYRO 1
// ------------------------------------------------------------------------------------------------------

// Utility
#define CALIBRATE 0
// ------------------------------------------------------------------------------------------------------

/*
 * Set to 1 to enable the respective debugging, zero otherwise
 * To enable any of the XX_DEBUG, set DEBUG to 1 first
 * It is wise to enable only one debug at a time (i.e. LOOP_DEBUG or IO_DEBUG) due to the atmega328p memory constraints
 */
#define DEBUG 0
#define LOOP_DEBUG 0
#define IO_DEBUG 0
// ------------------------------------------------------------------------------------------------------
#endif