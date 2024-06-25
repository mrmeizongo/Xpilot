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

// Input pins
// ------------------------------------------------------------------------------------------------------
#define AILPIN_INPUT 2
#define ELEVPIN_INPUT 3
#define RUDDPIN_INPUT 4
#define MODEPIN_INPUT 5
// ------------------------------------------------------------------------------------------------------

// Output pins
// ------------------------------------------------------------------------------------------------------
#define AILPIN_OUTPUT 9
#define ELEVPIN_OUTPUT 10
#define RUDDPIN_OUTPUT 11
// ------------------------------------------------------------------------------------------------------

// ISR vectors
// All input pins use pin change interrupts
// Change these values to match your selected input pins
// if you modify these values you will also have to modify the settings in the PinChangeInterrupt library
// A cleaner config will probaby be useful here
// ------------------------------------------------------------------------------------------------------
#define AILPIN_INT 18
#define ELEVPIN_INT 19
#define RUDDPIN_INT 20
#define MODEPIN_INT 21
// ------------------------------------------------------------------------------------------------------

// In degree radians
// ------------------------------------------------------------------------------------------------------
#define ROLL_LIMIT 45
#define PITCH_LIMIT 45
// ------------------------------------------------------------------------------------------------------

// PID GAINs
// ------------------------------------------------------------------------------------------------------
// Roll
#define ROLL_KP 7.0f
#define ROLL_KI 0.01f
#define ROLL_KD 0.001f
// Pitch
#define PITCH_KP 8.0f
#define PITCH_KI 0.03f
#define PITCH_KD 0.001f
// Yaw
#define YAW_KP 6.0f
#define YAW_KI 0.01f
#define YAW_KD 0.001f
// ------------------------------------------------------------------------------------------------------

// Servo PWM range
// Edit this to suit your servo PWN values
// You can find these values in the datasheet that comes with your servo
// These values are conservative and should work for most servos
// ------------------------------------------------------------------------------------------------------
#define SERVO_MIN_PWM 1000
#define SERVO_MAX_PWM 2000
#define SERVO_MID_PWM 1500
// ------------------------------------------------------------------------------------------------------

// Transmitter/Receiver values
// ------------------------------------------------------------------------------------------------------
#define INPUT_THRESHOLD 200 // Used in to separate the 3 flight modes input threshold
// ------------------------------------------------------------------------------------------------------

// Set to 1 to enable the respective debugging, zero otherwise
// To enable any of the XX_DEBUG, set DEBUG to 1 first
// It is wise to enable only one debug at a time (i.e. LOOP_DEBUG or IO_DEBUG) due to the atmega328p memory constraints
// ------------------------------------------------------------------------------------------------------
#define DEBUG 0
#define LOOP_DEBUG 0
#define IO_DEBUG 0
// ------------------------------------------------------------------------------------------------------