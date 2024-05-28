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

// Pin interrupt connections
// Aileron and elevator inputs use hardware interrupts
// Mode switch uses pin change interrupts
#define AILPIN_INT 2
#define ELEVPIN_INT 3
#define RUDDPIN_INT 4
#define MODEPIN_INT 5
// #define THROTTLEPIN_INT 6

#define AILPIN_OUT 9
#define ELEVPIN_OUT 10
// #define RUDDPIN_OUT 11

#define INPUT_THRESHOLD 200
#define RECEIVER_LOW 1000
#define RECEIVER_MID 1500
#define RECEIVER_HIGH 2000

// In degree radians
#define ROLL_LIMIT 45
#define PITCH_LIMIT 45

// Servo
#define SERVO_MIN_PWM 1000
#define SERVO_MAX_PWM 2000
#define SERVO_MID_PWM 1500

// Forbid copying of object
#define CLASS_NO_COPY(c)        \
    c(const c &other) = delete; \
    c &operator=(const c &) = delete;

// Set to 1 to enable the respective debugging, zero otherwise
// To enable any of the XX_DEBUG, set DEBUG to 1 first
#define DEBUG 1
#define LOOP_DEBUG 0
#define IO_DEBUG 1