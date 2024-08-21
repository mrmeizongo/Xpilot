// 08/19/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
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
#ifndef _ACTUATORS_H
#define _ACTUATORS_H
#include <Servo.h>
#include <inttypes.h>
#include "config.h"

#if MAX_SERVO_CHANNELS > 12
#error "Too many servos installed. Max 12 servos per permitted."
#endif

// Maintain consecutive surface numbering starting from 0
enum Channel : uint8_t
{
    AILERON1 = 0U,
    AILERON2,
    ELEVATOR,
    RUDDER,
    CHANNEL_COUNT // If adding more control surfaces, ensure this is always at the bottom
};

class Actuators
{
public:
    Actuators(void);
    void init(void);
    void writeServos(void);
    void setServoOut(const int16_t (&SRVout)[MAX_SERVO_CHANNELS]);
    int16_t getServoOut(Channel);

private:
    Servo controlServo[MAX_SERVO_CHANNELS]; // Control servos
    int16_t channelOut[MAX_SERVO_CHANNELS]; // Servo output values
};

extern Actuators actuators;

#endif //_ACTUATORS_H