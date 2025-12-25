// Started - 08/19/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
// Updated - 02/25/2025 by Jamal Meizongo
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
#include <PlaneConfig.h>

class Actuators
{
public:
    // Maintain consecutive flight control surface numbering starting from 0
    // Ensure NUM_CHANNELS is the last enum in the list and does not exceed MAX_SERVOS in Servo library
    // MAX_SERVOS for the Arduino Nano is 12 servos because it has 1 16 bit timer
    enum Channel : uint8_t
    {
        CH1 = 0U,
        CH2,
        CH3,
        CH4,
#if defined(USE_AUXOUT1)
        CH5, // Auxiliary output channel 1
#endif
#if defined(USE_AUXOUT2)
        CH6, // Auxiliary output channel 2
#endif
        NUM_CHANNELS
    };

    Actuators(void);
    void init(void);
    void writeServos(void);
    void writeServos(const int16_t (&SRVout)[NUM_CHANNELS]);
    void setServoOut(Channel, int16_t);
    void setServoOut(const int16_t (&SRVout)[NUM_CHANNELS]);
    int16_t getServoOut(Channel);

private:
    Servo controlServo[NUM_CHANNELS]; // Control servos
    int16_t channelOut[NUM_CHANNELS]; // Servo output values
};

extern Actuators actuators;

#endif //_ACTUATORS_H