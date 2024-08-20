/* ============================================
Copyright (C) 2024 Jamal Meizongo
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

#ifndef _PID_H
#define _PID_H
#include <inttypes.h>

class PID
{
public:
    PID();                    // Empty Constructor
    PID(float, float, float); // Constructor with initialization parameters
    void ResetPID(void);      // Reset PID controller parameters
    int16_t Compute(float);   // Generate the PID output to be added to the servo

    float getKp(void) { return Kp; }
    float getKi(void) { return Ki; }
    float getKd(void) { return Kd; }

    void setKp(float _Kp) { Kp = _Kp; }
    void setKi(float _Ki) { Ki = _Ki; }
    void setKd(float _Kd) { Kd = _Kd; }

private:
    float Kp;
    float Kd;
    float Ki;

    float integral;
    float previousError;
    unsigned long previousTime;
};
#endif //_PID_H