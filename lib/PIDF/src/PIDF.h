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

#ifndef _PIDF_H
#define _PIDF_H
#include <inttypes.h>
#include "Filter.h"

class PIDF
{
public:
    PIDF();                                  // Empty Constructor
    PIDF(float, float, float, float, float); // Constructor with initialization parameters
    void Reset(void);                        // Reset PIDF integrator and derivative
    int16_t Compute(float, float);           // Generate the PIDF output to be added to the servo

    float getKp(void) { return Kp; }
    float getKi(void) { return Ki; }
    float getKd(void) { return Kd; }
    float getKf(void) { return Kf; }
    float getIMax(void) { return IMax; }

    void setKp(float _Kp) { Kp = _Kp; }
    void setKi(float _Ki) { Ki = _Ki; }
    void setKd(float _Kd) { Kd = _Kd; }
    void setKf(float _Kf) { Kf = _Kf; }
    void setIMax(float _IMax) { IMax = _IMax; }

private:
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float IMax;

    // Second orderow pass filter for derivative term
    Filter derivative_lpf;

    float integrator = 0;
    float previousError = 0;
    unsigned long previousTime = 0;
};
#endif //_PIDF_H