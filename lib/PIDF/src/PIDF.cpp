/* ============================================

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
#include <Arduino.h>
#include "PIDF.h"

PIDF::PIDF() {}

PIDF::PIDF(float _Kp, float _Ki, float _Kd, float _Kf, float _IMax)
    : Kp{_Kp}, Ki{_Ki}, Kd{_Kd}, Kf{_Kf}, IMax{_IMax}, derivative_lpf(20.0f)
{
}

// Resets PIDF
void PIDF::Reset(void)
{
    integrator = 0;
    derivative_lpf.Reset();
}

// Main function to be called to get PIDF control value
int16_t PIDF::Compute(float setPoint, float currentPoint)
{
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - previousTime;
    float currentError = setPoint - currentPoint;
    float output = 0.0f;
    float deltaTime;

    /*
     * If this PIDF hasn't been used for a full second then reset the PIDF.
     * This prevents I buildup from a previous fight mode from causing a
     * massive return before the integrator gets a chance to correct itself
     */
    if (previousTime == 0 || dt > 1000)
    {
        dt = 0;
        Reset();
    }

    deltaTime = (float)dt * 0.001f;
    // Save last time Compute was run
    previousTime = currentTime;
    // Compute proportional component
    output += currentError * Kp;

    // Compute integral component if time has elapsed
    if ((fabsf(Ki) > 0) && (dt > 0))
    {
        integrator += (currentError * Ki) * deltaTime;
        // Limit integrator wind up
        if (integrator < -IMax)
        {
            integrator = -IMax;
        }
        else if (integrator > IMax)
        {
            integrator = IMax;
        }
        output += integrator;
    }

    // Compute derivative component if time has elapsed
    if ((fabsf(Kd) > 0) && (dt > 0))
    {
        float derivative = (currentError - previousError) / deltaTime;
        derivative = derivative_lpf.Process(derivative, deltaTime); // process low pass filter

        // Update state
        previousError = currentError;
        // Add in derivative component
        output += derivative * Kd;
    }

    // Compute feedforward component if time has elapsed
    if ((fabsf(Kf) > 0) && (dt > 0))
        output += setPoint * Kf;

    return static_cast<int16_t>(output);
}