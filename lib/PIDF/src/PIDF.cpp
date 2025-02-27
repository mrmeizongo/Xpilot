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
    : Kp{_Kp}, Ki{_Ki}, Kd{_Kd}, Kf{_Kf}, IMax{_IMax}
{
    // 20Hz because anything over that is probably noise
    RC = 1.0f / (2.0f * M_PI * 20.0f);
    previousDerivative = NAN;
    integrator = 0;
    previousError = 0;
    previousTime = 0;
}

// Resets PIDF
void PIDF::Reset(void)
{
    integrator = 0;
    // Set previousDerivative as invalid on reset
    previousDerivative = NAN;
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
     * If this PIDF just started or hasn't been used for a full second then reset the PIDF.
     * If it hasn't been used for a full second, it prevents I buildup from a previous fight mode
     * from causing a massive return before the integrator gets a chance to correct itself
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
        float derivative = 0;
        if (isnanf(previousDerivative))
        {
            /*
             * Reset called.
             * Suppress first derivative term as we don't want a sudden change in input to cause a large D output change
             */
            derivative = 0;
            previousDerivative = 0;
        }
        else
            derivative = (currentError - previousError) / deltaTime;

        // Apply low pass filter to eliminate high frequency noise in the derivative term
        derivative = previousDerivative + ((deltaTime / (RC + deltaTime)) * (derivative - previousDerivative));

        // Update state
        previousError = currentError;
        previousDerivative = derivative;

        // Add in derivative component
        output += derivative * Kd;
    }

    // Compute feedforward component if time has elapsed
    if ((fabsf(Kf) > 0) && (dt > 0))
        output += setPoint * Kf;

    return static_cast<int16_t>(output);
}