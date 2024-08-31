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
#include "PID.h"

PID::PID() {}

PID::PID(float _Kp, float _Ki, float _Kd, float _IMax)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    IMax = _IMax;
    ResetI();
}

// Resets PID
void PID::ResetI(void)
{
    integrator = 0;
    previousDerivative = NAN;
}

// Main function to be called to get PID control value
int16_t PID::Compute(float currentError)
{
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - previousTime;
    float output = 0.0f;
    float deltaTime = (float)dt * 0.001f;

    // if this PID hasn't been used for a full second then zero
    // the intergator term. This prevents I buildup from a
    // previous fight mode from causing a massive return before
    // the integrator gets a chance to correct itself
    if (previousTime == 0 || dt >= 1000)
    {
        dt = 0;
        ResetI();
    }

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
        float derivative;

        if (isnanf(previousDerivative))
        {
            // Reset called. Suppress first derivative term
            // as we don't want a sudden change in input to cause
            // a large D output change
            derivative = 0;
            previousDerivative = 0;
        }
        else
            derivative = (currentError - previousError) / deltaTime;

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        float RC = 1 / (2 * M_PI * fCut);
        derivative = previousDerivative +
                     ((deltaTime / (RC + deltaTime)) *
                      (derivative - previousDerivative));

        // Update state
        previousError = currentError;
        previousDerivative = derivative;

        // Add in derivative component
        output += derivative * Kd;
    }

    // Save last time Compute was run
    previousTime = currentTime;
    return static_cast<int16_t>(output);
}