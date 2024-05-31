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

#include "PID.h"

PID::PID() {}

PID::PID(float _Kp, float _Ki, float _Kd)
{
    Initialize(_Kp, _Ki, _Kd);
}

// A call to an empty constructor must be followed by a call to the Initiailize() function to set the gain values
void PID::Initialize(float _Kp, float _Ki, float _Kd)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    ResetPID();
}

// Resets PID
void PID::ResetPID(void)
{
    previousError = 0;
    integral = 0;
}

// Main function to be called to get PID control value
float PID::Compute(float currentError, double interval)
{
    float proportional = currentError;
    integral = integral + (currentError * interval);
    double derivative = (currentError - previousError) / interval;
    previousError = currentError;
    return (float)((Kp * proportional) + (Ki * integral) + (Kd * derivative));
}