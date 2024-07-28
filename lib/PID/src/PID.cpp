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

PID::PID(float _Kp, float _Ki, float _Kd)
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
    previousTime = millis();
}

// Main function to be called to get PID control value
int PID::Compute(int16_t currentError)
{
    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) * 0.001;
    integral += (currentError * dt);
    double derivative = (currentError - previousError) / dt;
    previousError = currentError;
    previousTime = currentTime;
    int16_t result = static_cast<int16_t>((Kp * currentError) + (Ki * integral) + (Kd * derivative));
    return result;
}