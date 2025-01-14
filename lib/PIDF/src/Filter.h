/* ============================================
Copyright (C) 2025 Jamal Meizongo
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
#ifndef FILTER_H
#define FILTER_H
#include <Arduino.h>

#define SQRT2 1.4142135623730950488f

class Filter
{
public:
    virtual float Process(float, float) = 0;
    virtual ~Filter() = default;
};

// Computationally less expensive than second order filter but less effective, has slower roll-off and limited flexibility
class FirstOrderLPF : public Filter
{
public:
    FirstOrderLPF(float cutoffFrequency)
    {
        rc = 1.0f / (2.0f * M_PI * cutoffFrequency);
        prevOutput = 0.0f;
    }

    // Filter input signal to remove unwanted high frequency noise
    float Process(float input, float samplingFrequency)
    {
        // Calculate alpha based on the cutoff frequency and sampling frequency
        prevOutput = prevOutput + ((samplingFrequency / (rc + samplingFrequency)) * (input - prevOutput));
        return prevOutput;
    }

private:
    float prevOutput; // Previous output value
    float rc;
};

class SecondOrderLPF : public Filter
{
public:
    SecondOrderLPF(float cutoffFrequency)
        : cutoffFrequency(cutoffFrequency), prevInput1(0.0f), prevInput2(0.0f), prevOutput1(0.0f), prevOutput2(0.0f) {}

    void CalculateCoEfficients(float samplingFrequency)
    {
        // Calculate normalized cutoff frequency
        float omega = 2.0f * M_PI * (cutoffFrequency * samplingFrequency);
        float sinOmega = sin(omega);
        float cosOmega = cos(omega);

        // Compute Butterworth coefficients
        float alpha = sinOmega / SQRT2;
        float scale = 1.0f / (1.0f + alpha);

        b0 = (1.0f - cosOmega) / (2.0f * scale);
        b1 = (1.0f - cosOmega) * scale;
        b2 = b0;
        a1 = -2.0f * cosOmega * scale;
        a2 = (1.0f - alpha) * scale;
    }

    // Filter input signal to remove unwanted high frequency noise
    float Process(float input, float samplingFrequency)
    {
        CalculateCoEfficients(samplingFrequency);
        float output = (b0 * input) + (b1 * prevInput1) + (b2 * prevInput2) - (a1 * prevOutput1) - (a2 * prevOutput2);

        // Update previous values
        prevInput2 = prevInput1;
        prevInput1 = input;
        prevOutput2 = prevOutput1;
        prevOutput1 = output;

        return output;
    }

private:
    float cutoffFrequency;
    float a1, a2, b0, b1, b2;                               // Filter coefficients
    float prevInput1, prevInput2, prevOutput1, prevOutput2; // Previous input and output values
};
#endif // FILTER_H