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
#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H
#include "Filter.h"

enum class FilterType : uint8_t
{
    FIRST_ORDER = 0U,
    SECOND_ORDER
};

class LowPassFilter
{
public:
    // Use default filter of first order if no filter type is specified
    LowPassFilter(float cutoffFrequency, FilterType _filterType = FilterType::FIRST_ORDER)
    {
        filterType = _filterType;
        switch (filterType)
        {
        case FilterType::FIRST_ORDER:
            lpf = new FirstOrderLPF(cutoffFrequency);
            break;
        case FilterType::SECOND_ORDER:
            lpf = new SecondOrderLPF(cutoffFrequency);
            break;
        default:
            break;
        }
    }

    float Process(float input, float samplingFrequency)
    {
        return lpf->Process(input, samplingFrequency);
    }

    FilterType getFilterType() { return filterType; }

    ~LowPassFilter()
    {
        delete lpf;
    }

private:
    Filter *lpf;
    FilterType filterType;
};

#endif // LOWPASSFILTER_H