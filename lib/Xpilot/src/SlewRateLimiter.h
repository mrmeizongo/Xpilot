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

#ifndef _SLEWRATE
#define _SLEWRATE

template <typename T, typename U> class SlewRateLimiter
{
public:
    SlewRateLimiter()
        : _output{U{}}
        , _ratePerSecond{1000}
        , _maxChangeRate{1}
    {
    }

    SlewRateLimiter(uint16_t ratePerSecond, float dt)
        : _output{U{}}
        , _ratePerSecond{ratePerSecond}
    {
        _maxChangeRate = ratePerSecond * dt;
    }

    SlewRateLimiter(SlewRateLimiter&&) = default;
    SlewRateLimiter& operator=(SlewRateLimiter&&) = default;

    U update(T target)
    {
        const T error = target - _output;

        if (error > _maxChangeRate)
            _output += _maxChangeRate;
        else if (error < -_maxChangeRate)
            _output -= _maxChangeRate;
        else
            _output = target;

        return _output;
    }

    void setRate(uint16_t newRatePerSecond)
    {
        float dt = static_cast<float>(_maxChangeRate) / _ratePerSecond;
        _maxChangeRate = newRatePerSecond * dt;
        _ratePerSecond = newRatePerSecond;

        reset();
    }

    void reset(U prevOutput = U{}) { _output = prevOutput; }

private:
    U _output;
    uint16_t _ratePerSecond;
    uint8_t _maxChangeRate; // Under reasonable conditions, 0 < maxChangeRate < 255
};
#endif // _SLEWRATE