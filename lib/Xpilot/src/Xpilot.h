// 03/13/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
// This and other library code in this repository
// are partial releases and work is still in progress.
// Please keep this in mind as you use this piece of software.

/* ============================================
Flight stabilization software
    Copyright (C) 2024 Jamal Meizongo (mrmeizongo@outlook.com)

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

#ifndef _XPILOT_H
#define _XPILOT_H

class Xpilot
{
public:
    Xpilot(void);

    // Only functions called from the Arduino setup and loop functions
    void setup(void);
    void loop(void);

    void processOutput(void);

private:
    friend class ModeController; // Flight mode controller
    friend void printIO(void);   // Print servo values

    int16_t aileron1_out = 0; // Aileron servo output variable
    int16_t aileron2_out = 0; // Aileron servo output variable
    int16_t elevator_out = 0; // Elevator servo output variable
    int16_t rudder_out = 0;   // Rudder servo output variable
};

extern Xpilot xpilot;
#endif // _XPILOT_H