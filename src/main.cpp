// 03/13/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
// This and other library code in this repository
// are partial releases and work is still in progress.
// Please keep this in mind as you use this piece of software.

/* ============================================
Xpilot library code is placed under the MIT license
Copyright (c) 2024 by
Author: Jamal Meizongo (mrmeizongo@outlook.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <Arduino.h>
#include <Xpilot.h>

unsigned long now_ms, last_ms = 0;

void setup()
{
    xpilot.setup();
}

void loop()
{
    // Read input, read imu data, process output to servos
    // Process input at 50Hz, everything else runs at loop speed
    now_ms = millis();
    if (now_ms - last_ms >= 20)
    {
        xpilot.processInput();
        last_ms = now_ms;
    }

    if (xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::MANUAL)
        xpilot.processIMU();

    xpilot.processOutput();
}