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

#pragma once

// Pin interrupt connections
// Aileron and elevator inputs use hardware interrupts
// Mode switch uses pin change interrupts
#define AILPIN_INT 2
#define ELEVPIN_INT 3
#define MODEPIN_INT 4
// #define THROTTLEPIN_INT 5
// #define RUDDPIN_INT 6

#define AILPIN_OUT 9
#define ELEVPIN_OUT 10
// #define RUDDPIN_OUT 11

#define INPUT_THRESHOLD 200
#define RECEIVER_LOW 1000
#define RECEIVER_MID 1500
#define RECEIVER_HIGH 2000

// In degree radians
#define ROLL_LIMIT 45
#define PITCH_LIMIT 45

// In angles
#define AILERON_DEFLECTION_LIM 45
#define ELEVATOR_DEFLECTION_LIM 45

#define CENTER_DEFLECTION_POS 90

// Forbid copying of object
#define CLASS_NO_COPY(c)        \
    c(const c &other) = delete; \
    c &operator=(const c &) = delete;

// Set to 1 to enable the respective debugging, zero otherwise
// To enable any of the XX_DEBUG, set DEBUG to 1 first
#define DEBUG 0
#define LOOP_DEBUG 0
#define IO_DEBUG 0