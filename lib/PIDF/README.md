MIT License

## Copyright (c) 2024 Jamal Meizongo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

This is a simple PIDF controller I implemented for use in my RC plane flight stabilization software.  
A PIDF controller is an extension of the traditional PID (Proportional-Integral-Derivative) controller, with an added feed forward component. Here's a breakdown of the gains in a PIDF controller:

## Proportional Gain (P):

This gain determines the reaction to the current error value. A higher proportional gain results in a larger correction based on the current error. However, if the gain is too high, it can lead to system instability and oscillations.

## Integral Gain (I):

This gain accounts for the accumulation of past errors. It helps eliminate residual steady-state errors that the proportional gain alone cannot correct. The integral gain sums up the past errors and applies a correction based on the cumulative error. If the integral gain is too high, it can cause the system to become sluggish and oscillate.

## Derivative Gain (D):

This gain predicts future error based on the rate of change of the error. It helps to dampen the system response and reduce overshoot. A higher derivative gain can improve system stability, but if it's too high, it can amplify noise and cause instability.

## Feedforward (F):

The Feedforward (F) term helps to anticipate the control action required to achieve the desired setpoint, improving the system's response. The PID components act on the error between the setpoint and desired point whereas the F acts on the setpoint itself.  
The feedforward term helps to improve the system's response by providing an immediate control action based on the desired setpoint, reducing the lag and improving overall performance. It is very useful when the setpoint changes rapidly like in a RC vehicle.

# DISCLAIMER:

Do not expect this software to out perform other more established PIDF control softwares. This code shall be considered as highly experimental and is not designed or written to any safety critical, or mission critical standards. It is given/shared for free with the knowledge and understanding that this open source PIDF control software is only for personal or hobby use. It is intended to be used or modified to suit your needs. The author(s) shall not be held responsible or accountable for any damage, injury or loss that may be inflicted or incurred as a result of the use of misuse of this code. Use and modify at your own risk.

By using this, or any part of this software, you are agreeing to [this license agreement.](LICENSE)
