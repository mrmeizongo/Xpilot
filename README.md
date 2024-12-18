![XPilot Logo](assets/img/logo.jpg)

# Xpilot

Flight stabilization system intended to run on the atmega328 chip in the Arduino Nano and UNO microcontrollers.
Copyright (C) 2024 Jamal Meizongo

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

### Developed using PlatformIO on vscode

Xpilot is a flight stabilization system based on the ATMEGA328P chip in the Arduino Nano, Uno and Mini microcontrollers and MPU6050.
It is capable of stabilizing various airplane types including the traditional airplane, V-tail and flying wing.
It also works for various configurations of these airplanes i.e. 1 channel AIL, ELEV, and RUDD(traditional tail and V-tail),
2 channel AIL, ELEV and RUDD(traditional tail, V-tail and flying wing) or ELEV and RUDD only(traditional tail and V-tail).
See [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h) for airplane type configuration.

## Stabilization system loop

The atmega328p chip is capable of running the entire stabilization loop in 3ms.
This gives us an update frequency of ~333Hz. Output to the control servos are updated at 50Hz for standard servos.
In the context of RC planes, an execution loop of 333Hz provides a smooth and responsive control, which is particularly beneficial for maintaining stability in changing flight conditions.

## Setup

Connect MPU6050 to Arduino Nano as shown below

| PIN | VALUE |
| :-: | :---: |
| VIN |  5v   |
| GND |  GND  |
| SCL |  A5   |
| SDA |  A4   |

Connect receiver to Arduino Nano as shown below. This can be changed in [BoardConfig.h](lib/BoardConfig/src/BoardConfig.h). However changing the input pin numbers will require some modification to the PinChangeInterrupt library.

|    CHANNEL    | PIN |
| :-----------: | :-: |
|    Aileron    |  2  |
|   Elevator    |  3  |
|    Rudder     |  4  |
| AUX1/2 - Mode |  5  |

Can use both aileron channel outputs to individual aileron servos or both aileron servos can be connected to one aileron channel output using a Y-cable extension. Flying wings require individual aileron channel control.
Connect ailerons, elevator and rudder servos to Arduino Nano as shown below. This can be changed in [BoardConfig.h](lib/BoardConfig/src/BoardConfig.h).

| CHANNEL  | PIN |
| :------: | :-: |
| Aileron1 |  8  |
| Aileron2 |  9  |
| Elevator | 10  |
|  Rudder  | 11  |

Set up a 3-position switch on the transmitter to act as the Mode switch.
When properly setup, mode switch states is shown below.

| AUX Switch Position |    Mode     |
| :-----------------: | :---------: |
|          0          | Passthrough |
|          1          |    Rate     |
|          2          |  Stabilize  |

After setup, enable IO_DEBUG in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h) to verify proper operation.

DO NOT power the servos using the 5v power output from the Arduino Nano as this might harm the microcontroller.
However, the Nano, MPU6050 and servos can be powered from the same external 5VDC power source(ESC BEC). Make use of a 1-female/2-male Y cable splitter to power the Xpilot board and receiver.
It is a good idea to add a 0.47uF decoupling capacitors close to the individual servos for a stable power supply.

These pin numbers with the exception of MPU6050 can be reconfigured in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h). However, changing the pins for the channel inputs to Xpilot will require modification of the PinChangeInterrupt library.
Ensure all components share a common ground. The Nano and MPU6050 do not require decoupling capacitors as the breakout boards come with their own voltage regulators and decoupling capacitors.

![Schematics](assets/img/Schematics.png)

## Flight modes

There are 3 flight modes; 1 = passthrough/manual, 2 = rate, and 3 - stabilize.

Rate mode is the most popular among inexperienced flyers and is also the default mode of operation if mode switch is not configured. Passthrough is for advanced flyers. Rudder mixing for coordinated turns is enabled automatically in rate and stabilize modes and off by default in passthrough mode. You can override this and/or set roll % to be mixed with rudder in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h).

|      Flight mode       |                                     Description                                     |
| :--------------------: | :---------------------------------------------------------------------------------: |
| Manual/Passthrough - 1 |                 Manual flight control surface movement, passthrough                 |
|        Rate - 2        |                               Gyro based rate control                               |
|     Stabilize - 3      | Surfaces follow stick movement up-to set limits with wing-leveling on stick release |

## Airplane Selection

See [this](lib/PlaneConfigs/README.md) for instructions on how to set up configuration files for multi airplane use.

## NOTICE

Throttle is always under manual control. Signal wire for throttle goes directly to ESC for motor control.

Rate/Expo should NOT be used for Rate(2)/Stabilize(3) flight modes. You can however the mode switch on your transmitter to enable Rate/Expo in passthrough(1) flight mode.

Even though a calibration function is provided(recommended), the MPU6050 does not really need to be calibrated as long as it passes the self test function. Uncomment SELF_TEST_ACCEL_GYRO in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h) to enable. Be sure to comment it when done. Uncomment IMU_DEBUG in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h) and place the plane on a level surface to view the reported(roll, pitch, and yaw) sensor values. Adjust IMU_XXX_TRIM(pitch and roll) values to bring reported values to as close to zero as possible. This might require several attempts. Aim for a +/- .5 value when plane is placed on a level surface and held still.

A calibration function is also provided. Uncommenting CALIBRATE runs the calibration function and stores the X, Y, and Z accel/gyro biases in volatile memory. This will have to be performed on every startup. Ensure the airplane is held level and still throughout the calibration process.

(RECOMMENDED)You can uncomment CALIBRATE_DEBUG which runs the calibration function, stores the biases in volatile memory, and prints the biases in the serial monitor. You should record these values and assign them to the IMU ACCEL & GYRO bias definitions in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h). Ensure the airplane is held level and still throughout the calibration process.

After all debug operations, be sure to uncomment and reupload Xpilot for normal operation.

## Preflight

Be sure to go through the entirety of [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h) and perform any required modifications and preflight checks before flight.  
May your landings be beautiful! ❤️

Pull requests are welcome. Please try to adhere to the coding style in the project. I will review and approve them as time and opportunity permits.

## Donate

If you like this project and want to support me create more open source projects like this, please consider donating to my PayPal.  
[![Please Donate](assets/img/paypal-donate-button.png)](https://www.paypal.com/donate/?business=G7TZRNVYLUCHW&no_recurring=0&item_name=Thank+you+for+supporting+the+Xpilot+Flight+Stabilization+project.+May+your+landings+be+beautiful.+%E2%9D%A4%EF%B8%8F&currency_code=USD)

## Disclaimer:

Do not expect this software to out perform other more established flight controller projects such as ArduPilot, inav, betaFlight etc. This code shall be considered as highly experimental and is not designed or written to any safety critical, or mission critical standards. It is given/shared for free with the knowledge and understanding that this open source flight controller software is only for small hobby based electrically powered model aircraft, or other small hobby radio controlled vehicles. It is intended to be used or modified to suit your needs for small models and is NOT to be used on any manned vehicles. The author(s) shall not be held responsible or accountable for any damage, injury or loss that may be inflicted or incurred as a result of the use or misuse of this code. Use and modify at your own risk and use within accordance of your country's laws and/or regulations.

By using this, or any part of this software you agree to [this license agreement.](https://github.com/mrmeizongo/Xpilot/blob/main/LICENSE)

To put it more bluntly:

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

## Credits

NicoHood - [PinChangeInterrupt library](https://github.com/NicoHood/PinChangeInterrupt)
