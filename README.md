<p align="center">
  <img src="assets/img/logo.jpg" />
</p>

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
It also works for various configurations of these airplanes i.e. 1 channel AIL, ELEV, and RUDD(traditional and V-tail),  
2 channel AIL, ELEV and RUDD(traditional tail, V-tail and flying wing), AIL and ELEV only(traditional and V-tail) or ELEV and RUDD only(traditional and V-tail).  
See [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h) for airplane type configuration.

## Stabilization system loop

The atmega328p chip is capable of running the entire stabilization loop in ~3.6ms.  
This gives us an update frequency of ~275Hz which is more than enough to provide a smooth and responsive control system for slow flying RC planes such as trainers, gliders and some mild acrobatic planes.  
Do not expect this to outperform the capabilities of more established flight control/stabilization softwares. Pull requests are welcome.

## Setup

Connect MPU6050 to Arduino Nano as shown below

| PIN | VALUE |
| :-: | :---: |
| VIN |  5v   |
| GND |  GND  |
| SCL |  A5   |
| SDA |  A4   |

Connect receiver to Arduino Nano as shown below. This can be changed in [SystemConfig.h](lib/SystemConfig/src/SystemConfig.h). However changing the input pin numbers will require some modification to the PinChangeInterrupt library.

|                                            CHANNEL                                            | PIN |
| :-------------------------------------------------------------------------------------------: | :-: |
|                                            Aileron                                            |  2  |
|                                           Elevator                                            |  3  |
|                                            Rudder                                             |  4  |
|                                          AUX1 - Mode                                          |  5  |
|   AUX2 - FLAPERON (if activated in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h))   |  6  |
| AUX3 - User defined (if activated in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h)) |  7  |

Can use both aileron channel outputs to individual aileron servos or both aileron servos can be connected to one aileron channel output using a Y-cable extension.  
If ailerons are wired independently, they can be used as flaperons by activating USE_FLAPERONS in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h)
Flying wings require individual aileron channel control.  
Connect ailerons, elevator and rudder servos to Arduino Nano as shown below. This can be changed in [SystemConfig.h](lib/SystemConfig/src/SystemConfig.h).

|  CHANNEL  | PIN |
| :-------: | :-: |
| Aileron1  |  8  |
| Aileron2  |  9  |
| Elevator  | 10  |
|  Rudder   | 11  |
| Auxiliary | 12  |

Set up a 3-position switch on the transmitter to act as the Mode switch.  
When properly setup, mode switch states is shown below.

| AUX Switch Position |    Mode     |
| :-----------------: | :---------: |
|          0          | Passthrough |
|          1          |    Rate     |
|          2          |  Stabilize  |

After setup, enable IO_DEBUG in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h) to verify proper operation.

## Info

DO NOT power the servos using the 5v power output from the Arduino Nano as this might harm the microcontroller.  
However, the Nano, MPU6050 and servos can be powered from the same external 5VDC high current power source(such as ESC BEC). Make use of a 1-female/2-male Y cable splitter to power the Xpilot board and receiver.  
It is a good idea to add a 0.47uF decoupling capacitors close to the individual servos for a stable power supply.

These pin numbers with the exception of MPU6050 can be reconfigured in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h). However, changing the pins for the channel inputs to Xpilot will require modification of the PinChangeInterrupt library.  
Ensure all components share a common ground. The Nano and MPU6050 do not require decoupling capacitors as the breakout boards come with their own voltage regulators and decoupling capacitors.

<p align="center">
  <img src="assets/img/Schematics.png" />
</p>

## Flight modes

There are 3 flight modes; 1 = passthrough/manual, 2 = rate, and 3 - stabilize.

Rate mode is the most popular among inexperienced flyers. If mode switch is not configured, any of the 3 modes can be used as the default mode. See [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h).  
Passthrough is for advanced flyers. Rudder mixing for coordinated turns is enabled automatically in rate and stabilize modes and off by default in passthrough mode. You can override this and/or set roll % to be mixed with rudder in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h). Default aileron-rudder mixing value is 30%.

|      Flight mode       |                                     Description                                     |
| :--------------------: | :---------------------------------------------------------------------------------: |
| Manual/Passthrough - 1 |                 Manual flight control surface movement, passthrough                 |
|        Rate - 2        |                               Gyro based rate control                               |
|     Stabilize - 3      | Surfaces follow stick movement up-to set limits with wing-leveling on stick release |

## Airplane Selection

See [README.md](lib/PlaneConfigs/README.md) for instructions on how to set up configuration files for multi airplane use.

## NOTICE

Throttle is always under manual control. Signal wire for throttle goes directly to ESC for motor control.

Rate/Expo should NOT be used for Rate(2)/Stabilize(3) flight modes. You can however configure Rate/Expo for passthrough(1) flight mode.

Two calibration functions are provided. Uncommenting CALIBRATE runs the calibration function and stores the X, Y, and Z accel/gyro biases in flash memory. Uncommenting CALIBRATE_DEBUG does same and prints out the calibration values in the serial monitor for inspection. Ensure the airplane is on a level surface and held still throughout the calibration process.

At any time, you can view the calibrated values on the serial monitor by uncommenting READ_CALIBRATION_FROM_EEPROM in [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h)

After all debug and calibration operations are complete, be sure to uncomment and reupload Xpilot for normal operation.

## Build & Upload

To build and upload the project to the microcontroller, download [vscode](https://code.visualstudio.com/download) if you don't already have it and install the PlatformIO extension. With the project folder open in vscode, run the _platformio.exe run --target upload_ command in the terminal or click on the _PlatformIO:Upload button_ in the bottom status bar.

## Preflight

Be sure to go through [DefaultConfig.h](lib/PlaneConfigs/src/DefaultConfig.h) and perform any required modifications and preflight checks before flight.

Pull requests are welcome. Please try to adhere to the coding style in the project. I will review and approve them as time and opportunity permits.

## Donate

If you like this project and want to support me create more open source projects like this, please consider donating to my PayPal.  
[![Please Donate](assets/img/paypal-donate-button.png)](https://www.paypal.com/donate/?business=G7TZRNVYLUCHW&no_recurring=0&item_name=Thank+you+for+supporting+the+Xpilot+Flight+Stabilization+project.+May+your+landings+be+beautiful.+%E2%9D%A4%EF%B8%8F&currency_code=USD)

## Disclaimer:

Do not expect this software to out perform other more established flight controller projects such as ArduPilot, inav, betaFlight etc. This code shall be considered as highly experimental and is not designed or written to any safety critical, or mission critical standards. It is given/shared for free with the knowledge and understanding that this open source flight stabilization software is only for small hobby based electrically powered model aircraft, or other small hobby radio controlled vehicles. It is intended to be used or modified to suit your needs for small models and is NOT to be used on any manned vehicles. The author(s) shall not be held responsible or accountable for any damage, injury or loss that may be inflicted or incurred as a result of the use or misuse of this code. Use and modify at your own risk and use within accordance of your country's laws and/or regulations.

By using this, or any part of this software you agree to [this license agreement.](https://github.com/mrmeizongo/Xpilot/blob/main/LICENSE)

## Credits

NicoHood - [PinChangeInterrupt library](https://github.com/NicoHood/PinChangeInterrupt)
