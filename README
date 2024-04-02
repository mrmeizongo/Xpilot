# Xpilot

### Developed using PlatformIO on vscode

Flight stabilization system intended to run on the atmega328 chip in the Arduino Nano and UNO microcontrollers.
Designed with a 3-channel system(AIL, ELEV, \*RUDD) in mind; some modifications are necessary to convert it for a 2-channel system.

RUDD is directly connected to receiver for now as stabilization input is only applied to AIL and ELEV channels for roll and pitch control.
Work is in progress to include RUDD control

During testing I used a GY-91 10DOF sensor for testing but an MPU9250 will work as it is the same 9DOF sensor on the GY-91.
I did not use the barometer/altitude sensor output for this program(pull requests are welcome).

I utilized the MP9250 and Mahogany AHRS library by Jeff Rowberg so a big thank you to Jeff. [MPU9250-AHRS](https://github.com/jremington/MPU-9250-AHRS)

## Setup

Connect GY-91 / MPU9250 to Arduino Nano as shown below

| PIN | VALUE |
| :-: | :---: |
| VIN |  5v   |
| GND |  GND  |
| SCL |  A5   |
| SDA |  A4   |

To get receiver input to the Arduino Nano, I used hardware interrupts(pins 2 & 3) for the aileron and elevator respectively, and pin change interrupts for the mode switch.
These pins numbers can be reconfigured in [xpilot_config.h](lib/Xpilot/src/xpilot_config.h)
