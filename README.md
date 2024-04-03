# Xpilot

### Developed using PlatformIO on vscode

Flight stabilization system intended to run on the atmega328 chip in the Arduino Nano and UNO microcontrollers.
Designed with a 3-channel system(AIL, ELEV, \*RUDD) in mind; some modifications are necessary to convert it for a 2-channel system.

RUDD is directly connected to receiver for now as stabilization input is only applied to AIL and ELEV channels for roll and pitch control.
Work is in progress to include RUDD control

During testing I used a GY-91 10DOF sensor for testing but an MPU9250 will work as it is the same 9DOF sensor on the GY-91.
I did not utilize the barometer/altitude sensor output for this program(pull requests are welcome).

I utilized the MPU9250 and Mahogany AHRS library by Jeff Rowberg so a big thank you to Jeff. [MPU9250-AHRS](https://github.com/jremington/MPU-9250-AHRS)

## Setup

Connect GY-91/MPU9250 to Arduino Nano as shown below

| PIN | VALUE |
| :-: | :---: |
| VIN |  5v   |
| GND |  GND  |
| SCL |  A5   |
| SDA |  A4   |

Connect receiver to Arduino Nano as shown below

| CHANNEL  | PIN |
| :------: | :-: |
| Aileron  |  2  |
| Elevator |  3  |
|   Mode   |  4  |

Connect aileron and elevator servos to Arduino Nano as shown below. DO NOT power the servos using the 5v power output from the Arduino Nano as this might harm the microcontroller.
However, the Nano, GY-91 and servos can be powered from one 5VDC power source. It is also a good idea to make use of 0.47uF decoupling capacitors close to the individual servos for stability.

| CHANNEL  | PIN |
| :------: | :-: |
| Aileron  |  9  |
| Elevator | 10  |

These pin numbers with the exception of GY-91/MPU9250 can be reconfigured in [xpilot_config.h](lib/Xpilot/src/xpilot_config.h).
Ensure all components share a common ground. The Nano and GY-91/MPU9250 do not require decoupling capacitors as the breakout boards come with their own decoupling capacitors.

## Flight modes

There are 3 flight modes; modes 1 = manual/acro, 2 = fly-by-wire, and 3 - stabilize.

|   Flight mode   |                     Description                      |
| :-------------: | :--------------------------------------------------: |
| Manual/Acro - 1 | Manual flight control surface movement, passthrough  |
| Fly-by-wire - 2 |  Roll and pitch follow stick input up to set limits  |
|  Stabilize - 3  | Like fly-by-wire with wing-leveling on stick release |

Fly-by-wire mode is the most popular among inexperienced flyers. That is also the default mode of operation if mode switch has not been configured.
Note that there is currently no aileron and rudder mixing available to coordinate turns.
This is mainly due to a limited(2) number of hardware interrupt pins on the atmega328p chip. I'm considering using pin change interrupts for rudder channel input in a subsequent release.

Pull requests are welcome. Please try to adhere to the coding style in the project. I will review and approve them as time and opportunity permits.
