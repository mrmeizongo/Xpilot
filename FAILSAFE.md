# XPilot Failsafe Setup

Due to the raw pulse width modulation input capture mechanism used in XPilot to accommodate as many tx-rx combination as possible, failsafe detection requires a multi-step process for proper operation. Xpilot uses the throttle channel as the primary receiver failsafe indicator. On tx-rx signal loss, XPilot detects it and defaults to stabilize mode. The throttle input is overridden and set to a value slightly lower than trim.

The receiver must be configured so that, on loss of transmitter signal, the throttle output moves to a PWM value that is lower than the normal calibrated throttle range. XPilot detects this out-of-range throttle signal and enters failsafe.

## Setup

1. Calibrate the throttle channel normally in XPilot. The calibrated throttle range should represent the full usable throttle command range. 

Example:  
On my spektrum NX8 transmitter, my normal throttle operating range is 100%. This means:  
Minimum throttle PWM: 1100us  
Maximum throttle PWM: 1900us.

2. If you haven't already, assign any 2-position switch on your transmitter as your throttle cut activator. Configure throttle cut to output a value lower than 100% of transmitter's low throttle range.

Example:  
On my spektrum NX8 transmitter, I set my throttle cut to -125%. This means:  
Throttle cut PWM:  1000us.

3. Decent receivers usually allow a preset failsafe position for all control sticks and switches during the binding process. Activate throttle cut during this setup and leave all other sticks untouched(or not, it doesn't matter we don't use them anyway).

4. In [SysConfig.h](lib/Config/src/SysConfig.h), uncomment IO_DEBUG and upload to view input and output values on the serial monitor/bus. Activate the throttle cut switch and monitor the throttle pwm input. If done correctly, it should display ~1000us and enter failsafe after 2 seconds. If not, restart this guide.

5. At this point, manually activating the throttle cut will also activate the failsafe. If this behavior is not wanted(I don't know why you would), adjust the transmitter's failsafe position to a value between -101% and -113%. Failsafe tolerance is set to 52us(adjustable), verify IO_DEBUG displays between 1100 and 1050 in a commanded throttle cut and ensure failsafe is not activated. Then verify actual failsafe activation by turning off transmitter while monitoring the serial output.

## Important

REMOVE PROPPELLER FROM MOTOR BEFORE PERFORMING THIS FAILSAFE SETUP!!!

CALIBRATE ESC PER THE INSTRUCTIONS FOR YOUR ESC AND MOTOR COMBINATION. USE 100% THROTTLE RANGE. 

Do not configure the receiver failsafe throttle value at or near the normal minimum-throttle PWM.

The failsafe value must be clearly outside the calibrated throttle range so XPilot can distinguish between:

Normal minimum throttle

and:

Receiver signal loss

Failsafe detection should be based on the raw throttle PWM signal before normal throttle scaling or clamping. This ensures the intentionally out-of-range receiver failsafe value remains detectable.

The throttle failsafe signal indicates receiver-transmitter link loss. XPilot may also independently detect invalid or stale channel inputs as channel faults.