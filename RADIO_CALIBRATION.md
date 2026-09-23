# XPilot radio calibration

During radio calibration, XPilot takes a coherent snapshot of throttle, roll, pitch, and yaw every 50 ms
while streaming is active. Each snapshot is transmitted as four existing-size
nine-byte packets. The stream stops automatically and emits a NACK if any of
the four primary receiver inputs becomes invalid or stale.

## PC workflow

If not already configured, set transmitter rates to 100%.  
This establishes baseline mininum and maximum of outputs for all axes.  
Run this command in the interactive utility:

```text
./xp_serial.py  OR  python xp_serial.py
calibrate_radio
```

The utility performs two capture phases:

1. Capture the normal minimum and maximum of throttle, roll, pitch, yaw and the auxiliary channels.
2. Capture centered roll, pitch, yaw and auxiliary channel values and calculate their median.

Throttle trim is calculated as the midpoint between its captured normal
minimum and maximum. Deadband and reversal settings are not changed.

After validation, the utility displays the proposed results and requires user
confirmation before changing the active configuration in RAM. It then applies the 12
values with existing `SET` commands and verifies them with `GET` commands. If
application or verification fails, it attempts to restore the original 12
values.

The utility never issues the `SAVE` command when complete. Configuration saved in 
EEPROM remains unchanged until the user explicitly sends the `SAVE` command separately.

User retains the ability to modify the min, trim and max values for all 4 axes separately.
This can be performed by utilizing the `SET` command followed by the config ID.

Example

```bash
set RC_ROLL_MIN 1100

set RC_THROTTLE_MAX 1904
```

After every `SET` operation, be sure to verify the operation succeeded by using the `GET` command.


## Safety and validation

- Propulsion must be physically disconnected before calibration.
- Throttle cut must remain off during endpoint capture.
- A throttle minimum below 1050 us produces a warning because it may represent
  the throttle-cut signal rather than the normal endpoint.
- Roll, pitch, yaw and auxiliary center data must contain at least 20 samples and remain
  within a 30 us spread.
