# XPilot Serial Configuration Tool

`xp_serial.py` provides a serial interface for reading, modifying, loading, and inspecting XPilot configuration parameters from a host computer.

The tool communicates directly with XPilot's `ConfigManager` over the serial configuration protocol.

---

## Commands

```bash
GET <configID>
SET <configID> <value>
SAVE
LOAD
DEFAULTS
IMU_CALIBRATE
```

## Value Types

```bash
FLOAT
UINT16
INT16
UINT8
INT8
BOOL
```

## ConfigIDs

```bash
    ConfigID                            Value Type

    // AirframeConfig
    AIRFRAME_TYPE                       UINT8

    // RCConfig
    RC_ROLL_MIN                         INT16
    RC_ROLL_TRIM                        INT16
    RC_ROLL_MAX                         INT16
    RC_ROLL_DB                          UINT8

    RC_PITCH_MIN                        INT16
    RC_PITCH_TRIM                       INT16
    RC_PITCH_MAX                        INT16
    RC_PITCH_DB                         UINT8

    RC_YAW_MIN                          INT16
    RC_YAW_TRIM                         INT16
    RC_YAW_MAX                          INT16
    RC_YAW_DB                           UINT8

    // SRVConfig
    SRV_MIN                             INT16
    SRV_TRIM                            INT16
    SRV_MAX                             INT16

    // FlightConfig
    FLIGHT_CONTROL_RES                  INT16

    FLIGHT_MAX_ROLL_RATE_DEGS           INT16
    FLIGHT_MAX_PITCH_RATE_DEGS          INT16
    FLIGHT_MAX_YAW_RATE_DEGS            INT16

    FLIGHT_MAX_ROLL_ANGLE_DEGS          INT16
    FLIGHT_MAX_PITCH_ANGLE_DEGS         INT16

    FLIGHT_FLAPERON_SCALE_FACTOR        FLOAT
    FLIGHT_MAX_FLAPERON                 INT16

    FLIGHT_REVERSE_RUDDER_MIX           BOOL
    FLIGHT_RUDDER_MIX_SCALE_FACTOR      FLOAT

    // PIDFConfig
    PIDF_ROLL_KP                        FLOAT
    PIDF_ROLL_KI                        FLOAT
    PIDF_ROLL_KD                        FLOAT
    PIDF_ROLL_KF                        FLOAT
    PIDF_ROLL_I_WINDUP_MAX              FLOAT

    PIDF_PITCH_KP                       FLOAT
    PIDF_PITCH_KI                       FLOAT
    PIDF_PITCH_KD                       FLOAT
    PIDF_PITCH_KF                       FLOAT
    PIDF_PITCH_I_WINDUP_MAX             FLOAT

    PIDF_YAW_KP                         FLOAT
    PIDF_YAW_KI                         FLOAT
    PIDF_YAW_KD                         FLOAT
    PIDF_YAW_KF                         FLOAT
    PIDF_YAW_I_WINDUP_MAX               FLOAT

    // IMUConfig
    IMU_ACC_BIAS_X                      FLOAT
    IMU_ACC_BIAS_Y                      FLOAT
    IMU_ACC_BIAS_Z                      FLOAT

    IMU_GYRO_BIAS_X                     FLOAT
    IMU_GYRO_BIAS_Y                     FLOAT
    IMU_GYRO_BIAS_Z                     FLOAT

    IMU_CALIBRATED                      BOOL

    // FilterConfig
    FILTER_SLEW_RATE                    INT16
    FILTER_LPF_FREQ                     INT16
    FILTER_PROCESS_DT                   FLOAT
```

## Airframe Types

```bash
CONVENTIONAL                            0
V_TAIL                                  1
FLYING_WING_RUDDER                      2
FLYING_WING_NO_RUDDER                   3
RUDDER_ELEVATOR                         4
AILERON_ELEVATOR                        5
CUSTOM                                  6
```

---

## Starting the Tool

From the XPilot project directory, run:

```bash
py xp_serial.py
```

If the script is configured to run directly from your environment, it may also be launched with:

```bash
xp_serial.py
```

It can also be launched with:
```bash
./xp_serial.py
```

The tool will attempt to locate and connect to the XPilot serial port.

If automatic port detection is unavailable or unsuccessful, select the appropriate COM port manually.

Example:

```text
COM3
```

---

# Configuration Commands

XPilot configuration values are identified by `ConfigID`.

`xp_serial.py` accepts the human-readable `ConfigID` names, so it is normally unnecessary to enter the hexadecimal parameter ID manually.

For example:

```text
FLIGHT_CONTROL_RESOLUTION
FILTER_SLEW_RATE
ROLL_RC_MIN
ROLL_RC_TRIM
ROLL_RC_MAX
```

Use the names defined by the current `ConfigID` table in `xp_serial.py`.

---

## View Configuration

Use the configuration display command to view the current XPilot config table.

```text
config
```

---

# GET

`GET` reads a single configuration parameter from XPilot.

Use:

```text
GET <ConfigID>
```

Example:

```text
GET FILTER_SLEW_RATE
```

The tool sends the corresponding `ConfigID` to XPilot and displays the value returned by the firmware.

The numeric/hexadecimal `ConfigID` may also be used when necessary, but using the enum name is recommended because it is easier to read and less prone to mistakes.

---

# SET

`SET` changes a configuration parameter.

Use:

```text
SET <ConfigID> <value>
```

Example:

```text
SET FILTER_SLEW_RATE 500
```

The requested value is sent to `ConfigManager::set()`.

If the value passes validation and the operation succeeds:

1. The live XPilot configuration is updated.
2. The configuration is marked as modified.
3. Registered configuration subscribers are notified.
4. Subsystems that cache the affected parameter can update their runtime state.

This allows supported parameters to take effect without requiring a reboot.
Some parameters intentionally require a reboot before their full effects are applied.

---

## Verify a SET Operation

After changing an important parameter, verify it with the `GET` command:

```text
SET FILTER_SLEW_RATE 500
GET FILTER_SLEW_RATE
```

---

# Configuration Subscribers

The XPilot configuration manager uses a subscriber system to notify runtime components when configuration values change.

Subsystems register callbacks with `ConfigManager` on initialization.

When a successful `SET` operation occurs:

```text
xp_serial.py
    ↓
SerialConfigTask
    ↓
ConfigManager::set()
    ↓
Configuration updated
    ↓
Subscribers notified
    ↓
Affected subsystem updates cached state
```

This is primarily used for parameters whose values are cached during subsystem initialization.

For example, changing:

```text
FILTER_SLEW_RATE
```

can cause the Mode subsystem to update its roll, pitch, and yaw slew-rate controllers immediately.

Not every configuration parameter is necessarily intended to be applied live. Parameters that substantially affect XPilot initialization or system architecture may require a reboot.

---

# LOAD

`LOAD` reloads the stored XPilot configuration from EEPROM.

Use:

```text
LOAD
```

A configuration reload should be treated differently from changing an individual parameter with `SET`.

Because loading a complete configuration can affect many subsystems simultaneously, XPilot may require a reboot after a configuration reload.

This prevents partially reinitialized subsystems from operating with inconsistent configuration state.

---

# Read-Only Configuration Values

Not every `ConfigID` that can be read is externally writable.

Some parameters represent system-generated state rather than user configuration.

A notable example is IMU calibration bias data.

The following types of values are intentionally read-only from the normal `SET` interface:

```text
IMU calibration biases
IMU calibrated status
```

These values may be retrieved with `GET`, but they cannot be arbitrarily changed through `SET`.

This prevents inconsistent states such as:

```text
IMU_CALIBRATED = true
```

while no valid calibration has actually been performed.

---

# IMU Calibration

IMU calibration is performed using the dedicated calibration action rather than by manually setting calibration values. Place the airplane on a level surface and begin the calibration process.

Use the IMU calibration command:

```text
IMU_CALIBRATE
```

XPilot performs the calibration procedure internally and generates the required bias values.

After successful calibration, XPilot updates its system-managed calibration configuration.

The calibration flag and generated biases can then be inspected through their corresponding readable configuration entries.

The intended configuration flow is:

```text
IMU_CALIBRATE
       ↓
XPilot collects IMU samples
       ↓
Biases calculated
       ↓
Biases stored
       ↓
Calibration marked valid
```

Do not attempt to reproduce an IMU calibration by manually modifying calibration-related configuration values.

After calibration run the `SAVE` command to store the biases in EEPROM

```bash
SAVE
```

---

# ConfigID Names vs Numeric IDs

Prefer symbolic names:

```text
GET FILTER_SLEW_RATE
```

over numeric IDs such as:

```text
GET 0x33
```

Symbolic names are preferable because configuration IDs may be added, removed, or reorganized as XPilot develops.

The Python `ConfigID` table and firmware `ConfigID` definitions must remain synchronized.

If the firmware configuration IDs are changed, the corresponding definitions in `xp_serial.py` must also be updated.

---

# Maintaining ConfigID Synchronization

Whenever a configuration parameter is:

* Added
* Removed
* Renamed
* Reassigned
* Made read-only
* Changed to system-managed state

review both:

```text
Firmware ConfigID definitions
xp_serial.py ConfigID definitions
```

The configuration count used by `xp_serial.py` must also be updated when the number of exposed configuration parameters changes.

A mismatch can produce errors such as:

```text
KeyError
```

when displaying the configuration table.

---

# Recommended Workflow

For normal parameter tuning:

```text
1. Start xp_serial.py
2. Run config
3. SET the desired parameter
4. GET the parameter to verify it
5. Test the new behavior
6. Run config when needed to inspect the complete configuration
7. SAVE config to EEPROM
```

Example:

```text
config

SET FILTER_SLEW_RATE 400

GET FILTER_SLEW_RATE

SAVE
```

---

# Parameters That Should Be Verified Before Flight

At minimum, verify configuration related to:

```text
Airframe type
Flight control resolution

Radio minimum values
Radio trim values
Radio maximum values
Radio deadbands

Servo minimum values
Servo trim values
Servo maximum values

PID/PIDF gains

Maximum angular rates
Maximum attitude limits

Filter settings
Control slew rate

Control-surface mixing
Flaperon configuration
Rudder mixing

IMU calibration status
```

The exact parameter names depend on the current XPilot `ConfigID` definitions.

---

# Troubleshooting

## SET succeeds but behavior does not change

First verify the value:

```text
GET <ConfigID>
```

If the new value is stored correctly but the subsystem continues using the old value, check whether that subsystem caches the parameter internally.

Cached parameters should either:

* register a `ConfigManager` subscriber, or
* intentionally require reboot/reinitialization.

---

## Value works after reboot but not immediately

This normally indicates one of two things:

1. The affected subsystem only reads the configuration during initialization.
2. Its configuration subscriber is not updating the cached runtime value.

Check the subsystem's configuration callback.

---

## Configuration table produces a KeyError

Verify that:

```text
Firmware ConfigID
xp_serial.py ConfigID
configuration count
```

are synchronized.

This commonly occurs after adding, removing, or renaming configuration entries without updating the Python-side configuration table/count.

---

## Settings disappear after firmware changes

If the binary layout of the stored `Config` structure changes, previously stored EEPROM configuration may no longer be compatible with the new firmware.

Examples include:

* Adding configuration members
* Removing configuration members
* Changing configuration member types
* Reordering stored members
* Replacing multiple parameters with a new parameter

After a configuration schema change, EEPROM configuration may need to be regenerated or reset depending on the firmware's version/checksum handling.

---

# Design Notes

`xp_serial.py` is an interface to XPilot's configuration system; it is not the owner of configuration state.

The authoritative configuration remains inside XPilot:

```text
xp_serial.py
      ↓
Serial protocol
      ↓
SerialConfigTask
      ↓
ConfigManager
      ↓
Config
```
