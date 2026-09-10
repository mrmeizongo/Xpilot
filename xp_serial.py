#!/usr/bin/env python

import argparse
import struct
import sys
import time

import serial
from serial.tools import list_ports


# ============================================================
# XPilot serial protocol
# ============================================================

START_BYTE = 0xAA
PACKET_SIZE = 9

class Color:
    RESET   = "\033[0m"
    RED     = "\033[31m"
    GREEN   = "\033[32m"
    YELLOW  = "\033[33m"
    BLUE    = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN    = "\033[36m"
    WHITE   = "\033[37m"

    BOLD    = "\033[1m"
    DIM     = "\033[2m"

class Command:
    GET = 0x01
    SET = 0x02
    SAVE = 0x03
    LOAD = 0x04
    DEFAULTS = 0x05
    CALIBRATE_IMU = 0x06

    ACK = 0x80
    NACK = 0x81
    VALUE = 0x82


class ValueType:
    FLOAT = 0x00
    UINT16 = 0x01
    INT16 = 0x02
    UINT8 = 0x03
    INT8 = 0x04
    BOOL = 0x05


COMMAND_NAMES = {
    Command.GET: "GET",
    Command.SET: "SET",
    Command.SAVE: "SAVE",
    Command.LOAD: "LOAD",
    Command.DEFAULTS: "DEFAULTS",
    Command.CALIBRATE_IMU: "CALIBRATE_IMU",

    Command.ACK: "ACK",
    Command.NACK: "NACK",
    Command.VALUE: "VALUE",
}


COMMAND_IDS_BY_NAME = {
    "GET": Command.GET,
    "SET": Command.SET,
    "SAVE": Command.SAVE,
    "LOAD": Command.LOAD,
    "DEFAULTS": Command.DEFAULTS,
    "CALIBRATE_IMU": Command.CALIBRATE_IMU,
}


TYPE_NAMES = {
    ValueType.FLOAT: "FLOAT",
    ValueType.UINT16: "UINT16",
    ValueType.INT16: "INT16",
    ValueType.UINT8: "UINT8",
    ValueType.INT8: "INT8",
    ValueType.BOOL: "BOOL",
}


# ============================================================
# AirframeType table
#
# Keep this synchronized Xpilot AirframeType enum in Config.
# ============================================================

AIRFRAME_TYPES = {
    "CONVENTIONAL": 0,
    "V_TAIL": 1,
    "FLYING_WING_RUDDER": 2,
    "FLYING_WING_NO_RUDDER": 3,
    "RUDDER_ELEVATOR": 4,
    "AILERON_ELEVATOR": 5,
    "CUSTOM": 6,
    
}


AIRFRAME_NAMES = {
    value: name
    for name, value in AIRFRAME_TYPES.items()
}


# ============================================================
# ConfigID table
# ============================================================

CONFIG_NAMES = {
    0: "AIRFRAME_TYPE",

    1: "RC_THROTTLE_MIN",
    2: "RC_THROTTLE_TRIM",
    3: "RC_THROTTLE_MAX",
    4: "RC_THROTTLE_DB",
    5: "RC_THROTTLE_REVERSE",

    6: "RC_ROLL_MIN",
    7: "RC_ROLL_TRIM",
    8: "RC_ROLL_MAX",
    9: "RC_ROLL_DB",
    10: "RC_ROLL_REVERSE",

    11: "RC_PITCH_MIN",
    12: "RC_PITCH_TRIM",
    13: "RC_PITCH_MAX",
    14: "RC_PITCH_DB",
    15: "RC_PITCH_REVERSE",

    16: "RC_YAW_MIN",
    17: "RC_YAW_TRIM",
    18: "RC_YAW_MAX",
    19: "RC_YAW_DB",
    20: "RC_YAW_REVERSE",

    21: "SRV_THROTTLE_MIN",
    22: "SRV_THROTTLE_TRIM",
    23: "SRV_THROTTLE_MAX",
    24: "SRV_THROTTLE_REVERSE",

    25: "SRV_ROLL_MIN",
    26: "SRV_ROLL_TRIM",
    27: "SRV_ROLL_MAX",
    28: "SRV_ROLL_REVERSE",

    29: "SRV_PITCH_MIN",
    30: "SRV_PITCH_TRIM",
    31: "SRV_PITCH_MAX",
    32: "SRV_PITCH_REVERSE",

    33: "SRV_YAW_MIN",
    34: "SRV_YAW_TRIM",
    35: "SRV_YAW_MAX",
    36: "SRV_YAW_REVERSE",

    37: "SRV_AUX_MIN",
    38: "SRV_AUX_TRIM",
    39: "SRV_AUX_MAX",
    40: "SRV_AUX_REVERSE",

    41: "FLIGHT_MAX_ROLL_RATE_DEGS",
    42: "FLIGHT_MAX_PITCH_RATE_DEGS",
    43: "FLIGHT_MAX_YAW_RATE_DEGS",

    44: "FLIGHT_MAX_ROLL_ANGLE_DEGS",
    45: "FLIGHT_MAX_PITCH_ANGLE_DEGS",

    46: "FLIGHT_ROLL_ANGLE_KP",
    47: "FLIGHT_PITCH_ANGLE_KP",

    48: "FLIGHT_FLAPERON_SCALE_FACTOR",
    49: "FLIGHT_MAX_FLAPERON",

    50: "FLIGHT_REVERSE_RUDDER_MIX",
    51: "FLIGHT_RUDDER_MIX_SCALE_FACTOR",

    52: "PIDF_ROLL_KP",
    53: "PIDF_ROLL_KI",
    54: "PIDF_ROLL_KD",
    55: "PIDF_ROLL_KF",
    56: "PIDF_ROLL_I_WINDUP_MAX",

    57: "PIDF_PITCH_KP",
    58: "PIDF_PITCH_KI",
    59: "PIDF_PITCH_KD",
    60: "PIDF_PITCH_KF",
    61: "PIDF_PITCH_I_WINDUP_MAX",

    62: "PIDF_YAW_KP",
    63: "PIDF_YAW_KI",
    64: "PIDF_YAW_KD",
    65: "PIDF_YAW_KF",
    66: "PIDF_YAW_I_WINDUP_MAX",

    67: "IMU_ACC_BIAS_X",
    68: "IMU_ACC_BIAS_Y",
    69: "IMU_ACC_BIAS_Z",

    70: "IMU_GYRO_BIAS_X",
    71: "IMU_GYRO_BIAS_Y",
    72: "IMU_GYRO_BIAS_Z",

    73: "IMU_CALIBRATED",

    74: "CONTROL_SLEW_RATE",
    75: "CONTROL_LPF_FREQ",
    76: "CONTROL_RESOLUTION",
    77: "CONTROL_DT",
}


CONFIG_COUNT = 78


CONFIG_IDS_BY_NAME = {
    name: param_id
    for param_id, name in CONFIG_NAMES.items()
}


# ============================================================
# Config types
# ============================================================

CONFIG_TYPES = {
    # Airframe type
    0: ValueType.UINT8,

    # THROTTLERC
    1: ValueType.INT16,
    2: ValueType.INT16,
    3: ValueType.INT16,
    4: ValueType.UINT8,
    5: ValueType.BOOL,

    # RollRC
    6: ValueType.INT16,
    7: ValueType.INT16,
    8: ValueType.INT16,
    9: ValueType.UINT8,
    10: ValueType.BOOL,

    # PitchRC
    11: ValueType.INT16,
    12: ValueType.INT16,
    13: ValueType.INT16,
    14: ValueType.UINT8,
    15: ValueType.BOOL,

    # YawRC
    16: ValueType.INT16,
    17: ValueType.INT16,
    18: ValueType.INT16,
    19: ValueType.UINT8,
    20: ValueType.BOOL,

    # Throrrle SrvConfig
    21: ValueType.INT16,
    22: ValueType.INT16,
    23: ValueType.INT16,
    24: ValueType.BOOL,

    # Roll SrvConfig
    25: ValueType.INT16,
    26: ValueType.INT16,
    27: ValueType.INT16,
    28: ValueType.BOOL,

    # Pitch SrvConfig
    29: ValueType.INT16,
    30: ValueType.INT16,
    31: ValueType.INT16,
    32: ValueType.BOOL,

    # Yaw SrvConfig
    33: ValueType.INT16,
    34: ValueType.INT16,
    35: ValueType.INT16,
    36: ValueType.BOOL,

    # Aux SrvConfig
    37: ValueType.INT16,
    38: ValueType.INT16,
    39: ValueType.INT16,
    40: ValueType.BOOL,

    # FlightConfig
    41: ValueType.INT16,
    42: ValueType.INT16,
    43: ValueType.INT16,

    44: ValueType.INT16,
    45: ValueType.INT16,

    46: ValueType.FLOAT,
    47: ValueType.FLOAT,

    48: ValueType.FLOAT,
    49: ValueType.INT16,

    50: ValueType.BOOL,
    51: ValueType.FLOAT,

    # RollPIDF
    52: ValueType.FLOAT,
    53: ValueType.FLOAT,
    54: ValueType.FLOAT,
    55: ValueType.FLOAT,
    56: ValueType.FLOAT,

    # PitchPIDF
    57: ValueType.FLOAT,
    58: ValueType.FLOAT,
    59: ValueType.FLOAT,
    60: ValueType.FLOAT,
    61: ValueType.FLOAT,

    # YawPIDF
    62: ValueType.FLOAT,
    63: ValueType.FLOAT,
    64: ValueType.FLOAT,
    65: ValueType.FLOAT,
    66: ValueType.FLOAT,

    # IMUConfig
    67: ValueType.FLOAT,
    68: ValueType.FLOAT,
    69: ValueType.FLOAT,

    70: ValueType.FLOAT,
    71: ValueType.FLOAT,
    72: ValueType.FLOAT,

    73: ValueType.BOOL,

    # ControlConfig
    74: ValueType.UINT16,
    75: ValueType.UINT16,
    76: ValueType.INT16,
    77: ValueType.FLOAT,
}


# ============================================================
# Serial port discovery
# ============================================================

def get_serial_ports():
    return list(list_ports.comports())


def likely_arduino_port(port):
    text = " ".join([
        port.description or "",
        port.manufacturer or "",
        port.product or "",
        port.hwid or "",
    ]).lower()

    keywords = (
        "arduino",
        "nano",
        "ch340",
        "ch341",
        "wch",
        "ftdi",
        "usb serial",
        "usb-serial",
    )

    return any(keyword in text for keyword in keywords)


def print_ports(ports):
    if not ports:
        print(f"{Color.YELLOW}No serial ports found.{Color.RESET}")
        return

    print("\nAvailable serial ports:")

    for index, port in enumerate(ports):
        print(
            f"  [{index}] "
            f"{port.device:<8} "
            f"{port.description}"
        )

    print()


def select_port(manual_port=None):
    ports = get_serial_ports()

    if manual_port:
        return manual_port

    if not ports:
        raise RuntimeError(
            f"{Color.RED}No serial ports were found.{Color.RESET}"
        )

    for port in ports:
        if port.device.upper() == "COM3":
            print(
                f"Automatically selected "
                f"{port.device}: "
                f"{port.description}"
            )
            return port.device

    candidates = [
        port
        for port in ports
        if likely_arduino_port(port)
    ]

    if len(candidates) == 1:
        port = candidates[0]

        print(
            f"Automatically selected "
            f"{port.device}: "
            f"{port.description}"
        )

        return port.device

    if len(ports) == 1:
        port = ports[0]

        print(
            f"Only one serial port found. "
            f"Using {port.device}: "
            f"{port.description}"
        )

        return port.device

    print_ports(ports)

    while True:
        entry = input(
            "Enter port number or port name "
            "(example: 0 or COM3): "
        ).strip()

        if entry.isdigit():
            index = int(entry)

            if 0 <= index < len(ports):
                return ports[index].device

        else:
            for port in ports:
                if port.device.lower() == entry.lower():
                    return port.device

        print(f"{Color.RED}Invalid port selection.{Color.RESET}")


# ============================================================
# Parsing helpers
# ============================================================

def parse_command(text):
    text = text.strip().upper()

    if text in COMMAND_IDS_BY_NAME:
        return COMMAND_IDS_BY_NAME[text]

    # Raw HEX fallback.
    raw = text

    if raw.startswith("0X"):
        raw = raw[2:]

    try:
        value = int(raw, 16)

    except ValueError:
        raise ValueError(
            f"Unknown command '{text}'."
        )

    if not 0 <= value <= 0xFF:
        raise ValueError(
            f"{Color.RED}Command must fit in one byte.{Color.RESET}"
        )

    return value


def parse_config_id(text):
    text = text.strip()
    upper = text.upper()

    if upper in CONFIG_IDS_BY_NAME:
        return CONFIG_IDS_BY_NAME[upper]

    try:
        value = int(text, 10)

    except ValueError:
        raise ValueError(
            f"{Color.RED}Unknown ConfigID '{text}'.{Color.RESET}"
        )

    if not 0 <= value < CONFIG_COUNT:
        raise ValueError(
            f"{Color.RED}ConfigID must be between {Color.RESET}"
            f"{Color.RED}0 and {CONFIG_COUNT - 1}.{Color.RESET}"
        )

    return value


def additive_checksum(data):
    return sum(data) & 0xFF


def bytes_to_hex(data):
    return " ".join(
        f"{byte:02X}"
        for byte in data
    )


# ============================================================
# Airframe parsing
# ============================================================

def parse_airframe_type(text):
    """
    Accepts:

        CONVENTIONAL
        V_TAIL
        FLYING_WING_RUDDER
        FLYING_WING_NO_RUDDER
        RUDDER_ELEVATOR
        AILERON_ELEVATOR
        CUSTOM

    Numeric values are also accepted.
    """

    upper = text.strip().upper()

    if upper in AIRFRAME_TYPES:
        return AIRFRAME_TYPES[upper]

    try:
        value = int(text, 0)

    except ValueError:
        valid = ", ".join(AIRFRAME_TYPES.keys())

        raise ValueError(
            f"{Color.RED}Unknown AirframeType '{text}'. {Color.RESET}"
            f"{Color.RED}Valid values: {valid}{Color.RESET}"
        )

    if value not in AIRFRAME_NAMES:
        raise ValueError(
            f"{Color.RED}Invalid AirframeType value {value}.{Color.RESET}"
        )

    return value


# ============================================================
# User value encoding
# ============================================================

def encode_user_value(
    param_id,
    value_type,
    text
):
    # --------------------------------------------------------
    # AIRFRAME_TYPE
    # --------------------------------------------------------

    if param_id == CONFIG_IDS_BY_NAME["AIRFRAME_TYPE"]:
        return parse_airframe_type(text)

    # --------------------------------------------------------
    # Normal values
    # --------------------------------------------------------

    if value_type == ValueType.FLOAT:
        value = float(text)

        return struct.unpack(
            "<I",
            struct.pack("<f", value)
        )[0]

    if value_type == ValueType.UINT16:
        value = int(text, 0)

        if not 0 <= value <= 0xFFFF:
            raise ValueError(
                f"{Color.RED}UINT16 must be between{Color.RESET} "
                f"{Color.RED}0 and 65535.{Color.RESET}"
            )

        return value

    if value_type == ValueType.INT16:
        value = int(text, 0)

        if not -32768 <= value <= 32767:
            raise ValueError(
                f"{Color.RED}INT16 must be between {Color.RESET}"
                f"{Color.RED}-32768 and 32767.{Color.RESET}"
            )

        return value & 0xFFFF

    if value_type == ValueType.UINT8:
        value = int(text, 0)

        if not 0 <= value <= 0xFF:
            raise ValueError(
                f"{Color.RED}UINT8 must be between{Color.RESET} "
                f"{Color.RED}0 and 255.{Color.RESET}"
            )

        return value

    if value_type == ValueType.BOOL:
        lowered = text.lower()

        if lowered in (
            "1",
            "true",
            "on",
            "yes",
        ):
            return 1

        if lowered in (
            "0",
            "false",
            "off",
            "no",
        ):
            return 0

        raise ValueError(
            f"{Color.RED}BOOL must be one of: {Color.RESET}"
            f"{Color.RED}0, 1, false, true, off, on.{Color.RESET}"
        )

    raise ValueError(
        f"{Color.RED}Unsupported configuration value type.{Color.RESET}"
    )


# ============================================================
# Packet construction
# ============================================================

def build_packet(
    command,
    param_id=0,
    value_type=0,
    value=0
):
    packet = bytearray(PACKET_SIZE)

    packet[0] = START_BYTE
    packet[1] = command & 0xFF
    packet[2] = param_id & 0xFF
    packet[3] = value_type & 0xFF

    packet[4:8] = struct.pack(
        "<I",
        value & 0xFFFFFFFF
    )

    packet[8] = additive_checksum(
        packet[:8]
    )

    return bytes(packet)


# ============================================================
# Response decoding
# ============================================================

def decode_value(
    value_type,
    raw_bytes
):
    if value_type == ValueType.FLOAT:
        return struct.unpack(
            "<f",
            raw_bytes
        )[0]

    if value_type == ValueType.UINT16:
        return struct.unpack(
            "<H",
            raw_bytes[:2]
        )[0]

    if value_type == ValueType.INT16:
        return struct.unpack(
            "<h",
            raw_bytes[:2]
        )[0]

    if value_type == ValueType.UINT8:
        return raw_bytes[0]

    if value_type == ValueType.BOOL:
        return raw_bytes[0] != 0

    return int.from_bytes(
        raw_bytes,
        byteorder="little"
    )


def decode_packet(packet):
    if len(packet) != PACKET_SIZE:
        print(f"{Color.YELLOW}RX: Invalid packet length.{Color.RESET}")
        return

    if packet[0] != START_BYTE:
        print(f"{Color.YELLOW}RX: Invalid start byte.{Color.RESET}")
        return

    calculated_checksum = additive_checksum(
        packet[:8]
    )

    if packet[8] != calculated_checksum:
        print(
            f"{Color.RED}RX: CHECKSUM ERROR "
            f"received=0x{packet[8]:02X} "
            f"calculated="
            f"0x{calculated_checksum:02X}{Color.RESET}"
        )
        return

    print(
        f"{Color.GREEN}RX:{Color.RESET}",
        bytes_to_hex(packet)
    )

    command = packet[1]
    param_id = packet[2]
    value_type = packet[3]
    raw_value = packet[4:8]

    # --------------------------------------------------------
    # ACK
    # --------------------------------------------------------

    if command == Command.ACK:
        original_command = COMMAND_NAMES.get(
            param_id,
            f"0x{param_id:02X}"
        )

        print(
            f"  {Color.GREEN}ACK: {original_command}{Color.RESET}"
        )
        return

    # --------------------------------------------------------
    # NACK
    # --------------------------------------------------------

    if command == Command.NACK:
        original_command = COMMAND_NAMES.get(
            param_id,
            f"0x{param_id:02X}"
        )

        print(
            f"  {Color.RED} NACK: {original_command}{Color.RESET}"
        )
        return

    # --------------------------------------------------------
    # VALUE
    # --------------------------------------------------------

    if command == Command.VALUE:
        name = CONFIG_NAMES.get(
            param_id,
            "UNKNOWN"
        )

        type_name = TYPE_NAMES.get(
            value_type,
            f"UNKNOWN(0x{value_type:02X})"
        )

        decoded = decode_value(
            value_type,
            raw_value
        )

        print(
            f"  ConfigID: "
            f"{param_id} [{name}]"
        )

        print(
            f"  Wire ID:  "
            f"0x{param_id:02X}"
        )

        print(
            f"  Type:     "
            f"{type_name}"
        )

        # Special display for AIRFRAME_TYPE.
        if (
            param_id
            == CONFIG_IDS_BY_NAME["AIRFRAME_TYPE"]
        ):
            airframe_name = AIRFRAME_NAMES.get(
                decoded,
                "UNKNOWN"
            )

            print(
                f"  Value:    "
                f"{decoded} [{airframe_name}]"
            )

        else:
            print(
                f"  Value:    "
                f"{decoded}"
            )

        return

    command_name = COMMAND_NAMES.get(
        command,
        f"UNKNOWN(0x{command:02X})"
    )

    print(
        f"  Command: {command_name}"
    )


# ============================================================
# User command parsing
# ============================================================

def parse_user_command(line):
    parts = line.split()

    if not parts:
        return None

    command = parse_command(
        parts[0]
    )

    # --------------------------------------------------------
    # No-argument commands
    # --------------------------------------------------------

    if command in (
        Command.SAVE,
        Command.LOAD,
        Command.DEFAULTS,
        Command.CALIBRATE_IMU,
    ):
        if len(parts) != 1:
            name = COMMAND_NAMES.get(
                command,
                f"0x{command:02X}"
            )

            raise ValueError(
                f"{Color.RED}{name} does not accept arguments.{Color.RESET}"
            )

        return build_packet(
            command=command
        )

    # --------------------------------------------------------
    # GET
    #
    # GET AIRFRAME_TYPE
    # GET IMU_CALIBRATED
    # --------------------------------------------------------

    if command == Command.GET:
        if len(parts) != 2:
            raise ValueError(
                "GET syntax:\n"
                "  GET CONFIG_ID\n\n"
                "Example:\n"
                "  GET AIRFRAME_TYPE"
            )

        param_id = parse_config_id(
            parts[1]
        )

        return build_packet(
            command=command,
            param_id=param_id
        )

    # --------------------------------------------------------
    # SET
    #
    # SET AIRFRAME_TYPE V_TAIL
    # SET PIDF_ROLL_KP 0.5
    # --------------------------------------------------------

    if command == Command.SET:
        if len(parts) != 3:
            raise ValueError(
                "SET syntax:\n"
                "  SET CONFIG_ID VALUE\n\n"
                "Examples:\n"
                "  SET AIRFRAME_TYPE V_TAIL\n"
                "  SET PIDF_ROLL_KP 0.5"
            )

        param_id = parse_config_id(
            parts[1]
        )

        value_type = CONFIG_TYPES.get(
            param_id
        )

        if value_type is None:
            raise ValueError(
                f"{Color.RED}No type registered for "
                f"ConfigID {param_id}.{Color.RESET}"
            )

        value = encode_user_value(
            param_id,
            value_type,
            parts[2]
        )

        return build_packet(
            command=command,
            param_id=param_id,
            value_type=value_type,
            value=value
        )

    raise ValueError(
        f"{Color.RED}Unsupported command "
        f"0x{command:02X}.{Color.RESET}"
    )


# ============================================================
# Receive one packet
# ============================================================

def receive_packet(
    ser,
    timeout=2.0
):
    start_time = time.monotonic()
    buffer = bytearray()

    while (
        time.monotonic() - start_time
        < timeout
    ):
        if ser.in_waiting > 0:
            byte = ser.read(1)[0]

            if not buffer:
                if byte != START_BYTE:
                    continue

            buffer.append(byte)

            if len(buffer) == PACKET_SIZE:
                return bytes(buffer)

        else:
            time.sleep(0.005)

    return None


# ============================================================
# Tables
# ============================================================

def print_config_table():
    print(
        "\nXPilot ConfigID Table"
    )

    print(
        "-------------------------------------------------------------"
    )

    print(
        f"{'DEC':>3}  "
        f"{'HEX':>4}  "
        f"{'NAME':<35} "
        f"TYPE"
    )

    print(
        "-------------------------------------------------------------"
    )

    for param_id in range(CONFIG_COUNT):
        name = CONFIG_NAMES[
            param_id
        ]

        value_type = CONFIG_TYPES.get(
            param_id
        )

        type_name = TYPE_NAMES.get(
            value_type,
            "UNKNOWN"
        )

        print(
            f"{param_id:3d}  "
            f"0x{param_id:02X}  "
            f"{name:<35} "
            f"{type_name}"
        )

    print()


def print_airframe_table():
    print(
        "\nXPilot Airframe Types"
    )

    print(
        "----------------------------"
    )

    for name, value in AIRFRAME_TYPES.items():
        print(
            f"{value:3d}  "
            f"0x{value:02X}  "
            f"{name}"
        )

    print()


def print_command_table():
    print(
        "\nXPilot Commands"
    )

    print(
        "--------------------------------"
    )

    for name, command in COMMAND_IDS_BY_NAME.items():
        print(
            f"0x{command:02X}  {name}"
        )

    print()


# ============================================================
# Help
# ============================================================

def print_help():
    print(f"""
{Color.GREEN}XPilot Serial Utility{Color.RESET}
=====================


{Color.MAGENTA}GET{Color.RESET}
---

    GET CONFIG_ID

Examples:

    GET AIRFRAME_TYPE
    GET IMU_CALIBRATED
    GET PIDF_ROLL_KP


{Color.MAGENTA}SET{Color.RESET}
---

    SET CONFIG_ID VALUE

Examples:

    SET PIDF_ROLL_KP 0.5

    SET RC_ROLL_TRIM 1500

    SET FLIGHT_REVERSE_RUDDER_MIX true


{Color.MAGENTA}AIRFRAME TYPE{Color.RESET}
-------------

Airframe names can be used directly.

Examples:

    SET AIRFRAME_TYPE CONVENTIONAL

    SET AIRFRAME_TYPE V_TAIL

    SET AIRFRAME_TYPE FLYING_WING

To read the current airframe:

    GET AIRFRAME_TYPE

The response will display both the numeric value
and the airframe name.


{Color.MAGENTA}SYSTEM COMMANDS{Color.RESET}
---------------

    SAVE

    LOAD

    DEFAULTS

    CALIBRATE_IMU


{Color.MAGENTA}UTILITY COMMANDS{Color.RESET}
----------------

config
    Display ConfigID values.

airframes
    Display supported AirframeType values.

commands
    Display command IDs.

ports
    Display available serial ports.

help
    Display this help.

quit
exit
    Exit the utility.


{Color.MAGENTA}RAW PROTOCOL{Color.RESET}
------------

Raw HEX command bytes are still supported.

For example:

    01 48

is equivalent to:

    GET IMU_CALIBRATED
{Color.RESET}"""
    )


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description=(
            "XPilot binary serial "
            "configuration utility"
        )
    )

    parser.add_argument(
        "-p",
        "--port",
        help=(
            "Serial port, "
            "for example COM3"
        )
    )

    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=250000,
        help=(
            "Serial baud rate "
            "(default: 250000)"
        )
    )

    args = parser.parse_args()

    try:
        port = select_port(
            args.port
        )

    except RuntimeError as error:
        print(error)
        return 1

    print(
        f"\nConnecting to Xpilot on {port} "
        f"at {args.baud} baud..."
    )

    try:
        ser = serial.Serial(
            port=port,
            baudrate=args.baud,
            timeout=0,
            write_timeout=0.1
        )

    except serial.SerialException as error:
        print(
            f"{Color.RED}Unable to open {Color.RESET}"
            f"{Color.RED}{port}: {error}{Color.RESET}"
        )

        return 1

    # Nano commonly resets when serial opens.
    time.sleep(2.0)

    ser.reset_input_buffer()

    print("Connected.")
    print(
        "Type 'help' for commands.\n"
    )

    try:
        while True:
            line = input(
                "\nXPilot> "
            ).strip()

            if not line:
                continue

            lowered = line.lower()

            if lowered in (
                "quit",
                "exit",
            ):
                break

            if lowered == "help":
                print_help()
                continue

            if lowered == "config":
                print_config_table()
                continue

            if lowered == "airframes":
                print_airframe_table()
                continue

            if lowered == "commands":
                print_command_table()
                continue

            if lowered == "ports":
                print_ports(
                    get_serial_ports()
                )
                continue

            try:
                packet = parse_user_command(
                    line
                )

                if packet is None:
                    continue

                print(
                    f"{Color.GREEN}TX:{Color.RESET}",
                    bytes_to_hex(packet)
                )

                ser.write(packet)

                response = receive_packet(
                    ser,
                    timeout=2.0
                )

                if response is None:
                    print(
                        f"{Color.RED}RX: No response received.{Color.RESET}"
                    )

                else:
                    decode_packet(
                        response
                    )

            except ValueError as error:
                print(
                    f"Command error: {error}"
                )

            except serial.SerialException as error:
                print(
                    f"Serial error: {error}"
                )

                break

    except KeyboardInterrupt:
        print(
            f"\n{Color.RED}Stopping...{Color.RESET}"
        )

    finally:
        if ser.is_open:
            ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())