#!/usr/bin/env python3

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

    1: "RC_ROLL_MIN",
    2: "RC_ROLL_TRIM",
    3: "RC_ROLL_MAX",
    4: "RC_ROLL_DB",
    5: "RC_ROLL_REVERSE",

    6: "RC_PITCH_MIN",
    7: "RC_PITCH_TRIM",
    8: "RC_PITCH_MAX",
    9: "RC_PITCH_DB",
    10: "RC_PITCH_REVERSE",

    11: "RC_YAW_MIN",
    12: "RC_YAW_TRIM",
    13: "RC_YAW_MAX",
    14: "RC_YAW_DB",
    15: "RC_YAW_REVERSE",

    16: "SRV_ROLL_MIN",
    17: "SRV_ROLL_TRIM",
    18: "SRV_ROLL_MAX",
    19: "SRV_ROLL_REVERSE",

    20: "SRV_PITCH_MIN",
    21: "SRV_PITCH_TRIM",
    22: "SRV_PITCH_MAX",
    23: "SRV_PITCH_REVERSE",

    24: "SRV_YAW_MIN",
    25: "SRV_YAW_TRIM",
    26: "SRV_YAW_MAX",
    27: "SRV_YAW_REVERSE",

    28: "SRV_AUX_MIN",
    29: "SRV_AUX_TRIM",
    30: "SRV_AUX_MAX",
    31: "SRV_AUX_REVERSE",

    32: "FLIGHT_MAX_ROLL_RATE_DEGS",
    33: "FLIGHT_MAX_PITCH_RATE_DEGS",
    34: "FLIGHT_MAX_YAW_RATE_DEGS",

    35: "FLIGHT_MAX_ROLL_ANGLE_DEGS",
    36: "FLIGHT_MAX_PITCH_ANGLE_DEGS",

    37: "FLIGHT_ROLL_ANGLE_KP",
    38: "FLIGHT_PITCH_ANGLE_KP",

    39: "FLIGHT_FLAPERON_SCALE_FACTOR",
    40: "FLIGHT_MAX_FLAPERON",

    41: "FLIGHT_REVERSE_RUDDER_MIX",
    42: "FLIGHT_RUDDER_MIX_SCALE_FACTOR",

    43: "PIDF_ROLL_KP",
    44: "PIDF_ROLL_KI",
    45: "PIDF_ROLL_KD",
    46: "PIDF_ROLL_KF",
    47: "PIDF_ROLL_I_WINDUP_MAX",

    48: "PIDF_PITCH_KP",
    49: "PIDF_PITCH_KI",
    50: "PIDF_PITCH_KD",
    51: "PIDF_PITCH_KF",
    52: "PIDF_PITCH_I_WINDUP_MAX",

    53: "PIDF_YAW_KP",
    54: "PIDF_YAW_KI",
    55: "PIDF_YAW_KD",
    56: "PIDF_YAW_KF",
    57: "PIDF_YAW_I_WINDUP_MAX",

    58: "IMU_ACC_BIAS_X",
    59: "IMU_ACC_BIAS_Y",
    60: "IMU_ACC_BIAS_Z",

    61: "IMU_GYRO_BIAS_X",
    62: "IMU_GYRO_BIAS_Y",
    63: "IMU_GYRO_BIAS_Z",

    64: "IMU_CALIBRATED",

    65: "PROCESSING_SLEW_RATE",
    66: "PROCESSING_LPF_FREQ",
    67: "PROCESSING_DT",
}


CONFIG_COUNT = 68


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

    # RollRC
    1: ValueType.INT16,
    2: ValueType.INT16,
    3: ValueType.INT16,
    4: ValueType.UINT8,
    5: ValueType.BOOL,

    # PitchRC
    6: ValueType.INT16,
    7: ValueType.INT16,
    8: ValueType.INT16,
    9: ValueType.UINT8,
    10: ValueType.BOOL,

    # YawRC
    11: ValueType.INT16,
    12: ValueType.INT16,
    13: ValueType.INT16,
    14: ValueType.UINT8,
    15: ValueType.BOOL,

    # Roll SrvConfig
    16: ValueType.INT16,
    17: ValueType.INT16,
    18: ValueType.INT16,
    19: ValueType.BOOL,

    # Pitch SrvConfig
    20: ValueType.INT16,
    21: ValueType.INT16,
    22: ValueType.INT16,
    23: ValueType.BOOL,

    # Yaw SrvConfig
    24: ValueType.INT16,
    25: ValueType.INT16,
    26: ValueType.INT16,
    27: ValueType.BOOL,

    # Aux SrvConfig
    28: ValueType.INT16,
    29: ValueType.INT16,
    30: ValueType.INT16,
    31: ValueType.BOOL,

    # FlightConfig
    32: ValueType.INT16,
    33: ValueType.INT16,
    34: ValueType.INT16,

    35: ValueType.INT16,
    36: ValueType.INT16,

    37: ValueType.FLOAT,
    38: ValueType.FLOAT,

    39: ValueType.FLOAT,
    40: ValueType.INT16,

    41: ValueType.BOOL,
    42: ValueType.FLOAT,

    # RollPIDF
    43: ValueType.FLOAT,
    44: ValueType.FLOAT,
    45: ValueType.FLOAT,
    46: ValueType.FLOAT,
    47: ValueType.FLOAT,

    # PitchPIDF
    48: ValueType.FLOAT,
    49: ValueType.FLOAT,
    50: ValueType.FLOAT,
    51: ValueType.FLOAT,
    52: ValueType.FLOAT,

    # YawPIDF
    53: ValueType.FLOAT,
    54: ValueType.FLOAT,
    55: ValueType.FLOAT,
    56: ValueType.FLOAT,
    57: ValueType.FLOAT,

    # IMUConfig
    58: ValueType.FLOAT,
    59: ValueType.FLOAT,
    60: ValueType.FLOAT,

    61: ValueType.FLOAT,
    62: ValueType.FLOAT,
    63: ValueType.FLOAT,

    64: ValueType.BOOL,

    # ProcessingConfig
    65: ValueType.UINT16,
    66: ValueType.UINT16,
    67: ValueType.FLOAT,
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