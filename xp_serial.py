#!/usr/bin/env python

import argparse
import statistics
import struct
import sys
import time

import serial
from serial.tools import list_ports

from prompt_toolkit import prompt
from prompt_toolkit.history import FileHistory


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
    START_RADIO_STREAM = 0x07
    STOP_RADIO_STREAM = 0x08

    ACK = 0x80
    NACK = 0x81
    VALUE = 0x82
    RADIO_VALUE = 0x83


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
    Command.START_RADIO_STREAM: "START_RADIO_STREAM",
    Command.STOP_RADIO_STREAM: "STOP_RADIO_STREAM",

    Command.ACK: "ACK",
    Command.NACK: "NACK",
    Command.VALUE: "VALUE",
    Command.RADIO_VALUE: "RADIO_VALUE",
}


SERIAL_COMMAND_IDS_BY_NAME = {
    "GET": Command.GET,
    "SET": Command.SET,
    "SAVE": Command.SAVE,
    "LOAD": Command.LOAD,
    "DEFAULTS": Command.DEFAULTS,
    "CALIBRATE_IMU": Command.CALIBRATE_IMU,
}


RADIO_CHANNELS = {
    "THROTTLE": 0,
    "ROLL": 1,
    "PITCH": 2,
    "YAW": 3,
    "AUX1": 4,
    "AUX2": 5,
}


RADIO_CHANNEL_NAMES = {
    value: name
    for name, value in RADIO_CHANNELS.items()
}


PRIMARY_RADIO_CHANNELS = tuple(
    RADIO_CHANNELS.values()
)


RADIO_CONFIG_IDS = {
    "THROTTLE": (1, 2, 3),
    "ROLL": (6, 7, 8),
    "PITCH": (11, 12, 13),
    "YAW": (16, 17, 18),
    "AUX1": (21, 22, 23),
    "AUX2": (26, 27, 28),
}


MIN_ENDPOINT_SAMPLES = 10
MIN_CENTER_SAMPLES = 20
MIN_CHANNEL_SPAN_US = 400
MIN_TRIM_MARGIN_US = 100
MAX_CENTER_SPREAD_US = 30
THROTTLE_CUT_WARNING_US = 1050


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
    "ELEVON_WITH_RUDDER": 2,
    "ELEVON_NO_RUDDER": 3,
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
    
    21: "RC_AUX1_MIN",
    22: "RC_AUX1_TRIM",
    23: "RC_AUX1_MAX",
    24: "RC_AUX1_DB",
    25: "RC_AUX1_REVERSE",
    
    26: "RC_AUX2_MIN",
    27: "RC_AUX2_TRIM",
    28: "RC_AUX2_MAX",
    29: "RC_AUX2_DB",
    30: "RC_AUX2_REVERSE",

    31: "SRV_THROTTLE_MIN",
    32: "SRV_THROTTLE_TRIM",
    33: "SRV_THROTTLE_MAX",
    34: "SRV_THROTTLE_REVERSE",

    35: "SRV_ROLL_MIN",
    36: "SRV_ROLL_TRIM",
    37: "SRV_ROLL_MAX",
    38: "SRV_ROLL_REVERSE",

    39: "SRV_PITCH_MIN",
    40: "SRV_PITCH_TRIM",
    41: "SRV_PITCH_MAX",
    42: "SRV_PITCH_REVERSE",

    43: "SRV_YAW_MIN",
    44: "SRV_YAW_TRIM",
    45: "SRV_YAW_MAX",
    46: "SRV_YAW_REVERSE",

    47: "FLIGHT_MAX_ROLL_RATE_DEGS",
    48: "FLIGHT_MAX_PITCH_RATE_DEGS",
    49: "FLIGHT_MAX_YAW_RATE_DEGS",

    50: "FLIGHT_MAX_ROLL_ANGLE_DEGS",
    51: "FLIGHT_MAX_PITCH_ANGLE_DEGS",

    52: "FLIGHT_ROLL_ANGLE_KP",
    53: "FLIGHT_PITCH_ANGLE_KP",

    54: "FLIGHT_FLAPERON_SCALE_FACTOR",
    55: "FLIGHT_MAX_FLAPERON",

    56: "FLIGHT_REVERSE_RUDDER_MIX",
    57: "FLIGHT_RUDDER_MIX_SCALE_FACTOR",

    58: "PIDF_ROLL_KP",
    69: "PIDF_ROLL_KI",
    60: "PIDF_ROLL_KD",
    61: "PIDF_ROLL_KF",
    62: "PIDF_ROLL_I_WINDUP_MAX",

    63: "PIDF_PITCH_KP",
    64: "PIDF_PITCH_KI",
    65: "PIDF_PITCH_KD",
    66: "PIDF_PITCH_KF",
    67: "PIDF_PITCH_I_WINDUP_MAX",

    68: "PIDF_YAW_KP",
    69: "PIDF_YAW_KI",
    70: "PIDF_YAW_KD",
    71: "PIDF_YAW_KF",
    72: "PIDF_YAW_I_WINDUP_MAX",

    73: "IMU_ACC_BIAS_X",
    74: "IMU_ACC_BIAS_Y",
    75: "IMU_ACC_BIAS_Z",

    76: "IMU_GYRO_BIAS_X",
    77: "IMU_GYRO_BIAS_Y",
    78: "IMU_GYRO_BIAS_Z",

    79: "IMU_CALIBRATED",

    80: "CONTROL_SLEW_RATE",
    81: "CONTROL_LPF_FREQ",
    82: "CONTROL_RESOLUTION",
    83: "CONTROL_DT",
}


CONFIG_COUNT = 84


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

    # Aux1RC
    21: ValueType.INT16,
    22: ValueType.INT16,
    23: ValueType.INT16,
    24: ValueType.UINT8,
    25: ValueType.BOOL,

    # Aux2RC
    26: ValueType.INT16,
    27: ValueType.INT16,
    28: ValueType.INT16,
    29: ValueType.UINT8,
    30: ValueType.BOOL,

    # Throttle SrvConfig
    31: ValueType.INT16,
    32: ValueType.INT16,
    33: ValueType.INT16,
    34: ValueType.BOOL,

    # Roll SrvConfig
    35: ValueType.INT16,
    36: ValueType.INT16,
    37: ValueType.INT16,
    38: ValueType.BOOL,

    # Pitch SrvConfig
    39: ValueType.INT16,
    40: ValueType.INT16,
    41: ValueType.INT16,
    42: ValueType.BOOL,

    # Yaw SrvConfig
    43: ValueType.INT16,
    44: ValueType.INT16,
    45: ValueType.INT16,
    46: ValueType.BOOL,

    # FlightConfig
    47: ValueType.INT16,
    48: ValueType.INT16,
    49: ValueType.INT16,

    50: ValueType.INT16,
    51: ValueType.INT16,

    52: ValueType.FLOAT,
    53: ValueType.FLOAT,

    54: ValueType.FLOAT,
    55: ValueType.INT16,

    56: ValueType.BOOL,
    57: ValueType.FLOAT,

    # RollPIDF
    58: ValueType.FLOAT,
    59: ValueType.FLOAT,
    60: ValueType.FLOAT,
    61: ValueType.FLOAT,
    62: ValueType.FLOAT,

    # PitchPIDF
    63: ValueType.FLOAT,
    64: ValueType.FLOAT,
    65: ValueType.FLOAT,
    66: ValueType.FLOAT,
    67: ValueType.FLOAT,

    # YawPIDF
    68: ValueType.FLOAT,
    69: ValueType.FLOAT,
    70: ValueType.FLOAT,
    71: ValueType.FLOAT,
    72: ValueType.FLOAT,

    # IMUConfig
    73: ValueType.FLOAT,
    74: ValueType.FLOAT,
    75: ValueType.FLOAT,

    76: ValueType.FLOAT,
    77: ValueType.FLOAT,
    78: ValueType.FLOAT,

    79: ValueType.BOOL,

    # ControlConfig
    80: ValueType.UINT16,
    81: ValueType.UINT16,
    82: ValueType.INT16,
    83: ValueType.FLOAT,
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

    if text in SERIAL_COMMAND_IDS_BY_NAME:
        return SERIAL_COMMAND_IDS_BY_NAME[text]

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
        ELEVON_RUDDER
        ELEVON_NO_RUDDER
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
# Machine-readable response handling
# ============================================================

class RadioCalibrationError(RuntimeError):
    pass


def parse_response_packet(packet):
    if len(packet) != PACKET_SIZE:
        raise RadioCalibrationError(
            "Invalid packet length."
        )

    if packet[0] != START_BYTE:
        raise RadioCalibrationError(
            "Invalid packet start byte."
        )

    calculated_checksum = additive_checksum(
        packet[:8]
    )

    if packet[8] != calculated_checksum:
        raise RadioCalibrationError(
            "Packet checksum mismatch."
        )

    return {
        "command": packet[1],
        "param_id": packet[2],
        "value_type": packet[3],
        "value": decode_value(
            packet[3],
            packet[4:8]
        ),
    }


def wait_for_ack(
    ser,
    original_command,
    timeout=2.0,
    radio_handler=None
):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        packet = receive_packet(
            ser,
            timeout=min(0.25, remaining)
        )

        if packet is None:
            continue

        response = parse_response_packet(packet)
        command = response["command"]

        if command == Command.RADIO_VALUE:
            if radio_handler is not None:
                radio_handler(response)
            continue

        if (
            command == Command.ACK
            and response["param_id"] == original_command
        ):
            return

        if (
            command == Command.NACK
            and response["param_id"] == original_command
        ):
            name = COMMAND_NAMES.get(
                original_command,
                f"0x{original_command:02X}"
            )

            raise RadioCalibrationError(
                f"XPilot rejected {name}."
            )

    name = COMMAND_NAMES.get(
        original_command,
        f"0x{original_command:02X}"
    )

    raise RadioCalibrationError(
        f"Timed out waiting for {name} acknowledgement."
    )


def send_command_and_wait_for_ack(
    ser,
    command,
    packet=None,
    radio_handler=None
):
    if packet is None:
        packet = build_packet(
            command=command
        )

    ser.write(packet)

    wait_for_ack(
        ser,
        command,
        radio_handler=radio_handler
    )


def request_config_value(ser, param_id):
    ser.write(
        build_packet(
            command=Command.GET,
            param_id=param_id
        )
    )

    deadline = time.monotonic() + 2.0

    while time.monotonic() < deadline:
        packet = receive_packet(
            ser,
            timeout=min(
                0.25,
                deadline - time.monotonic()
            )
        )

        if packet is None:
            continue

        response = parse_response_packet(packet)

        if (
            response["command"] == Command.VALUE
            and response["param_id"] == param_id
        ):
            return response["value"]

        if (
            response["command"] == Command.NACK
            and response["param_id"] == Command.GET
        ):
            raise RadioCalibrationError(
                f"GET {CONFIG_NAMES[param_id]} was rejected."
            )

    raise RadioCalibrationError(
        f"Timed out reading {CONFIG_NAMES[param_id]}."
    )


def set_config_value(ser, param_id, value):
    value_type = CONFIG_TYPES[param_id]
    encoded = encode_user_value(
        param_id,
        value_type,
        str(value)
    )

    packet = build_packet(
        command=Command.SET,
        param_id=param_id,
        value_type=value_type,
        value=encoded
    )

    send_command_and_wait_for_ack(
        ser,
        Command.SET,
        packet=packet
    )


def enter_pressed():
    if sys.platform.startswith("win"):
        import msvcrt

        pressed = False

        while msvcrt.kbhit():
            char = msvcrt.getwch()

            if char in ("\r", "\n"):
                pressed = True

        return pressed

    import select

    readable, _, _ = select.select(
        [sys.stdin],
        [],
        [],
        0
    )

    if readable:
        sys.stdin.readline()
        return True

    return False


def capture_radio_phase(ser, captured_channels):
    samples = {
        channel: []
        for channel in captured_channels
    }

    def record_radio_value(response):
        channel = response["param_id"]

        if channel not in PRIMARY_RADIO_CHANNELS:
            raise RadioCalibrationError(
                f"Invalid streamed radio channel {channel}."
            )

        if response["value_type"] != ValueType.UINT16:
            raise RadioCalibrationError(
                "Streamed radio value has the wrong type."
            )

        pwm = response["value"]

        if not 600 <= pwm <= 2400:
            name = RADIO_CHANNEL_NAMES[channel]
            raise RadioCalibrationError(
                f"Invalid {name} PWM value: {pwm} us."
            )

        if channel in samples:
            samples[channel].append(pwm)

    ser.reset_input_buffer()

    send_command_and_wait_for_ack(
        ser,
        Command.START_RADIO_STREAM
    )

    streaming = True

    try:
        while not enter_pressed():
            packet = receive_packet(
                ser,
                timeout=0.1
            )

            if packet is None:
                continue

            response = parse_response_packet(packet)

            if response["command"] == Command.RADIO_VALUE:
                record_radio_value(response)
                continue

            if (
                response["command"] == Command.NACK
                and response["param_id"]
                == Command.START_RADIO_STREAM
            ):
                streaming = False

                raise RadioCalibrationError(
                    "Radio stream stopped because one or more "
                    "receiver channels became invalid, stale "
                    f"or number of requested radio channels exceeded {len(RADIO_CHANNELS)}."
                )

    finally:
        if streaming:
            ser.write(
                build_packet(
                    command=Command.STOP_RADIO_STREAM
                )
            )

            # Stream packets already queued ahead of the STOP ACK are
            # still valid members of the phase and are recorded here.
            wait_for_ack(
                ser,
                Command.STOP_RADIO_STREAM,
                radio_handler=record_radio_value
            )

        ser.reset_input_buffer()

    return samples


def print_radio_calibration(calibration):
    print(
        "\nProposed radio calibration"
    )
    print(
        "---------------------------------------------"
    )
    print(
        f"{'CHANNEL':<10} "
        f"{'MIN':>7} "
        f"{'TRIM':>7} "
        f"{'MAX':>7}"
    )
    print(
        "---------------------------------------------"
    )

    for name in RADIO_CHANNELS:
        values = calibration[name]

        print(
            f"{name:<10} "
            f"{values['min']:>7} "
            f"{values['trim']:>7} "
            f"{values['max']:>7}"
        )

    print()


def calculate_radio_calibration(endpoint_samples, center_samples):
    calibration = {}

    for channel in PRIMARY_RADIO_CHANNELS:
        count = len(endpoint_samples[channel])

        if count < MIN_ENDPOINT_SAMPLES:
            name = RADIO_CHANNEL_NAMES[channel]

            raise RadioCalibrationError(
                f"Not enough endpoint samples for {name}: "
                f"received {count}, need at least "
                f"{MIN_ENDPOINT_SAMPLES}."
            )

    throttle_samples = endpoint_samples[
        RADIO_CHANNELS["THROTTLE"]
    ]

    throttle_min = min(throttle_samples)
    throttle_max = max(throttle_samples)

    calibration["THROTTLE"] = {
        "min": throttle_min,
        "trim": (
            throttle_min
            + throttle_max
        ) // 2,
        "max": throttle_max,
    }

    for name in ("ROLL", "PITCH", "YAW", "AUX1", "AUX2"):
        channel = RADIO_CHANNELS[name]
        endpoints = endpoint_samples[channel]
        centered = center_samples[channel]

        if len(centered) < MIN_CENTER_SAMPLES:
            raise RadioCalibrationError(
                f"Not enough centered samples for {name}: "
                f"received {len(centered)}, need at least "
                f"{MIN_CENTER_SAMPLES}."
            )

        calibration[name] = {
            "min": min(endpoints),
            "trim": int(round(
                statistics.median(centered)
            )),
            "max": max(endpoints),
        }

    for name, values in calibration.items():
        minimum = values["min"]
        trim = values["trim"]
        maximum = values["max"]

        if not 600 <= minimum <= 2400:
            raise RadioCalibrationError(
                f"{name} minimum is outside the accepted range."
            )

        if not 600 <= maximum <= 2400:
            raise RadioCalibrationError(
                f"{name} maximum is outside the accepted range."
            )

        if not minimum < trim < maximum:
            raise RadioCalibrationError(
                f"{name} does not satisfy MIN < TRIM < MAX."
            )

        if maximum - minimum < MIN_CHANNEL_SPAN_US:
            raise RadioCalibrationError(
                f"{name} span is too small: "
                f"{maximum - minimum} us."
            )

        if name != "THROTTLE":
            if (
                trim - minimum < MIN_TRIM_MARGIN_US
                or maximum - trim < MIN_TRIM_MARGIN_US
            ):
                raise RadioCalibrationError(
                    f"{name} trim is too close to an endpoint."
                )

    return calibration


def apply_radio_calibration(ser, calibration):
    proposed = {}

    for name, config_ids in RADIO_CONFIG_IDS.items():
        values = calibration[name]

        proposed[config_ids[0]] = values["min"]
        proposed[config_ids[1]] = values["trim"]
        proposed[config_ids[2]] = values["max"]

    original = {
        param_id: request_config_value(
            ser,
            param_id
        )
        for param_id in proposed
    }

    try:
        for param_id, value in proposed.items():
            set_config_value(
                ser,
                param_id,
                value
            )

        for param_id, expected in proposed.items():
            actual = request_config_value(
                ser,
                param_id
            )

            if actual != expected:
                raise RadioCalibrationError(
                    f"Verification failed for "
                    f"{CONFIG_NAMES[param_id]}: "
                    f"expected {expected}, received {actual}."
                )

    except Exception:
        rollback_errors = []

        for param_id, value in original.items():
            try:
                set_config_value(
                    ser,
                    param_id,
                    value
                )

            except Exception as error:
                rollback_errors.append(
                    f"{CONFIG_NAMES[param_id]}: {error}"
                )

        if rollback_errors:
            raise RadioCalibrationError(
                "Calibration failed and rollback was incomplete: "
                + "; ".join(rollback_errors)
            )

        raise


def run_radio_calibration(ser):
    print(f"""
{Color.MAGENTA}Radio Input Calibration{Color.RESET}
=======================

{Color.RED}Disconnect propulsion before continuing.{Color.RESET}
Keep throttle cut OFF throughout endpoint capture.
The calibration updates RAM only; it will not write EEPROM.
""")

    input(
        f"Press {Color.GREEN}Enter{Color.RESET} after the aircraft is safe "
        "and the transmitter is ready..."
    )

    print(f"""
Move all channels through their full normal ranges.
Briefly hold every control at each endpoint.

Press {Color.GREEN}Enter{Color.RESET} when endpoint capture is complete.
""")

    endpoint_samples = capture_radio_phase(
        ser,
        PRIMARY_RADIO_CHANNELS
    )

    print(f"""
    {Color.GREEN}Input captured{Color.RESET}
    """)

    input(f"""
Release all channels and leave them in the centered position.
Throttle position is ignored during this phase.

Press {Color.GREEN}Enter{Color.RESET} when all controls are centered
and ready for capture...
""")

    print(f"""
Capture started.
Keep all controls centered.

Press {Color.GREEN}Enter{Color.RESET} when center capture is complete.
""")

    center_samples = capture_radio_phase(
        ser,
        (
            RADIO_CHANNELS["ROLL"],
            RADIO_CHANNELS["PITCH"],
            RADIO_CHANNELS["YAW"],
            RADIO_CHANNELS["AUX1"],
            RADIO_CHANNELS["AUX2"],
        )
    )

    print(f"""
    {Color.GREEN}Input captured{Color.RESET}
    """)

    calibration = calculate_radio_calibration(
        endpoint_samples,
        center_samples
    )

    print_radio_calibration(
        calibration
    )

    if (
        calibration["THROTTLE"]["min"]
        < THROTTLE_CUT_WARNING_US
    ):
        print(
            f"{Color.YELLOW}WARNING: The throttle minimum is below normal range\n"
        )

    confirmation = input(
        "Apply these values to the active RAM configuration? "
        "[y/N]: "
    ).strip().lower()

    if confirmation not in ("y", "yes"):
        print(
            f"{Color.YELLOW}Calibration discarded. "
            f"No configuration values were changed.{Color.RESET}"
        )
        return

    apply_radio_calibration(
        ser,
        calibration
    )

    print(
        f"{Color.GREEN}Radio calibration applied and verified "
        f"in RAM.{Color.RESET}"
    )
    print(
        "Run SAVE separately to persist it to EEPROM."
    )


UTILITY_COMMANDS = {
    "CALIBRATE_RADIO": run_radio_calibration,
}
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

    for name, command in SERIAL_COMMAND_IDS_BY_NAME.items():
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


{Color.MAGENTA}RADIO CALIBRATION{Color.RESET}
-----------------

    CALIBRATE_RADIO

Streams throttle, roll, pitch, and yaw while you capture
their normal endpoints, then captures centered roll,
pitch, and yaw trims. Results are displayed for approval
before the active RAM configuration is changed.

This command never writes EEPROM. Run SAVE separately
after inspecting and testing the applied calibration.


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
        "Type 'help' for commands."
    )
    
    history = FileHistory(".xp_serial_history")

    try:
        while True:
            line = prompt("\nXpilot> ", history=history).strip()

            if not line:
                continue

            command_text = line.upper()

            if command_text in (
                "QUIT",
                "EXIT",
            ):
                break

            if command_text == "HELP":
                print_help()
                continue

            if command_text == "CONFIG":
                print_config_table()
                continue

            if command_text == "AIRFRAMES":
                print_airframe_table()
                continue

            if command_text == "COMMANDS":
                print_command_table()
                continue

            if command_text == "PORTS":
                print_ports(
                    get_serial_ports()
                )
                continue


            utility_command = UTILITY_COMMANDS.get(command_text)

            if utility_command is not None:
                try:
                    utility_command(
                        ser
                    )

                except RadioCalibrationError as error:
                    print(
                        f"{Color.RED}Radio calibration failed: "
                        f"{error}{Color.RESET}"
                    )

                except serial.SerialException as error:
                    print(
                        f"{Color.RED}Serial error: "
                        f"{error}{Color.RESET}"
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
