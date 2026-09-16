"""
Modbus RTU protocol handling and relay controller logic.
"""
from typing import Dict, Tuple, Optional, List
import math
import threading
import serial
import serial.tools.list_ports


def calculate_crc(data: bytes) -> bytes:
    """
    Compute Modbus RTU 16-bit CRC (polynomial 0xA001).
    Returns 2 bytes in little-endian order (Low byte first, High byte second).
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_relay_command(address: int, relay_index: int, state: bool) -> bytes:
    """
    Construct an 8-byte Modbus RTU single coil write command (Function 0x05).

    :param address: Board address between 1 and 255 (0x01 to 0xFF).
    :param relay_index: Relay index between 0 and 7.
    :param state: True to turn ON (0xFF00), False to turn OFF (0x0000).
    :return: 8-byte command frame with CRC-16.
    """
    if not (1 <= address <= 255):
        raise ValueError(f"Address must be between 1 and 255, got {address}")
    if not (0 <= relay_index <= 7):
        raise ValueError(f"Relay index must be between 0 and 7, got {relay_index}")

    value_hi = 0xFF if state else 0x00
    value_lo = 0x00

    frame = bytes([address, 0x05, 0x00, relay_index, value_hi, value_lo])
    crc = calculate_crc(frame)
    return frame + crc


def build_read_inputs_command(address: int, start: int = 0, count: int = 8, function_code: int = 0x02) -> bytes:
    """
    Construct a Modbus RTU read discrete inputs command (Function 0x02) or read coils (0x01).

    :param address: Board address between 1 and 255 (0x01 to 0xFF).
    :param start: Starting input address (default 0).
    :param count: Number of input points to read (default 8 for 8 input channels).
    :param function_code: Function code (0x02 for Read Discrete Inputs, 0x01 for Read Coils).
    :return: 8-byte command frame with CRC-16.
    """
    if not (1 <= address <= 255):
        raise ValueError(f"Address must be between 1 and 255, got {address}")
    if not (1 <= count <= 2000):
        raise ValueError(f"Count must be between 1 and 2000, got {count}")
    if not (0 <= start <= 65535):
        raise ValueError(f"Start address must be between 0 and 65535, got {start}")

    frame = bytes([
        address,
        function_code,
        (start >> 8) & 0xFF,
        start & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF,
    ])
    crc = calculate_crc(frame)
    return frame + crc


def parse_read_inputs_response(
    response: bytes, expected_address: Optional[int] = None, expected_count: int = 8
) -> Optional[List[bool]]:
    """
    Parse a Modbus RTU response for read inputs/coils (Function 0x02 / 0x01).

    Expected response frame format:
    [Address, FunctionCode, ByteCount, DataBytes..., CRC_Lo, CRC_Hi]

    :param response: Raw response bytes received from device.
    :param expected_address: Optional expected slave address to validate.
    :param expected_count: Number of input states expected (default: 8).
    :return: List of booleans representing input states (True = Active/High, False = Inactive/Low),
             or None if frame is invalid / CRC mismatch.
    """
    if len(response) < 5:
        return None

    # Validate CRC
    data_part = response[:-2]
    expected_crc = response[-2:]
    if calculate_crc(data_part) != expected_crc:
        return None

    addr = response[0]
    func = response[1]
    byte_count = response[2]

    if expected_address is not None and addr != expected_address:
        return None

    # Error response check (Function code with MSB set, e.g., 0x82)
    if func & 0x80:
        return None

    if len(response) != 3 + byte_count + 2:
        return None

    data_bytes = response[3 : 3 + byte_count]
    states: List[bool] = []
    for i in range(expected_count):
        byte_idx = i // 8
        bit_idx = i % 8
        if byte_idx < len(data_bytes):
            is_active = bool((data_bytes[byte_idx] >> bit_idx) & 0x01)
            states.append(is_active)
        else:
            states.append(False)

    return states


def parse_hex_string(data: bytes) -> str:
    """Format bytes as uppercase space-separated hex string."""
    return data.hex(" ").upper()


class RelayController:
    """
    Manages serial connection, board addressing, relay states, and digital input polling.
    """

    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        # Relay State tracking: (address, relay_index) -> bool (True=ON, False=OFF)
        self.relay_states: Dict[Tuple[int, int], bool] = {}
        # Input State tracking: (address, input_index) -> bool (True=Active, False=Inactive)
        self.input_states: Dict[Tuple[int, int], bool] = {}

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self.serial_port is not None and self.serial_port.is_open

    def get_relay_state(self, address: int, relay_index: int) -> bool:
        return self.relay_states.get((address, relay_index), False)

    def set_local_state(self, address: int, relay_index: int, state: bool) -> None:
        self.relay_states[(address, relay_index)] = state

    def get_input_state(self, address: int, input_index: int) -> bool:
        return self.input_states.get((address, input_index), False)

    def set_input_state(self, address: int, input_index: int, state: bool) -> None:
        self.input_states[(address, input_index)] = state

    def connect(
        self,
        port: str,
        baudrate: int = 9600,
        bytesize: int = 8,
        stopbits: float = 1,
        parity: str = "N",
        rtscts: bool = False,
        xonxoff: bool = False,
        dsrdtr: bool = False,
        timeout: float = 0.5,
    ) -> None:
        """Open serial port connection."""
        with self._lock:
            if self.serial_port is not None and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None

            # Map stopbits
            stopbits_map = {
                1: serial.STOPBITS_ONE,
                1.5: serial.STOPBITS_ONE_POINT_FIVE,
                2: serial.STOPBITS_TWO,
            }
            sb = stopbits_map.get(stopbits, serial.STOPBITS_ONE)

            # Map parity
            parity_map = {
                "N": serial.PARITY_NONE,
                "None": serial.PARITY_NONE,
                "E": serial.PARITY_EVEN,
                "Even": serial.PARITY_EVEN,
                "O": serial.PARITY_ODD,
                "Odd": serial.PARITY_ODD,
                "M": serial.PARITY_MARK,
                "Mark": serial.PARITY_MARK,
                "S": serial.PARITY_SPACE,
                "Space": serial.PARITY_SPACE,
            }
            par = parity_map.get(parity, serial.PARITY_NONE)

            # Map bytesize
            bytesize_map = {
                5: serial.FIVEBITS,
                6: serial.SIXBITS,
                7: serial.SEVENBITS,
                8: serial.EIGHTBITS,
            }
            bs = bytesize_map.get(bytesize, serial.EIGHTBITS)

            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=bs,
                parity=par,
                stopbits=sb,
                rtscts=rtscts,
                xonxoff=xonxoff,
                dsrdtr=dsrdtr,
                timeout=timeout,
                write_timeout=timeout,
            )

    def disconnect(self) -> None:
        """Close serial port connection."""
        with self._lock:
            if self.serial_port is not None:
                try:
                    if self.serial_port.is_open:
                        self.serial_port.close()
                finally:
                    self.serial_port = None

    def send_command(self, cmd: bytes, expected_length: int = 8) -> bytes:
        """
        Send raw command bytes over serial port and read response.
        Thread-safe serial transmission and reception.
        """
        with self._lock:
            if self.serial_port is None or not self.serial_port.is_open:
                raise RuntimeError("Serial port is not connected.")

            self.serial_port.reset_input_buffer()
            self.serial_port.write(cmd)
            self.serial_port.flush()

            response = self.serial_port.read(expected_length)
            return response

    def set_relay(self, address: int, relay_index: int, state: bool) -> Tuple[bytes, bytes]:
        """
        Build and send relay command.
        Returns (command_bytes, response_bytes).
        """
        cmd = build_relay_command(address, relay_index, state)
        response = b""
        if self.is_connected:
            response = self.send_command(cmd, expected_length=8)
        self.set_local_state(address, relay_index, state)
        return cmd, response

    def read_inputs(
        self, address: int, count: int = 8, function_code: int = 0x02
    ) -> Tuple[bytes, bytes, Optional[List[bool]]]:
        """
        Build and send read inputs command, and parse response.
        Returns (command_bytes, response_bytes, Optional[List[bool]]).
        """
        cmd = build_read_inputs_command(address=address, start=0, count=count, function_code=function_code)
        response = b""
        states: Optional[List[bool]] = None

        expected_byte_count = math.ceil(count / 8)
        expected_resp_len = 3 + expected_byte_count + 2  # [Addr, Func, ByteCount, Data..., CRC_L, CRC_H]

        if self.is_connected:
            response = self.send_command(cmd, expected_length=expected_resp_len)
            states = parse_read_inputs_response(response, expected_address=address, expected_count=count)
            if states is not None:
                for idx, val in enumerate(states):
                    self.set_input_state(address, idx, val)

        return cmd, response, states

    @staticmethod
    def list_available_ports():
        """Return list of available COM port names and descriptions."""
        return serial.tools.list_ports.comports()
