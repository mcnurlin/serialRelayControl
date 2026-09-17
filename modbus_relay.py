"""
Modbus RTU protocol handling and relay controller logic.
"""
from typing import Dict, Tuple, Optional, List, Callable, Any
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


def build_read_analog_inputs_command(
    address: int, start: int = 0, count: int = 8, function_code: int = 0x04
) -> bytes:
    """
    Construct a Modbus RTU read analog input registers command (Function 0x04 or 0x03).

    :param address: Board address between 1 and 255 (0x01 to 0xFF).
    :param start: Starting register address (default 0).
    :param count: Number of 16-bit analog registers to read (default 8 for 8 AI channels).
    :param function_code: Function code (0x04 for Read Input Registers, 0x03 for Read Holding Registers).
    :return: 8-byte command frame with CRC-16.
    """
    if not (1 <= address <= 255):
        raise ValueError(f"Address must be between 1 and 255, got {address}")
    if not (1 <= count <= 125):
        raise ValueError(f"Register count must be between 1 and 125, got {count}")
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


def parse_read_analog_inputs_response(
    response: bytes, expected_address: Optional[int] = None, expected_count: int = 8
) -> Optional[List[int]]:
    """
    Parse a Modbus RTU response for read analog input/holding registers (Function 0x04 / 0x03).

    Expected response frame format:
    [Address, FunctionCode, ByteCount (2*N), Reg0_Hi, Reg0_Lo, ..., RegN_Hi, RegN_Lo, CRC_Lo, CRC_Hi]

    :param response: Raw response bytes received from device.
    :param expected_address: Optional expected slave address to validate.
    :param expected_count: Number of 16-bit registers expected (default: 8).
    :return: List of 16-bit integer values, or None if frame is invalid / CRC mismatch.
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

    # Error response check (Function code with MSB set, e.g., 0x84 or 0x83)
    if func & 0x80:
        return None

    if byte_count != expected_count * 2 or len(response) != 3 + byte_count + 2:
        return None

    registers: List[int] = []
    for i in range(expected_count):
        offset = 3 + i * 2
        val = (response[offset] << 8) | response[offset + 1]
        registers.append(val)

    return registers


def raw_to_ma(raw_value: int, scale_mode: str = "auto") -> float:
    """
    Convert raw 16-bit register value to mA for 4-20mA current loop inputs.

    Supported scaling modes:
    - "auto": Auto-detect scaling format:
        * 3500 <= raw <= 21000: value in microamps / 1000.0 (e.g., 4000 = 4.00 mA, 20000 = 20.00 mA)
        * 0 <= raw <= 4095: 12-bit ADC mapping -> 4.0 + (raw / 4095.0) * 16.0 mA
        * 0 <= raw <= 10000: 0.001 * raw mA or 0.002 * raw mA
        * raw > 21000: 16-bit full scale mapping (4.0 + (raw / 65535.0) * 16.0 mA)
    - "4000-20000": raw / 1000.0 mA (4000 = 4mA, 20000 = 20mA)
    - "0-20000": raw / 1000.0 mA (0 = 0mA, 4000 = 4mA, 20000 = 20mA)
    - "12bit_adc": 4.0 + (raw / 4095.0) * 16.0 mA
    - "10bit_adc": 4.0 + (raw / 1023.0) * 16.0 mA
    - "16bit_adc": 4.0 + (raw / 65535.0) * 16.0 mA

    :param raw_value: Unsigned 16-bit register value.
    :param scale_mode: Conversion preset mode.
    :return: Float value representing current in milliamperes (mA).
    """
    if scale_mode == "4000-20000":
        return round(raw_value / 1000.0, 3)
    elif scale_mode == "0-20000":
        return round(raw_value / 1000.0, 3)
    elif scale_mode == "12bit_adc":
        clamped = max(0, min(4095, raw_value))
        return round(4.0 + (clamped / 4095.0) * 16.0, 3)
    elif scale_mode == "10bit_adc":
        clamped = max(0, min(1023, raw_value))
        return round(4.0 + (clamped / 1023.0) * 16.0, 3)
    elif scale_mode == "16bit_adc":
        clamped = max(0, min(65535, raw_value))
        return round(4.0 + (clamped / 65535.0) * 16.0, 3)
    else:  # "auto"
        if 3500 <= raw_value <= 21000:
            return round(raw_value / 1000.0, 3)
        elif raw_value <= 4095:
            return round(4.0 + (raw_value / 4095.0) * 16.0, 3)
        elif raw_value <= 10000:
            return round(4.0 + (raw_value / 10000.0) * 16.0, 3)
        else:
            return round(4.0 + (raw_value / 65535.0) * 16.0, 3)


def parse_hex_string(data: bytes) -> str:
    """Format bytes as uppercase space-separated hex string."""
    return data.hex(" ").upper()


def format_device_response_string(data: Optional[bytes]) -> str:
    """
    Format the raw response frame or string received from a device into a human-readable representation.
    Returns the space-separated hex bytes, appended with printable ASCII text if alphanumeric characters are present.
    """
    if not data:
        return "(No response string)"

    hex_str = parse_hex_string(data)

    # Check for printable ASCII characters in the response
    printable_chars = [chr(b) if (32 <= b <= 126) else "." for b in data]
    clean_ascii = "".join(printable_chars).strip(".")

    # If there are 2 or more alphanumeric characters in ASCII form, include decoded string
    if len(clean_ascii) >= 2 and any(c.isalnum() for c in clean_ascii):
        return f"{hex_str} [ASCII: '{clean_ascii}']"
    return hex_str


def is_valid_modbus_response(response: bytes, expected_address: Optional[int] = None) -> bool:
    """
    Check if received bytes form a valid Modbus RTU response frame with correct CRC.

    :param response: Raw response bytes.
    :param expected_address: Optional expected slave address.
    :return: True if frame is valid Modbus RTU response and CRC matches, False otherwise.
    """
    if len(response) < 5:
        return False
    if expected_address is not None and response[0] != expected_address:
        return False
    data_part = response[:-2]
    expected_crc = response[-2:]
    return calculate_crc(data_part) == expected_crc


class RelayController:
    """
    Manages serial connection, board addressing, relay states, digital inputs, and analog inputs.
    """

    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        # Relay State tracking: (address, relay_index) -> bool (True=ON, False=OFF)
        self.relay_states: Dict[Tuple[int, int], bool] = {}
        # Input State tracking: (address, input_index) -> bool (True=Active, False=Inactive)
        self.input_states: Dict[Tuple[int, int], bool] = {}
        # Analog State tracking: (address, ai_index) -> int (raw 16-bit register value)
        self.analog_states: Dict[Tuple[int, int], int] = {}
        # Last network scan device responses: address -> raw response bytes
        self.last_scan_device_responses: Dict[int, bytes] = {}

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

    def get_analog_state(self, address: int, ai_index: int) -> int:
        return self.analog_states.get((address, ai_index), 0)

    def set_analog_state(self, address: int, ai_index: int, value: int) -> None:
        self.analog_states[(address, ai_index)] = value

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

    def read_analog_inputs(
        self, address: int, count: int = 8, function_code: int = 0x04
    ) -> Tuple[bytes, bytes, Optional[List[int]]]:
        """
        Build and send read analog inputs/registers command (Function 0x04 or 0x03), and parse response.
        Returns (command_bytes, response_bytes, Optional[List[int]]).
        """
        cmd = build_read_analog_inputs_command(address=address, start=0, count=count, function_code=function_code)
        response = b""
        registers: Optional[List[int]] = None

        expected_byte_count = count * 2
        expected_resp_len = 3 + expected_byte_count + 2  # [Addr, Func, ByteCount(2*N), Data..., CRC_L, CRC_H]

        if self.is_connected:
            response = self.send_command(cmd, expected_length=expected_resp_len)
            registers = parse_read_analog_inputs_response(response, expected_address=address, expected_count=count)
            if registers is not None:
                for idx, val in enumerate(registers):
                    self.set_analog_state(address, idx, val)

        return cmd, response, registers

    def _probe_modbus_command(self, cmd: bytes, expected_address: int, timeout: float) -> Optional[bytes]:
        """
        Send a Modbus RTU probe frame and check for any valid response or exception from expected_address.
        Returns the raw response frame bytes if valid, otherwise None.
        """
        if self.serial_port is None or not self.serial_port.is_open:
            return None

        old_timeout = getattr(self.serial_port, "timeout", None)
        try:
            if hasattr(self.serial_port, "timeout"):
                self.serial_port.timeout = timeout
            if hasattr(self.serial_port, "reset_input_buffer"):
                self.serial_port.reset_input_buffer()
            self.serial_port.write(cmd)
            if hasattr(self.serial_port, "flush"):
                self.serial_port.flush()

            # Read initial 3-byte header
            header = self.serial_port.read(3)
            if not header or len(header) < 3:
                return None

            # If mock or buffer already returned full frame in header
            if len(header) >= 5 and header[0] == expected_address and calculate_crc(header[:-2]) == header[-2:]:
                return header

            addr = header[0]
            func = header[1]
            if addr != expected_address:
                return None

            # Exception response (5 bytes total: [addr, func|0x80, exception_code, crc_l, crc_h])
            if func & 0x80:
                tail = self.serial_port.read(2)
                frame = header + (tail or b"")
                if len(frame) == 5 and calculate_crc(frame[:-2]) == frame[-2:]:
                    return frame
                return None

            # Normal read functions (0x01, 0x02, 0x03, 0x04)
            if func in (0x01, 0x02, 0x03, 0x04):
                byte_count = header[2]
                if byte_count > 250:
                    return None
                tail = self.serial_port.read(byte_count + 2)
                frame = header + (tail or b"")
                if len(frame) == (3 + byte_count + 2) and calculate_crc(frame[:-2]) == frame[-2:]:
                    return frame
                return None

            # Single coil / register write response (0x05, 0x06, 0x0F, 0x10) - 8 bytes total
            if func in (0x05, 0x06, 0x0F, 0x10):
                tail = self.serial_port.read(5)
                frame = header + (tail or b"")
                if len(frame) == 8 and calculate_crc(frame[:-2]) == frame[-2:]:
                    return frame
                return None

            # Fallback: check if any remaining bytes in buffer form valid frame
            in_waiting = getattr(self.serial_port, "in_waiting", 0)
            rest = self.serial_port.read(in_waiting) if in_waiting > 0 else b""
            frame = header + rest
            if len(frame) >= 5 and frame[0] == expected_address and calculate_crc(frame[:-2]) == frame[-2:]:
                return frame
            return None

        except Exception:
            return None
        finally:
            if self.serial_port is not None and hasattr(self.serial_port, "timeout") and old_timeout is not None:
                self.serial_port.timeout = old_timeout

    def probe_device(self, address: int, timeout: float = 0.06) -> Optional[bytes]:
        """
        Probe a device at the given address to verify presence on the Modbus network.
        Tries standard Modbus function codes (0x01 Read Coils for relay boards, 0x03 Read Holding Registers,
        0x02 Read Discrete Inputs, 0x04 Read Input Registers).
        Returns the raw response frame bytes as soon as a valid Modbus RTU response or exception frame with matching CRC is received.
        """
        if not (1 <= address <= 255):
            return None
        if not self.is_connected:
            return None

        # Probe queries to test in order:
        # 1. Function 0x01 (Read Coils, count=8) - Standard for 8-channel Modbus relay boards
        # 2. Function 0x03 (Read Holding Register 0, count=1) - For register-based controllers/sensors
        # 3. Function 0x02 (Read Discrete Inputs, count=8) - For digital input modules
        # 4. Function 0x04 (Read Input Register 0, count=1) - For analog input modules
        probe_commands = [
            build_read_inputs_command(address=address, start=0, count=8, function_code=0x01),
            build_read_analog_inputs_command(address=address, start=0, count=1, function_code=0x03),
            build_read_inputs_command(address=address, start=0, count=8, function_code=0x02),
            build_read_analog_inputs_command(address=address, start=0, count=1, function_code=0x04),
        ]

        with self._lock:
            for cmd in probe_commands:
                resp = self._probe_modbus_command(cmd, expected_address=address, timeout=timeout)
                if resp is not None:
                    return resp

        return None

    def ping_device(self, address: int, timeout: float = 0.06) -> bool:
        """
        Probe a device at the given address to verify presence on the Modbus network.
        Returns True as soon as a valid Modbus RTU response or exception frame with matching CRC is received.
        """
        return self.probe_device(address, timeout=timeout) is not None

    def scan_network(
        self,
        start_address: int = 1,
        end_address: int = 255,
        timeout_per_device: float = 0.06,
        progress_callback: Optional[Callable] = None,
        stop_event: Optional[threading.Event] = None,
    ) -> List[int]:
        """
        Scan the Modbus RTU network for responsive devices in the specified address range.
        Stores discovered device response bytes in self.last_scan_device_responses.

        :param start_address: Starting slave address (1-255).
        :param end_address: Ending slave address (1-255).
        :param timeout_per_device: Timeout per address probe in seconds (default: 0.06s).
        :param progress_callback: Optional callback(current_addr, total_addresses, found_addresses)
                                  or callback(current_addr, total_addresses, found_addresses, device_responses).
        :param stop_event: Optional threading.Event to signal scan cancellation.
        :return: List of responding device addresses.
        """
        start = max(1, min(255, start_address))
        end = max(1, min(255, end_address))
        if start > end:
            start, end = end, start

        found_devices: List[int] = []
        self.last_scan_device_responses = {}
        total_addresses = end - start + 1

        for addr in range(start, end + 1):
            if stop_event is not None and stop_event.is_set():
                break

            resp = self.probe_device(addr, timeout=timeout_per_device)
            if resp is not None:
                found_devices.append(addr)
                self.last_scan_device_responses[addr] = resp

            if progress_callback is not None:
                try:
                    try:
                        progress_callback(addr, total_addresses, list(found_devices), dict(self.last_scan_device_responses))
                    except TypeError:
                        progress_callback(addr, total_addresses, list(found_devices))
                except Exception:
                    pass

        return found_devices

    @staticmethod
    def list_available_ports():
        """Return list of available COM port names and descriptions."""
        return serial.tools.list_ports.comports()
