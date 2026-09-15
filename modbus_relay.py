"""
Modbus RTU protocol handling and relay controller logic.
"""
from typing import Dict, Tuple, Optional
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


def parse_hex_string(data: bytes) -> str:
    """Format bytes as uppercase space-separated hex string."""
    return data.hex(" ").upper()


class RelayController:
    """
    Manages serial connection, board addressing, and relay states.
    """

    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        # State tracking: (address, relay_index) -> bool (True=ON, False=OFF)
        self.relay_states: Dict[Tuple[int, int], bool] = {}

    @property
    def is_connected(self) -> bool:
        return self.serial_port is not None and self.serial_port.is_open

    def get_relay_state(self, address: int, relay_index: int) -> bool:
        return self.relay_states.get((address, relay_index), False)

    def set_local_state(self, address: int, relay_index: int, state: bool) -> None:
        self.relay_states[(address, relay_index)] = state

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
        if self.is_connected:
            self.disconnect()

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
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            finally:
                self.serial_port = None

    def send_command(self, cmd: bytes) -> bytes:
        """
        Send raw command bytes over serial port and read response if available.
        """
        if not self.is_connected or self.serial_port is None:
            raise RuntimeError("Serial port is not connected.")

        self.serial_port.reset_input_buffer()
        self.serial_port.write(cmd)
        self.serial_port.flush()

        # Read back response (Modbus write single coil echo is usually 8 bytes)
        response = self.serial_port.read(8)
        return response

    def set_relay(self, address: int, relay_index: int, state: bool) -> Tuple[bytes, bytes]:
        """
        Build and send relay command.
        Returns (command_bytes, response_bytes).
        """
        cmd = build_relay_command(address, relay_index, state)
        response = b""
        if self.is_connected:
            response = self.send_command(cmd)
        self.set_local_state(address, relay_index, state)
        return cmd, response

    @staticmethod
    def list_available_ports():
        """Return list of available COM port names and descriptions."""
        return serial.tools.list_ports.comports()
