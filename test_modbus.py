"""
Tests for Modbus RTU command generation, input reading, and CRC-16 computation for serial relay control.
"""
import pytest
import threading
from unittest.mock import MagicMock
from modbus_relay import (
    calculate_crc,
    build_relay_command,
    build_read_inputs_command,
    build_read_analog_inputs_command,
    parse_read_inputs_response,
    parse_read_analog_inputs_response,
    is_valid_modbus_response,
    raw_to_ma,
    parse_hex_string,
    format_device_response_string,
    RelayController,
)


def test_calculate_crc_modbus():
    # 01 05 00 00 FF 00 -> CRC low 8C, high 3A -> 0x8C, 0x3A
    data = bytes([0x01, 0x05, 0x00, 0x00, 0xFF, 0x00])
    crc = calculate_crc(data)
    assert crc == bytes([0x8C, 0x3A])

    # 01 05 00 00 00 00 -> CRC low CD, high CA -> 0xCD, 0xCA
    data_off = bytes([0x01, 0x05, 0x00, 0x00, 0x00, 0x00])
    crc_off = calculate_crc(data_off)
    assert crc_off == bytes([0xCD, 0xCA])


def test_exact_issue_specification_commands():
    """
    Verify all 16 command frames given in the specification:
    Relay 0 on:  01 05 00 00 FF 00 8C 3A | off: 01 05 00 00 00 00 CD CA
    Relay 1 on:  01 05 00 01 FF 00 DD FA | off: 01 05 00 01 00 00 9C 0A
    Relay 2 on:  01 05 00 02 FF 00 2D FA | off: 01 05 00 02 00 00 6C 0A
    Relay 3 on:  01 05 00 03 FF 00 7C 3A | off: 01 05 00 03 00 00 3D CA
    Relay 4 on:  01 05 00 04 FF 00 CD FB | off: 01 05 00 04 00 00 8C 0B
    Relay 5 on:  01 05 00 05 FF 00 9C 3B | off: 01 05 00 05 00 00 DD CB
    Relay 6 on:  01 05 00 06 FF 00 6C 3B | off: 01 05 00 06 00 00 2D CB
    Relay 7 on:  01 05 00 07 FF 00 3D FB | off: 01 05 00 07 00 00 7C 0B
    """
    expected_commands = {
        0: {
            True: "01 05 00 00 FF 00 8C 3A",
            False: "01 05 00 00 00 00 CD CA",
        },
        1: {
            True: "01 05 00 01 FF 00 DD FA",
            False: "01 05 00 01 00 00 9C 0A",
        },
        2: {
            True: "01 05 00 02 FF 00 2D FA",
            False: "01 05 00 02 00 00 6C 0A",
        },
        3: {
            True: "01 05 00 03 FF 00 7C 3A",
            False: "01 05 00 03 00 00 3D CA",
        },
        4: {
            True: "01 05 00 04 FF 00 CD FB",
            False: "01 05 00 04 00 00 8C 0B",
        },
        5: {
            True: "01 05 00 05 FF 00 9C 3B",
            False: "01 05 00 05 00 00 DD CB",
        },
        6: {
            True: "01 05 00 06 FF 00 6C 3B",
            False: "01 05 00 06 00 00 2D CB",
        },
        7: {
            True: "01 05 00 07 FF 00 3D FB",
            False: "01 05 00 07 00 00 7C 0B",
        },
    }

    for relay_idx, states in expected_commands.items():
        for state, expected_hex in states.items():
            cmd = build_relay_command(address=1, relay_index=relay_idx, state=state)
            expected_bytes = bytes.fromhex(expected_hex)
            assert cmd == expected_bytes, f"Mismatch on Relay {relay_idx} state={state}: got {cmd.hex(' ').upper()}, expected {expected_hex}"


def test_build_read_inputs_command():
    # Address 1, Read 8 Discrete Inputs starting at 0: 01 02 00 00 00 08 -> CRC 79 CC
    cmd = build_read_inputs_command(address=1, start=0, count=8, function_code=0x02)
    expected = bytes.fromhex("01 02 00 00 00 08 79 CC")
    assert cmd == expected

    # Address 2, Read 8 Discrete Inputs: 02 02 00 00 00 08 79 FF
    cmd_addr2 = build_read_inputs_command(address=2, start=0, count=8, function_code=0x02)
    expected_addr2 = bytes.fromhex("02 02 00 00 00 08 79 FF")
    assert cmd_addr2 == expected_addr2

    # Validation errors
    with pytest.raises(ValueError):
        build_read_inputs_command(address=0)

    with pytest.raises(ValueError):
        build_read_inputs_command(address=256)

    with pytest.raises(ValueError):
        build_read_inputs_command(address=1, count=0)


def test_parse_read_inputs_response():
    # 01 02 01 05 -> CRC is 61 8B. 0x05 is binary 00000101 (Input 0=True, Input 1=False, Input 2=True, rest False)
    payload = bytes([0x01, 0x02, 0x01, 0x05])
    crc = calculate_crc(payload)
    resp = payload + crc

    states = parse_read_inputs_response(resp, expected_address=1, expected_count=8)
    assert states is not None
    assert states == [True, False, True, False, False, False, False, False]

    # All ON (0xFF)
    payload_all_on = bytes([0x01, 0x02, 0x01, 0xFF])
    resp_all_on = payload_all_on + calculate_crc(payload_all_on)
    states_all_on = parse_read_inputs_response(resp_all_on, expected_address=1, expected_count=8)
    assert states_all_on == [True] * 8

    # All OFF (0x00)
    payload_all_off = bytes([0x01, 0x02, 0x01, 0x00])
    resp_all_off = payload_all_off + calculate_crc(payload_all_off)
    states_all_off = parse_read_inputs_response(resp_all_off, expected_address=1, expected_count=8)
    assert states_all_off == [False] * 8

    # Invalid CRC
    corrupt_resp = resp[:-1] + bytes([0x00])
    assert parse_read_inputs_response(corrupt_resp, expected_address=1) is None

    # Mismatched address
    assert parse_read_inputs_response(resp, expected_address=2) is None

    # Too short
    assert parse_read_inputs_response(bytes([0x01, 0x02]), expected_address=1) is None


def test_address_range_validation():
    # Valid addresses 1 to 255
    cmd_1 = build_relay_command(address=1, relay_index=0, state=True)
    assert cmd_1[0] == 1

    cmd_255 = build_relay_command(address=255, relay_index=7, state=False)
    assert cmd_255[0] == 255
    assert len(cmd_255) == 8

    # Invalid addresses
    with pytest.raises(ValueError):
        build_relay_command(address=0, relay_index=0, state=True)

    with pytest.raises(ValueError):
        build_relay_command(address=256, relay_index=0, state=True)

    # Invalid relay index (must be 0 to 7)
    with pytest.raises(ValueError):
        build_relay_command(address=1, relay_index=-1, state=True)

    with pytest.raises(ValueError):
        build_relay_command(address=1, relay_index=8, state=True)


def test_relay_controller_state_tracking():
    controller = RelayController()
    assert controller.get_relay_state(address=1, relay_index=0) is False

    controller.set_local_state(address=1, relay_index=0, state=True)
    assert controller.get_relay_state(address=1, relay_index=0) is True
    assert controller.get_relay_state(address=2, relay_index=0) is False

    # Input state tracking
    assert controller.get_input_state(address=1, input_index=0) is False
    controller.set_input_state(address=1, input_index=0, state=True)
    assert controller.get_input_state(address=1, input_index=0) is True
    assert controller.get_input_state(address=2, input_index=0) is False

    # Analog state tracking
    assert controller.get_analog_state(address=1, ai_index=0) == 0
    controller.set_analog_state(address=1, ai_index=0, value=12000)
    assert controller.get_analog_state(address=1, ai_index=0) == 12000
    assert controller.get_analog_state(address=2, ai_index=0) == 0


def test_build_read_analog_inputs_command():
    # Address 1, Read 8 Analog Input Registers (0x04) starting at 0: 01 04 00 00 00 08 -> CRC F1 CC
    cmd = build_read_analog_inputs_command(address=1, start=0, count=8, function_code=0x04)
    expected = bytes.fromhex("01 04 00 00 00 08 F1 CC")
    assert cmd == expected

    # Address 2, Read 8 Analog Input Registers: 02 04 00 00 00 08 F1 FF
    cmd_addr2 = build_read_analog_inputs_command(address=2, start=0, count=8, function_code=0x04)
    expected_addr2 = bytes.fromhex("02 04 00 00 00 08 F1 FF")
    assert cmd_addr2 == expected_addr2

    # Validation errors
    with pytest.raises(ValueError):
        build_read_analog_inputs_command(address=0)

    with pytest.raises(ValueError):
        build_read_analog_inputs_command(address=256)

    with pytest.raises(ValueError):
        build_read_analog_inputs_command(address=1, count=0)

    with pytest.raises(ValueError):
        build_read_analog_inputs_command(address=1, count=126)


def test_parse_read_analog_inputs_response():
    # 8 registers with values: AI0=4000, AI1=8000, AI2=12000, AI3=16000, AI4=20000, AI5=10000, AI6=5000, AI7=15000
    values = [4000, 8000, 12000, 16000, 20000, 10000, 5000, 15000]
    payload = bytearray([0x01, 0x04, 0x10])  # Address 1, Func 4, ByteCount 16 (0x10)
    for v in values:
        payload.append((v >> 8) & 0xFF)
        payload.append(v & 0xFF)

    crc = calculate_crc(bytes(payload))
    resp = bytes(payload) + crc

    parsed = parse_read_analog_inputs_response(resp, expected_address=1, expected_count=8)
    assert parsed is not None
    assert parsed == values

    # Test scaling
    assert raw_to_ma(4000, "auto") == 4.0
    assert raw_to_ma(12000, "auto") == 12.0
    assert raw_to_ma(20000, "auto") == 20.0
    assert raw_to_ma(4095, "12bit_adc") == 20.0
    assert raw_to_ma(0, "12bit_adc") == 4.0

    # Invalid CRC
    corrupt_resp = resp[:-1] + bytes([0x00])
    assert parse_read_analog_inputs_response(corrupt_resp, expected_address=1) is None

    # Mismatched address
    assert parse_read_analog_inputs_response(resp, expected_address=2) is None

    # Invalid byte count / short frame
    assert parse_read_analog_inputs_response(bytes([0x01, 0x04, 0x02, 0x00]), expected_address=1) is None


def test_is_valid_modbus_response():
    # Valid discrete input response: 01 02 01 00 CRC_L CRC_H
    payload = bytes([0x01, 0x02, 0x01, 0x00])
    resp = payload + calculate_crc(payload)
    assert is_valid_modbus_response(resp) is True
    assert is_valid_modbus_response(resp, expected_address=1) is True
    assert is_valid_modbus_response(resp, expected_address=2) is False

    # Valid exception response: 05 82 02 CRC_L CRC_H
    ex_payload = bytes([0x05, 0x82, 0x02])
    ex_resp = ex_payload + calculate_crc(ex_payload)
    assert is_valid_modbus_response(ex_resp) is True
    assert is_valid_modbus_response(ex_resp, expected_address=5) is True

    # Invalid CRC
    bad_crc_resp = resp[:-1] + bytes([0x99])
    assert is_valid_modbus_response(bad_crc_resp) is False

    # Too short
    assert is_valid_modbus_response(bytes([0x01, 0x02])) is False


def test_scan_network_mock():
    controller = RelayController()

    mock_serial = MagicMock()
    mock_serial.is_open = True

    # Mock responses: device at address 1 (relay board Function 0x01), device at address 5 respond
    def fake_read(size):
        written = mock_serial.write.call_args[0][0]
        addr = written[0]
        func = written[1]
        if addr == 1 and func == 0x01:
            # 8 coils status response: 01 01 01 00 CRC
            payload = bytes([0x01, 0x01, 0x01, 0x00])
            return payload + calculate_crc(payload)
        elif addr == 5:
            # Function 0x01 response for device 5
            payload = bytes([0x05, 0x01, 0x01, 0x03])
            return payload + calculate_crc(payload)
        return b""

    mock_serial.read.side_effect = fake_read
    controller.serial_port = mock_serial

    # Ping test
    assert controller.ping_device(address=1) is True
    assert controller.ping_device(address=2) is False
    assert controller.ping_device(address=5) is True

    # Scan test across addresses 1 to 8
    progress_log = []

    def on_prog(curr, total, found):
        progress_log.append((curr, list(found)))

    found_devices = controller.scan_network(
        start_address=1,
        end_address=8,
        timeout_per_device=0.01,
        progress_callback=on_prog,
    )

    assert found_devices == [1, 5]
    assert len(progress_log) == 8
    assert progress_log[-1][1] == [1, 5]
    # Check that device responses are captured
    assert 1 in controller.last_scan_device_responses
    assert 5 in controller.last_scan_device_responses
    assert controller.last_scan_device_responses[1] == bytes([0x01, 0x01, 0x01, 0x00]) + calculate_crc(bytes([0x01, 0x01, 0x01, 0x00]))


def test_format_device_response_string():
    # Hex only
    raw1 = bytes([0x01, 0x01, 0x01, 0x00, 0x51, 0x88])
    formatted1 = format_device_response_string(raw1)
    assert "01 01 01 00 51 88" in formatted1

    # Hex + ASCII
    raw2 = bytes([0x01, 0x03, 0x04]) + b"REL8" + bytes([0x12, 0x34])
    formatted2 = format_device_response_string(raw2)
    assert "REL8" in formatted2
    assert "01 03 04 52 45 4C 38 12 34" in formatted2

    # None or empty
    assert format_device_response_string(None) == "(No response string)"
    assert format_device_response_string(b"") == "(No response string)"


def test_ping_device_various_functions():
    controller = RelayController()
    mock_serial = MagicMock()
    mock_serial.is_open = True
    controller.serial_port = mock_serial

    # Test 1: Device at address 1 responds to Function 0x01 (Read Coils)
    def fake_read_coils(size):
        written = mock_serial.write.call_args[0][0]
        if written[0] == 1 and written[1] == 0x01:
            payload = bytes([0x01, 0x01, 0x01, 0x00])
            return payload + calculate_crc(payload)
        return b""

    mock_serial.read.side_effect = fake_read_coils
    assert controller.ping_device(address=1) is True

    # Test 2: Device at address 3 only responds to Function 0x03 (Holding Registers)
    def fake_read_holding(size):
        written = mock_serial.write.call_args[0][0]
        if written[0] == 3 and written[1] == 0x03:
            payload = bytes([0x03, 0x03, 0x02, 0x00, 0x00])
            return payload + calculate_crc(payload)
        return b""

    mock_serial.read.side_effect = fake_read_holding
    assert controller.ping_device(address=3) is True

    # Test 3: Device at address 4 responds with Modbus Exception 0x81 (Illegal Function)
    def fake_read_exception(size):
        written = mock_serial.write.call_args[0][0]
        if written[0] == 4:
            payload = bytes([0x04, 0x81, 0x01])
            return payload + calculate_crc(payload)
        return b""

    mock_serial.read.side_effect = fake_read_exception
    assert controller.ping_device(address=4) is True


def test_scan_network_stop_event():
    controller = RelayController()
    mock_serial = MagicMock()
    mock_serial.is_open = True
    mock_serial.read.return_value = b""
    controller.serial_port = mock_serial

    stop_event = threading.Event()
    stop_event.set()  # Stop immediately

    found = controller.scan_network(
        start_address=1,
        end_address=50,
        stop_event=stop_event,
    )
    assert found == []
