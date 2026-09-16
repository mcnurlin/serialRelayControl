"""
Tests for Modbus RTU command generation, input reading, and CRC-16 computation for serial relay control.
"""
import pytest
from modbus_relay import (
    calculate_crc,
    build_relay_command,
    build_read_inputs_command,
    parse_read_inputs_response,
    parse_hex_string,
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
