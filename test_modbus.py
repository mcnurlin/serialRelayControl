"""
Tests for Modbus RTU command generation and CRC-16 computation for serial relay control.
"""
import pytest
from modbus_relay import calculate_crc, build_relay_command, parse_hex_string, RelayController


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
