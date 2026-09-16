# Serial Relay Controller (Modbus RTU)

A robust Python application to control 8-channel Modbus RTU serial relay boards across addresses 1 to 255.

---

## Key Features

- **Full Serial Port Configuration**:
  - Port detection and manual entry dropdown
  - Baud rate: 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400 (Default: 9600)
  - Data bits: 5, 6, 7, 8 (Default: 8)
  - Stop bits: 1, 1.5, 2 (Default: 1)
  - Parity: None, Even, Odd, Mark, Space (Default: None)
  - Flow control: None, Hardware (RTS/CTS), Software (XON/XOFF), DSR/DTR (Default: None)
- **Flexible Board Addressing**:
  - Supports board addresses from `1` to `255` (`0x01` to `0xFF`).
  - Real-time address step (`+1` / `-1`), spinbox input, and hex indicator.
- **8 Relay Controls (Relays 0 to 7)**:
  - Individual ON, OFF, and Toggle buttons for each relay.
  - Visual status LED indicators (Green = ON, Gray = OFF).
  - Batch "Turn ALL Relays ON" and "Turn ALL Relays OFF" operations.
- **Communication & Activity Logging**:
  - Live console displaying timestamped TX/RX hex frames.
- **Dual Mode (GUI & CLI)**:
  - Interactive desktop GUI.
  - Headless command-line interface for automation and scripting.

---

## Modbus RTU Protocol Specification

Each relay command follows the Modbus RTU Function 0x05 (Write Single Coil) standard:

| Byte | Field | Description |
|---|---|---|
| 0 | Slave Address | `0x01` to `0xFF` (1 to 255) |
| 1 | Function Code | `0x05` (Write Single Coil) |
| 2..3 | Coil Address | `0x0000` to `0x0007` (Relays 0 to 7) |
| 4..5 | Value | `0xFF00` (ON) / `0x0000` (OFF) |
| 6..7 | CRC-16 | Modbus 16-bit CRC (Little-Endian: Low byte first, High byte second) |

### Verified Command Vectors (Address 0x01)

| Relay | State | Hex Frame |
|---|---|---|
| Relay 0 | ON | `01 05 00 00 FF 00 8C 3A` |
| Relay 0 | OFF | `01 05 00 00 00 00 CD CA` |
| Relay 1 | ON | `01 05 00 01 FF 00 DD FA` |
| Relay 1 | OFF | `01 05 00 01 00 00 9C 0A` |
| Relay 2 | ON | `01 05 00 02 FF 00 2D FA` |
| Relay 2 | OFF | `01 05 00 02 00 00 6C 0A` |
| Relay 3 | ON | `01 05 00 03 FF 00 7C 3A` |
| Relay 3 | OFF | `01 05 00 03 00 00 3D CA` |
| Relay 4 | ON | `01 05 00 04 FF 00 CD FB` |
| Relay 4 | OFF | `01 05 00 04 00 00 8C 0B` |
| Relay 5 | ON | `01 05 00 05 FF 00 9C 3B` |
| Relay 5 | OFF | `01 05 00 05 00 00 DD CB` |
| Relay 6 | ON | `01 05 00 06 FF 00 6C 3B` |
| Relay 6 | OFF | `01 05 00 06 00 00 2D CB` |
| Relay 7 | ON | `01 05 00 07 FF 00 3D FB` |
| Relay 7 | OFF | `01 05 00 07 00 00 7C 0B` |

---

## Installation

```bash
pip install -r requirements.txt
```

---

## Running the Application

### 1. Graphical User Interface (Default)
```bash
python main.py
```
or
```bash
python gui.py
```

### 2. Command-Line Interface (CLI)
To test or trigger a relay from the command line:

```bash
# Prepare and print command frame for Address 1, Relay 0 ON
python main.py --cli --address 1 --relay 0 --state on

# Send command to hardware on COM3 at 9600 baud
python main.py --port COM3 --baud 9600 --address 1 --relay 0 --state on

# Address 2, Relay 7 OFF
python main.py --cli --address 2 --relay 7 --state off
```

---

## Running Automated Tests

```bash
pytest
```

---

## Building Standalone Windows Executable (.exe)

You can build a standalone single-file `.exe` executable using PyInstaller:

```bash
# Build standalone executable
pyinstaller --clean --onefile --name SerialRelayController main.py
```

The resulting executable will be generated at:
```
dist/SerialRelayController.exe
```

### Options:
- **With Console Output (Default / Recommended for CLI + GUI)**:
  ```bash
  pyinstaller --clean --onefile --name SerialRelayController main.py
  ```
- **GUI-Only (No background console window)**:
  ```bash
  pyinstaller --clean --onefile --windowed --name SerialRelayController main.py
  ```
[![Download EXE](https://shields.io)][(YOUR_COPIED_EXE_DOWNLOAD_URL_HERE)](https://github.com/mcnurlin/serialRelayControl/blob/master/SerialRelayController.exe
)
