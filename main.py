"""
Main entry point for Serial Relay Controller.
Supports both Graphical User Interface (default) and Command-Line Interface (CLI).
"""
import sys
import argparse
from modbus_relay import (
    RelayController,
    build_relay_command,
    build_read_inputs_command,
    build_read_analog_inputs_command,
    parse_read_inputs_response,
    parse_read_analog_inputs_response,
    raw_to_ma,
    parse_hex_string,
    format_device_response_string,
)


def run_cli(args):
    """Execute command-line relay operation, digital input reading, analog input reading, or network device scan."""
    controller = RelayController()

    if args.scan:
        start_addr = max(1, min(255, args.scan_start))
        end_addr = max(1, min(255, args.scan_end))
        if start_addr > end_addr:
            start_addr, end_addr = end_addr, start_addr

        print(f"Scanning Modbus RTU network for devices (addresses {start_addr} to {end_addr})...")
        if args.port:
            flow = args.flow.lower()
            rtscts = "hw" in flow or "rts" in flow
            xonxoff = "sw" in flow or "xon" in flow
            dsrdtr = "dsr" in flow

            print(f"Connecting to {args.port} at {args.baud} baud...")
            controller.connect(
                port=args.port,
                baudrate=args.baud,
                bytesize=args.databits,
                stopbits=args.stopbits,
                parity=args.parity,
                rtscts=rtscts,
                xonxoff=xonxoff,
                dsrdtr=dsrdtr,
                timeout=args.timeout,
            )
            try:
                def progress(curr, total, found, responses=None):
                    if found and curr == found[-1]:
                        resp = (responses or {}).get(curr) or controller.last_scan_device_responses.get(curr, b"")
                        resp_str = format_device_response_string(resp) if resp else ""
                        if resp_str:
                            print(f"  [Found] Device detected at Address {curr} (0x{curr:02X}) | String: {resp_str}")
                        else:
                            print(f"  [Found] Device detected at Address {curr} (0x{curr:02X})")

                devices = controller.scan_network(
                    start_address=start_addr,
                    end_address=end_addr,
                    timeout_per_device=args.scan_timeout,
                    progress_callback=progress,
                )
                print(f"Scan complete: {len(devices)} device(s) found on network.")
                if devices:
                    print("Active Devices:")
                    for d in devices:
                        resp = controller.last_scan_device_responses.get(d, b"")
                        resp_str = format_device_response_string(resp) if resp else ""
                        if resp_str:
                            print(f"  - Address {d:3d} (0x{d:02X}) | String: {resp_str}")
                        else:
                            print(f"  - Address {d:3d} (0x{d:02X})")
            finally:
                controller.disconnect()
                print("Disconnected.")
        else:
            print("[Note] Specify --port to perform live serial scan across devices.")
        return

    if args.read_analog:
        cmd = build_read_analog_inputs_command(address=args.address, start=0, count=8)
        print(f"[TX Frame] Board Address: {args.address} (0x{args.address:02X}) | Read Analog Inputs 4-20mA (AI 0 to 7)")
        print(f"[Bytes]    {parse_hex_string(cmd)}")

        if args.port:
            flow = args.flow.lower()
            rtscts = "hw" in flow or "rts" in flow
            xonxoff = "sw" in flow or "xon" in flow
            dsrdtr = "dsr" in flow

            print(f"Connecting to {args.port} at {args.baud} baud...")
            controller.connect(
                port=args.port,
                baudrate=args.baud,
                bytesize=args.databits,
                stopbits=args.stopbits,
                parity=args.parity,
                rtscts=rtscts,
                xonxoff=xonxoff,
                dsrdtr=dsrdtr,
                timeout=args.timeout,
            )
            try:
                _, resp, registers = controller.read_analog_inputs(address=args.address, count=8)
                print(f"[RX Frame] {parse_hex_string(resp) if resp else '(No response/timeout)'}")
                if registers is not None:
                    print("Analog Inputs 4-20mA (AI 0 to 7):")
                    for i, val in enumerate(registers):
                        ma = raw_to_ma(val, scale_mode=args.scaling)
                        print(f"  AI {i}: {ma:.2f} mA (Raw: {val} / 0x{val:04X})")
            finally:
                controller.disconnect()
                print("Disconnected.")
        return

    if args.read_inputs:
        cmd = build_read_inputs_command(address=args.address, start=0, count=8)
        print(f"[TX Frame] Board Address: {args.address} (0x{args.address:02X}) | Read Discrete Inputs (0 to 7)")
        print(f"[Bytes]    {parse_hex_string(cmd)}")

        if args.port:
            flow = args.flow.lower()
            rtscts = "hw" in flow or "rts" in flow
            xonxoff = "sw" in flow or "xon" in flow
            dsrdtr = "dsr" in flow

            print(f"Connecting to {args.port} at {args.baud} baud...")
            controller.connect(
                port=args.port,
                baudrate=args.baud,
                bytesize=args.databits,
                stopbits=args.stopbits,
                parity=args.parity,
                rtscts=rtscts,
                xonxoff=xonxoff,
                dsrdtr=dsrdtr,
                timeout=args.timeout,
            )
            try:
                _, resp, states = controller.read_inputs(address=args.address, count=8)
                print(f"[RX Frame] {parse_hex_string(resp) if resp else '(No response/timeout)'}")
                if states is not None:
                    print("Input States (0 to 7):")
                    for i, val in enumerate(states):
                        print(f"  Input {i}: {'HIGH (Active)' if val else 'LOW (Inactive)'}")
            finally:
                controller.disconnect()
                print("Disconnected.")
        return

    state = args.state.lower() in ("on", "1", "true")
    cmd = build_relay_command(address=args.address, relay_index=args.relay, state=state)

    print(f"[TX Frame] Board Address: {args.address} (0x{args.address:02X}) | Relay: {args.relay} | State: {'ON' if state else 'OFF'}")
    print(f"[Bytes]    {parse_hex_string(cmd)}")

    if args.port:
        flow = args.flow.lower()
        rtscts = "hw" in flow or "rts" in flow
        xonxoff = "sw" in flow or "xon" in flow
        dsrdtr = "dsr" in flow

        print(f"Connecting to {args.port} at {args.baud} baud...")
        controller.connect(
            port=args.port,
            baudrate=args.baud,
            bytesize=args.databits,
            stopbits=args.stopbits,
            parity=args.parity,
            rtscts=rtscts,
            xonxoff=xonxoff,
            dsrdtr=dsrdtr,
            timeout=args.timeout,
        )
        try:
            resp = controller.send_command(cmd)
            print(f"[RX Frame] {parse_hex_string(resp) if resp else '(No response/timeout)'}")
        finally:
            controller.disconnect()
            print("Disconnected.")


def main():
    parser = argparse.ArgumentParser(
        description="Serial Relay Control Application (Modbus RTU Control, Digital & 4-20mA Analog Input Monitoring)"
    )
    parser.add_argument("--cli", action="store_true", help="Run in command-line mode instead of launching GUI")
    parser.add_argument("--port", type=str, default="", help="Serial port name (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--databits", type=int, choices=[5, 6, 7, 8], default=8, help="Data bits (default: 8)")
    parser.add_argument("--stopbits", type=float, choices=[1, 1.5, 2], default=1, help="Stop bits (default: 1)")
    parser.add_argument("--parity", choices=["N", "E", "O", "M", "S", "None", "Even", "Odd", "Mark", "Space"], default="N", help="Parity (default: None)")
    parser.add_argument("--flow", choices=["none", "hw", "sw", "dsr"], default="none", help="Flow control (default: none)")
    parser.add_argument("--address", type=int, default=1, help="Board address between 1 and 255 (default: 1)")
    parser.add_argument("--relay", type=int, choices=range(8), default=0, help="Relay index (0 to 7)")
    parser.add_argument("--state", choices=["on", "off", "ON", "OFF", "1", "0"], default="on", help="Relay target state (on/off)")
    parser.add_argument("--read-inputs", action="store_true", help="Read digital input ports 0 to 7")
    parser.add_argument("--read-analog", "--read-ai", dest="read_analog", action="store_true", help="Read 4-20mA analog inputs (AI 0 to 7)")
    parser.add_argument("--scaling", choices=["auto", "4000-20000", "0-20000", "12bit_adc", "10bit_adc", "16bit_adc"], default="auto", help="Analog scaling preset (default: auto)")
    parser.add_argument("--scan", action="store_true", help="Scan Modbus RTU network to detect all device addresses")
    parser.add_argument("--scan-start", type=int, default=1, help="Starting scan address (default: 1)")
    parser.add_argument("--scan-end", type=int, default=255, help="Ending scan address (default: 255)")
    parser.add_argument("--scan-timeout", type=float, default=0.05, help="Timeout per device in scan in seconds (default: 0.05)")
    parser.add_argument("--timeout", type=float, default=0.5, help="Serial timeout in seconds (default: 0.5)")

    args = parser.parse_args()

    if args.cli or args.port or args.read_inputs or args.read_analog or args.scan:
        run_cli(args)
    else:
        # Launch GUI
        import gui
        gui.main()


if __name__ == "__main__":
    main()
