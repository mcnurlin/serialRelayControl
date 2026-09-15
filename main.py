"""
Main entry point for Serial Relay Controller.
Supports both Graphical User Interface (default) and Command-Line Interface (CLI).
"""
import sys
import argparse
from modbus_relay import RelayController, build_relay_command, parse_hex_string


def run_cli(args):
    """Execute command-line relay operation."""
    controller = RelayController()
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
        description="Serial Relay Control Application (Modbus RTU Single Coil Control for 8-Channel Relay Boards)"
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
    parser.add_argument("--timeout", type=float, default=0.5, help="Serial timeout in seconds (default: 0.5)")

    args = parser.parse_args()

    if args.cli or args.port:
        run_cli(args)
    else:
        # Launch GUI
        import gui
        gui.main()


if __name__ == "__main__":
    main()
