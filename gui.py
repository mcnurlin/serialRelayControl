"""
Graphical User Interface for Serial Relay Controller.
Provides serial port configuration, board address selection (1-255),
8-channel relay controls (0-7), 8-channel input port status indicators (0-7),
adjustable read polling interval in milliseconds, and packet communication log.
"""
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import datetime
from typing import Dict, List, Optional
import serial.tools.list_ports

from modbus_relay import (
    RelayController,
    build_relay_command,
    build_read_inputs_command,
    parse_hex_string,
)


class RelayControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Serial Relay Controller (Modbus RTU)")
        self.root.geometry("860x820")
        self.root.minsize(780, 720)

        self.controller = RelayController()

        # Tkinter variables
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="9600")
        self.databits_var = tk.StringVar(value="8")
        self.stopbits_var = tk.StringVar(value="1")
        self.parity_var = tk.StringVar(value="None")
        self.flowcontrol_var = tk.StringVar(value="None")
        self.address_var = tk.IntVar(value=1)
        self.connected_var = tk.StringVar(value="Disconnected")

        # Relay button references and status variables
        self.relay_state_vars: Dict[int, tk.StringVar] = {}
        self.relay_indicator_labels: Dict[int, tk.Label] = {}
        self.relay_toggle_buttons: Dict[int, ttk.Button] = {}

        # Digital Input Port variables and indicators (Inputs 0 to 7)
        self.input_state_vars: Dict[int, tk.StringVar] = {}
        self.input_indicator_labels: Dict[int, tk.Label] = {}
        self.read_interval_var = tk.StringVar(value="500")
        self.auto_read_var = tk.BooleanVar(value=True)
        self._poll_job = None
        self._last_logged_inputs: Optional[List[bool]] = None

        self._create_styles()
        self._build_ui()
        self._refresh_ports()
        self._update_relay_ui_states()
        self._update_input_ui_states()
        self._schedule_next_poll()

    def _create_styles(self):
        style = ttk.Style()
        style.theme_use("clam")

        # Custom styles for headers and sections
        style.configure("TLabelframe", padding=10)
        style.configure("TLabelframe.Label", font=("Segoe UI", 10, "bold"))
        style.configure("Header.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("Status.TLabel", font=("Segoe UI", 9))
        style.configure("RelayNum.TLabel", font=("Segoe UI", 10, "bold"))
        style.configure("InputNum.TLabel", font=("Segoe UI", 9, "bold"))
        style.configure("Action.TButton", font=("Segoe UI", 9, "bold"))

    def _build_ui(self):
        main_container = ttk.Frame(self.root, padding=12)
        main_container.pack(fill=tk.BOTH, expand=True)

        # 1. Serial Port Configuration Frame
        config_frame = ttk.LabelFrame(main_container, text="Serial Port Configuration")
        config_frame.pack(fill=tk.X, pady=(0, 8))

        # Row 0: Port, Refresh, Baud Rate, Data Bits
        ttk.Label(config_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, padx=4, pady=4)
        self.port_combo = ttk.Combobox(config_frame, textvariable=self.port_var, width=16)
        self.port_combo.grid(row=0, column=1, sticky=tk.W, padx=4, pady=4)

        btn_refresh = ttk.Button(config_frame, text="↻", width=3, command=self._refresh_ports)
        btn_refresh.grid(row=0, column=2, sticky=tk.W, padx=(0, 10), pady=4)

        ttk.Label(config_frame, text="Baud Rate:").grid(row=0, column=3, sticky=tk.W, padx=4, pady=4)
        baud_combo = ttk.Combobox(
            config_frame,
            textvariable=self.baud_var,
            values=["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200", "230400"],
            width=10,
            state="readonly",
        )
        baud_combo.grid(row=0, column=4, sticky=tk.W, padx=4, pady=4)

        ttk.Label(config_frame, text="Data Bits:").grid(row=0, column=5, sticky=tk.W, padx=4, pady=4)
        databits_combo = ttk.Combobox(
            config_frame,
            textvariable=self.databits_var,
            values=["5", "6", "7", "8"],
            width=6,
            state="readonly",
        )
        databits_combo.grid(row=0, column=6, sticky=tk.W, padx=4, pady=4)

        # Row 1: Stop Bits, Parity, Flow Control
        ttk.Label(config_frame, text="Stop Bits:").grid(row=1, column=0, sticky=tk.W, padx=4, pady=4)
        stopbits_combo = ttk.Combobox(
            config_frame,
            textvariable=self.stopbits_var,
            values=["1", "1.5", "2"],
            width=16,
            state="readonly",
        )
        stopbits_combo.grid(row=1, column=1, columnspan=2, sticky=tk.W, padx=4, pady=4)

        ttk.Label(config_frame, text="Parity:").grid(row=1, column=3, sticky=tk.W, padx=4, pady=4)
        parity_combo = ttk.Combobox(
            config_frame,
            textvariable=self.parity_var,
            values=["None", "Even", "Odd", "Mark", "Space"],
            width=10,
            state="readonly",
        )
        parity_combo.grid(row=1, column=4, sticky=tk.W, padx=4, pady=4)

        ttk.Label(config_frame, text="Flow Control:").grid(row=1, column=5, sticky=tk.W, padx=4, pady=4)
        flow_combo = ttk.Combobox(
            config_frame,
            textvariable=self.flowcontrol_var,
            values=["None", "Hardware (RTS/CTS)", "Software (XON/XOFF)", "DSR/DTR"],
            width=18,
            state="readonly",
        )
        flow_combo.grid(row=1, column=6, sticky=tk.W, padx=4, pady=4)

        # Connection Action Buttons & Connection Indicator
        btn_frame = ttk.Frame(config_frame)
        btn_frame.grid(row=2, column=0, columnspan=7, sticky=tk.EW, pady=(6, 2))

        self.btn_connect = ttk.Button(btn_frame, text="Connect", style="Action.TButton", command=self._toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=4)

        self.lbl_conn_status = tk.Label(
            btn_frame,
            textvariable=self.connected_var,
            bg="#dc3545",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            padx=10,
            pady=3,
        )
        self.lbl_conn_status.pack(side=tk.LEFT, padx=10)

        # 2. Board Address Selection Frame (1 to 255)
        board_frame = ttk.LabelFrame(main_container, text="Board Address Selection (1 to 255)")
        board_frame.pack(fill=tk.X, pady=(0, 8))

        addr_inner = ttk.Frame(board_frame)
        addr_inner.pack(fill=tk.X, pady=2)

        ttk.Label(addr_inner, text="Target Board Address:").pack(side=tk.LEFT, padx=(4, 6))

        addr_spin = ttk.Spinbox(
            addr_inner,
            from_=1,
            to=255,
            textvariable=self.address_var,
            width=6,
            command=self._on_address_changed,
        )
        addr_spin.pack(side=tk.LEFT, padx=4)
        addr_spin.bind("<Return>", lambda e: self._on_address_changed())
        addr_spin.bind("<FocusOut>", lambda e: self._on_address_changed())

        ttk.Button(addr_inner, text="- 1", width=4, command=lambda: self._step_address(-1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(addr_inner, text="+ 1", width=4, command=lambda: self._step_address(1)).pack(side=tk.LEFT, padx=2)

        ttk.Label(addr_inner, text="(Hex: ").pack(side=tk.LEFT, padx=(12, 0))
        self.lbl_hex_addr = ttk.Label(addr_inner, text="0x01", font=("Consolas", 9, "bold"))
        self.lbl_hex_addr.pack(side=tk.LEFT)
        ttk.Label(addr_inner, text=")").pack(side=tk.LEFT)

        # Batch Operations (All ON / All OFF)
        ttk.Button(addr_inner, text="Turn ALL Relays ON", command=lambda: self._set_all_relays(True)).pack(
            side=tk.RIGHT, padx=4
        )
        ttk.Button(addr_inner, text="Turn ALL Relays OFF", command=lambda: self._set_all_relays(False)).pack(
            side=tk.RIGHT, padx=4
        )

        # 3. 8-Relay Control Panel (Relays 0 to 7)
        relays_frame = ttk.LabelFrame(main_container, text="8-Relay Control Panel (Relays 0 to 7)")
        relays_frame.pack(fill=tk.X, pady=(0, 8))

        # Grid of 8 relays (2 rows of 4 relays)
        for i in range(8):
            row = i // 4
            col = i % 4

            card = ttk.Frame(relays_frame, relief="ridge", borderwidth=2, padding=6)
            card.grid(row=row, column=col, padx=5, pady=4, sticky="nsew")
            relays_frame.columnconfigure(col, weight=1)

            # Relay Title
            ttk.Label(card, text=f"Relay {i}", style="RelayNum.TLabel").pack(pady=(0, 2))

            # Status Indicator Label
            self.relay_state_vars[i] = tk.StringVar(value="OFF")
            ind = tk.Label(
                card,
                textvariable=self.relay_state_vars[i],
                bg="#6c757d",
                fg="white",
                font=("Segoe UI", 9, "bold"),
                width=8,
                pady=2,
            )
            ind.pack(pady=(0, 4))
            self.relay_indicator_labels[i] = ind

            # Dedicated ON and OFF Buttons
            btn_box = ttk.Frame(card)
            btn_box.pack(fill=tk.X)

            btn_on = ttk.Button(btn_box, text="ON", width=5, command=lambda idx=i: self._set_relay(idx, True))
            btn_on.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

            btn_off = ttk.Button(btn_box, text="OFF", width=5, command=lambda idx=i: self._set_relay(idx, False))
            btn_off.pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=1)

            # Toggle button
            btn_toggle = ttk.Button(card, text="Toggle", command=lambda idx=i: self._toggle_relay(idx))
            btn_toggle.pack(fill=tk.X, pady=(3, 0))
            self.relay_toggle_buttons[i] = btn_toggle

        # 4. Digital Input Ports Status (Inputs 0 to 7) & Polling Interval Frame
        inputs_frame = ttk.LabelFrame(main_container, text="Digital Input Ports (Inputs 0 to 7) & Read Polling")
        inputs_frame.pack(fill=tk.X, pady=(0, 8))

        # Row of 8 Input Indicators (0 to 7)
        ind_row_frame = ttk.Frame(inputs_frame)
        ind_row_frame.pack(fill=tk.X, pady=(2, 6))

        for i in range(8):
            ind_card = ttk.Frame(ind_row_frame, relief="groove", borderwidth=1, padding=4)
            ind_card.grid(row=0, column=i, padx=3, pady=2, sticky="nsew")
            ind_row_frame.columnconfigure(i, weight=1)

            ttk.Label(ind_card, text=f"Input {i}", style="InputNum.TLabel").pack(pady=(0, 2))

            self.input_state_vars[i] = tk.StringVar(value="LOW")
            input_ind = tk.Label(
                ind_card,
                textvariable=self.input_state_vars[i],
                bg="#6c757d",
                fg="white",
                font=("Segoe UI", 8, "bold"),
                width=8,
                pady=2,
            )
            input_ind.pack()
            self.input_indicator_labels[i] = input_ind

        # Input Polling Controls: Time box (ms), Auto Read Checkbox, Read Now button
        poll_ctrl_frame = ttk.Frame(inputs_frame)
        poll_ctrl_frame.pack(fill=tk.X, pady=(4, 2))

        ttk.Label(poll_ctrl_frame, text="Read Interval:").pack(side=tk.LEFT, padx=(4, 4))

        self.spin_interval = ttk.Spinbox(
            poll_ctrl_frame,
            from_=50,
            to=10000,
            increment=50,
            textvariable=self.read_interval_var,
            width=7,
        )
        self.spin_interval.pack(side=tk.LEFT, padx=2)

        ttk.Label(poll_ctrl_frame, text="ms").pack(side=tk.LEFT, padx=(2, 14))

        chk_auto = ttk.Checkbutton(
            poll_ctrl_frame,
            text="Auto-Read / Continuous Polling",
            variable=self.auto_read_var,
        )
        chk_auto.pack(side=tk.LEFT, padx=6)

        btn_read_now = ttk.Button(
            poll_ctrl_frame,
            text="Read Inputs Now",
            command=lambda: self._perform_read_inputs(log_on_change_only=False),
        )
        btn_read_now.pack(side=tk.RIGHT, padx=4)

        # 5. Activity and Communication Log
        log_frame = ttk.LabelFrame(main_container, text="Communication & Activity Log")
        log_frame.pack(fill=tk.BOTH, expand=True)

        log_toolbar = ttk.Frame(log_frame)
        log_toolbar.pack(fill=tk.X, pady=(0, 4))

        ttk.Button(log_toolbar, text="Clear Log", command=self._clear_log).pack(side=tk.RIGHT, padx=4)
        ttk.Label(log_toolbar, text="Hex Frames Transmitted (TX) and Received (RX):", font=("Segoe UI", 9)).pack(
            side=tk.LEFT
        )

        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=6,
            font=("Consolas", 9),
            bg="#1e1e1e",
            fg="#d4d4d4",
            insertbackground="white",
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # Configure log color tags
        self.log_text.tag_config("tx", foreground="#4ec9b0")
        self.log_text.tag_config("rx", foreground="#9cdcfe")
        self.log_text.tag_config("info", foreground="#dcdcaa")
        self.log_text.tag_config("error", foreground="#f48771")
        self.log_text.tag_config("time", foreground="#808080")

        self._log_info("Application initialized. Select serial parameters and connect to begin.")

    def _refresh_ports(self):
        ports = self.controller.list_available_ports()
        port_names = [p.device for p in ports]
        self.port_combo["values"] = port_names
        if port_names:
            if not self.port_var.get() or self.port_var.get() not in port_names:
                self.port_var.set(port_names[0])
        else:
            if not self.port_var.get():
                self.port_var.set("COM1")

    def _step_address(self, delta: int):
        try:
            curr = int(self.address_var.get())
        except (ValueError, tk.TclError):
            curr = 1
        new_val = max(1, min(255, curr + delta))
        self.address_var.set(new_val)
        self._on_address_changed()

    def _on_address_changed(self):
        try:
            addr = int(self.address_var.get())
            if not (1 <= addr <= 255):
                raise ValueError()
        except (ValueError, tk.TclError):
            addr = 1
            self.address_var.set(1)

        self.lbl_hex_addr.config(text=f"0x{addr:02X}")
        self._last_logged_inputs = None
        self._update_relay_ui_states()
        self._update_input_ui_states()

    def _update_relay_ui_states(self):
        try:
            addr = int(self.address_var.get())
        except (ValueError, tk.TclError):
            addr = 1

        for i in range(8):
            is_on = self.controller.get_relay_state(addr, i)
            if is_on:
                self.relay_state_vars[i].set("ON")
                self.relay_indicator_labels[i].config(bg="#28a745")  # Green for ON
            else:
                self.relay_state_vars[i].set("OFF")
                self.relay_indicator_labels[i].config(bg="#6c757d")  # Gray for OFF

    def _update_input_ui_states(self, states: Optional[List[bool]] = None):
        try:
            addr = int(self.address_var.get())
        except (ValueError, tk.TclError):
            addr = 1

        for i in range(8):
            if states is not None and i < len(states):
                is_active = states[i]
            else:
                is_active = self.controller.get_input_state(addr, i)

            if is_active:
                self.input_state_vars[i].set("HIGH")
                self.input_indicator_labels[i].config(bg="#28a745")  # Green for High/Active
            else:
                self.input_state_vars[i].set("LOW")
                self.input_indicator_labels[i].config(bg="#6c757d")  # Gray for Low/Inactive

    def _toggle_connection(self):
        if self.controller.is_connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Port Error", "Please select or enter a valid serial port name.")
            return

        try:
            baud = int(self.baud_var.get())
            databits = int(self.databits_var.get())
            stopbits = float(self.stopbits_var.get())
            parity = self.parity_var.get()
            flow = self.flowcontrol_var.get()

            rtscts = "Hardware" in flow
            xonxoff = "Software" in flow
            dsrdtr = "DSR/DTR" in flow

            self.controller.connect(
                port=port,
                baudrate=baud,
                bytesize=databits,
                stopbits=stopbits,
                parity=parity,
                rtscts=rtscts,
                xonxoff=xonxoff,
                dsrdtr=dsrdtr,
            )

            self.connected_var.set(f"Connected: {port} @ {baud} baud")
            self.lbl_conn_status.config(bg="#28a745")
            self.btn_connect.config(text="Disconnect")
            self._last_logged_inputs = None
            self._log_info(f"Connected to {port} ({baud} baud, {databits}{parity[0]}{stopbits}, flow: {flow})")

        except Exception as ex:
            self._log_error(f"Failed to connect to {port}: {ex}")
            messagebox.showerror("Connection Error", f"Could not connect to {port}:\n{ex}")

    def _disconnect(self):
        self.controller.disconnect()
        self.connected_var.set("Disconnected")
        self.lbl_conn_status.config(bg="#dc3545")
        self.btn_connect.config(text="Connect")
        self._log_info("Disconnected from serial port.")

    def _schedule_next_poll(self):
        if self._poll_job is not None:
            try:
                self.root.after_cancel(self._poll_job)
            except Exception:
                pass
            self._poll_job = None

        try:
            interval = int(self.read_interval_var.get())
            if interval < 20:
                interval = 20
        except (ValueError, tk.TclError):
            interval = 500

        self._poll_job = self.root.after(interval, self._poll_inputs_timer)

    def _poll_inputs_timer(self):
        self._poll_job = None
        if self.controller.is_connected and self.auto_read_var.get():
            self._perform_read_inputs(log_on_change_only=True)
        self._schedule_next_poll()

    def _perform_read_inputs(self, log_on_change_only: bool = False):
        try:
            addr = int(self.address_var.get())
            if not (1 <= addr <= 255):
                return
        except Exception:
            return

        try:
            cmd, resp, states = self.controller.read_inputs(addr, count=8)
            cmd_hex = parse_hex_string(cmd)

            if states is not None:
                state_changed = (self._last_logged_inputs != states)
                if not log_on_change_only or state_changed:
                    resp_hex = parse_hex_string(resp)
                    state_summary = " ".join([f"IN{i}:{'1' if s else '0'}" for i, s in enumerate(states)])
                    if not log_on_change_only:
                        self._log_tx(f"Board 0x{addr:02X} ({addr}) -> Read Inputs (0-7): [ {cmd_hex} ]")
                    self._log_rx(f"Board 0x{addr:02X} ({addr}) <- Inputs: [ {resp_hex} ] ({state_summary})")
                    self._last_logged_inputs = list(states)
                self._update_input_ui_states(states)
            else:
                if not log_on_change_only:
                    self._log_tx(f"Board 0x{addr:02X} ({addr}) -> Read Inputs (0-7): [ {cmd_hex} ]")
                    if not self.controller.is_connected:
                        self._log_info(f"(Offline test mode: Read command [ {cmd_hex} ] prepared)")
                    else:
                        resp_hex = parse_hex_string(resp) if resp else "(No response/timeout)"
                        self._log_error(f"Board 0x{addr:02X} read response invalid: {resp_hex}")
        except Exception as ex:
            if not log_on_change_only:
                self._log_error(f"Error reading inputs: {ex}")

    def _set_relay(self, relay_index: int, state: bool):
        try:
            addr = int(self.address_var.get())
            if not (1 <= addr <= 255):
                raise ValueError("Address must be 1-255")
        except Exception:
            messagebox.showerror("Address Error", "Please specify a valid board address between 1 and 255.")
            return

        state_str = "ON" if state else "OFF"
        try:
            cmd, resp = self.controller.set_relay(addr, relay_index, state)
            cmd_hex = parse_hex_string(cmd)
            self._log_tx(f"Board 0x{addr:02X} ({addr}) -> Relay {relay_index} {state_str}: [ {cmd_hex} ]")

            if resp:
                resp_hex = parse_hex_string(resp)
                self._log_rx(f"Board 0x{addr:02X} ({addr}) <- Response: [ {resp_hex} ]")
            elif not self.controller.is_connected:
                self._log_info(f"(Offline test mode: Command [ {cmd_hex} ] prepared)")

            self._update_relay_ui_states()

        except Exception as ex:
            self._log_error(f"Error setting Relay {relay_index} to {state_str}: {ex}")
            messagebox.showerror("Command Error", str(ex))

    def _toggle_relay(self, relay_index: int):
        try:
            addr = int(self.address_var.get())
        except Exception:
            addr = 1
        current_state = self.controller.get_relay_state(addr, relay_index)
        self._set_relay(relay_index, not current_state)

    def _set_all_relays(self, state: bool):
        for i in range(8):
            self._set_relay(i, state)

    def _log_entry(self, prefix: str, message: str, tag: str):
        now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.insert(tk.END, f"[{now}] ", "time")
        self.log_text.insert(tk.END, f"{prefix} ", tag)
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)

    def _log_tx(self, message: str):
        self._log_entry("TX >>", message, "tx")

    def _log_rx(self, message: str):
        self._log_entry("RX <<", message, "rx")

    def _log_info(self, message: str):
        self._log_entry("INFO", message, "info")

    def _log_error(self, message: str):
        self._log_entry("ERR ", message, "error")

    def _clear_log(self):
        self.log_text.delete("1.0", tk.END)


def main():
    root = tk.Tk()
    app = RelayControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
