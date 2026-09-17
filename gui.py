"""
Graphical User Interface for Serial Relay Controller.
Provides serial port configuration, board address selection (1-255),
8-channel relay controls (0-7), 8-channel digital input port status indicators (0-7),
8-channel 4-20mA analog input indicators (0-7),
adjustable read polling interval in milliseconds, and packet communication log.
"""
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import datetime
import threading
from typing import Dict, List, Optional
import serial.tools.list_ports

from modbus_relay import (
    RelayController,
    build_relay_command,
    build_read_inputs_command,
    build_read_analog_inputs_command,
    raw_to_ma,
    parse_hex_string,
    format_device_response_string,
)


class RelayControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Serial Relay Controller (Modbus RTU & 4-20mA Analog Inputs)")
        self.root.geometry("1760x860")
        self.root.minsize(1200, 700)

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

        # Analog Input Port variables and indicators (AI 0 to 7, 4-20mA)
        self.analog_ma_vars: Dict[int, tk.StringVar] = {}
        self.analog_raw_vars: Dict[int, tk.StringVar] = {}
        self.analog_ma_labels: Dict[int, tk.Label] = {}
        self.analog_progress_bars: Dict[int, ttk.Progressbar] = {}
        self.analog_scale_mode_var = tk.StringVar(value="Auto")

        # Polling variables
        self.read_interval_var = tk.StringVar(value="500")
        self.auto_read_var = tk.BooleanVar(value=True)
        self.poll_digital_var = tk.BooleanVar(value=True)
        self.poll_analog_var = tk.BooleanVar(value=True)
        self._poll_job = None
        self._last_logged_inputs: Optional[List[bool]] = None
        self._last_logged_analogs: Optional[List[int]] = None

        # Network Device Scanner variables
        self._is_scanning = False
        self._scan_stop_event: Optional[threading.Event] = None
        self._scan_thread: Optional[threading.Thread] = None
        self.discovered_devices: List[int] = []
        self.scan_start_var = tk.StringVar(value="1")
        self.scan_end_var = tk.StringVar(value="255")
        self.scan_timeout_var = tk.StringVar(value="60")
        self.scan_status_var = tk.StringVar(value="Status: Ready to scan (1-255)")

        self._create_styles()
        self._build_ui()
        self._refresh_ports()
        self._update_relay_ui_states()
        self._update_input_ui_states()
        self._update_analog_ui_states()
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
        style.configure("AINum.TLabel", font=("Segoe UI", 9, "bold"))
        style.configure("Action.TButton", font=("Segoe UI", 9, "bold"))

    def _build_ui(self):
        main_container = ttk.Frame(self.root, padding=10)
        main_container.pack(fill=tk.BOTH, expand=True)

        # Top Horizontal Container: Serial Port Configuration (Left) & Board Address / Network Scanner (Right)
        top_frame = ttk.Frame(main_container)
        top_frame.pack(fill=tk.X, pady=(0, 6))

        # 1. Serial Port Configuration Frame (Left)
        config_frame = ttk.LabelFrame(top_frame, text="Serial Port Configuration")
        config_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))

        # Row 0: Port, Refresh, Baud Rate, Data Bits
        ttk.Label(config_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, padx=4, pady=3)
        self.port_combo = ttk.Combobox(config_frame, textvariable=self.port_var, width=14)
        self.port_combo.grid(row=0, column=1, sticky=tk.W, padx=2, pady=3)

        btn_refresh = ttk.Button(config_frame, text="↻", width=3, command=self._refresh_ports)
        btn_refresh.grid(row=0, column=2, sticky=tk.W, padx=(0, 6), pady=3)

        ttk.Label(config_frame, text="Baud Rate:").grid(row=0, column=3, sticky=tk.W, padx=4, pady=3)
        baud_combo = ttk.Combobox(
            config_frame,
            textvariable=self.baud_var,
            values=["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200", "230400"],
            width=9,
            state="readonly",
        )
        baud_combo.grid(row=0, column=4, sticky=tk.W, padx=2, pady=3)

        ttk.Label(config_frame, text="Data Bits:").grid(row=0, column=5, sticky=tk.W, padx=4, pady=3)
        databits_combo = ttk.Combobox(
            config_frame,
            textvariable=self.databits_var,
            values=["5", "6", "7", "8"],
            width=5,
            state="readonly",
        )
        databits_combo.grid(row=0, column=6, sticky=tk.W, padx=2, pady=3)

        # Row 1: Stop Bits, Parity, Flow Control
        ttk.Label(config_frame, text="Stop Bits:").grid(row=1, column=0, sticky=tk.W, padx=4, pady=3)
        stopbits_combo = ttk.Combobox(
            config_frame,
            textvariable=self.stopbits_var,
            values=["1", "1.5", "2"],
            width=14,
            state="readonly",
        )
        stopbits_combo.grid(row=1, column=1, columnspan=2, sticky=tk.W, padx=2, pady=3)

        ttk.Label(config_frame, text="Parity:").grid(row=1, column=3, sticky=tk.W, padx=4, pady=3)
        parity_combo = ttk.Combobox(
            config_frame,
            textvariable=self.parity_var,
            values=["None", "Even", "Odd", "Mark", "Space"],
            width=9,
            state="readonly",
        )
        parity_combo.grid(row=1, column=4, sticky=tk.W, padx=2, pady=3)

        ttk.Label(config_frame, text="Flow Control:").grid(row=1, column=5, sticky=tk.W, padx=4, pady=3)
        flow_combo = ttk.Combobox(
            config_frame,
            textvariable=self.flowcontrol_var,
            values=["None", "Hardware (RTS/CTS)", "Software (XON/XOFF)", "DSR/DTR"],
            width=16,
            state="readonly",
        )
        flow_combo.grid(row=1, column=6, sticky=tk.W, padx=2, pady=3)

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

        # 2. Board Address Selection & Network Scanner Frame (Right of Serial Port Configuration)
        board_frame = ttk.LabelFrame(top_frame, text="Board Address Selection & Network Scanner (1 to 255)")
        board_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 0))

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

        # Network Device Scanner & List Box section
        scan_sep = ttk.Separator(board_frame, orient="horizontal")
        scan_sep.pack(fill=tk.X, pady=(4, 4))

        scan_container = ttk.Frame(board_frame)
        scan_container.pack(fill=tk.BOTH, expand=True, padx=2, pady=(0, 2))

        # Left Column: Scan Controls, Range & Progress
        scan_left = ttk.Frame(scan_container)
        scan_left.pack(side=tk.LEFT, fill=tk.Y, padx=(4, 12))

        scan_btn_row = ttk.Frame(scan_left)
        scan_btn_row.pack(fill=tk.X, pady=(0, 2))

        self.btn_scan = ttk.Button(
            scan_btn_row,
            text="SCAN",
            style="Action.TButton",
            command=self._start_scan,
        )
        self.btn_scan.pack(side=tk.LEFT, padx=(0, 4))

        self.btn_stop_scan = ttk.Button(
            scan_btn_row,
            text="Stop",
            state="disabled",
            command=self._stop_scan,
        )
        self.btn_stop_scan.pack(side=tk.LEFT, padx=2)

        scan_range_row = ttk.Frame(scan_left)
        scan_range_row.pack(fill=tk.X, pady=2)
        ttk.Label(scan_range_row, text="Range:").pack(side=tk.LEFT, padx=(0, 2))
        ttk.Spinbox(scan_range_row, from_=1, to=255, textvariable=self.scan_start_var, width=4).pack(side=tk.LEFT, padx=1)
        ttk.Label(scan_range_row, text="to").pack(side=tk.LEFT, padx=2)
        ttk.Spinbox(scan_range_row, from_=1, to=255, textvariable=self.scan_end_var, width=4).pack(side=tk.LEFT, padx=1)
        ttk.Label(scan_range_row, text="Timeout:").pack(side=tk.LEFT, padx=(6, 2))
        ttk.Spinbox(scan_range_row, from_=10, to=1000, increment=10, textvariable=self.scan_timeout_var, width=4).pack(side=tk.LEFT, padx=1)
        ttk.Label(scan_range_row, text="ms").pack(side=tk.LEFT, padx=(1, 0))

        self.lbl_scan_status = ttk.Label(scan_left, textvariable=self.scan_status_var, font=("Segoe UI", 8))
        self.lbl_scan_status.pack(anchor=tk.W, pady=(2, 0))

        self.scan_progress_bar = ttk.Progressbar(scan_left, orient="horizontal", length=200, mode="determinate")
        self.scan_progress_bar.pack(fill=tk.X, pady=(2, 0))

        # Right Column: Discovered Devices List Box with Scrollbar
        scan_right = ttk.Frame(scan_container)
        scan_right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 2))

        list_header = ttk.Frame(scan_right)
        list_header.pack(fill=tk.X)
        ttk.Label(list_header, text="Discovered Devices (Address & String):", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        self.lbl_device_count = ttk.Label(list_header, text="(0 found)", font=("Segoe UI", 8))
        self.lbl_device_count.pack(side=tk.LEFT, padx=4)

        ttk.Button(list_header, text="Use Selected", command=self._apply_selected_device).pack(side=tk.RIGHT, padx=2)
        ttk.Button(list_header, text="Clear List", command=self._clear_scan_results).pack(side=tk.RIGHT, padx=2)

        list_scroll_frame = ttk.Frame(scan_right)
        list_scroll_frame.pack(fill=tk.BOTH, expand=True, pady=(2, 0))

        self.devices_listbox = tk.Listbox(
            list_scroll_frame,
            height=3,
            font=("Consolas", 9),
            selectmode=tk.SINGLE,
            exportselection=False,
        )
        self.devices_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.devices_listbox.bind("<Double-Button-1>", lambda e: self._apply_selected_device())
        self.devices_listbox.bind("<<ListboxSelect>>", lambda e: self._on_device_list_selected())

        lb_scroll = ttk.Scrollbar(list_scroll_frame, orient="vertical", command=self.devices_listbox.yview)
        lb_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.devices_listbox.config(yscrollcommand=lb_scroll.set)

        # 3. 8-Relay Control Panel (Relays 0 to 7)
        relays_frame = ttk.LabelFrame(main_container, text="8-Relay Control Panel (Relays 0 to 7)")
        relays_frame.pack(fill=tk.X, pady=(0, 6))

        # Batch operations toolbar in 8-Relay Control Panel
        relay_toolbar = ttk.Frame(relays_frame)
        relay_toolbar.pack(fill=tk.X, pady=(0, 4))

        ttk.Label(relay_toolbar, text="Individual Channel Controls:", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=2)
        ttk.Button(relay_toolbar, text="Turn ALL Relays OFF", command=lambda: self._set_all_relays(False)).pack(
            side=tk.RIGHT, padx=4
        )
        ttk.Button(relay_toolbar, text="Turn ALL Relays ON", command=lambda: self._set_all_relays(True)).pack(
            side=tk.RIGHT, padx=4
        )

        # Grid of 8 relays (1 row of 8 columns for wide window)
        relays_grid = ttk.Frame(relays_frame)
        relays_grid.pack(fill=tk.X)

        for i in range(8):
            card = ttk.Frame(relays_grid, relief="ridge", borderwidth=2, padding=5)
            card.grid(row=0, column=i, padx=3, pady=2, sticky="nsew")
            relays_grid.columnconfigure(i, weight=1)

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
            ind.pack(pady=(0, 3))
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
            btn_toggle.pack(fill=tk.X, pady=(2, 0))
            self.relay_toggle_buttons[i] = btn_toggle

        # 4. Digital Input Ports Status (Inputs 0 to 7)
        inputs_frame = ttk.LabelFrame(main_container, text="Digital Input Ports (Inputs 0 to 7)")
        inputs_frame.pack(fill=tk.X, pady=(0, 6))

        ind_row_frame = ttk.Frame(inputs_frame)
        ind_row_frame.pack(fill=tk.X, pady=(1, 3))

        for i in range(8):
            ind_card = ttk.Frame(ind_row_frame, relief="groove", borderwidth=1, padding=3)
            ind_card.grid(row=0, column=i, padx=2, pady=1, sticky="nsew")
            ind_row_frame.columnconfigure(i, weight=1)

            ttk.Label(ind_card, text=f"Input {i}", style="InputNum.TLabel").pack(pady=(0, 1))

            self.input_state_vars[i] = tk.StringVar(value="LOW")
            input_ind = tk.Label(
                ind_card,
                textvariable=self.input_state_vars[i],
                bg="#6c757d",
                fg="white",
                font=("Segoe UI", 8, "bold"),
                width=8,
                pady=1,
            )
            input_ind.pack()
            self.input_indicator_labels[i] = input_ind

        # 5. 8 Analog Input Ports (AI 0 to 7, 4-20mA) & Unified Polling Controls
        analog_frame = ttk.LabelFrame(main_container, text="Analog Inputs (AI 0 to 7) — 4-20mA Current Inputs")
        analog_frame.pack(fill=tk.X, pady=(0, 6))

        ai_row_frame = ttk.Frame(analog_frame)
        ai_row_frame.pack(fill=tk.X, pady=(1, 4))

        for i in range(8):
            ai_card = ttk.Frame(ai_row_frame, relief="groove", borderwidth=1, padding=3)
            ai_card.grid(row=0, column=i, padx=2, pady=1, sticky="nsew")
            ai_row_frame.columnconfigure(i, weight=1)

            ttk.Label(ai_card, text=f"AI {i}", style="AINum.TLabel").pack(pady=(0, 1))

            self.analog_ma_vars[i] = tk.StringVar(value="4.00 mA")
            self.analog_raw_vars[i] = tk.StringVar(value="Raw: 0")

            lbl_ma = tk.Label(
                ai_card,
                textvariable=self.analog_ma_vars[i],
                bg="#23272e",
                fg="#61afef",
                font=("Segoe UI", 8, "bold"),
                width=9,
                pady=1,
            )
            lbl_ma.pack(pady=(0, 2))
            self.analog_ma_labels[i] = lbl_ma

            pb = ttk.Progressbar(ai_card, orient="horizontal", length=60, mode="determinate", maximum=100)
            pb.pack(fill=tk.X, padx=2, pady=(0, 2))
            pb["value"] = 0
            self.analog_progress_bars[i] = pb

            lbl_raw = ttk.Label(ai_card, textvariable=self.analog_raw_vars[i], font=("Consolas", 7))
            lbl_raw.pack()

        # Polling and Read Controls Bar
        poll_ctrl_frame = ttk.Frame(analog_frame)
        poll_ctrl_frame.pack(fill=tk.X, pady=(3, 2))

        ttk.Label(poll_ctrl_frame, text="Read Interval:").pack(side=tk.LEFT, padx=(4, 2))
        self.spin_interval = ttk.Spinbox(
            poll_ctrl_frame,
            from_=50,
            to=10000,
            increment=50,
            textvariable=self.read_interval_var,
            width=6,
        )
        self.spin_interval.pack(side=tk.LEFT, padx=2)
        ttk.Label(poll_ctrl_frame, text="ms").pack(side=tk.LEFT, padx=(2, 8))

        chk_auto = ttk.Checkbutton(
            poll_ctrl_frame,
            text="Auto Polling",
            variable=self.auto_read_var,
        )
        chk_auto.pack(side=tk.LEFT, padx=4)

        ttk.Label(poll_ctrl_frame, text="Scaling:").pack(side=tk.LEFT, padx=(6, 2))
        scale_combo = ttk.Combobox(
            poll_ctrl_frame,
            textvariable=self.analog_scale_mode_var,
            values=["Auto", "4000-20000", "0-20000", "12bit_adc", "10bit_adc", "16bit_adc"],
            width=10,
            state="readonly",
        )
        scale_combo.pack(side=tk.LEFT, padx=2)
        scale_combo.bind("<<ComboboxSelected>>", lambda e: self._update_analog_ui_states())

        btn_read_all = ttk.Button(
            poll_ctrl_frame,
            text="Read All Now",
            command=self._perform_read_all,
        )
        btn_read_all.pack(side=tk.RIGHT, padx=4)

        btn_read_analog = ttk.Button(
            poll_ctrl_frame,
            text="Read Analog (AI)",
            command=lambda: self._perform_read_analog_inputs(log_on_change_only=False),
        )
        btn_read_analog.pack(side=tk.RIGHT, padx=2)

        btn_read_dig = ttk.Button(
            poll_ctrl_frame,
            text="Read Digital",
            command=lambda: self._perform_read_inputs(log_on_change_only=False),
        )
        btn_read_dig.pack(side=tk.RIGHT, padx=2)

        # 6. Activity and Communication Log
        log_frame = ttk.LabelFrame(main_container, text="Communication & Activity Log")
        log_frame.pack(fill=tk.BOTH, expand=True)

        log_toolbar = ttk.Frame(log_frame)
        log_toolbar.pack(fill=tk.X, pady=(0, 2))

        ttk.Button(log_toolbar, text="Clear Log", command=self._clear_log).pack(side=tk.RIGHT, padx=4)
        ttk.Label(log_toolbar, text="Hex Frames Transmitted (TX) and Received (RX):", font=("Segoe UI", 9)).pack(
            side=tk.LEFT
        )

        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=5,
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
        self._last_logged_analogs = None
        self._update_relay_ui_states()
        self._update_input_ui_states()
        self._update_analog_ui_states()

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

    def _update_analog_ui_states(self, values: Optional[List[int]] = None):
        try:
            addr = int(self.address_var.get())
        except (ValueError, tk.TclError):
            addr = 1

        scale_mode = self.analog_scale_mode_var.get().lower()

        for i in range(8):
            if values is not None and i < len(values):
                raw_val = values[i]
            else:
                raw_val = self.controller.get_analog_state(addr, i)

            ma_val = raw_to_ma(raw_val, scale_mode=scale_mode)
            self.analog_ma_vars[i].set(f"{ma_val:.2f} mA")
            self.analog_raw_vars[i].set(f"Raw: {raw_val}")

            # Percentage for 4-20mA progress bar (4mA = 0%, 20mA = 100%)
            pct = ((ma_val - 4.0) / 16.0) * 100.0 if ma_val >= 4.0 else 0.0
            pct = max(0.0, min(100.0, pct))
            if i in self.analog_progress_bars:
                self.analog_progress_bars[i]["value"] = pct

            # Badge color coding
            if i in self.analog_ma_labels:
                if ma_val < 3.8:  # Open loop / low fault
                    self.analog_ma_labels[i].config(fg="#e5c07b", bg="#2c2820")
                elif ma_val > 20.5:  # Over-range / high fault
                    self.analog_ma_labels[i].config(fg="#e06c75", bg="#322225")
                else:  # Normal 4-20mA range
                    self.analog_ma_labels[i].config(fg="#61afef", bg="#1f2d3d")

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
            self._last_logged_analogs = None
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
        if self.controller.is_connected and self.auto_read_var.get() and not self._is_scanning:
            self._perform_read_inputs(log_on_change_only=True)
            self._perform_read_analog_inputs(log_on_change_only=True)
        self._schedule_next_poll()

    def _perform_read_all(self):
        self._perform_read_inputs(log_on_change_only=False)
        self._perform_read_analog_inputs(log_on_change_only=False)

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
                        self._log_tx(f"Board 0x{addr:02X} ({addr}) -> Read Digital Inputs (0-7): [ {cmd_hex} ]")
                    self._log_rx(f"Board 0x{addr:02X} ({addr}) <- Digital: [ {resp_hex} ] ({state_summary})")
                    self._last_logged_inputs = list(states)
                self._update_input_ui_states(states)
            else:
                if not log_on_change_only:
                    self._log_tx(f"Board 0x{addr:02X} ({addr}) -> Read Digital Inputs (0-7): [ {cmd_hex} ]")
                    if not self.controller.is_connected:
                        self._log_info(f"(Offline test mode: Digital read command [ {cmd_hex} ] prepared)")
                    else:
                        resp_hex = parse_hex_string(resp) if resp else "(No response/timeout)"
                        self._log_error(f"Board 0x{addr:02X} digital read response invalid: {resp_hex}")
        except Exception as ex:
            if not log_on_change_only:
                self._log_error(f"Error reading digital inputs: {ex}")

    def _perform_read_analog_inputs(self, log_on_change_only: bool = False):
        try:
            addr = int(self.address_var.get())
            if not (1 <= addr <= 255):
                return
        except Exception:
            return

        try:
            cmd, resp, registers = self.controller.read_analog_inputs(addr, count=8)
            cmd_hex = parse_hex_string(cmd)

            if registers is not None:
                reg_changed = (self._last_logged_analogs != registers)
                if not log_on_change_only or reg_changed:
                    resp_hex = parse_hex_string(resp)
                    scale_mode = self.analog_scale_mode_var.get().lower()
                    ai_summary = " ".join([f"AI{i}:{raw_to_ma(v, scale_mode):.2f}mA" for i, v in enumerate(registers)])
                    if not log_on_change_only:
                        self._log_tx(f"Board 0x{addr:02X} ({addr}) -> Read Analog Inputs 4-20mA (0-7): [ {cmd_hex} ]")
                    self._log_rx(f"Board 0x{addr:02X} ({addr}) <- Analog: [ {resp_hex} ] ({ai_summary})")
                    self._last_logged_analogs = list(registers)
                self._update_analog_ui_states(registers)
            else:
                if not log_on_change_only:
                    self._log_tx(f"Board 0x{addr:02X} ({addr}) -> Read Analog Inputs 4-20mA (0-7): [ {cmd_hex} ]")
                    if not self.controller.is_connected:
                        self._log_info(f"(Offline test mode: Analog read command [ {cmd_hex} ] prepared)")
                    else:
                        resp_hex = parse_hex_string(resp) if resp else "(No response/timeout)"
                        self._log_error(f"Board 0x{addr:02X} analog read response invalid: {resp_hex}")
        except Exception as ex:
            if not log_on_change_only:
                self._log_error(f"Error reading analog inputs: {ex}")

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

    def _start_scan(self):
        if not self.controller.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to a serial port first to scan the network.")
            self._log_error("Scan failed: Serial port is not connected.")
            return

        if self._is_scanning:
            return

        try:
            start_addr = int(self.scan_start_var.get())
            end_addr = int(self.scan_end_var.get())
        except (ValueError, tk.TclError):
            start_addr, end_addr = 1, 255

        try:
            timeout_ms = float(self.scan_timeout_var.get())
            timeout_per_device = max(0.01, timeout_ms / 1000.0)
        except (ValueError, tk.TclError):
            timeout_per_device = 0.06

        start_addr = max(1, min(255, start_addr))
        end_addr = max(1, min(255, end_addr))
        if start_addr > end_addr:
            start_addr, end_addr = end_addr, start_addr

        self._is_scanning = True
        self._scan_stop_event = threading.Event()
        self.discovered_devices.clear()
        self.devices_listbox.delete(0, tk.END)
        self.lbl_device_count.config(text="(0 found)")
        self.scan_progress_bar["value"] = 0
        self.btn_scan.config(state="disabled")
        self.btn_stop_scan.config(state="normal")
        self.scan_status_var.set(f"Scanning addresses {start_addr} to {end_addr}...")

        self._log_info(f"Starting Modbus RTU network scan for addresses {start_addr} to {end_addr} (timeout {int(timeout_per_device*1000)}ms)...")

        def scan_worker():
            def on_progress(curr_addr, total, found_list, responses=None):
                self.root.after(0, self._on_scan_progress, curr_addr, total, found_list, start_addr, end_addr, responses)

            try:
                self.controller.scan_network(
                    start_address=start_addr,
                    end_address=end_addr,
                    timeout_per_device=timeout_per_device,
                    progress_callback=on_progress,
                    stop_event=self._scan_stop_event,
                )
            except Exception as ex:
                self.root.after(0, self._log_error, f"Scan encountered error: {ex}")
            finally:
                self.root.after(0, self._on_scan_finished)

        self._scan_thread = threading.Thread(target=scan_worker, daemon=True)
        self._scan_thread.start()

    def _stop_scan(self):
        if self._is_scanning and self._scan_stop_event is not None:
            self._scan_stop_event.set()
            self.scan_status_var.set("Stopping scan...")

    def _on_scan_progress(
        self,
        current_addr: int,
        total: int,
        found_list: List[int],
        start_addr: int,
        end_addr: int,
        responses: Optional[Dict[int, bytes]] = None,
    ):
        if not self._is_scanning:
            return

        progress_pct = ((current_addr - start_addr + 1) / total) * 100.0
        self.scan_progress_bar["value"] = progress_pct

        # Check for newly discovered devices
        for dev_addr in found_list:
            if dev_addr not in self.discovered_devices:
                self.discovered_devices.append(dev_addr)
                resp_bytes = (responses or {}).get(dev_addr) or self.controller.last_scan_device_responses.get(dev_addr, b"")
                resp_str = format_device_response_string(resp_bytes) if resp_bytes else ""
                if resp_str:
                    item_text = f"Address {dev_addr:3d} (0x{dev_addr:02X})  |  String: {resp_str}"
                    self._log_info(f"Discovered device at Address {dev_addr} (0x{dev_addr:02X}) | String: {resp_str}")
                else:
                    item_text = f"Address {dev_addr:3d} (0x{dev_addr:02X})"
                    self._log_info(f"Discovered device at Address {dev_addr} (0x{dev_addr:02X})")

                self.devices_listbox.insert(tk.END, item_text)
                self.devices_listbox.see(tk.END)

        count = len(self.discovered_devices)
        self.lbl_device_count.config(text=f"({count} found)")
        self.scan_status_var.set(f"Scanning address {current_addr}/{end_addr} ({count} found)...")

    def _on_scan_finished(self):
        self._is_scanning = False
        self.btn_scan.config(state="normal")
        self.btn_stop_scan.config(state="disabled")
        self.scan_progress_bar["value"] = 100

        count = len(self.discovered_devices)
        if self._scan_stop_event and self._scan_stop_event.is_set():
            msg = f"Scan stopped. Found {count} device(s)."
        else:
            msg = f"Scan completed. Found {count} device(s)."

        self.scan_status_var.set(msg)
        self.lbl_device_count.config(text=f"({count} found)")
        self._log_info(f"{msg} Discovered addresses: {self.discovered_devices}")

    def _on_device_list_selected(self):
        sel = self.devices_listbox.curselection()
        if sel and sel[0] < len(self.discovered_devices):
            addr = self.discovered_devices[sel[0]]
            self.address_var.set(addr)
            self._on_address_changed()
            resp_bytes = self.controller.last_scan_device_responses.get(addr, b"")
            if resp_bytes:
                resp_str = format_device_response_string(resp_bytes)
                self.scan_status_var.set(f"Selected: Address {addr} (0x{addr:02X}) | String: {resp_str}")

    def _apply_selected_device(self):
        sel = self.devices_listbox.curselection()
        if sel and sel[0] < len(self.discovered_devices):
            addr = self.discovered_devices[sel[0]]
            self.address_var.set(addr)
            self._on_address_changed()
            resp_bytes = self.controller.last_scan_device_responses.get(addr, b"")
            resp_str = format_device_response_string(resp_bytes) if resp_bytes else ""
            if resp_str:
                self._log_info(f"Switched active board address to {addr} (0x{addr:02X}) | String: {resp_str}")
            else:
                self._log_info(f"Switched active board address to {addr} (0x{addr:02X})")
        else:
            if self.discovered_devices:
                addr = self.discovered_devices[0]
                self.address_var.set(addr)
                self._on_address_changed()
                resp_bytes = self.controller.last_scan_device_responses.get(addr, b"")
                resp_str = format_device_response_string(resp_bytes) if resp_bytes else ""
                if resp_str:
                    self._log_info(f"Switched active board address to {addr} (0x{addr:02X}) | String: {resp_str}")
                else:
                    self._log_info(f"Switched active board address to {addr} (0x{addr:02X})")

    def _clear_scan_results(self):
        self.discovered_devices.clear()
        self.devices_listbox.delete(0, tk.END)
        self.lbl_device_count.config(text="(0 found)")
        self.scan_status_var.set("Status: Ready to scan (1-255)")
        self.scan_progress_bar["value"] = 0

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
