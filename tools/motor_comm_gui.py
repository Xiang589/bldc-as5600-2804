#!/usr/bin/env python3
"""Tkinter GUI for the STM32 AS5600 BLDC motor UART command protocol."""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
except ImportError:  # pragma: no cover - depends on local environment
    serial = None


DEFAULT_PORT = "COM5"
DEFAULT_BAUD = "115200"
DEFAULT_REFRESH_MS = "500"
READ_TIMEOUT_S = 0.1
WRITE_TIMEOUT_S = 0.5
QUEUE_POLL_MS = 50

STATUS_FIELDS = (
    "en",
    "mode",
    "target",
    "angle",
    "vel",
    "raw",
    "md",
    "ml",
    "mh",
    "vlim",
    "wlim",
    "uq",
    "z",
)


class SerialSession:
    """Small serial wrapper with a background line reader."""

    def __init__(self, rx_queue: "queue.Queue[tuple[str, str]]") -> None:
        self.rx_queue = rx_queue
        self.ser = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    def is_connected(self) -> bool:
        return self.ser is not None and bool(self.ser.is_open)

    def connect(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        if self.is_connected():
            return
        self.stop_event.clear()
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=READ_TIMEOUT_S,
            write_timeout=WRITE_TIMEOUT_S,
        )
        self.thread = threading.Thread(target=self._read_loop, name="motor-uart-rx", daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.stop_event.set()
        ser = self.ser
        if ser is not None:
            try:
                ser.close()
            except Exception as exc:  # pragma: no cover - hardware dependent
                self.rx_queue.put(("err", f"serial close failed: {exc}"))
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.thread = None
        self.ser = None

    def send_line(self, line: str) -> None:
        ser = self.ser
        if ser is None or not ser.is_open:
            raise RuntimeError("serial port is not connected")
        data = (line.strip() + "\n").encode("ascii", errors="replace")
        with self.write_lock:
            ser.write(data)
            ser.flush()

    def _read_loop(self) -> None:
        while not self.stop_event.is_set():
            ser = self.ser
            if ser is None:
                break
            try:
                raw = ser.readline()
            except Exception as exc:  # pragma: no cover - hardware dependent
                if not self.stop_event.is_set():
                    self.rx_queue.put(("err", f"serial read failed: {exc}"))
                break
            if raw:
                text = raw.decode("ascii", errors="replace").strip()
                if text:
                    self.rx_queue.put(("rx", text))


class MotorCommGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("BLDC AS5600 Motor Control")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.rx_queue: "queue.Queue[tuple[str, str]]" = queue.Queue()
        self.session = SerialSession(self.rx_queue)
        self.status_vars = {name: tk.StringVar(value="--") for name in STATUS_FIELDS}
        self.stop_reason_var = tk.StringVar(value="--")
        self.fault_var = tk.StringVar(value="--")
        self.magnet_var = tk.StringVar(value="--")
        self.connection_var = tk.StringVar(value="Disconnected")

        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD)
        self.refresh_var = tk.StringVar(value=DEFAULT_REFRESH_MS)
        self.voltage_var = tk.StringVar(value="0.2")
        self.velocity_var = tk.StringVar(value="0.5")
        self.position_var = tk.StringVar(value="0.1")
        self.limit_voltage_var = tk.StringVar(value="1.0")
        self.limit_velocity_var = tk.StringVar(value="5.0")
        self.manual_var = tk.StringVar()

        self.control_buttons: list[tk.Widget] = []
        self.status_due = time.monotonic()

        self._build_ui()
        self._set_connected_ui(False)
        self.root.after(QUEUE_POLL_MS, self._poll_gui)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(5, weight=1)

        self._build_connection_frame()
        self._build_status_frame()
        self._build_control_frame()
        self._build_target_frame()
        self._build_manual_frame()
        self._build_log_frame()

    def _build_connection_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Connection")
        frame.grid(row=0, column=0, padx=8, pady=6, sticky="ew")
        for col in (1, 3, 5):
            frame.columnconfigure(col, weight=1)

        ttk.Label(frame, text="Port").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(frame, textvariable=self.port_var, width=12).grid(row=0, column=1, padx=4, pady=4)
        ttk.Label(frame, text="Baud").grid(row=0, column=2, padx=4, pady=4)
        ttk.Entry(frame, textvariable=self.baud_var, width=10).grid(row=0, column=3, padx=4, pady=4)
        ttk.Label(frame, text="Refresh ms").grid(row=0, column=4, padx=4, pady=4)
        ttk.Entry(frame, textvariable=self.refresh_var, width=8).grid(row=0, column=5, padx=4, pady=4)

        self.connect_button = ttk.Button(frame, text="Connect", command=self.connect)
        self.disconnect_button = ttk.Button(frame, text="Disconnect", command=self.disconnect)
        self.connect_button.grid(row=0, column=6, padx=4, pady=4)
        self.disconnect_button.grid(row=0, column=7, padx=4, pady=4)
        ttk.Label(frame, textvariable=self.connection_var, width=14).grid(row=0, column=8, padx=4, pady=4)

    def _build_status_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Status")
        frame.grid(row=1, column=0, padx=8, pady=6, sticky="ew")
        for col in range(8):
            frame.columnconfigure(col, weight=1)

        for idx, name in enumerate(STATUS_FIELDS):
            row = idx // 4
            col = (idx % 4) * 2
            ttk.Label(frame, text=f"{name}:").grid(row=row, column=col, padx=4, pady=3, sticky="e")
            ttk.Label(frame, textvariable=self.status_vars[name], width=12).grid(
                row=row, column=col + 1, padx=4, pady=3, sticky="w"
            )

        self.magnet_label = tk.Label(frame, textvariable=self.magnet_var, width=14, relief="groove")
        self.magnet_label.grid(row=4, column=0, columnspan=2, padx=4, pady=4, sticky="w")
        ttk.Label(frame, text="stop_reason:").grid(row=4, column=2, padx=4, pady=4, sticky="e")
        ttk.Label(frame, textvariable=self.stop_reason_var, width=8).grid(
            row=4, column=3, padx=4, pady=4, sticky="w"
        )
        ttk.Label(frame, text="fault:").grid(row=4, column=4, padx=4, pady=4, sticky="e")
        ttk.Label(frame, textvariable=self.fault_var, width=8).grid(
            row=4, column=5, padx=4, pady=4, sticky="w"
        )

    def _build_control_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Controls")
        frame.grid(row=2, column=0, padx=8, pady=6, sticky="ew")

        buttons = (
            ("ENABLE", lambda: self.send_command("ENABLE 1")),
            ("DISABLE", lambda: self.send_command("ENABLE 0")),
            ("ZERO / FCAL", self.confirm_zero),
            ("STOP", lambda: self.send_command("STOP")),
        )
        for col, (text, command) in enumerate(buttons):
            button = ttk.Button(frame, text=text, command=command)
            button.grid(row=0, column=col, padx=4, pady=4, sticky="ew")
            self.control_buttons.append(button)

        self.estop_button = tk.Button(
            frame,
            text="ESTOP",
            command=lambda: self.send_command("ESTOP"),
            bg="#c62828",
            fg="white",
            activebackground="#b71c1c",
            activeforeground="white",
        )
        self.estop_button.grid(row=0, column=4, padx=4, pady=4, sticky="ew")
        self.control_buttons.append(self.estop_button)
        for col in range(5):
            frame.columnconfigure(col, weight=1)

    def _build_target_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Targets and Limits")
        frame.grid(row=3, column=0, padx=8, pady=6, sticky="ew")
        for col in range(6):
            frame.columnconfigure(col, weight=1)

        self._add_target_row(frame, 0, "FOC Voltage Uq (V)", self.voltage_var, -3.0, 3.0, "VOLT")
        self._add_target_row(frame, 1, "Velocity (rad/s)", self.velocity_var, -20.0, 20.0, "VEL")
        self._add_target_row(frame, 2, "Position (rad)", self.position_var, -6.28, 6.28, "POS")

        ttk.Label(frame, text="voltage_limit").grid(row=3, column=0, padx=4, pady=4, sticky="e")
        ttk.Entry(frame, textvariable=self.limit_voltage_var, width=10).grid(
            row=3, column=1, padx=4, pady=4, sticky="ew"
        )
        ttk.Label(frame, text="velocity_limit").grid(row=3, column=2, padx=4, pady=4, sticky="e")
        ttk.Entry(frame, textvariable=self.limit_velocity_var, width=10).grid(
            row=3, column=3, padx=4, pady=4, sticky="ew"
        )
        button = ttk.Button(frame, text="Send LIMIT", command=self.send_limits)
        button.grid(row=3, column=4, columnspan=2, padx=4, pady=4, sticky="ew")
        self.control_buttons.append(button)

    def _add_target_row(
        self,
        frame: ttk.LabelFrame,
        row: int,
        label: str,
        variable: tk.StringVar,
        min_value: float,
        max_value: float,
        command_name: str,
    ) -> None:
        ttk.Label(frame, text=label).grid(row=row, column=0, padx=4, pady=4, sticky="e")
        ttk.Entry(frame, textvariable=variable, width=10).grid(row=row, column=1, padx=4, pady=4, sticky="ew")
        ttk.Label(frame, text=f"{min_value:g}..{max_value:g}").grid(
            row=row, column=2, padx=4, pady=4, sticky="w"
        )
        button = ttk.Button(
            frame,
            text=f"Send {command_name}",
            command=lambda: self.send_bounded_value(command_name, variable, min_value, max_value),
        )
        button.grid(row=row, column=3, columnspan=3, padx=4, pady=4, sticky="ew")
        self.control_buttons.append(button)

    def _build_manual_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Manual Command")
        frame.grid(row=4, column=0, padx=8, pady=6, sticky="ew")
        frame.columnconfigure(0, weight=1)

        entry = ttk.Entry(frame, textvariable=self.manual_var)
        entry.grid(row=0, column=0, padx=4, pady=4, sticky="ew")
        entry.bind("<Return>", lambda _event: self.send_manual())
        button = ttk.Button(frame, text="Send", command=self.send_manual)
        button.grid(row=0, column=1, padx=4, pady=4)
        self.control_buttons.append(button)

    def _build_log_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Log")
        frame.grid(row=5, column=0, padx=8, pady=6, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(frame, height=14, wrap="word", state="disabled")
        self.log_text.grid(row=0, column=0, padx=4, pady=4, sticky="nsew")
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)
        ttk.Button(frame, text="Clear Log", command=self.clear_log).grid(row=1, column=0, padx=4, pady=4, sticky="e")

    def connect(self) -> None:
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud rate must be an integer.")
            return
        if not port:
            messagebox.showerror("Invalid port", "Serial port cannot be empty.")
            return

        try:
            self.session.connect(port, baud)
        except Exception as exc:
            self.log(f"ERR: connect failed: {exc}")
            messagebox.showerror("Connect failed", str(exc))
            self._set_connected_ui(False)
            return

        self._set_connected_ui(True)
        self.log(f"Connected to {port} at {baud}")
        self.send_command("PING")
        self.send_command("STATUS?")
        self.status_due = time.monotonic() + self._refresh_interval_s()

    def disconnect(self) -> None:
        self.session.disconnect()
        self._set_connected_ui(False)
        self.log("Disconnected")

    def confirm_zero(self) -> None:
        if not self.session.is_connected():
            return
        ok = messagebox.askyesno(
            "Confirm FOC zero calibration",
            "ZERO / FCAL applies a short alignment voltage and may move or hold the motor briefly.\n\nContinue?",
        )
        if ok:
            self.send_command("ZERO")

    def send_bounded_value(
        self,
        command_name: str,
        variable: tk.StringVar,
        min_value: float,
        max_value: float,
    ) -> None:
        value = self._parse_float(variable.get(), command_name)
        if value is None:
            return
        if value < min_value or value > max_value:
            messagebox.showerror(
                "Out of range",
                f"{command_name} must be between {min_value:g} and {max_value:g}.",
            )
            return
        self.send_command(f"{command_name} {value:g}")

    def send_limits(self) -> None:
        voltage = self._parse_float(self.limit_voltage_var.get(), "voltage_limit")
        velocity = self._parse_float(self.limit_velocity_var.get(), "velocity_limit")
        if voltage is None or velocity is None:
            return
        if voltage < 0.0 or velocity < 0.0:
            messagebox.showerror("Out of range", "LIMIT values must be non-negative.")
            return
        self.send_command(f"LIMIT {voltage:g} {velocity:g}")

    def send_manual(self) -> None:
        command = self.manual_var.get().strip()
        if not command:
            return
        self.send_command(command)
        self.manual_var.set("")

    def send_command(self, command: str) -> bool:
        if not self.session.is_connected():
            self.log("ERR: serial port is not connected")
            return False
        try:
            self.session.send_line(command)
        except Exception as exc:
            self.log(f"ERR: send failed: {exc}")
            self.session.disconnect()
            self._set_connected_ui(False)
            return False
        self.log(f"TX: {command}")
        return True

    def _parse_float(self, text: str, name: str) -> float | None:
        try:
            return float(text.strip())
        except ValueError:
            messagebox.showerror("Invalid input", f"{name} must be a number.")
            return None

    def _refresh_interval_s(self) -> float:
        try:
            value = int(self.refresh_var.get().strip())
        except ValueError:
            value = 500
        if value < 100:
            value = 100
        if value > 10000:
            value = 10000
        return value / 1000.0

    def _poll_gui(self) -> None:
        self._process_rx_queue()
        if self.session.is_connected() and time.monotonic() >= self.status_due:
            self.send_command("STATUS?")
            self.status_due = time.monotonic() + self._refresh_interval_s()
        self.root.after(QUEUE_POLL_MS, self._poll_gui)

    def _process_rx_queue(self) -> None:
        while True:
            try:
                kind, text = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "rx":
                self.log(f"RX: {text}")
                self._handle_rx_line(text)
            else:
                self.log(f"ERR: {text}")
                self.session.disconnect()
                self._set_connected_ui(False)

    def _handle_rx_line(self, line: str) -> None:
        tokens = line.split()
        if len(tokens) >= 2 and tokens[0] in {"OK", "EVT"} and tokens[1] == "STATUS":
            fields = self._parse_fields(tokens[2:])
            self._update_status(fields)
        elif len(tokens) >= 2 and tokens[0] == "ERR" and tokens[1] == "STATE":
            fields = self._parse_fields(tokens[2:])
            if "sr" in fields:
                self.stop_reason_var.set(fields["sr"])
            if "fault" in fields:
                self.fault_var.set(fields["fault"])

    def _parse_fields(self, tokens: list[str]) -> dict[str, str]:
        fields: dict[str, str] = {}
        for token in tokens:
            if "=" not in token:
                continue
            name, value = token.split("=", 1)
            fields[name] = value
        return fields

    def _update_status(self, fields: dict[str, str]) -> None:
        for name in STATUS_FIELDS:
            self.status_vars[name].set(fields.get(name, "--"))
        self._update_magnet_status(fields)

    def _update_magnet_status(self, fields: dict[str, str]) -> None:
        md = fields.get("md")
        ml = fields.get("ml")
        mh = fields.get("mh")
        if md == "1" and ml == "0" and mh == "0":
            self.magnet_var.set("Mag OK")
            self.magnet_label.configure(bg="#2e7d32", fg="white")
        elif md == "0":
            self.magnet_var.set("No magnet")
            self.magnet_label.configure(bg="#c62828", fg="white")
        elif ml == "1":
            self.magnet_var.set("Mag weak")
            self.magnet_label.configure(bg="#ef6c00", fg="white")
        elif mh == "1":
            self.magnet_var.set("Mag strong")
            self.magnet_label.configure(bg="#ef6c00", fg="white")
        else:
            self.magnet_var.set("--")
            self.magnet_label.configure(bg=self.root.cget("bg"), fg="black")

    def _set_connected_ui(self, connected: bool) -> None:
        self.connection_var.set("Connected" if connected else "Disconnected")
        self.connect_button.configure(state="disabled" if connected else "normal")
        self.disconnect_button.configure(state="normal" if connected else "disabled")
        for button in self.control_buttons:
            button.configure(state="normal" if connected else "disabled")

    def log(self, text: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def on_close(self) -> None:
        if self.session.is_connected():
            try:
                self.session.send_line("STOP")
                self.log("TX: STOP")
            except Exception as exc:
                self.log(f"ERR: stop on close failed: {exc}")
            self.session.disconnect()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    MotorCommGui(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
