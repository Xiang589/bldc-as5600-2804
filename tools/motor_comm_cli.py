#!/usr/bin/env python3
"""Small serial CLI for the STM32 AS5600 BLDC motor command protocol."""

from __future__ import annotations

import argparse
import sys
import time

import serial


DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.4


def open_serial(args: argparse.Namespace) -> serial.Serial:
    return serial.Serial(args.port, args.baud, timeout=READ_TIMEOUT)


def send_line(ser: serial.Serial, line: str) -> None:
    ser.write((line.strip() + "\n").encode("ascii"))
    ser.flush()


def read_available(ser: serial.Serial, wait_s: float = READ_TIMEOUT) -> list[str]:
    deadline = time.monotonic() + wait_s
    lines: list[str] = []
    while time.monotonic() < deadline:
        raw = ser.readline()
        if raw:
            lines.append(raw.decode("ascii", errors="replace").rstrip())
            deadline = time.monotonic() + 0.05
        else:
            time.sleep(0.01)
    return lines


def request(args: argparse.Namespace, command: str) -> int:
    with open_serial(args) as ser:
        ser.reset_input_buffer()
        send_line(ser, command)
        for line in read_available(ser):
            print(line)
    return 0


def stream(args: argparse.Namespace) -> int:
    with open_serial(args) as ser:
        ser.reset_input_buffer()
        send_line(ser, f"STREAM {args.period_ms}")
        for line in read_available(ser):
            print(line)
        if args.period_ms == 0:
            return 0
        try:
            while True:
                raw = ser.readline()
                if raw:
                    print(raw.decode("ascii", errors="replace").rstrip())
        except KeyboardInterrupt:
            send_line(ser, "STREAM 0")
            return 0


def interactive(args: argparse.Namespace) -> int:
    with open_serial(args) as ser:
        ser.reset_input_buffer()
        print("Enter protocol commands. Ctrl-C or EOF exits.")
        try:
            while True:
                text = input("> ").strip()
                if not text:
                    continue
                send_line(ser, text)
                for line in read_available(ser):
                    print(line)
        except (KeyboardInterrupt, EOFError):
            print()
            return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32 BLDC UART command CLI")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")

    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping")
    sub.add_parser("status")
    sub.add_parser("enable")
    sub.add_parser("disable")

    mode_p = sub.add_parser("mode")
    mode_p.add_argument("mode", choices=["idle", "open", "vel", "pos"])

    vel_p = sub.add_parser("vel")
    vel_p.add_argument("rad_s", type=float)

    pos_p = sub.add_parser("pos")
    pos_p.add_argument("rad", type=float)

    volt_p = sub.add_parser("volt")
    volt_p.add_argument("volts", type=float)

    limit_p = sub.add_parser("limit")
    limit_p.add_argument("volts", type=float)
    limit_p.add_argument("rad_s", type=float)

    sub.add_parser("stop")
    sub.add_parser("estop")
    sub.add_parser("zero")
    sub.add_parser("keepalive")

    stream_p = sub.add_parser("stream")
    stream_p.add_argument("period_ms", type=int)

    sub.add_parser("interactive")
    return parser


def command_from_args(args: argparse.Namespace) -> str | None:
    if args.cmd == "ping":
        return "PING"
    if args.cmd == "status":
        return "STATUS?"
    if args.cmd == "enable":
        return "ENABLE 1"
    if args.cmd == "disable":
        return "ENABLE 0"
    if args.cmd == "mode":
        return f"MODE {args.mode.upper()}"
    if args.cmd == "vel":
        return f"VEL {args.rad_s:g}"
    if args.cmd == "pos":
        return f"POS {args.rad:g}"
    if args.cmd == "volt":
        return f"VOLT {args.volts:g}"
    if args.cmd == "limit":
        return f"LIMIT {args.volts:g} {args.rad_s:g}"
    if args.cmd == "stop":
        return "STOP"
    if args.cmd == "estop":
        return "ESTOP"
    if args.cmd == "zero":
        return "ZERO"
    if args.cmd == "keepalive":
        return "KEEPALIVE"
    return None


def main(argv: list[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.cmd == "stream":
        return stream(args)
    if args.cmd == "interactive":
        return interactive(args)

    command = command_from_args(args)
    if command is None:
        parser.error("unsupported command")
    return request(args, command)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
