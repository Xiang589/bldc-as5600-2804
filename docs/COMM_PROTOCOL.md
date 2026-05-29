# UART DMA Motor Command Protocol

This document describes the first UART command protocol for the STM32F103C8T6 + AS5600 + 2804 BLDC demo.

The firmware uses USART2 at `115200 8N1`. RX and TX use DMA. UART callbacks only maintain DMA/RX/TX state; command parsing and motor control happen in `CommTask`.

## Safety Model

- Power-up state is `disabled`, `IDLE`, `target=0`.
- `ENABLE 1` only arms command acceptance. It does not start the motor by itself.
- `VEL`, `POS`, `VOLT`, and `TARGET` return `ERR DISABLED` before `ENABLE 1`.
- `CommTask` never touches PWM/TIM directly. It calls `motor_command`, which calls the public `motor_control` API.
- If the motor is enabled and no control command or `KEEPALIVE` is received for 1000 ms, the firmware stops/disables the motor and sends one `EVT COMM_TIMEOUT`.
- `ZERO` only runs the existing software FOC zero calibration. It never writes AS5600 OTP/BURN registers.
- `PIDV` and `PIDP` currently return `ERR UNSUPPORTED`; runtime PID tuning is not implemented yet.

## Line Format

Commands are ASCII text lines terminated by `\n` or `\r\n`.

- Command names are case-insensitive.
- Arguments are separated by spaces or tabs.
- Empty lines are ignored.
- Maximum line length is 128 bytes. Longer lines return `ERR ARG line too long`.
- One UART packet may contain partial lines or multiple commands.

Responses:

```text
OK <CMD> ...
ERR <code> <message>
EVT <event> ...
```

## Units

- `VEL` uses rad/s.
- `POS` uses rad.
- `VOLT` and the first `LIMIT` argument use volts.
- Firmware parsing converts decimal command arguments to fixed-point milli-units internally.
- `STATUS?` reports `target`, `angle`, `vel`, `vlim`, and `wlim` as milli-units:
  - `target`: current mode target in mV, mrad/s, or mrad.
  - `angle`: mrad.
  - `vel`: mrad/s.
  - `vlim`: mV.
  - `wlim`: mrad/s.
- AS5600 `raw` is 0..4095.
- `md/ml/mh` are AS5600 magnet detected / too weak / too strong flags.

## Commands

| Command | Description |
| --- | --- |
| `PING` | Link test. Returns `OK PING`. |
| `HELP` | Prints the supported command list. |
| `ID?` | Returns firmware protocol identity. |
| `STATUS?` | Returns enable, mode, target, AS5600 feedback, limits, Uq, and FOC zero flag. |
| `ENABLE <0\|1>` | Disable or arm command acceptance. `ENABLE 1` does not start output. |
| `MODE <IDLE\|OPEN\|VEL\|POS>` | Select logical command mode. `POS` target is initialized to current position. |
| `TARGET <float>` | Apply target for current logical mode. In `OPEN`, the target is treated as an open-loop voltage command. |
| `VOLT <float>` | Apply voltage-mode FOC command in volts. |
| `VEL <float>` | Apply FOC velocity target in rad/s. |
| `POS <float>` | Apply FOC position target in rad. |
| `LIMIT <voltage_limit> <velocity_limit>` | Set command-layer voltage and velocity limits. |
| `PIDV ...` | Reserved for future velocity PID tuning. Currently unsupported. |
| `PIDP ...` | Reserved for future position PID tuning. Currently unsupported. |
| `STREAM <period_ms\|0>` | Send `EVT STATUS` every 20..5000 ms, or stop stream with `0`. |
| `STOP` | Stop motor output and disable command output. |
| `ESTOP` | Emergency stop and latch disabled state until `ENABLE 1`. |
| `ZERO` | Run existing software FOC zero calibration; no AS5600 OTP/BURN access. |
| `KEEPALIVE` | Refresh communication watchdog. |

## Example Session

```text
PING
OK PING

ENABLE 1
OK ENABLE

VEL 5
OK VEL

STATUS?
OK STATUS en=1 mode=VEL target=5000 angle=1234 vel=4980 raw=321 md=1 ml=0 mh=0 vlim=1000 wlim=12566 uq=420 z=1

STOP
OK STOP
```

## Python CLI

The helper script only depends on `pyserial`:

```bash
python tools/motor_comm_cli.py --port COM5 ping
python tools/motor_comm_cli.py --port COM5 status
python tools/motor_comm_cli.py --port COM5 enable
python tools/motor_comm_cli.py --port COM5 vel 5
python tools/motor_comm_cli.py --port COM5 stop
python tools/motor_comm_cli.py --port COM5 stream 100
python tools/motor_comm_cli.py --port COM5 interactive
```

## Implementation Boundaries

- `comm_uart_dma`: circular RX DMA polling, non-blocking TX DMA, TX busy state.
- `comm_task`: line assembly, command execution, stream status, watchdog.
- `comm_protocol`: HAL/RTOS-independent parser.
- `motor_command`: safety wrapper around `motor_control`.

The FOC voltage/speed/position paths remain experimental. This protocol does not make FOC, PID tuning, current loop, torque loop, or production safety complete.
