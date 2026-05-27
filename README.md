# STM32 AS5600 BLDC Motor Control Platform

基于 STM32F103C8T6、AS5600 磁编码器与三相 BLDC 驱动的电机控制实验平台。

This repository is a board-validated embedded motor-control experiment, not a production-ready FOC controller. The current firmware has completed OPEN-mode sine PWM drive, AS5600 angle/RPM feedback, an LCD/touch debug UI, and a basic safety state machine. CLSPD speed closed-loop control and PID tuning are still prototype / in-progress work. FOC is not implemented and is kept as a future research direction.

## Project Overview / 项目简介

The main STM32CubeIDE project is:

```text
examples/stm32f103c8t6_as5600_adc_i2c_compare
```

The project combines motor drive, magnetic encoder feedback, and on-board UI tooling into a compact STM32F103 experiment platform:

- STM32F103C8T6 runs the HAL-based firmware at 72 MHz.
- TIM1 CH1/CH2/CH3 generate three-phase PWM for U/V/W motor phases.
- AS5600 is sampled over I2C1 for raw angle and RPM estimation.
- ILI9341 LCD + XPT2046 touch provide local start/stop, mode, speed, duty, and fault visibility.
- OPEN mode is used for stable open-loop sine-drive validation.
- CLSPD mode is a prototype speed-feedback path that adjusts the sine phase period using AS5600 RPM.

## Current Status / 当前状态

Completed:

- STM32CubeIDE project for STM32F103C8T6.
- TIM1 three-phase PWM output and driver enable control.
- OPEN-mode sine-wave open-loop motor drive.
- AS5600 I2C raw angle acquisition.
- Angle conversion and RPM estimation from AS5600 delta.
- Direction-aligned RPM sign convention.
- Feedback snapshot diagnostics and I2C recovery after sensor power sequencing.
- ILI9341 LCD / XPT2046 touch debug UI.
- Basic motor state, stop reason, and fault state handling.

Prototype / In Progress:

- CLSPD speed closed-loop mode.
- Conservative speed PID prototype that adjusts phase advance period.
- PID debug display on the LCD UI.
- Board-side PID gain tuning and behavior validation.

Planned:

- More systematic PID tuning under different speed/load conditions.
- Improved control-loop observability.
- FOC-oriented research and possible future implementation.
- Cleaner long-term separation between generated CubeMX code and application modules.

## Features / 功能完成度表格

| Feature | Status | Notes |
| --- | --- | --- |
| STM32F103C8T6 CubeIDE project | Completed | Main project is under `examples/stm32f103c8t6_as5600_adc_i2c_compare`. |
| Three-phase PWM output | Completed | TIM1 CH1/CH2/CH3 drive U/V/W phase outputs through `motor_driver`. |
| Safe driver enable/disable | Completed | Stop path clears PWM and disables `FOCMINI_EN`. |
| OPEN sine PWM drive | Completed | Uses sine lookup table and phase accumulator. |
| AS5600 angle feedback | Completed | Reads 12-bit raw angle over I2C1. |
| RPM estimation | Completed | Computes RPM from angle delta with direction-aligned sign. |
| Feedback snapshot diagnostics | Completed | Provides coherent angle/speed/error data to UI and control code. |
| AS5600 I2C recovery | Completed | Rate-limited I2C DeInit/ReInit after repeated sensor read failures. |
| LCD/touch debug UI | Completed | Local control for start/stop, direction, mode, speed level, duty, calibration, and fault clearing. |
| LCD SPI TX DMA large fills | Completed | Large fills use DMA; small text/shape updates use blocking SPI. |
| CLSPD speed closed loop | Prototype / In Progress | First-pass speed feedback mode, still requires tuning and validation. |
| Speed PID | Prototype / In Progress | Adjusts sine phase period only; not a complete production speed controller. |
| PID parameter tuning | Prototype / In Progress | Gains are conservative and require board-side testing. |
| Current loop | Planned | Not implemented. |
| Position loop | Planned | Not implemented. |
| FOC | Planned | Not implemented. |

## Hardware / 硬件组成

- MCU: STM32F103C8T6.
- Magnetic encoder: AS5600, connected through I2C1.
- Motor driver: three-phase BLDC driver stage with U/V/W PWM inputs and enable pin.
- Display: ILI9341 240x320 SPI LCD.
- Touch: XPT2046 resistive touch controller.
- Debug output: USART2 printf-style serial logging.
- Optional comparison input: ADC1 IN0 for earlier ADC/I2C angle comparison experiments.
- Programmer/debugger: ST-Link or compatible SWD debugger.

## Pin & Peripheral Usage / 引脚与外设使用概览

| Peripheral / Signal | Pin(s) | Usage |
| --- | --- | --- |
| TIM1_CH1 | PA8 | Motor phase U PWM. |
| TIM1_CH2 | PA9 | Motor phase V PWM. |
| TIM1_CH3 | PA10 | Motor phase W PWM. |
| `FOCMINI_EN` | PB12 | Motor driver enable GPIO. |
| I2C1_SCL / I2C1_SDA | PB6 / PB7 | AS5600 magnetic encoder bus. |
| SPI1_SCK / MISO / MOSI | PA5 / PA6 / PA7 | Shared SPI bus for LCD and touch. |
| LCD CS / DC / RES / BLK | PA4 / PB10 / PB11 / PB13 | ILI9341 display control pins. |
| Touch CS / PEN | PB1 / PB0 | XPT2046 chip select and pen detect. |
| USART2_TX / USART2_RX | PA2 / PA3 | Serial debug output/input. |
| ADC1_IN0 | PA0 | Optional analog angle comparison input. |
| SWD | PA13 / PA14 | ST-Link debug and programming. |
| PC13 GPIO | PC13 | Run/status LED. |

## Software Architecture / 软件结构

```text
Core/Src/as5600.c          AS5600 register access and raw angle helpers
Core/Src/motor_feedback.c  Angle sampling, RPM estimation, feedback snapshot, I2C recovery
Core/Src/motor_driver.c    TIM1 PWM output and motor driver enable/disable wrapper
Core/Src/motor_control.c   Motor state machine, OPEN/CLSPD modes, prototype speed PID
Core/Src/motor_ui.c        LCD/touch user interface and status rendering
Core/Src/lcd_ili9341.c     ILI9341 drawing primitives and SPI/DMA fill path
Core/Src/touch_xpt2046.c   Touch sampling, calibration, and coordinate mapping
```

The firmware intentionally keeps the low-level PWM driver simple. The current control code drives a three-phase sine table and changes phase advance speed; it does not implement FOC, current sensing, current-loop control, or rotor alignment.

## Control Modes / 控制模式

### OPEN - Completed

OPEN mode uses a sine lookup table and phase accumulator to generate three-phase PWM. The speed level maps to an open-loop phase period, and the duty setting is converted to a bounded modulation amplitude. This mode is suitable for validating:

- PWM output path.
- Motor driver enable/disable behavior.
- Direction switching.
- Basic motor response at conservative speed and duty settings.

### CLSPD - Prototype / In Progress

CLSPD mode uses AS5600 RPM feedback to adjust the sine phase advance period. The current implementation is intentionally conservative:

- It reuses the same three-phase sine PWM output path as OPEN mode.
- It runs PID updates only when a new AS5600 speed sample is available.
- It clamps PID output, integrator, and phase period to reduce unsafe jumps.
- It detects feedback startup timeout and runtime feedback loss.

Important limitations:

- PID gains are not fully tuned.
- The implementation is not production-grade closed-loop speed control.
- No FOC, alignment, current loop, or position loop is implemented.

## Build & Flash / 编译与烧录说明

1. Open STM32CubeIDE.
2. Import or open the project at:

   ```text
   examples/stm32f103c8t6_as5600_adc_i2c_compare
   ```

3. Connect the STM32F103C8T6 target board and ST-Link debugger.
4. Build the project in STM32CubeIDE.
5. Flash the firmware to the target board.
6. Validate behavior on real hardware.

Compilation alone is not enough for this project. Motor behavior, sensor recovery, LCD/touch interaction, and closed-loop speed response must be verified on the actual board with the real motor driver, AS5600 sensor, power supply, and motor.

## Safety Notes / 安全说明

- Before power-up, verify the three-phase driver wiring, motor connection, power supply polarity, AS5600 supply, and common ground.
- Use a low-voltage or current-limited power supply for first bring-up.
- Start with the motor unloaded or under very light load.
- Keep initial speed level and duty low.
- Be ready to disconnect power immediately.
- If the motor vibrates abnormally, heats up, squeals, stalls, or runs away, cut power first and debug only after the system is safe.
- Treat CLSPD mode as an experimental prototype until PID tuning is validated on the target hardware.

## Roadmap / 后续计划

- Tune CLSPD speed PID gains across low, medium, and high speed levels.
- Add clearer runtime diagnostics for target RPM, actual RPM, PID output, and feedback freshness.
- Improve fault handling and recovery workflows for long-running tests.
- Evaluate current-sensing hardware requirements before any current-loop or FOC work.
- Explore FOC only after the open-loop drive, feedback path, and speed-control prototype are well understood.

## Notes

- This repository includes STM32CubeMX-generated HAL project files for reproducibility.
- Generated code and application code currently live in the same CubeIDE project tree.
- The code is intended for learning, hardware bring-up, and control experiments.
- It should not be used as-is in safety-critical or production motor-control applications.
