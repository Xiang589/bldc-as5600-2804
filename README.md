# STM32 AS5600 BLDC Motor Control Platform

基于 STM32F103 与 AS5600 磁编码器的无刷电机控制实验平台

Recommended repository name: `stm32-as5600-bldc-control`

## 项目简介

This repository contains an STM32CubeIDE-based embedded motor-control experiment built around an STM32F103C8T6 MCU, an AS5600 magnetic angle sensor, and a three-phase BLDC driver stage. The project focuses on practical firmware structure for sensor feedback, safe PWM output, LCD/touch interaction, and the early stages of closed-loop speed-control development.

本项目定位为一个可上板验证的无刷电机控制实验平台，而不是完整量产级 FOC 控制器。当前已完成开环三相正弦 PWM 驱动、AS5600 角度采集、RPM 估算、LCD/touch 调试界面和基础安全状态机；CLSPD 速度闭环与 PID 参数整定仍处于 prototype / in-progress 阶段。

## Project Overview

The main STM32CubeIDE project is located at:

```text
examples/stm32f103c8t6_as5600_adc_i2c_compare
```

The firmware is organized as a small embedded control stack:

- AS5600 sensor acquisition over I2C.
- Motor feedback snapshot and RPM estimation.
- Three-phase TIM1 PWM output with safe enable/disable control.
- Open-loop sine-commutation motor drive.
- Prototype closed-loop speed mode using AS5600 RPM feedback.
- ILI9341 LCD and XPT2046 touch UI for board-side tuning and status display.

## Features

| Feature | Status | Notes |
| --- | --- | --- |
| Three-phase PWM output on TIM1 | Completed | U/V/W PWM outputs are driven through `motor_driver`. |
| Safe motor enable/disable sequence | Completed | PWM is cleared before disabling the driver enable pin. |
| OPEN mode sine-wave open-loop drive | Completed | Uses a sine lookup table and phase accumulator. |
| AS5600 raw angle acquisition | Completed | I2C-based angle readout with 12-bit raw angle conversion. |
| RPM estimation from AS5600 delta | Completed | Includes direction-aligned RPM sign handling. |
| Feedback snapshot diagnostics | Completed | Provides consistent angle/speed/error snapshot data to UI and control logic. |
| I2C recovery after sensor power sequencing | Completed | Performs rate-limited I2C DeInit/ReInit after repeated AS5600 read failures. |
| ILI9341 LCD + XPT2046 touch UI | Completed | Used for mode switching, speed/duty tuning, status, and calibration. |
| LCD SPI TX DMA fill optimization | Completed | Uses DMA for large fills and blocking SPI for small text/shape updates. |
| CLSPD closed-loop speed mode | In Progress / Prototype | Runs a conservative first-pass speed PID that adjusts phase period. |
| PID parameter tuning | In Progress | Requires board-side tuning and validation across speed/load conditions. |
| FOC algorithm | Planned | Not implemented in the current firmware. |

## Hardware

- MCU: STM32F103C8T6, 72 MHz, STM32Cube HAL.
- Magnetic encoder: AS5600 over I2C1.
- Motor output: three-phase BLDC PWM through TIM1 CH1/CH2/CH3.
- Driver enable: `FOCMINI_EN` GPIO output.
- Display: ILI9341 240x320 SPI LCD.
- Touch: XPT2046 resistive touch controller sharing SPI.
- Debug: USART2 printf-style serial output.
- Optional analog comparison path: ADC1 input for earlier ADC/I2C angle comparison experiments.

## Software Architecture

- `Core/Src/as5600.c`, `Core/Inc/as5600.h`
  Low-level AS5600 register access and raw angle conversion helpers.

- `Core/Src/motor_feedback.c`, `Core/Inc/motor_feedback.h`
  Periodic AS5600 sampling, angle validity, RPM estimation, direction-aligned delta/RPM semantics, snapshot diagnostics, error counters, and I2C recovery.

- `Core/Src/motor_driver.c`, `Core/Inc/motor_driver.h`
  Hardware PWM output layer for TIM1 CH1/CH2/CH3 and the motor driver enable pin. This layer intentionally stays simple and does not implement FOC.

- `Core/Src/motor_control.c`, `Core/Inc/motor_control.h`
  Motor state machine, OPEN/CLSPD mode handling, stop/fault reasons, safe shutdown sequencing, sine-commutation phase update, and prototype speed PID period adjustment.

- `Core/Src/motor_ui.c`, `Core/Inc/motor_ui.h`
  Board-side LCD/touch UI for start/stop, direction, mode, speed level, duty/amplitude, feedback display, and fault clearing.

- `Core/Src/lcd_ili9341.c`, `Core/Src/touch_xpt2046.c`
  Lightweight display and touch drivers used by the tuning UI.

## Control Modes

### OPEN - Completed

OPEN mode drives the motor using a three-phase sine lookup table and a phase accumulator. The speed level maps to an open-loop phase period, while the duty input is converted into a bounded modulation amplitude. This mode is suitable for validating PWM output, driver enable behavior, motor direction, and basic UI tuning.

### CLSPD - In Progress / Prototype

CLSPD mode uses AS5600 RPM feedback to adjust the phase advance period. The current implementation is a conservative first-pass speed PID prototype:

- The motor still uses the same three-phase sine PWM output path as OPEN mode.
- The PID adjusts phase period only; it does not perform FOC, alignment, current control, or position control.
- Feedback loss and startup feedback timeout are represented in the motor state/fault logic.
- PID gains and behavior still need board-side tuning before the mode can be considered complete.

## Key Implementation Details

- Consistent feedback snapshot: `MotorFeedback_GetSnapshot()` copies all feedback fields in a short critical section so UI/control code reads a coherent update cycle.
- Direction-aligned RPM: AS5600 raw delta is converted so FWD reports positive RPM and REV reports negative RPM.
- Feedback recovery: repeated AS5600 read failures trigger rate-limited I2C peripheral recovery, helping the system recover when the MCU/LCD powers up before the sensor supply.
- Safety-first stop path: stop/fault handling clears PWM first, disables the driver enable pin, then updates state/reason/fault.
- State and fault visibility: the UI shows STOP/RUN/STARTUP/FAULT states and fault reasons such as feedback lost or startup feedback timeout.
- LCD update efficiency: small text/shape draws use blocking SPI, while large LCD fills use SPI TX DMA to avoid unnecessary 1024-byte buffer fills for small UI updates.
- Prototype PID loop: the first-pass speed PID runs only on new AS5600 speed samples and clamps output, integrator, and phase period to reduce unsafe jumps.

## Current Status

Completed:

- STM32CubeIDE project for STM32F103C8T6.
- AS5600 I2C angle readout.
- RPM estimation from AS5600 angle delta.
- Direction-correct RPM sign convention.
- TIM1 three-phase PWM output.
- Safe driver enable/disable wrapper.
- OPEN mode sine-wave motor drive.
- LCD/touch UI for local testing and tuning.
- Feedback diagnostics and I2C recovery.

In Progress / Prototype:

- CLSPD speed closed-loop mode.
- Conservative PID period adjustment.
- PID debug information on the LCD UI.
- Board-side PID gain tuning and behavior validation.

Planned:

- More systematic PID tuning and load testing.
- Better control-loop observability.
- FOC-oriented control research and possible future implementation.
- Cleaner separation between generated CubeMX code and application modules.

## Resume Description

Built a STM32F103-based BLDC motor control experimental platform using an AS5600 magnetic encoder, TIM1 three-phase PWM, and an ILI9341 touch display. Implemented open-loop sine commutation, angle/RPM feedback acquisition, safe motor state/fault handling, LCD-based tuning UI, I2C recovery for sensor power sequencing, and a prototype speed PID loop for closed-loop speed-control experiments.

简历描述示例：

> 基于 STM32F103C8T6、AS5600 磁编码器和三相 BLDC 驱动板搭建电机控制实验平台；完成开环正弦 PWM 驱动、角度/RPM 反馈采集、安全停机/故障状态机、LCD 触摸调试界面与 I2C 传感器恢复机制，并实现第一版速度闭环 PID 原型用于后续参数整定与 FOC 研究。

## Notes

- This project is an educational and experimental embedded-control platform.
- The current firmware is not a production-ready motor controller.
- FOC, current-loop control, and position-loop control are not implemented.
- CubeMX-generated files are kept in the STM32CubeIDE project for reproducibility.
- Build and board validation are expected to be performed with STM32CubeIDE and the target hardware.
