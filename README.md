# STM32 AS5600 无刷电机控制实验平台

基于 STM32F103C8T6、AS5600 磁编码器与三相无刷电机驱动的电机控制实验平台。

本仓库是一个可上板验证的嵌入式电机控制实验项目，不是量产级 FOC 控制器。当前固件已完成 OPEN 开环正弦 PWM 驱动、AS5600 角度/RPM 反馈、LCD 触摸调试界面和基础安全状态机。CLSPD 速度闭环与 PID 调参仍处于原型和开发中阶段。FOC 尚未实现，只作为后续研究方向。

## 项目简介

主 STM32CubeIDE 工程位于：

```text
examples/stm32f103c8t6_as5600_adc_i2c_compare
```

项目把电机驱动、磁编码器反馈和板载调试界面组合成一个紧凑的 STM32F103 实验平台：

- STM32F103C8T6 运行基于 HAL 的固件，系统时钟 72 MHz。
- TIM1 CH1、CH2、CH3 输出三相 U/V/W PWM。
- AS5600 通过 I2C1 采集原始角度，并用于 RPM 估算。
- ILI9341 LCD 与 XPT2046 触摸用于本地启动、停止、方向、模式、速度、DUTY 和故障状态显示。
- OPEN 模式用于开环正弦驱动验证。
- CLSPD 模式是速度反馈原型路径，通过 AS5600 RPM 调整正弦相位推进周期。

## 当前状态

已完成：

- STM32F103C8T6 的 STM32CubeIDE 工程。
- TIM1 三相 PWM 输出与驱动使能控制。
- OPEN 开环正弦波电机驱动。
- AS5600 I2C 原始角度采集。
- 角度换算与基于角度差分的 RPM 估算。
- 与电机方向一致的 RPM 正负号语义。
- 反馈快照诊断与传感器后上电时的 I2C 恢复机制。
- ILI9341 LCD 与 XPT2046 触摸调试界面。
- 电机运行状态、停机原因和故障状态管理。

原型 / 开发中：

- CLSPD 速度闭环模式。
- 通过调节相位推进周期实现的保守速度 PID 原型。
- LCD 上的 PID 调试信息显示。
- 基于真实硬件的 PID 参数整定与行为验证。

计划中：

- 在不同速度和负载条件下进行系统化 PID 调参。
- 增强控制环路可观测性。
- 研究 FOC，并在条件成熟后评估是否实现。
- 长期整理 CubeMX 生成代码与应用代码的边界。

## 功能完成度

| 功能 | 状态 | 说明 |
| --- | --- | --- |
| STM32F103C8T6 CubeIDE 工程 | 已完成 | 主工程位于 `examples/stm32f103c8t6_as5600_adc_i2c_compare`。 |
| 三相 PWM 输出 | 已完成 | TIM1 CH1/CH2/CH3 通过 `motor_driver` 输出 U/V/W 三相 PWM。 |
| 驱动安全使能与关闭 | 已完成 | 停机路径会先清 PWM，再关闭 `FOCMINI_EN`。 |
| OPEN 开环正弦驱动 | 已完成 | 使用正弦查表与相位累加器生成三相 PWM。 |
| AS5600 角度反馈 | 已完成 | 通过 I2C1 读取 12 位原始角度。 |
| RPM 估算 | 已完成 | 基于 AS5600 角度差分计算 RPM，并对齐电机方向语义。 |
| 反馈快照诊断 | 已完成 | 为 UI 和控制逻辑提供同一更新周期的角度、速度和错误信息。 |
| AS5600 I2C 恢复 | 已完成 | 连续读取失败后限流执行 I2C DeInit/ReInit。 |
| LCD 触摸调试界面 | 已完成 | 支持启停、方向、模式、速度档、DUTY、校准和故障清除。 |
| LCD SPI TX DMA 大块填充 | 已完成 | 大面积填充走 DMA，小文本和小图形走阻塞 SPI。 |
| CLSPD 速度闭环 | 原型 / 开发中 | 第一版速度反馈路径仍需要调参与验证。 |
| 速度 PID | 原型 / 开发中 | 目前只调节正弦相位推进周期，不是完整生产级速度控制器。 |
| PID 参数整定 | 原型 / 开发中 | 当前参数保守，需要上板测试。 |
| 电流环 | 计划中 | 尚未实现。 |
| 位置环 | 计划中 | 尚未实现。 |
| FOC | 计划中 | 尚未实现。 |

## 硬件组成

- 主控：STM32F103C8T6。
- 磁编码器：AS5600，通过 I2C1 连接。
- 电机驱动：三相无刷电机驱动级，包含 U/V/W PWM 输入和使能引脚。
- 显示：ILI9341 240x320 SPI LCD。
- 触摸：XPT2046 电阻触摸控制器。
- 调试输出：USART2 串口打印。
- 可选比较输入：ADC1 IN0，用于早期 ADC/I2C 角度对比实验。
- 烧录调试：ST-Link 或兼容 SWD 调试器。

## 引脚与外设使用概览

| 外设 / 信号 | 引脚 | 用途 |
| --- | --- | --- |
| TIM1_CH1 | PA8 | 电机 U 相 PWM。 |
| TIM1_CH2 | PA9 | 电机 V 相 PWM。 |
| TIM1_CH3 | PA10 | 电机 W 相 PWM。 |
| `FOCMINI_EN` | PB12 | 电机驱动使能。 |
| I2C1_SCL / I2C1_SDA | PB6 / PB7 | AS5600 磁编码器总线。 |
| SPI1_SCK / MISO / MOSI | PA5 / PA6 / PA7 | LCD 与触摸共用 SPI 总线。 |
| LCD CS / DC / RES / BLK | PA4 / PB10 / PB11 / PB13 | ILI9341 控制引脚。 |
| Touch CS / PEN | PB1 / PB0 | XPT2046 片选与触摸检测。 |
| USART2_TX / USART2_RX | PA2 / PA3 | 串口调试输入输出。 |
| ADC1_IN0 | PA0 | 可选模拟角度对比输入。 |
| SWD | PA13 / PA14 | ST-Link 调试与烧录。 |
| PC13 GPIO | PC13 | 运行状态指示灯。 |

## 软件结构

```text
Core/Src/as5600.c          AS5600 寄存器访问与原始角度读取
Core/Src/motor_feedback.c  角度采样、RPM 估算、反馈快照、I2C 恢复
Core/Src/motor_driver.c    TIM1 PWM 输出与电机驱动使能封装
Core/Src/motor_control.c   电机状态机、OPEN/CLSPD 模式、速度 PID 原型
Core/Src/motor_ui.c        LCD 触摸界面与状态显示
Core/Src/lcd_ili9341.c     ILI9341 绘图接口与 SPI/DMA 填充路径
Core/Src/touch_xpt2046.c   触摸采样、校准与坐标映射
```

底层 PWM 驱动刻意保持简单。当前控制逻辑通过三相正弦表和相位推进速度驱动电机；未实现 FOC、电流采样、电流环控制或转子对齐流程。

## 控制模式

### OPEN：已完成

OPEN 模式使用正弦查表和相位累加器生成三相 PWM。速度档位映射到开环相位周期，DUTY 输入会转换为受限的调制幅度。该模式适合验证：

- PWM 输出路径。
- 电机驱动使能和关闭行为。
- 方向切换。
- 低速、低 DUTY 下的基础电机响应。

### CLSPD：原型 / 开发中

CLSPD 模式使用 AS5600 RPM 反馈调整正弦相位推进周期。当前实现保持保守：

- 复用 OPEN 模式相同的三相正弦 PWM 输出路径。
- 只有收到新的 AS5600 速度样本时才更新 PID。
- 对 PID 输出、积分项和相位周期进行限幅，减少不安全跳变。
- 能识别启动阶段反馈超时和运行中反馈丢失。

重要限制：

- PID 参数尚未完整整定。
- 当前实现不是生产级速度闭环控制。
- 未实现 FOC、对齐流程、电流环或位置环。

## 编译与烧录

1. 打开 STM32CubeIDE。
2. 导入或打开以下工程：

   ```text
   examples/stm32f103c8t6_as5600_adc_i2c_compare
   ```

3. 连接 STM32F103C8T6 目标板和 ST-Link 调试器。
4. 在 STM32CubeIDE 中编译工程。
5. 将固件下载到目标板。
6. 在真实硬件上验证功能。

本项目不能只依赖编译结果判断是否可用。电机行为、传感器恢复、LCD 触摸交互和速度闭环响应都需要在真实电机驱动板、AS5600、供电系统和电机上验证。

## 安全说明

- 上电前确认三相驱动、电机连接、电源极性、AS5600 供电和共地连接正确。
- 初次上电使用低电压或限流电源。
- 初次启动建议电机空载或低负载。
- 初始速度档和 DUTY 保持较低。
- 测试时应随时准备断电。
- 如果出现异常震动、发热、啸叫、堵转或失控，先立即断电，再排查原因。
- 在 PID 调参充分验证前，CLSPD 模式应视为实验原型。

## 后续计划

- 在低、中、高不同速度档下整定 CLSPD 速度 PID。
- 增加更清晰的运行诊断信息，例如目标 RPM、实际 RPM、PID 输出和反馈新鲜度。
- 改进长时间测试中的故障处理和恢复流程。
- 在进行电流环或 FOC 前，评估电流采样硬件需求。
- 在开环驱动、反馈路径和速度闭环原型充分稳定后，再继续研究 FOC。

## 说明

- 本仓库包含 STM32CubeMX 生成的 HAL 工程文件，便于复现实验环境。
- 当前 CubeMX 生成代码和应用代码仍位于同一个 CubeIDE 工程树下。
- 本项目用于学习、硬件 bring-up 和控制实验。
- 不应直接用于安全关键或量产电机控制场景。

---

# STM32 AS5600 BLDC Motor Control Platform

An experimental motor-control platform based on STM32F103C8T6, the AS5600 magnetic encoder, and a three-phase BLDC driver stage.

This repository is a board-validated embedded motor-control experiment, not a production-ready FOC controller. The current firmware has completed OPEN-mode sine PWM drive, AS5600 angle/RPM feedback, an LCD touch debug UI, and a basic safety state machine. CLSPD speed closed-loop control and PID tuning are still prototype / in-progress work. FOC is not implemented and is kept as a future research direction.

## Project Overview

The main STM32CubeIDE project is:

```text
examples/stm32f103c8t6_as5600_adc_i2c_compare
```

The project combines motor drive, magnetic encoder feedback, and on-board UI tooling into a compact STM32F103 experiment platform:

- STM32F103C8T6 runs the HAL-based firmware at 72 MHz.
- TIM1 CH1, CH2, and CH3 generate three-phase PWM for U/V/W motor phases.
- AS5600 is sampled over I2C1 for raw angle and RPM estimation.
- ILI9341 LCD and XPT2046 touch provide local start/stop, direction, mode, speed, duty, and fault visibility.
- OPEN mode is used for stable open-loop sine-drive validation.
- CLSPD mode is a prototype speed-feedback path that adjusts the sine phase period using AS5600 RPM.

## Current Status

Completed:

- STM32CubeIDE project for STM32F103C8T6.
- TIM1 three-phase PWM output and driver enable control.
- OPEN-mode sine-wave open-loop motor drive.
- AS5600 I2C raw angle acquisition.
- Angle conversion and RPM estimation from AS5600 delta.
- Direction-aligned RPM sign convention.
- Feedback snapshot diagnostics and I2C recovery after sensor power sequencing.
- ILI9341 LCD and XPT2046 touch debug UI.
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

## Features

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
| LCD touch debug UI | Completed | Local control for start/stop, direction, mode, speed level, duty, calibration, and fault clearing. |
| LCD SPI TX DMA large fills | Completed | Large fills use DMA; small text/shape updates use blocking SPI. |
| CLSPD speed closed loop | Prototype / In Progress | First-pass speed feedback mode, still requires tuning and validation. |
| Speed PID | Prototype / In Progress | Adjusts sine phase period only; not a complete production speed controller. |
| PID parameter tuning | Prototype / In Progress | Gains are conservative and require board-side testing. |
| Current loop | Planned | Not implemented. |
| Position loop | Planned | Not implemented. |
| FOC | Planned | Not implemented. |

## Hardware

- MCU: STM32F103C8T6.
- Magnetic encoder: AS5600, connected through I2C1.
- Motor driver: three-phase BLDC driver stage with U/V/W PWM inputs and enable pin.
- Display: ILI9341 240x320 SPI LCD.
- Touch: XPT2046 resistive touch controller.
- Debug output: USART2 printf-style serial logging.
- Optional comparison input: ADC1 IN0 for earlier ADC/I2C angle comparison experiments.
- Programmer/debugger: ST-Link or compatible SWD debugger.

## Pin and Peripheral Usage

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
| USART2_TX / USART2_RX | PA2 / PA3 | Serial debug input/output. |
| ADC1_IN0 | PA0 | Optional analog angle comparison input. |
| SWD | PA13 / PA14 | ST-Link debug and programming. |
| PC13 GPIO | PC13 | Run/status LED. |

## Software Architecture

```text
Core/Src/as5600.c          AS5600 register access and raw angle helpers
Core/Src/motor_feedback.c  Angle sampling, RPM estimation, feedback snapshot, I2C recovery
Core/Src/motor_driver.c    TIM1 PWM output and motor driver enable/disable wrapper
Core/Src/motor_control.c   Motor state machine, OPEN/CLSPD modes, prototype speed PID
Core/Src/motor_ui.c        LCD touch user interface and status rendering
Core/Src/lcd_ili9341.c     ILI9341 drawing primitives and SPI/DMA fill path
Core/Src/touch_xpt2046.c   Touch sampling, calibration, and coordinate mapping
```

The firmware intentionally keeps the low-level PWM driver simple. The current control code drives a three-phase sine table and changes phase advance speed; it does not implement FOC, current sensing, current-loop control, or rotor alignment.

## Control Modes

### OPEN: Completed

OPEN mode uses a sine lookup table and phase accumulator to generate three-phase PWM. The speed level maps to an open-loop phase period, and the duty setting is converted to a bounded modulation amplitude. This mode is suitable for validating:

- PWM output path.
- Motor driver enable/disable behavior.
- Direction switching.
- Basic motor response at conservative speed and duty settings.

### CLSPD: Prototype / In Progress

CLSPD mode uses AS5600 RPM feedback to adjust the sine phase advance period. The current implementation is intentionally conservative:

- It reuses the same three-phase sine PWM output path as OPEN mode.
- It runs PID updates only when a new AS5600 speed sample is available.
- It clamps PID output, integrator, and phase period to reduce unsafe jumps.
- It detects feedback startup timeout and runtime feedback loss.

Important limitations:

- PID gains are not fully tuned.
- The implementation is not production-grade closed-loop speed control.
- No FOC, alignment, current loop, or position loop is implemented.

## Build and Flash

1. Open STM32CubeIDE.
2. Import or open the project at:

   ```text
   examples/stm32f103c8t6_as5600_adc_i2c_compare
   ```

3. Connect the STM32F103C8T6 target board and ST-Link debugger.
4. Build the project in STM32CubeIDE.
5. Flash the firmware to the target board.
6. Validate behavior on real hardware.

Compilation alone is not enough for this project. Motor behavior, sensor recovery, LCD touch interaction, and closed-loop speed response must be verified on the actual board with the real motor driver, AS5600 sensor, power supply, and motor.

## Safety Notes

- Before power-up, verify the three-phase driver wiring, motor connection, power supply polarity, AS5600 supply, and common ground.
- Use a low-voltage or current-limited power supply for first bring-up.
- Start with the motor unloaded or under very light load.
- Keep initial speed level and duty low.
- Be ready to disconnect power immediately.
- If the motor vibrates abnormally, heats up, squeals, stalls, or runs away, cut power first and debug only after the system is safe.
- Treat CLSPD mode as an experimental prototype until PID tuning is validated on the target hardware.

## Roadmap

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
