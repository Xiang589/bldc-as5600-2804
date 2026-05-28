# STM32 AS5600 BLDC Motor Control Platform

基于 STM32F103C8T6、AS5600 磁编码器与三相 BLDC 驱动的电机控制实验平台。

本仓库是一个可上板验证的嵌入式电机控制实验项目，不是量产级 FOC 控制器。当前代码已经具备 OPEN 开环正弦 PWM 驱动、AS5600 角度/RPM 反馈、LCD/touch 调试界面、FreeRTOS 任务拆分、基础安全状态机，以及第一版 voltage-mode FOC / FOC speed loop / FOC position loop 代码框架。FOC 相关功能仍需要真实硬件校准、方向确认和 PID 参数整定；电流环、力矩闭环和生产级保护未实现。

## 中文说明

### Project Overview / 项目简介

主 STM32CubeIDE 工程位于：

```text
examples/stm32f103c8t6_as5600_adc_i2c_compare
```

项目使用 TIM1 CH1/CH2/CH3 生成 U/V/W 三相 PWM，通过 `FOCMINI_EN` 控制三相驱动板使能；AS5600 通过 I2C1 提供 12-bit raw angle，并由 `FeedbackTask` 周期采样形成一致的反馈快照；ILI9341 LCD 与 XPT2046 touch 提供本地调试界面；FreeRTOS 中的 `ControlTask`、`FeedbackTask`、`UITask`、`MonitorTask` 分别承担控制、传感器采样、UI、心跳/监控职责。

### Current Status / 当前状态

| 功能 | 状态 | 说明 |
| --- | --- | --- |
| STM32CubeIDE 工程 | Completed | 主工程位于 `examples/stm32f103c8t6_as5600_adc_i2c_compare`。 |
| TIM1 三相 PWM 输出 | Completed | `motor_driver` 封装 U/V/W PWM 和 EN 使能。 |
| OPEN 开环正弦驱动 | Completed | 使用正弦查表和相位累加器驱动电机。 |
| AS5600 angle/RPM feedback | Completed | I2C1 读取 raw angle，计算角度、RPM、speed sample seq。 |
| AS5600 I2C recovery | Completed | 传感器后上电或 I2C 异常后限流执行 DeInit/ReInit。 |
| AS5600 磁铁诊断快照 | Implemented / needs hardware validation | 读取 STATUS、AGC、MAGNITUDE，并暴露 magnet detected / weak / strong 状态。 |
| FreeRTOS task split | Completed | Control / Feedback / UI / Monitor task 已拆分。 |
| LCD/touch 调试界面 | Completed | 支持启停、方向、模式、速度/DUTY、状态和 fault 显示。 |
| CLSPD speed PID | Prototype / In Progress | 调节开环相位推进 period，已上板标定过一版，但仍不是完整生产级速度环。 |
| Voltage-mode FOC | Implemented / needs hardware tuning | `Uq` 电压命令锁定到 AS5600 电角度，`Ud=0`，复用三相 PWM 输出。 |
| FOC speed loop | Implemented / needs hardware tuning | target RPM -> speed PID -> `Uq` -> voltage FOC。参数保守，未完成实机整定。 |
| FOC position loop | Implemented / needs hardware tuning | target position -> position PID/P -> target velocity -> speed PID -> `Uq` -> voltage FOC。 |
| FOC zero alignment | Implemented / needs hardware validation | 提供低电压固定矢量校准入口，并保留启动时的软件零点捕获兜底。 |
| Current loop | Planned / Not Implemented | 当前工程没有已验证的相电流采样链路。 |
| Torque closed loop | Not Implemented | 当前 `Uq` 只是电压命令，不代表实际电流或真实力矩闭环。 |
| Production-grade PID tuning | Not Implemented | 速度环、位置环仍需上板调参。 |

### Hardware / 硬件组成

- MCU: STM32F103C8T6
- Magnetic encoder: AS5600, I2C1
- Motor: 2804 BLDC, 12N14P, pole pairs = 7
- Driver: SimpleFOC Mini / DRV8313-style 3PWM driver board or compatible three-phase driver
- Display: ILI9341 240x320 SPI LCD
- Touch: XPT2046 resistive touch controller
- Debug: USART2, ST-Link / SWD

### Pin & Peripheral Usage / 引脚与外设使用概览

| Peripheral / Signal | Pin(s) | Usage |
| --- | --- | --- |
| TIM1_CH1 | PA8 | Motor phase U PWM |
| TIM1_CH2 | PA9 | Motor phase V PWM |
| TIM1_CH3 | PA10 | Motor phase W PWM |
| `FOCMINI_EN` | PB12 | Motor driver enable |
| I2C1_SCL / I2C1_SDA | PB6 / PB7 | AS5600 |
| SPI1_SCK / MISO / MOSI | PA5 / PA6 / PA7 | LCD and touch SPI bus |
| LCD CS / DC / RES / BLK | PA4 / PB10 / PB11 / PB13 | ILI9341 control pins |
| Touch CS / PEN | PB1 / PB0 | XPT2046 chip select and pen detect |
| USART2_TX / RX | PA2 / PA3 | Debug serial |
| PC13 | PC13 | MonitorTask heartbeat LED |

### Software Architecture / 软件结构

```text
Core/Src/as5600.c          AS5600 register access: RAW_ANGLE, STATUS, AGC, MAGNITUDE
Core/Src/motor_feedback.c  FeedbackTask-side angle/RPM sampling, diagnostics, snapshot, I2C recovery
Core/Src/motor_driver.c    TIM1 PWM output and FOCMINI_EN enable/disable wrapper
Core/Src/motor_control.c   State machine, OPEN/CLSPD, voltage FOC, speed loop, position loop
Core/Src/motor_ui.c        LCD/touch UI, mode switching, status/debug display
Core/Src/freertos.c        ControlTask, FeedbackTask, UITask, MonitorTask
```

### Control Modes / 控制模式

- `OPEN`: 传统开环正弦 PWM，用 speed level 控制相位推进周期，用 DUTY 控制调制幅度。
- `CLSPD`: 原型速度闭环，AS5600 RPM -> PID -> open-loop phase period，保留用于对照测试。
- `FOCV`: voltage-mode FOC，AS5600 raw angle -> mechanical angle -> electrical angle，输出 `Ud=0`、`Uq=target_voltage` 的三相正弦电压。
- `FOCSPD`: FOC 速度环，target RPM -> speed PID -> `Uq` -> voltage FOC。
- `FOCPOS`: FOC 位置环，target position -> position controller -> target velocity -> speed PID -> `Uq` -> voltage FOC。

### Key Implementation Details / 关键实现要点

- 2804 电机按资料使用 `pole pairs = 7`。
- AS5600 `raw_angle` / `angle_x100` 保持传感器原始显示语义；`delta_raw`、`total_raw_turns`、`rpm_x10` 是 motor-direction aligned 语义。
- `MotorFeedback_GetSnapshot()` 使用短临界区一次性复制完整快照，control/UI 不直接在任务中访问 I2C。
- FOC 电角度计算：

  ```text
  electrical_angle = normalize(mechanical_angle * 7 + zero_electric_offset)
  ```

- voltage-mode FOC 使用 `Ud=0`、`Uq` 电压命令，不实现电流环。
- speed loop 只在新的 AS5600 speed sample 到来时更新，避免重复积分旧 RPM。
- position loop 级联到 speed loop，不直接把位置误差写入 PWM。
- STOP/FAULT 路径保持：先清 PWM，再关闭 EN，再更新 state/reason/fault。

### Build & Flash / 编译与烧录

1. 使用 STM32CubeIDE 打开主工程：

   ```text
   examples/stm32f103c8t6_as5600_adc_i2c_compare
   ```

2. 连接目标板和 ST-Link。
3. 编译并下载到 STM32F103C8T6。
4. 在真实硬件上验证 LCD/touch、OPEN、AS5600 angle/RPM、CLSPD、FOC voltage、FOC speed、FOC position。

仅编译通过不能证明电机控制可用。FOC 零点、方向、PID 参数、温升、电源限流和异常行为必须上板验证。

### Safety Notes / 安全说明

- 上电前确认三相驱动、电机、电源、AS5600 供电和共地连接正确。
- 初次 FOC 测试必须使用低电压/限流电源。
- 电机空载或低负载启动，远离转子。
- 从低 DUTY、低 voltage limit、低目标 RPM 开始。
- 出现异常震动、啸叫、发热、堵转或失控时立即断电。
- 当前没有电流环、过流保护、温度保护或堵转保护，不能用于安全关键或量产场景。

### Roadmap / 后续计划

- 上板确认 AS5600 磁铁诊断阈值、FOC 角度方向和零电角度校准流程。
- 逐步整定 FOC speed PID 和 position loop 参数。
- 评估是否需要把 AS5600 采样周期从 20ms 降低到更适合 FOC 的周期。
- 若硬件增加并验证相电流采样，再规划 current loop / torque loop。
- 后续可评估 SVPWM、速度滤波和更完整的故障保护。

## English Description

### Project Overview

This repository contains an STM32F103C8T6 + AS5600 + three-phase BLDC motor-control experiment. It is intended for learning, hardware bring-up, and control-loop prototyping, not as a production-ready FOC controller.

The main STM32CubeIDE project is:

```text
examples/stm32f103c8t6_as5600_adc_i2c_compare
```

### Current Status

Completed work includes the STM32CubeIDE project, TIM1 three-phase PWM output, OPEN-mode sine drive, AS5600 raw angle/RPM feedback, I2C recovery, FreeRTOS task split, LCD/touch UI, and the basic safety state machine.

Implemented but still requiring hardware tuning:

- AS5600 magnetic diagnostics through STATUS / AGC / MAGNITUDE.
- Voltage-mode FOC using AS5600 electrical angle and bounded `Uq` voltage command.
- FOC speed loop: target RPM -> speed PID -> `Uq` -> voltage FOC.
- FOC position loop: target position -> target velocity -> speed PID -> `Uq` -> voltage FOC.
- Low-voltage FOC zero alignment entry.

Still planned / not implemented:

- Current loop.
- Torque closed loop.
- Production-grade speed/position PID tuning.
- Over-current, temperature, and stall protection.

### Features

| Feature | Status | Notes |
| --- | --- | --- |
| OPEN sine PWM drive | Completed | Uses sine LUT and phase accumulator. |
| AS5600 angle/RPM feedback | Completed | Includes direction-aligned RPM semantics. |
| RTOS task split | Completed | Control, feedback, UI, and monitor tasks. |
| CLSPD speed PID | Prototype / In Progress | Adjusts open-loop phase period, kept as a comparison path. |
| AS5600 magnetic diagnostics | Implemented / needs hardware validation | STATUS, AGC, MAGNITUDE are sampled into the snapshot. |
| Voltage-mode FOC | Implemented / needs hardware tuning | `Ud=0`, bounded `Uq`, rotor electrical-angle locked sine output. |
| FOC speed loop | Implemented / needs hardware tuning | Conservative PID outputs `Uq`. |
| FOC position loop | Implemented / needs hardware tuning | Cascaded position -> velocity -> voltage FOC path. |
| Current loop | Planned / Not Implemented | No verified phase-current sampling path yet. |
| Torque loop | Not Implemented | `Uq` is voltage command only. |

### Control Modes

- `OPEN`: open-loop sine PWM.
- `CLSPD`: prototype AS5600 RPM loop that adjusts phase period.
- `FOCV`: voltage-mode FOC.
- `FOCSPD`: FOC speed loop.
- `FOCPOS`: cascaded FOC position loop.

### Build and Flash

Open the STM32CubeIDE project under `examples/stm32f103c8t6_as5600_adc_i2c_compare`, connect the target board and ST-Link, build, flash, and validate on real hardware. Motor behavior, FOC angle direction, zero alignment, speed-loop tuning, and position-loop tuning cannot be verified by compilation alone.

### Notes

This project intentionally does not claim completed current control, torque control, production-grade PID tuning, or complete safety protection. The FOC implementation is a buildable experimental framework that must still be tuned and validated on the target motor, driver, AS5600 sensor, power supply, and mechanical setup.
