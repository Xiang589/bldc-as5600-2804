# Wiring and RTOS Notes

本文档记录当前 STM32F103C8T6 + AS5600 + SimpleFOC Mini / 三相驱动模块 + ILI9341 LCD + XPT2046 触摸屏的杜邦线连接关系和 FreeRTOS 任务划分。

当前硬件是多个模块通过杜邦线连接，并没有完整 PCB 原理图。因此本文件以当前固件和 CubeMX 配置为准，用来帮助复现接线、调试和后续继续开发。

> 注意：本文档描述的是当前固件期望的连接方式。不同模块的丝印、排针顺序和电源输入命名可能不同，实际接线前必须再次核对模块丝印、数据手册和万用表连通性。

## 1. Project scope

主工程路径：

```text
examples/stm32f103c8t6_as5600_adc_i2c_compare
```

当前系统状态：

- MCU：STM32F103C8T6，系统时钟 72 MHz。
- 电机驱动：TIM1 CH1 / CH2 / CH3 输出三相 PWM。
- 角度反馈：AS5600，通过 I2C1 读取 RAW_ANGLE。
- 显示与触摸：ILI9341 LCD + XPT2046 触摸，共用 SPI1。
- 操作界面：LCD / touch UI 支持启动、停止、方向、速度档、DUTY、模式和故障显示。
- RTOS：FreeRTOS / CMSIS-RTOS2。
- 当前控制：OPEN 开环正弦 PWM 已完成；CLSPD / speed PID 仍为 prototype / in-progress；FOC 尚未实现。

## 2. Firmware-derived pin map

| STM32 pin | CubeMX / firmware name | Function | External module / signal | Notes |
| --- | --- | --- | --- | --- |
| PC13 | `PC13_RUN_LED` | GPIO output | Run LED | `MonitorTask` 500 ms heartbeat |
| PA0 | `ADC1_IN0` | ADC input | Reserved / legacy analog angle input | ADC1_IN0 is configured, but current RTOS path does not use the old ADC/I2C comparison print path |
| PA2 | `USART2_TX` | UART TX | USB-UART RX / debug serial | 115200 8N1 |
| PA3 | `USART2_RX` | UART RX | USB-UART TX / debug serial | 115200 8N1 |
| PA4 | `LCD_CS` | GPIO output | ILI9341 CS | Active low |
| PA5 | `SPI1_SCK` | SPI clock | ILI9341 SCK / XPT2046 CLK | Shared SPI1 bus |
| PA6 | `SPI1_MISO` | SPI MISO | XPT2046 MISO / LCD MISO if present | Required by touch controller readback |
| PA7 | `SPI1_MOSI` | SPI MOSI | ILI9341 MOSI / XPT2046 DIN | Shared SPI1 bus |
| PA8 | `TIM1_CH1` | PWM output | Motor driver phase U control input | Firmware maps TIM1 CH1 to phase U |
| PA9 | `TIM1_CH2` | PWM output | Motor driver phase V control input | Firmware maps TIM1 CH2 to phase V |
| PA10 | `TIM1_CH3` | PWM output | Motor driver phase W control input | Firmware maps TIM1 CH3 to phase W |
| PA13 | `SWDIO` | Debug | ST-Link SWDIO | Keep for programming/debugging |
| PA14 | `SWCLK` | Debug | ST-Link SWCLK | Keep for programming/debugging |
| PB0 | `TP_PEN` | GPIO input pull-up | XPT2046 PENIRQ / T_IRQ | Active low touch press detect |
| PB1 | `TP_CS` | GPIO output | XPT2046 CS | Active low |
| PB6 | `I2C1_SCL` | I2C clock | AS5600 SCL | Current firmware config: 100 kHz I2C |
| PB7 | `I2C1_SDA` | I2C data | AS5600 SDA | Current firmware config: 100 kHz I2C |
| PB10 | `LCD_DC` | GPIO output | ILI9341 D/C | Command/data select |
| PB11 | `LCD_RES` | GPIO output | ILI9341 RESET | LCD reset |
| PB12 | `FOCMINI_EN` | GPIO output | Motor driver enable | Default low; set high only when motor driver is enabled |
| PB13 | `LCD_BLK` | GPIO output | ILI9341 backlight | Backlight enable |
| PD0 | `RCC_OSC_IN` | HSE input | External crystal / oscillator | CubeMX HSE configuration |
| PD1 | `RCC_OSC_OUT` | HSE output | External crystal / oscillator | CubeMX HSE configuration |

## 3. Dupont wiring by module

### 3.1 STM32F103C8T6 to AS5600 magnetic encoder

| STM32 side | AS5600 module side | Notes |
| --- | --- | --- |
| 3.3 V or module-compatible VCC | VCC / VDD | Use the voltage required by the AS5600 breakout module. Many modules support 3.3 V or 5 V; confirm module schematic. |
| GND | GND | Must share ground with MCU and motor driver logic ground. |
| PB6 / I2C1_SCL | SCL | Pull-ups are required on I2C. Many breakout modules already include pull-ups. |
| PB7 / I2C1_SDA | SDA | I2C1 bus. |
| Optional fixed level | DIR | DIR selects raw angle increasing direction. Tie according to desired sign convention. Current firmware also applies `MOTOR_FEEDBACK_DELTA_SIGN` so FWD reports positive RPM. |
| Not used in current firmware | OUT | Analog/PWM output is not used by the current I2C feedback path. |
| Not used in current firmware | PGO | Programming option pin; not used during normal firmware operation. |

Current firmware details:

- AS5600 7-bit I2C address is `0x36`; HAL code uses `(0x36U << 1)`.
- Current driver reads `RAW_ANGLE` through blocking HAL I2C memory reads.
- RAW angle is 12-bit, `0~4095` represents one mechanical revolution.
- `FeedbackTask` owns I2C1 / AS5600 access in the RTOS architecture.
- `MotorFeedback_Update()` currently samples angle every 20 ms and updates RPM every 200 ms.

### 3.2 STM32F103C8T6 to SimpleFOC Mini / three-phase driver module

The current firmware treats the driver module as a simple three-phase PWM driver with one enable pin.

| STM32 side | Driver module side | Notes |
| --- | --- | --- |
| PA8 / TIM1_CH1 | U / IN1 / phase-U PWM input | Firmware maps CH1 to U. Check actual module silk screen. |
| PA9 / TIM1_CH2 | V / IN2 / phase-V PWM input | Firmware maps CH2 to V. Check actual module silk screen. |
| PA10 / TIM1_CH3 | W / IN3 / phase-W PWM input | Firmware maps CH3 to W. Check actual module silk screen. |
| PB12 / `FOCMINI_EN` | EN / driver enable | Default low at startup. `MotorDriver_Enable()` sets it high. |
| GND | GND | MCU ground and driver logic ground must be common. |
| External motor supply | VM / VCC / motor power input | Do not power the motor from the MCU 3.3 V pin. Use a proper external supply. |
| Motor wires | OUT1 / OUT2 / OUT3 or U / V / W | If motor direction or phase sequence is wrong, swap phase mapping or direction in firmware after controlled testing. |

Current firmware assumptions:

- `TIM1_CH1/CH2/CH3` are U/V/W PWM outputs.
- `TIM1` period is `3599`, APB2 timer clock is 72 MHz, giving about 20 kHz PWM.
- `MotorDriver_Init()` disables the driver first, starts the three TIM1 PWM channels, then clears all PWM outputs to zero.
- `MotorDriver_Enable()` only raises `FOCMINI_EN`; it does not automatically set duty or start motion.
- `MotorDriver_Disable()` clears PWM first, then lowers `FOCMINI_EN`.
- The firmware does not currently use driver `nSLEEP`, `nRESET`, `nFAULT`, current sense, or comparator outputs. If the physical driver module exposes these pins and requires fixed levels, wire them according to the module documentation before enabling the motor driver.

### 3.3 STM32F103C8T6 to ILI9341 LCD and XPT2046 touch

LCD and touch share SPI1 but use separate chip-select pins.

| STM32 side | LCD / touch side | Notes |
| --- | --- | --- |
| PA5 / SPI1_SCK | LCD SCK, touch CLK | Shared SPI clock. |
| PA7 / SPI1_MOSI | LCD MOSI, touch DIN | Shared MOSI. |
| PA6 / SPI1_MISO | Touch DOUT / MISO | Used by XPT2046 touch reads. |
| PA4 / `LCD_CS` | LCD CS | Active low. Firmware deselects touch before selecting LCD. |
| PB10 / `LCD_DC` | LCD D/C | Command/data select. |
| PB11 / `LCD_RES` | LCD RESET | LCD reset. |
| PB13 / `LCD_BLK` | LCD BL / LED | Backlight enable. |
| PB1 / `TP_CS` | XPT2046 CS | Active low. |
| PB0 / `TP_PEN` | XPT2046 PENIRQ / T_IRQ | Input with pull-up, active low. |
| 3.3 V / module VCC | LCD / touch VCC | Check module voltage requirement. |
| GND | GND | Common ground. |

Current firmware details:

- SPI1 is configured as master, mode 0, 8-bit, MSB first, prescaler 16, about 4.5 Mbit/s.
- LCD large fills use SPI1 TX DMA through DMA1 Channel 3.
- Small LCD transfers and touch reads use blocking SPI calls.
- `UITask` owns LCD / touch operations in the RTOS architecture.
- No SPI mutex is currently required because only `UITask` accesses SPI1.

### 3.4 Debug UART and programming

| STM32 side | External side | Notes |
| --- | --- | --- |
| PA2 / USART2_TX | USB-UART RX | 115200 8N1 debug output. |
| PA3 / USART2_RX | USB-UART TX | Reserved for debug input / future command interface. |
| GND | USB-UART GND | Common ground is required. |
| PA13 / SWDIO | ST-Link SWDIO | Programming/debugging. |
| PA14 / SWCLK | ST-Link SWCLK | Programming/debugging. |
| 3.3 V / GND | ST-Link reference / GND | Follow ST-Link wiring requirements. |

## 4. FreeRTOS task layout

Current tasks are generated through CubeMX / CMSIS-RTOS2 and recorded in `.ioc`.

| Task | Priority | Stack | Period | Owns / does | Must not do |
| --- | --- | --- | --- | --- | --- |
| `ControlTask` | High | 256 words | 1 ms | Calls `MotorControl_Tick1ms()`; motor state/control scheduling | No LCD, no touch, no I2C/SPI blocking reads, no printf, no malloc/free |
| `FeedbackTask` | AboveNormal | 256 words | 20 ms | Calls `MotorFeedback_Update(HAL_GetTick())`; owns AS5600 / I2C1 sampling | No LCD, no touch, no PWM output, no printf, no malloc/free |
| `UITask` | Normal | 512 words | 10 ms | Calls `MotorUi_Update(HAL_GetTick())`; owns LCD/touch UI; reads feedback snapshots | Should not drive AS5600 sampling timing; should not run motor control algorithm |
| `MonitorTask` | Low | 128 words | 500 ms | Toggles `PC13_RUN_LED`; optional stack high-water monitoring | No long blocking I/O, no LCD rendering, no periodic printf by default |

Current timing ownership:

- HAL timebase uses TIM2.
- FreeRTOS keeps SysTick.
- `ControlTask` and `FeedbackTask` use `osDelayUntil()` style fixed-period scheduling.
- `UITask` currently uses `osDelay(10)`.
- `MonitorTask` currently uses `osDelay(500)`.

## 5. Resource ownership rules

Current design deliberately avoids mutexes by assigning each external bus to one owning task.

| Resource | Current owner | Notes |
| --- | --- | --- |
| TIM1 CH1/CH2/CH3 PWM | `ControlTask` through motor control / motor driver path | Motor driver writes CCR registers. |
| FOCMINI_EN GPIO | Motor control / motor driver path | Keep default disabled on startup. |
| I2C1 / AS5600 | `FeedbackTask` | No other task should directly call AS5600 I2C functions. |
| SPI1 / LCD / touch | `UITask` | LCD and XPT2046 share SPI1; chip-select separation is handled in drivers. |
| USART2 debug printf | Startup / limited UI-related debug only | `USE_NEWLIB_REENTRANT` is not enabled. Avoid multi-task printf. |
| PC13 LED | `MonitorTask` | Heartbeat output. |

If future code makes multiple tasks access the same I2C/SPI/UART peripheral, add a FreeRTOS mutex or a single-owner service task before merging that change.

## 6. Stack and heap policy

Current FreeRTOS heap:

```text
configTOTAL_HEAP_SIZE = 8192
```

Current task stack allocation:

```text
ControlTask   256 words
FeedbackTask  256 words
UITask        512 words
MonitorTask   128 words
```

Stack monitoring policy:

- `ENABLE_RTOS_STACK_MONITOR` is currently `0U` by default.
- If enabled, `MonitorTask` samples `uxTaskGetStackHighWaterMark()` for all four tasks.
- Do not blindly increase all stacks.
- Suggested tuning rule:
  - Increase control / feedback / monitor stack only if remaining stack is below 64 words.
  - Increase UI stack only if remaining stack is below 128 words.
  - If task creation fails, then evaluate increasing `configTOTAL_HEAP_SIZE` to 10240.

## 7. Bring-up checklist after rewiring

Before enabling motor output:

1. Check all module grounds are common.
2. Check motor supply is connected to the driver power input, not to MCU 3.3 V.
3. Confirm `FOCMINI_EN` is low before firmware enables the driver.
4. Confirm TIM1 PWM pins are connected to the intended driver inputs.
5. Confirm AS5600 magnet is detected and angle changes smoothly.
6. Confirm LCD/touch work before motor testing.
7. Start in OPEN mode at conservative duty.
8. If motor direction or RPM sign is unexpected, do not raise duty first; verify U/V/W mapping, AS5600 direction, and firmware direction sign.

## 8. Known limitations and future notes

- This document is a wiring note for a Dupont-wire modular setup, not a formal product schematic.
- FOC is not implemented yet.
- Current AS5600 path is I2C-based and runs in `FeedbackTask`; it is suitable for angle/RPM feedback and early FOC research, but a future high-frequency current-loop FOC path may need faster angle/current feedback and ADC/PWM synchronized sampling.
- Current hardware documentation does not yet record driver current-sense wiring; current-loop FOC cannot be claimed until current sensing is defined, implemented, and tested.
- If future code adds `CommTask`, `LogTask`, current sensing, or FOC fast loop, update this document together with the code.
