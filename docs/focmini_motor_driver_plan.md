# FOCMini 电机驱动开发计划（安全骨架版）

## 1. FOCMini 3PWM 接线说明

当前项目采用 3PWM 输入模式，三相 PWM 输入如下：

- TIM1_CH1 (PA8) -> FOCMini IN1
- TIM1_CH2 (PA9) -> FOCMini IN2
- TIM1_CH3 (PA10) -> FOCMini IN3
- PB12 -> FOCMini EN
- GND 共地（STM32 与 FOCMini）

说明：第一版仅实现 PWM 和 EN 安全封装，不在固件中执行自动旋转或闭环控制。

## 2. STM32F103C8T6 引脚分配表

| 功能 | 引脚 | 备注 |
|---|---|---|
| AS5600 OUT (ADC) | PA0 (ADC1_IN0) | 读取模拟角度 |
| AS5600 I2C SCL | PB6 (I2C1_SCL) | 读取 RAW ANGLE |
| AS5600 I2C SDA | PB7 (I2C1_SDA) | 读取 RAW ANGLE |
| FOCMini IN1 | PA8 (TIM1_CH1) | 三相 PWM U 相 |
| FOCMini IN2 | PA9 (TIM1_CH2) | 三相 PWM V 相 |
| FOCMini IN3 | PA10 (TIM1_CH3) | 三相 PWM W 相 |
| FOCMini EN | PB12 | 驱动使能控制 |
| 调试串口 TX/RX | PA2/PA3 (USART2) | 115200 8N1 |
| 运行灯 | PC13 | 心跳闪烁 |

## 3. 为什么把 USART1 改到 USART2

将调试串口改到 USART2（PA2/PA3）主要是为了避免与其他外设/调试资源冲突，并与当前 CubeMX 工程配置保持一致。这样可确保打印通道稳定，便于后续电机驱动联调时观察日志。

## 4. TIM1 三路 PWM 的作用

TIM1 CH1/CH2/CH3 分别输出三相占空比命令（U/V/W）。在当前阶段：

- PWM 通道会初始化并启动输出；
- 默认占空比保持为 0；
- 不做电角度调制、不做 FOC 运算。

这为下一阶段开环/闭环控制保留统一接口。

## 5. PB12 EN 的安全逻辑

安全规则：

1. 上电初始化时必须先 `EN=Low`。
2. `Disable` 时先将三相 PWM 置零，再拉低 EN。
3. `Enable` 仅负责拉高 EN，不隐式修改占空比。
4. 占空比输入做 `0.0f ~ 1.0f` 限幅，避免越界写入比较寄存器。

## 6. 上电安全检查清单

每次上电或刷写后，先确认：

- [ ] 串口日志正常输出（USART2, 115200 8N1）
- [ ] `FOCMINI_EN` 默认低电平
- [ ] TIM1 CH1/CH2/CH3 占空比为 0
- [ ] 电机不自转
- [ ] AS5600 ADC/I2C 对比日志持续更新
- [ ] PC13 运行灯按预期闪烁

## 7. 后续开发路线

1. PWM 骨架（当前阶段）
2. 手动 enable 测试（仅人工触发）
3. 开环三相 PWM 测试（低占空比、限流保护）
4. 结合 AS5600 角度反馈
5. SPWM / SVPWM 调制
6. 闭环控制（速度环/电流环按阶段推进）
