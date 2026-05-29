# FOC Goal and Implementation Plan

本文档用于规划本项目后续 FOC 方向的开发目标。它基于当前仓库代码、AS5600 数据手册、2804 电机工程图、SimpleFOC Mini 原理图和一组 SimpleFOC 参考 PID 截图整理而成。

本文件只描述目标和实施计划，不表示当前固件已经完成 FOC、完整速度环、位置环、电流环或力矩闭环。

## 1. 项目现状分析

主工程位于：

```text
examples/stm32f103c8t6_as5600_adc_i2c_compare
```

当前工程已经具备做 FOC 前的基础平台能力：

- STM32F103C8T6 + STM32 HAL + STM32CubeIDE 工程。
- FreeRTOS/CMSIS-RTOS2 任务框架：
  - `ControlTask`：1ms 周期调用 `MotorControl_Tick1ms()`。
  - `FeedbackTask`：20ms 周期调用 `MotorFeedback_Update(HAL_GetTick())`。
  - `UITask`：LCD/touch UI 更新。
  - `MonitorTask`：PC13 heartbeat 和可选栈余量监控。
- TIM1 CH1/CH2/CH3 已用于三相 U/V/W PWM 输出。
- `FOCMINI_EN` 由 GPIO 控制，停机路径保持先清 PWM、再关闭 EN。
- `motor_driver.c` 已封装三相 PWM duty 写入和驱动使能/关闭。
- `motor_feedback.c` 已实现 AS5600 RAW_ANGLE 读取、角度换算、RPM 估算、反馈快照、I2C 恢复。
- `motor_control.c` 已实现：
  - OPEN 开环三相正弦驱动。
  - STOP/START/FAULT 基础状态机。
  - CLSPD 原型速度模式。
  - 保守 speed PID 原型，用于调整开环正弦相位推进 period。
- `motor_ui.c` 已提供 LCD/touch 调试界面，显示 angle、RPM、目标转速、period、PID output、状态和故障。

当前还没有实现：

- FOC 电角度闭环调制。
- 电角度零点标定流程。
- SVPWM 或基于 Clarke/Park 变换的正式调制层。
- 电流采样、电流环、力矩闭环。
- 位置环。
- 已整定的生产级 speed PID。

当前 CLSPD 仍应视为 speed closed-loop prototype。它根据 AS5600 RPM 调整正弦相位推进 period，但三相电压矢量并没有锁定到转子电角度，因此还不是 FOC。

## 2. AS5600 角度读取和诊断要点

AS5600 是 12-bit 磁编码器，适合作为本项目 FOC 的机械角度反馈来源。

### 基本寄存器

- I2C 7-bit 地址：`0x36`，STM32 HAL 中使用左移后的地址 `(0x36U << 1)`。
- `RAW_ANGLE`：`0x0C/0x0D`，未经过零点/量程映射的 12-bit 原始角度。
- `ANGLE`：`0x0E/0x0F`，经过芯片内部零点/最大角设置映射后的角度。
- `STATUS`：`0x0B`，磁铁状态诊断：
  - `MD`：magnet detected。
  - `ML`：magnet too weak。
  - `MH`：magnet too strong。
- `AGC`：`0x1A`，自动增益值，可用于判断磁铁距离是否合适。
- `MAGNITUDE`：`0x1B/0x1C`，磁场幅值诊断。

### 角度与方向

- AS5600 一圈为 `0..4095`，对应机械角 `0..360 deg`。
- 数据手册说明 DIR 引脚会改变角度递增方向：
  - DIR 接 GND：从芯片顶部观察，顺时针转动时角度递增。
  - DIR 接 VDD：逆时针转动时角度递增。
- 当前代码已经在 feedback 层把 `delta_raw`、`total_raw_turns`、`rpm_x10` 修正为 motor-direction aligned 语义：
  - FWD 显示正 RPM。
  - REV 显示负 RPM。
  - `raw_angle` / `angle_x100` 仍保留原始传感器角度显示语义。

### 诊断与可靠性

后续 FOC 前建议补强 AS5600 诊断：

- 读取 `STATUS`，启动和运行中要求 `MD=1`。
- 如果 `ML=1` 或 `MH=1`，禁止进入 FOC RUN，提示磁铁过弱或过强。
- 读取 `AGC` 和 `MAGNITUDE`，用于装配调试：
  - 数据手册建议 AGC 尽量位于量程中间。
  - 典型气隙约 `0.5 mm..3 mm`，实际取决于磁铁。
  - 推荐磁场范围约 `30 mT..90 mT`。
  - 磁场低于检测阈值时会触发 magnet missing 诊断。
- 保留当前 I2C recovery：
  - 连续读取失败后限流执行 `HAL_I2C_DeInit()` / `MX_I2C1_Init()`。
  - 传感器后上电时可以自动恢复。
- FOC 阶段需要更严格的 feedback freshness：
  - angle sample 过旧时必须禁止输出或降级停机。
  - speed sample 过旧时不得继续积分速度环。

### 采样周期注意事项

当前 `FeedbackTask` 为 20ms 周期，`motor_feedback.c` 内部 speed 更新周期为 200ms。这对 UI 和低速 speed prototype 足够，但对真正 FOC 电压模式偏慢。

FOC 电压模式正式启用前，应评估：

- 将 angle 采样周期降到 1ms、2ms 或 5ms。
- I2C1 目前为 100 kHz，必要时评估 400 kHz，但需要 CubeMX 配置和实测确认。
- 阻塞式 HAL I2C 读取不能放在中断中；仍应由 `FeedbackTask` 或专门 sensor task 执行。
- 控制侧只能读取 snapshot，不能直接在 `ControlTask` 中阻塞访问 I2C。

## 3. 2804 电机关键参数

来自 2804 电机工程图的关键参数如下，后续 FOC 计算必须优先使用这些机械与电气边界。

| 参数 | 数值 | 备注 |
| --- | --- | --- |
| 型号 | 2804 | 小型三相无刷电机 |
| 适用电压 | 7.4V..16V DC | 额定电压 12V，瞬时最大电压可到 20V |
| 槽极数 | 12N14P | 12 槽 14 极 |
| 极对数 | 7 | `pole pairs = 7`，FOC 电角度计算必须使用 |
| 定子尺寸 | 28mm | 2804 中 28 表示定子直径，04 表示定子高度 |
| 电机尺寸 | 34.5 x 15mm | 含径向磁铁总高约 19.5mm |
| 重量 | 37g | 小电机，热容量有限 |
| 绕线电阻 | 5.1 ohm | 工程图给出的绕组电阻 |
| 相电阻 Rs | 2.55 ohm | 可用于粗略电流估计 |
| 额定电流 | 0.5A | 工程图说明 1A 以内温升正常 |
| 最大电流 | 2A | 后续测试必须限流 |
| KV | 220 | 参考空载速度常数 |
| 转速 | 2600 RPM / 12V | 空载参考 |
| 绕线电感 | 2.8mH | 工程图给出的绕组电感 |
| 相电感 Ls | 0.86mH | 后续电流环建模参考，目前不实现电流环 |
| 磁链 Flux | 0.0035 Wb | 仅作模型参考 |
| 力矩 | 300 g.cm，约 0.03 Nm | 标注说明不同测试方式会有差异 |

最重要的 FOC 参数：

```text
MOTOR_POLE_PAIRS = 7
```

机械角和电角度关系：

```text
electrical_angle = normalize(pole_pairs * mechanical_angle + zero_electric_offset)
```

其中 `zero_electric_offset` 必须通过安全的低电压标定流程获得，不能随意猜测。

## 4. FOC 电压模式实现计划

第一版目标是 voltage-mode FOC prototype，不实现电流环，不实现力矩闭环。

### 阶段 0：准备与保护

- 新增或规划独立的 FOC 参数：
  - `MOTOR_POLE_PAIRS = 7`
  - `voltage_limit`
  - `alignment_voltage`
  - `zero_electric_offset`
  - `angle_direction`
  - `angle_fresh_timeout_ms`
- 扩展 AS5600 诊断：
  - 读取 `STATUS`、`AGC`、`MAGNITUDE`。
  - FOC RUN 前必须确认 magnet detected。
- 保留当前 OPEN 和 CLSPD，不直接替换现有可工作的调试路径。
- UI 增加 FOC 调试信息时必须复用缓存刷新逻辑，不做大面积频繁刷屏。

### 阶段 1：电角度基础

- 从 `MotorFeedbackSnapshot_t` 获取机械角：

  ```text
  mech_angle_q = raw_angle / 4096 turn
  ```

- 根据 7 极对数计算电角度：

  ```text
  elec_angle = normalize(7 * mech_angle + zero_electric_offset)
  ```

- 明确角度方向：
  - AS5600 raw angle 显示语义保持不变。
  - 控制内部需要单独定义 FOC angle direction。
  - 方向错误时，FOC 会表现为无法稳定出力或反向转矩，必须通过低电压测试确认。

### 阶段 2：电角度零点标定

FOC 必须知道电角度零点。建议做成单独的校准流程，不在上电后自动大动作执行。

建议流程：

1. 电机空载、低电压、限流电源。
2. 设置很小的 `alignment_voltage`。
3. 输出固定电压矢量，使转子吸附到已知电角度。
4. 等待短时间稳定。
5. 读取 AS5600 raw angle。
6. 计算并保存 `zero_electric_offset`。
7. 清 PWM、关闭 EN。

注意：

- 该流程是 FOC 所需的安全校准步骤，不是当前已实现功能。
- `alignment_voltage` 必须很小，且需要超时保护。
- 校准失败或 AS5600 诊断异常时不得进入 FOC RUN。

### 阶段 3：3PWM 电压模式 FOC

SimpleFOC Mini 原理图显示驱动级是 3PWM 类输入，包含三相 OUT1/OUT2/OUT3、IN1/IN2/IN3、EN、FAULT、SLEEP、RESET 等典型 DRV8313 风格接口。当前工程也正好使用：

```text
TIM1_CH1 PA8  -> U/IN1
TIM1_CH2 PA9  -> V/IN2
TIM1_CH3 PA10 -> W/IN3
PB12          -> FOCMINI_EN
```

第一版可采用 voltage-mode FOC：

- `Ud = 0`
- `Uq = target_voltage`
- 使用电角度做反 Park/Clarke 或等价三相正弦合成。
- 输出到 `MotorDriver_SetPwmDutyPermyriad()`。

概念公式：

```text
Ua = center + Uq * sin(theta_e)
Ub = center + Uq * sin(theta_e - 120 deg)
Uc = center + Uq * sin(theta_e + 120 deg)
```

现有 `kSineLut` 和三相 120 deg offset 可复用，但相位来源必须从“开环相位累加器”切换为“转子电角度 + 期望转矩角”。

关键边界：

- 不引入电流采样时，`Uq` 只是电压命令，不是实际电流或实际力矩。
- 不应把 voltage-mode FOC 描述成电流环或力矩闭环。
- 不应直接用当前 CLSPD 的 period PID 替代 FOC；FOC 输出应由转子角度锁相。

### 阶段 4：模式集成

建议新增独立模式，而不是直接改写 OPEN/CLSPD：

- `OPEN`：保留当前开环正弦调试。
- `CLSPD`：保留当前 speed PID prototype，直到 FOC speed loop 可替代。
- `FOC_VOLTAGE`：新增 voltage-mode FOC 调试模式。

`FOC_VOLTAGE` 进入 RUN 前必须满足：

- AS5600 angle valid。
- AS5600 诊断 OK。
- angle sample 新鲜。
- 已完成 `zero_electric_offset` 标定。
- voltage limit 非零且处于安全范围。
- PWM 为 0 后再 enable。

## 5. 速度环实现计划

速度环应建立在 voltage-mode FOC 之上，而不是继续调节开环 phase period。

### 输入与输出

- 输入：
  - `target_rpm`
  - `actual_rpm`
  - feedback freshness
- 输出：
  - `Uq` 电压命令
- 约束：
  - `Uq` 必须限幅。
  - `Uq` 必须有斜率限制。
  - speed sample 未更新时不得重复积分旧速度。

### 与当前 CLSPD 的关系

当前 CLSPD：

```text
target RPM -> speed PID -> phase period
```

FOC 速度模式应逐步变为：

```text
target RPM -> speed PID -> Uq voltage command -> voltage FOC -> three-phase PWM
```

速度环仍可复用当前已经验证过的思路：

- 只在新的 speed sample 到来时更新 PID。
- reset 后跳过旧 sample。
- integrator clamp。
- output clamp。
- fault/stop/mode change/direction change 时 reset PID。
- feedback lost 后清 PWM、关 EN、进入 FAULT。

### SimpleFOC 参考 PID 的使用边界

上传的参考图中，SimpleFOC 示例使用：

```text
BLDCMotor motor = BLDCMotor(7);
PIDParams vel_pid = {0.15f, 1, 0, 0.01f};
PIDParams pos_pid = {20, 25, 0, 0};
```

这些值只能作为参考，不应直接当作本项目已整定参数。

原因：

- MCU、PWM 频率、采样周期、驱动板、电源、电机负载都不同。
- 当前项目还没有电流采样。
- 当前 speed sample 周期和滤波策略与 SimpleFOC 示例不一定一致。
- 2804 小电机热容量有限，过大的 PI 参数可能导致突然加速、震荡或发热。

第一版建议：

- 先使用 P 或弱 PI。
- `Uq` voltage limit 从很小值开始。
- 低速空载验证，再逐步提高速度和电压限幅。
- 以实际 RPM 是否稳定接近目标为准，不追求快速响应。

## 6. 位置环实现计划

位置环应作为 FOC 电压模式和速度环稳定后的后续阶段。

推荐结构：

```text
target_position -> position PID -> target_velocity -> speed PID -> Uq -> voltage FOC
```

实现要点：

- 使用 AS5600 raw angle 和 `total_raw_turns` 构建连续机械位置。
- 区分单圈 absolute angle 和多圈位置。
- position loop 先只做慢速、低电压、空载实验。
- 初期建议 P 或 PD，不建议立刻加入很大的 I 项。
- 需要位置目标限幅、速度目标限幅、Uq 限幅。
- 大位置误差时使用速度限制，避免一步给出过大 `Uq`。
- 位置环不应和当前 UI 大幅耦合；UI 只提供目标设置和状态显示。

位置环进入条件：

- FOC zero offset 已标定。
- angle 方向确认正确。
- voltage FOC 能稳定输出。
- 速度环已经能可靠限速。
- STOP/FAULT/ClearFault 路径已验证。

## 7. 安全保护和限幅

本项目没有相电流采样，因此后续 FOC 计划必须把安全边界放在电压、时间、状态机和人工限流上。

### 必须保留的停机顺序

```text
MotorDriver_SetAllPwmZero()
MotorDriver_Disable()
更新 state / stop_reason / fault
```

### 电压与 duty 限幅

- 2804 额定电流约 0.5A，最大电流 2A。
- 相电阻约 2.55 ohm，可用于估算低速堵转时的电流风险。
- voltage-mode FOC 初始 `Uq` limit 应保守，例如从 0.5V..1.0V 级别开始，再按上板温升和电流逐步调整。
- duty 和调制幅度必须保持硬限幅。
- `Uq` 和 target velocity 必须有 slew rate limit。

### 传感器保护

- angle invalid：拒绝进入 FOC。
- AS5600 `MD=0`：拒绝进入 FOC。
- `ML=1` / `MH=1`：拒绝或至少进入 warning，不应继续高能量输出。
- angle sample stale：FAULT 或安全停机。
- I2C 连续失败：保持现有 recovery，同时控制侧必须停止使用过期反馈。

### 状态机保护

- fault 未清除时不得启动。
- Stop 不自动清 fault。
- ClearFault 不自动启动，不恢复 PWM，不打开 EN。
- mode change / direction change / target mode change 必须 reset 控制器积分状态。
- FOC calibration 和 FOC run 应有单独状态表达，避免 UI 上看不出当前正在吸附、运行或故障。

### 硬件保护

- 初次 FOC 测试使用限流电源。
- 不装桨，不带高惯量负载。
- 电机固定，手远离转子。
- 驱动板、电机、AS5600 供电和地线必须可靠。
- 发现异常震动、啸叫、发热、失控时立即断电。

## 8. 构建检查方式

后续每个实现 PR 至少执行以下本地源码检查：

```bash
git diff --check
rg -n "conflict marker pattern" examples/stm32f103c8t6_as5600_adc_i2c_compare/Core examples/stm32f103c8t6_as5600_adc_i2c_compare/stm32f103c8t6_as5600_adc_i2c_compare.ioc
```

其中 `conflict marker pattern` 指仓库约定的 Git 冲突标记检查正则。提交前实际执行时应使用 AGENTS.md 中的完整命令。

STM32CubeIDE / CubeMX 一致性检查：

- `.ioc` 与生成代码保持一致。
- 不提交 `Debug/`、`Release/`、`.settings/`、`*.launch`。
- 对 CubeMX 生成文件，业务代码尽量放在 USER CODE 区域。
- `MX_DMA_Init()` 仍只调用一次，并且在 `MX_SPI1_Init()` 之前。
- `stm32f1xx_it.c` 中不应重新出现 `MotorControl_Tick1ms()` 的中断调用。

编译和上板检查：

- 使用 STM32CubeIDE 编译主工程。
- 连接 ST-Link 烧录。
- 先验证 PC13 heartbeat、LCD/touch、OPEN 启停、AS5600 angle/RPM。
- 再验证 FOC calibration。
- 最后才验证 FOC voltage mode、speed loop 和 position loop。

如果本地没有命令行构建环境，应在 PR 描述中明确说明未运行命令行编译，并由上板测试记录补充。

## 9. README 更新要求

后续实现 FOC 相关功能时，README 必须按真实进度更新，不能提前夸大。

建议状态描述：

- OPEN open-loop sine PWM drive：Completed。
- AS5600 angle/RPM feedback：Completed。
- FreeRTOS task split：Completed。
- CLSPD speed PID：Prototype / In Progress。
- FOC voltage mode：Planned，直到完成上板验证后才能改为 Prototype / In Progress。
- FOC speed loop：Planned 或 Prototype / In Progress，取决于是否已经上板验证。
- FOC position loop：Planned，直到真实实现并验证。
- Current loop：Planned / Not Implemented。
- Torque closed-loop：Not Implemented。

README 中应补充：

- 2804 电机 `12N14P`，`pole pairs = 7`。
- FOC 仍是实验性质，不是量产级控制器。
- SimpleFOC 参考 PID 只作为调参参考，不是本项目已验证参数。
- 如果使用 SimpleFOC Mini / DRV8313 风格 3PWM 驱动，应说明当前没有相电流采样。
- 安全说明必须保留并强化。

## 10. 不应夸大的内容

以下内容不能写成已完成：

- 电流环。
- 力矩闭环。
- 生产级 speed PID。
- 生产级 position PID。
- FOC 实机整定完成。
- 完整 SVPWM 优化。
- 自动可靠的零电角度标定。
- 堵转保护。
- 过流保护。
- 温度保护。

可以准确描述为：

- 当前已经具备三相 PWM、AS5600 反馈、FreeRTOS 任务框架和 UI 调试基础。
- 当前 CLSPD 是基于 AS5600 RPM 的速度控制原型。
- 后续 FOC 第一阶段目标是 voltage-mode FOC prototype。
- 在没有电流采样前，`Uq` 只是电压命令，不是实际电流或真实力矩闭环。

## 推荐后续 PR 拆分

1. `feat: add AS5600 magnetic diagnostics`
   - 增加 `STATUS`、`AGC`、`MAGNITUDE` 读取。
   - 将 magnet detected / too weak / too strong 纳入 feedback snapshot。
   - UI 轻量显示磁铁诊断。

2. `feat: add FOC angle model`
   - 增加 pole pairs、mechanical angle、electrical angle、angle direction、zero offset 数据结构。
   - 只计算和显示，不输出 FOC PWM。

3. `feat: add safe FOC zero calibration`
   - 增加低电压电角度零点标定流程。
   - 必须有 timeout、voltage limit、STOP 可中断。

4. `feat: add voltage-mode FOC prototype`
   - `Ud=0`，`Uq` 手动限幅输入。
   - 复用 3PWM 输出路径。
   - 上板验证低速、低电压、空载行为。

5. `feat: add FOC speed loop prototype`
   - 速度 PID 输出 `Uq`。
   - 保守参数、限幅、anti-windup、freshness 检查。

6. `feat: add FOC position loop prototype`
   - position PID 输出 target velocity。
   - 速度环输出 `Uq`。
   - 限速、限幅、低速验证。
