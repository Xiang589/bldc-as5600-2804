# STM32F103C8T6 Flash Size Optimization Notes

本文档记录 `work/rtos-feedback-monitor` 分支中一次 Debug 构建 Flash 超限问题的定位、处理方式和后续经验总结。

相关提交：

```text
421a418 fix: unify motor sensor direction sign
e35a7cd fix: reduce Debug flash usage
```

## 1. 问题现象

在 STM32CubeIDE Debug 配置下编译时，C 文件基本都能正常编译成 object 文件，但最终链接 `.elf` 失败：

```text
section `.text' will not fit in region `FLASH'
region `FLASH' overflowed by 3448 bytes
collect2.exe: error: ld returned 1 exit status
```

这说明问题不是语法错误，而是链接阶段程序代码、只读常量和初始化数据放不进目标 Flash 区域。

本项目目标芯片按 STM32F103C8T6 处理，Flash 资源非常有限。当前 linker script 仍按 64KB 级别目标处理，不能简单把 linker script 里的 Flash 改大来绕过问题，否则会出现“编译通过但真实芯片放不下”的风险。

## 2. 优化原则

本次优化目标是让 Debug 构建通过，同时尽量保留电机控制核心功能。

保留内容：

```text
OPEN 开环正弦 PWM
CLSPD 原型速度闭环
FOCV 电压模式 FOC
FOCSPD FOC 速度环
FOCPOS FOC 位置环
AS5600 angle/RPM feedback
LCD/touch 正常按钮控制
FOC zero calibration 的 FCAL 入口
```

没有修改：

```text
.ioc
PWM / I2C / TIM / UART 引脚
FOC PID 参数
FOC 方向参数
linker script
```

主要优化方式：

```text
1. Debug 配置启用 size optimization
2. 关闭启动 UART / printf 测试日志
3. 通过宏真正裁剪触摸屏重新校准流程
4. 保留正常 LCD/touch 控制和 FOC FCAL 入口
5. 简短化部分状态显示字符串
```

## 3. Debug 构建改为 size optimization

Debug 配置通常偏向调试体验，可能使用 `-O0`，代码不会做体积优化。对于 64KB Flash 的 STM32F103C8T6，`-O0` 很容易导致 `.text` 超限。

本次在 `.cproject` 中把 C Compiler 和 C++ Compiler 的 Optimization level 改为 size optimization，即 `-Os`。

作用：

```text
- 优先减小代码体积
- 删除或合并部分冗余指令
- 配合 -ffunction-sections / -fdata-sections / --gc-sections，让链接器更容易丢弃未使用函数
```

注意：

```text
-Os 可能让单步调试体验变差一些。
但对于 64KB Flash 目标，先保证固件能放进 Flash 更重要。
```

## 4. 关闭启动 UART / printf 测试日志

`main.c` 中原本有一些启动阶段串口测试和调试打印，例如：

```text
UART direct test
printf test
ADC calibration result
AS5600 ADC/I2C compare start
```

这些打印用于早期确认 USART2 和 printf 重定向是否正常，不属于电机控制核心功能。

本次新增：

```c
#define ENABLE_BOOT_UART_LOG 0U
```

并把启动日志包到条件编译中：

```c
#if ENABLE_BOOT_UART_LOG
printf(...);
#endif
```

需要注意的是，ADC 校准动作本身仍然保留；关闭的只是 ADC 校准结果的 UART 打印。

这样可以减少：

```text
- 启动测试字符串
- printf 调用路径
- 不必要的早期调试输出
```

## 5. 真正裁剪触摸屏重新校准流程

之前代码里虽然已经有：

```c
#define ENABLE_TOUCH_CALIBRATION 0U
```

但很多触摸屏重新校准相关代码仍然参与编译，包括：

```text
CalPoint 结构体
CAL / YES / NO 按钮
5 点触摸校准状态机
Cal_Build()
Cal_Validate()
TouchCalStorage_Save() 相关路径
校准过程 printf
校准界面字符串
```

其中 `Cal_Build()` 使用了较多 `float` 运算。STM32F103C8T6 是 Cortex-M3，项目使用 soft-float，浮点计算可能额外拉入软件浮点相关代码，Flash 占用会比较明显。

本次把触摸屏重新校准流程真正包入：

```c
#if ENABLE_TOUCH_CALIBRATION
...
#endif
```

默认 `ENABLE_TOUCH_CALIBRATION = 0U` 时，以下内容不再参与编译：

```text
- 5 点触摸校准界面
- 触摸校准确认界面
- 校准用 float 仿射计算
- 校准结果验证函数
- 校准保存流程中的多处 printf
- CAL / YES / NO 相关按钮和状态变量
```

这次优化对 Flash 体积帮助较大。

## 6. 保留正常 LCD/touch 控制

关闭的是“触摸屏重新校准流程”，不是关闭触摸屏功能。

仍然保留：

```text
START
STOP / CLEAR
DIR
SET
MODE
SPD+ / SPD-
DUTY+ / DUTY-
FOC 模式下的 FCAL
状态显示
```

初始化逻辑变为：

```text
- 如果 Flash 中有触摸校准数据，则加载已保存校准。
- 如果没有保存过校准数据，则加载默认校准参数并直接进入主界面。
```

这样可以节省 Flash，但代价是：如果默认触摸参数不准确，按钮触摸可能不够准。

需要重新校准触摸屏时，可以临时打开：

```c
#define ENABLE_TOUCH_CALIBRATION 1U
```

完成校准保存后，再改回：

```c
#define ENABLE_TOUCH_CALIBRATION 0U
```

## 7. 精简部分状态字符串

LCD 状态显示中有一些较长字符串会进入 Flash 的只读数据区。本次顺手缩短了部分显示文本，例如：

```text
TgtRPM -> T
```

这种优化单独节省不大，但在 64KB Flash 目标上，字符串常量、HELP 文本、调试日志都应该尽量克制。

## 8. 优化结果

本次修复后，Debug 构建通过。

构建 size 结果：

```text
text = 59584
data = 316
bss  = 15556
```

Flash 主要消耗约为：

```text
text + data = 59584 + 316 = 59900 bytes
```

当前 FLASH region 约为 63KB，即约 64512 bytes，剩余约：

```text
64512 - 59900 = 4612 bytes
```

也就是说，本次从原先 overflow 3448 bytes 的状态，恢复到大约还有 4.6KB Flash 余量。

值得注意的是，本次直接运行旧 Debug makefile 时仍显示 `-O0`，但已经能链接通过。这说明仅靠代码裁剪已经节省了足够空间；后续 CubeIDE Clean / Rebuild 后真正应用 `.cproject` 中的 `-Os`，理论上还会有更多余量。

## 9. 经验总结

在 STM32F103C8T6 这类 64KB Flash MCU 上，以下内容很容易导致 Flash 超限：

```text
printf
长字符串
LCD UI 文本
浮点计算
触摸屏/传感器校准流程
调试日志
未真正被 #if 排除的可选功能
复杂状态机
```

推荐排查顺序：

```text
1. 确认错误发生在编译阶段还是链接阶段。
2. 如果是 FLASH overflow，不要先改 linker script。
3. 查看 size / map，确认大模块来源。
4. Debug 配置优先尝试 -Os。
5. 关闭不必要的 printf 和启动日志。
6. 把可选调试/校准功能真正放进 #if 条件编译。
7. 避免在小 MCU 上无必要使用 float。
8. 保留核心功能，裁剪非核心功能。
```

## 10. 后续新增串口上位机的注意事项

目前剩余 Flash 约 4.6KB，不算宽裕。后续如果继续新增串口协议和上位机控制，固件端必须控制体积。

建议：

```text
1. CommTask 可以做，但命令和回复要短。
2. 不要在固件里放大段 HELP 说明文本。
3. 不要引入 JSON 解析库。
4. STATUS 输出保持短格式。
5. 尽量使用整数单位，例如 rpm_x10、deg_x10、mV。
6. 避免大量 printf 浮点格式化。
7. PC 上位机复杂逻辑放到电脑端，STM32 只做简单命令解析。
```

推荐状态回复格式：

```text
OK
ERR
STAT M=FOCV S=STOP RPM=0 UQ=0 Z=1
```

不推荐：

```text
{
  "mode": "FOCV",
  "state": "STOP",
  "rpm": 0,
  "uq": 0,
  "zero_calibrated": true
}
```

JSON 可读性更好，但固件端解析和格式化成本更高，不适合作为当前 64KB Flash 版本的首选方案。

## 11. 结论

本次 Flash 优化不是靠扩大 Flash 区域，也不是删除 FOC 核心功能，而是通过：

```text
- Debug 使用 size optimization
- 关闭启动日志
- 真正裁剪触摸屏重新校准流程
- 减少部分字符串
```

让固件重新适配 STM32F103C8T6 64KB Flash 目标。

后续继续开发时，应优先保持“核心控制逻辑 + 最小调试入口”的思路，把复杂功能尽量放到 PC 上位机侧实现。
