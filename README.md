# 20261011 教具作业：重力前馈与双环 PID

**当前为 Step1 简化版：默认纯前馈，只有电流限幅和手动停机；不再自动执行角度、零位、超时等停机门槛。操作以 [Step1 简明步骤](docs/step1-simple.md) 为准。下文和旧指南中的保护说明仅描述旧版本，不用于当前版本。**

面向作业 Step1、Step2：按负载写重力前馈，实现位置保持、内外环在线整定、10°阶跃记录，配合 Keil 与 CubeMonitor 完成验收。默认上电停机，标定竖直向下零位后才允许输出。

## 打开与编译

- Keil：`MDK-ARM/261011_PID.uvprojx`，Arm Compiler 6。产物：`MDK-ARM/261011_PID/261011_PID.axf`。
- CLion：打开本目录，用 CMake Debug 配置和 GNU Arm 工具链。
- 命令行：`cmake --preset Debug`，`cmake --build --preset Debug`。CMake、Ninja、GNU Arm 工具链需要在 PATH 中。
- CubeMonitor 必须使用实际下载的那份 `.axf` 或 `.elf`，不能混用不同编译器产物的符号地址。

**先读 [调试与验收指南](docs/debug-guide.md)，再运行教具。**

## 初始配置

- 默认 **M2006 + C610，电调 ID=1**。使用 M3508 + C620 前修改 `UserCode/motor_config.hpp` 的 `MOTOR_MODEL=3508` 并重新编译。
- CAN1：PD0 RX / PD1 TX，1 Mbps；TIM6 控制周期 1 ms。
- 初始电流限幅 0.5 A、速度限幅 120°/s、角度范围 ±150°，都是保守调试起点，不是已实测达标参数。
- 上电 `stop_flag=1`。标零后人工置零才允许运行；反馈超时会停机并使零位失效。
- 停机在下一 1 ms 控制发送周期把 PID 和前馈的总输出覆盖为零，并取消旧的待发送命令。零电流不能主动保持负载，先做好支撑。
- 电机输出非零时不要暂停、单步或打断点；先置停机并确认零电流，现场保留物理断电手段。

## 实现结构

| 文件 | 用途 |
|---|---|
| `UserCode/PID.hpp` | 保留培训示例的构造参数、reset、calc 和成员命名，补齐初始化、微分滤波、积分限幅、抗饱和 |
| `UserCode/Motor.hpp` | 大端反馈解析、有符号速度和电流、编码器多圈累计、减速比换算 |
| `UserCode/motor_config.hpp` | 型号、ID、方向、负载参数与保护范围 |
| `UserCode/app.cpp` | 双环、前馈、Keil 调试变量、保护、500 点阶跃记录 |
| `tools/export_trace.py` | 导出的原始内存/Intel HEX 转 CSV |

控制链：目标角度 → 位置 PID → 目标速度 → 速度 PID → 电流修正 + 重力前馈 → 总限幅 → stop_flag 覆盖 → CAN1。

前馈为 `I_ff = ff_peak_current_a × ff_scale × sin(actual_angle_deg × π/180)`，零位竖直向下。2006 的 100 g、65 mm 对应峰值重力矩 0.063765 N·m，用手册 0.18 N·m/A 得初始峰值 0.35425 A。3508 的 500 g、55.24 mm 对应 0.2709522 N·m；缺少其电机转矩常数，不套用 2006 常数，初始电流幅值为零，按指南实测标定。

PID 保留每周期误差累加、误差差分，参数对应固定 1 ms，不显式乘除 dt。`calc(ref,fdb)` 仍可使用，新增第三个可选参数将前馈纳入速度环饱和判断，返回的仍是 PID 修正量。

## 工程检查与版本

原工程 CAN1 与 TIM6 配置正确，但 PLL 实际仍用 HSI。现同步修正 `main.c` 和 `.ioc`：12 MHz HSE，PLL M/N/P/Q=12/336/2/7，SYSCLK=168 MHz、PCLK1=42 MHz、TIM6 时钟=84 MHz。

当前 `.ioc` 生成 CMake；重新生成时保留 main 的 USER CODE。切换生成 Keil 后确认 app.cpp、UserCode 包含路径和 Arm Compiler 6 仍配置正确。控制源码只有一套，不分别维护 Keil/CLion 版本。

Git 保存源码、HAL/CMSIS 与已有许可证、工程配置和指南，忽略编译产物、IDE 私人配置、下载工具 ZIP。培训目录的 ST-LINK Server 与 Scope 压缩包未自动安装、不上传；本作业使用 CubeMonitor，无需引入 Scope。

软件编译和模拟验证不能冒充真实教具验收。本仓库不含虚构的达标截图、视频或硬件最终 PID 参数。
