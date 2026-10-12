# Step1 简化版：只做前馈

新版上电默认 control_mode=2（纯前馈）、stop_flag=1（停机）、限流0.5 A。
不会因为角度超过150°、未标零、反馈超时或发送错误把stop_flag强制改为1。
反馈状态只供观察，不作为启动门槛。非有限电流按零处理，2006电流硬上限10 A。

## 只需要三个操作变量

Watch 添加：stop_flag、zero_request、ff_scale。
可选观察：actual_angle_deg、command_current_a、rx_count。

1. 编译下载新版，进入Debug并Run，确认rx_count增加。
2. 保持stop_flag=1，负载重心放到轴正下方并静止。
3. 写zero_request=1，它自动回零；actual_angle_deg接近零。
4. 清理运动范围，写stop_flag=0，纯前馈开始输出。
5. 使用安全的拨动位置缓慢换到30°、60°等角度，松开检查是否基本静止，不把手放在夹点，不强行抵抗电机。
6. 往下落：停机后ff_scale增加0.05；往上走：停机后减少0.05。重新启用测试。
7. 正负多个角度能基本保持后录视频。target_angle_deg在此模式下不起作用，两个Kp不用改。
8. 结束前支撑负载，stop_flag=1。

不要运行中拔CAN线或让调试器暂停CPU；新版不做反馈超时自动停机，可能继续输出旧角度对应的电流。掉线/复位后手动停机、回到向下位置重新标零，不把本教学版本用于无人值守。

Step2仍保留：以后切control_mode=4使用双环，当前先不要调整PID。

## 本次验证

2026-10-12：Keil和GCC编译通过；已通过STM32CubeProgrammer下载到连接的STM32F407，并成功读回校验及复位。未自动启动电机，stop_flag上电保持1。
