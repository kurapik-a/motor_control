# PWM 移植说明

> 2026-11-03后续更新：ADC已接入PWM硬件同步触发，详见《ADC_PWM同步采样说明.md》。以下软件触发阶段的描述为移植历史；默认仍不使能桥臂，不绑定FOC电流环回调。

日期：2026-11-03。来源为已调通的 `single_motor_monitor_vofa`，目标为拆分中的 `motor_control`。
本次只接入 PWM 模块；没有下载固件或操作电机。

## 文件和原函数对应关系

| 原工程 | 新接口 | 用途 |
|---|---|---|
| pmsm_init.c / pwm_pins_init | pwm_init 内部 | PWM0，PE00~PE05，UH/UL/VH/VL/WH/WL |
| pmsm_init.c / pwmv2_duty_init | init_waveforms | 三对中心对齐互补 PWM，保留原比较器及影子寄存器分配 |
| pmsm_init.c / pwmv2_trigfor_adc_init | init_trigger | 周期末 ADC 比较点 16、TRGO0，影子寄存器 9 |
| pmsm_init.c / pwmv2_trigfor_currentctrl_init | init_trigger | 电流环比较点 17、TRGO1，影子寄存器 10 |
| bldc_foc_callback.c / bldc_foc_pwmset | pwm_set_counts | SVPWM 宽度计数转换为左右比较点 |
| bldc_foc_callback.c / disable_all_pwm_output | pwm_disable | 六路立即强制低电平，保留计数器运行 |
| bldc_foc_callback.c / enable_all_pwm_output | pwm_enable | 显式撤销六路软件强制关断 |
| pmsm_currentctrl.c / isr_current_loop | pwm_current_loop_isr | 预留 20 kHz 控制回调；当前没有绑定 FOC |

公开接口在 app/inc/pwm.h，实现在 app/src/pwm.c。CMake 已包含 pwm.c，保留用户原先添加的源文件条目。

## 上电行为

main.c 在 board_init 后调用 pwm_init，并通过 pwm_result 保留返回值。
首次初始化：配置时钟及三相互补波形 → 强制六路低电平 → 设置三相 50% 初值 → 准备比较触发点 → 映射 PE00~PE05 → 同步启动计数器。
不调用 pwm_enable；不绑定电流环回调。后续继续初始化编码器、ADC 和 VOFA。
重复调用 pwm_init 返回 0，保留当前输出、回调和配置，不重新配置正在运行的通道。

## 参数和接口

- 频率：motor_config.h 的 MOTOR_PWM_FREQUENCY_HZ，当前 20000 Hz。
- 周期计数：电机 PWM 时钟 / PWM 频率，由 pwm_get_reload() 读取，不写死时钟频率。
- 死区：MOTOR_PWM_DEADTIME_HALF_TICKS，当前 200；单位沿用芯片半周期计数，不直接当作 ns。
- 电流中断比较点：MOTOR_PWM_CURRENT_ISR_DELAY_TICKS，当前 200；它本身不代表 ADC/DMA 已经完成。
- pwm_set_counts(u,v,w)：范围 0~reload，保持原 SVPWM 计数接口尺度；奇数宽度按原实现取半宽整数。
- pwm_set_duty(u,v,w)：范围 0~1，例如 0.5 表示 50%；越界或 NaN/Inf 返回 -3，保留旧设置。
- pwm_enable()/pwm_disable()：仅管理输出；不会自动运行三环或完成电角度对齐。标定及故障门禁由之后的上层控制负责。
- pwm_set_current_loop_callback(fn)：绑定 PWM 周期回调并开放比较中断；不使能桥臂。传 NULL 关闭该中断并关闭桥臂。
- 所有三相更新和输出使能切换使用短临界区；回调在中断内执行，不能阻塞。

## ADC 与后续 FOC

当前 adc.c 是主循环软件触发、等待 DMA 完整数据的独立验证驱动。本次保留它，未连接 TRGM 的 PWM→ADC 路由，避免两种触发方式冲突。
PWM 中的 ADC 比较点和 TRGO 已准备好，但尚不触发当前 ADC。之后接入 FOC 时，需要一起改造 ADC 硬件触发、双路样本完成检查、控制回调、保护及使能流程。
不要在 PWM 回调内直接调用当前 adc_poll/adc_read；它们目前只支持主循环使用。

## 验证范围

motor_control 默认 MOTOR_CONTROL_BUILD_MOTOR=OFF，原保留 motor/core 扩展没有开启。
GCC 构建与 SES Debug 构建通过，验证了 SDK 1.9 接口和链接。未实测频率、死区、互补波形、ADC 同步或带电电机运行。
