# PWM、ADC 移植交接文档

更新日期：2026-11-03。本文记录 PWM/ADC 移植阶段结束、临时时序测量已撤除后的状态。

## 1. 两个工程的身份

| 工程 | 绝对路径 | 用途 |
|---|---|---|
| 原工程 | `C:\Users\Sinx\Desktop\HPM6E8Y\single_motor_monitor_vofa` | 已调通的完整电机控制工程，作为功能、接线和参数参考 |
| 当前工程 | `C:\Users\Sinx\Desktop\HPM6E8Y\motor_control` | 正在重新整理、分模块移植的工程；后续开发在这里继续 |

不要把“原工程已有三环闭环”理解为“当前工程也已接通三环”。当前阶段完成的是底层采样和输出基础。

## 2. 完成情况

| 模块 | 当前状态 | 尚未完成的边界 |
|---|---|---|
| 编码器 | QEI A/B 读取、位置及速度、输出轴换算已接入 | SPI 分支尚未接入；FOC 电角度对齐流程未接入 |
| PWM | PWM0 六路互补输出、死区、占空比/计数接口、显式使能关闭已移植 | 尚未由闭环算法实时生成占空比 |
| PWM 开环测试 | 已实际验证电机可以慢速转动 | 测试调用已从主循环和 VOFA 移除，不是当前默认功能 |
| 同步电流 ADC | PWM 硬件触发 U/V、双路完成确认、DMA 校验、快照发布、零偏标定已移植 | 闭环带载、全部占空比范围的采样窗口仍待联调 |
| 母线、温度 | 已恢复为独立周期采样 | 当前只提供原始码，尚未接入完整保护和物理量换算 |
| VOFA | 恢复三个编码器通道，100 Hz 上传 | 当前没有接入原工程的电机 ENABLE/SET 控制命令 |
| 电流/速度/位置闭环 | 尚未接通 | 配置头中的 PI 数值不等于控制算法已经运行 |

## 3. 当前启动和运行流程

入口：`app/src/main.c`。

1. `board_init()`：初始化板级基础资源。
2. `pwm_init()`：启动 PWM 计数及相关配置，六路功率输出保持强制低电平。
3. `encoder_init()`：初始化 QEI 编码器。
4. `motor_vofa_init()`：先完成串口等耗时初始化。
5. `adc_init()`：配置同步电流采样、独立辅助采样，开始自动零偏标定。
6. 主循环执行 `adc_poll()`、`adc_read()`，将快照放入调试变量 `adc_sample`；执行 `encoder_read()`、`motor_vofa_set_encoder()`、`motor_vofa_poll()`。

当前主循环不调用 `pwm_enable()`，上电不会自动驱动电机。PWM 计数器运行与桥臂输出使能是两个状态。

## 4. 关键文件和接口

以下路径均相对于当前工程根目录。

| 文件 | 职责与主要接口 |
|---|---|
| `motor/inc/motor_config.h` | PWM 频率、死区、ADC 比例、零偏、超时及电机/机械参数；有些原三环参数目前只是保留配置 |
| `app/inc/pwm.h`、`app/src/pwm.c` | `pwm_init`、`pwm_set_counts`、`pwm_set_duty`、`pwm_enable`、`pwm_disable`；ADC 触发路由和时间基准接口 |
| `app/inc/adc.h`、`app/src/adc.c` | `adc_init`、`adc_poll`、`adc_read`、`adc_start_offset_calibration`；数据字段和返回码说明见头文件 |
| `app/inc/encoder.h`、`app/src/encoder.c` | 编码器位置、测速与输出轴换算 |
| `app/inc/motor_vofa.h`、`app/src/motor_vofa.c` | 编码器快照及 JustFloat 上传 |
| `ADC_PWM同步采样说明.md` | 同步采样实现、配置及测量结束后的正式状态 |

`pwm_set_current_loop_callback()` 预留 PWM 比较中断回调，当前未绑定 FOC。同步采样使用独立 ADC 完成中断，不依赖这个回调。后续确定 FOC 运行入口时，需要同时核对数据新鲜度和 PWM 更新时刻，不能直接假设比较中断发生时本轮 DMA 已就绪。

## 5. PWM 与采样配置

- PWM0，PE00～PE05：UH/UL/VH/VL/WH/WL。20 kHz，周期约 50 μs；当前 reload=10000，PWM 时钟约 200 MHz。
- PWM 硬件计数器向上计数，三相脉冲通过比较点形成中心对齐的互补波形；不要将其误认为上下计数模式。
- ADC 电流触发比较点为 reload-1，使用 PWM0 TRGO0 同时触发两路抢占队列。
- U 相：ADC0，PF06/CH14；V 相：ADC1，PF08/CH10。
- 母线：ADC0，PF31/CH15；温度：ADC0，PF26/CH1，使用独立周期采样，不并入电流帧。
- 电流 ADC 使用完整 16 位码。零偏后按 `(offset-raw)` 换算 mA，W 相由 `-Iu-Iv` 推算。
- 当前硬件比例为参考 3.3 V、增益 10、采样电阻 1 mΩ，约 5.0355 mA/码。
- 自动零偏：先等待稳定，再收集 256 个独立样本，约 1 ms 接受一个标定样本。以 `offset_state==ADC_OFFSET_READY` 和 `current_valid` 为准。

零偏标定必须在桥臂关闭、无实际相电流条件下进行。初始化成功或 `adc_read()==0` 不能单独作为电流已可用于闭环的依据，还要检查 `current_valid`。

## 6. 同步采样流程及保留修复

PWM 触发 → 两路 ADC 转换并写 DMA → ADC 完成中断调用 `service_frame()` → 确认两路完成、校验 DMA 标识及截止时间 → 换算/发布完整快照 → 为下一轮重新接通触发。

主循环 `adc_poll()` 提供健康检查及延期重臂重试，并读取母线、温度周期结果；`adc_read()` 读取一致快照，不直接访问正在更新的 DMA。

测量结束后保留以下正式修复，不能将当时全部 Git 更改一键丢弃：

- 采样关键路径放入 ILM 快速执行区，关键 SDK 小函数强制内联；通过最终 ELF 检查，避免仅看源码属性便误判执行位置。
- 母线/温度结果读取移到关中断区外，只用短临界区提交数据。
- DMA 和完成状态先清理，再计算下一次触发时刻；接通路由前再次检查边沿余量。
- CPU/PWM 时间配对跨度过大或临近边沿时延期重试，避免错误的触发时间预测。
- DMA 读取前后都检查 10 μs 截止时间，拒绝超时旧帧。

采样故障会锁存并关闭触发路由和桥臂；需要重新 `adc_init()` 恢复。`-7` 表示超时，结合 `timeout_stage`、`timeout_completion_mask`、`irq_count` 和 `sample_count` 排查。初始化或重标定在桥臂使能时返回 -9。

## 7. 测量结论和验证范围

此前板上记录的 31 帧有效测量中，估计 PWM 触发至软件确认 DMA 就绪约 1.592～1.917 μs，平均约 1.814 μs；触发至发布最高约 2.048 μs。

这些值包含软件观测开销；触发时间来自 PWM/CPU 计数配对估算，不是硬件捕获时间戳。不能将其直接说成 ADC 纯转换时间，也不能据此保证所有 20 kHz 周期均小于 2 μs。旧工程在约 200 个 PWM 计数后进入读取路径，还包含中断响应与函数开销，不能把 200 个计数直接当成实际 DMA 读取时刻。

已完成：PWM 开环实际转动验证、同步采样和零偏实际观测、时序上报测量。撤除临时测量后的版本已通过 GCC、SES Debug 编译以及 ADC/PWM 主机回归和最终 ELF 快速区检查；该恢复版本仍需用户重新下载到板上确认运行。

可复用检查：

- `tests/run_adc_pwm_sync_tests.ps1`
- `tests/run_pwm_adc_timing_tests.ps1`
- `tests/run_pwm_test.ps1`：保留的开环测试逻辑回归，不代表主程序正在运行开环测试。
- `tests/check_adc_realtime_placement.py`：检查最终 ELF 的快速区放置和关键调用；使用 SDK Python，传入 ELF 路径及 `--objdump` 工具路径。
- GCC 构建入口：`build.ps1`；SES 工程：`build/segger_embedded_studio/motor_control.emProject`。

## 8. 现在如何观察

VOFA 使用 JustFloat、115200 bit/s，100 Hz：CH0=输出轴累计角度（度），CH1=输出轴原始转速（rpm），CH2=输出轴滤波转速（rpm）。减速比目前为 7.75；编码器每电机轴一圈 4000 计数，已包含四倍频。

临时时序结构、专用 16 通道上报和专用测量测试已移除。电流采样状态通过调试器查看 `adc_result`、`adc_sample`，PWM 初始化看 `pwm_result`。正常持续采样时应看到 `sample_count` 增长、`fault_code==0`、零偏就绪后 `current_valid==true`。

## 9. 下一阶段建议

1. 接入坐标变换、SVPWM、PI 等计算模块，先验证数学方向、单位与输出尺度。
2. 接入电角度获取/对齐、使能状态机和电流保护；AB 编码器累计角度不能直接当成已对齐电角度。
3. 打通电流闭环，确认使用本轮有效采样，并核对不同占空比下的实际采样窗口；先小电流验证方向和保护。
4. 电流环稳定后再接速度环、位置环，以及 VOFA 控制命令。

继续开发时不要把当前配置中保留的旧保护参数视为已经运行的保护。先建立明确的闭环使能、故障关闭和恢复流程，再允许电机自动输出。
