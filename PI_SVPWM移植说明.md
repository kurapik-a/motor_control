# PI、SVPWM 与电压开环

> 最新状态：已将 `pmsm_control.c/.h` 重命名为 `pmsm_foc.c/.h`，增加独立Clarke/Park/反Park，默认ENABLE启动正式电流对齐及闭环。当前用法见 [电流环移植说明.md](电流环移植说明.md)。以下保留为电压开环阶段记录，只有设置 `MOTOR_CONTROL_USE_CURRENT_LOOP=0` 才使用开环入口。

更新：2026-11-03。本阶段移植原工程的 PI、反 Park、六扇区 SVPWM，恢复调试器及 VOFA ENABLE 启动的电压开环。尚未闭合电流环，也没有执行编码器电角度对齐。

## 实际调用和数据流

```text
main 初始化 board → PWM → encoder → VOFA → ADC
while (1)
  adc_poll → adc_read → adc_sample（观察同步电流和状态）
  pwm_test_poll
    消费 pwm_test_request（启动/停止）
    检查ADC有效性、数据年龄、U/V/重建W绝对电流
    约每1ms：缓升电压/频率 → 积分生成电角度
      pmsm_voltage_to_pwm(Ud=0, Uq, 角度)
        反Park → pmsm_svpwm → pwm_set_counts
  encoder_read → motor_vofa_set_encoder → motor_vofa_poll

PWM硬件：20kHz触发ADC → DMA → ADC中断发布电流快照
PWM比较中断：test_tick，20kHz检查测试时长和前台采样心跳
VOFA：100Hz上传输出轴角度、原始rpm、滤波rpm
```

开环使用给定角度，不使用编码器角度。ADC电流用于启动门禁和软件过流检查，不参与电压调节。电压矢量更新仍在前台约1kHz执行，不能把本模块直接当成20kHz电流环。

## 文件和接口

- `motor/inc/pmsm_math.h`：沿用用户提供的数学类型头，本阶段仅支持默认浮点模式，文件未修改。
- `motor/inc/pid.h`、`motor/src/pid.c`：独立PI模块，不依赖SVPWM或外设。使用时包含 `pid.h`，接口名保持不变。
  - `pmsm_pi_reset`：清零积分和输出，保留增益、限幅。
  - `pmsm_pi_step`：误差=目标-反馈；候选积分=旧积分+Ki×误差；输出=Kp×误差+候选积分；输出饱和时冻结积分，与原 PI 一致。Ki 已包含每拍周期，不再乘 Ts。无 D 项。
- `motor/inc/pmsm_control.h`、`motor/src/pmsm_control.c`：反Park、SVPWM纯算法，不访问外设。
  - `pmsm_svpwm`：保留原六扇区分配、整数截断、14189/16384系数以及过调制等比例压缩。乘法使用64位中间量，增加输入检查；失败不改写输出。
  - `pmsm_voltage_to_pwm`：电角度归一化后计算 sin/cos，反 Park 得到 alpha/beta，再执行 SVPWM。本阶段用 sinf/cosf，未搬原正弦查表。反 Park 在此函数内部，没有单独函数名。Clarke 和正 Park 尚未移植：开环直接给电压，不计算相电流反馈的 Id/Iq。
- `app/src/pwm_test.c`：复用原测试的启停、缓升、过流现场和心跳超时，把原三相直接正弦占空比改成反 Park + SVPWM。
- `app/src/pwm.c`：PWM写入、使能、关闭、回调管理位于ILM；相关SDK函数强制内联，避免关中断时调用Flash。测试中断和停止路径也放入ILM。
- `app/src/main.c`：在 ADC 快照读取后调用 `pwm_test_poll()`；默认不调用启动函数。
- `app/CMakeLists.txt`：默认编译新纯算法和开环测试；旧 `pmsm_init.c`/core控制扩展仍未启用。

## 单位

PI 接口不自行换算单位。后续电流环可用配置头的 d/q Kp、Ki 和电压限幅初始化两个 `pmsm_pi_t`：输入误差为 mA，输出为原工程的电压 PWM 计数尺度。

反 Park、SVPWM 的 Ud/Uq、alpha/beta 使用原工程电压计数尺度，非伏特；最终 u/v/w 是0～reload脉宽计数。开环 `amplitude` 沿用等效相正弦占空比峰值，内部 Uq=amplitude×reload×sqrt(3)。SVPWM增加共模偏移，所以三相占空比和不恒为1.5；线电压关系保持对应正弦关系。

## 如何上板启动

1. 编译、下载，保持运行。上电默认关桥。
2. 等待 `adc_result==0`、`adc_sample.current_valid==true`、`offset_state==ADC_OFFSET_READY`，通常约1.26秒后。
3. VOFA 发送 `ENABLE,1` 并附加实际 CR 或 LF 字符，启动默认正向；发送 `ENABLE,0` 或 `STOP` 并附加 CR/LF 停止。上位机发送框的 `\r` 表示回车转义，不要把反斜杠和字母 r 当成普通文本发送。调试器仍可写入 `pwm_test_request=1` 请求正向、2反向、3停止，请求由主循环消费后清零。标定未完成时启动会被拒绝，需要重新请求，不会排队等待标定完成。
4. 观察 `pwm_test_monitor.result`、`sector`、`phase_deg`、三相 `duty_*`、`peak_current_ma` 和 `trip`。VOFA仍只上传三个编码器输出轴通道。

默认参数：电角频率1Hz、幅值0.01、先500ms缓升电压再500ms缓升频率、总时长5秒、相电流软件门限3000mA、10ms无新采样心跳则停机。参数集中在 `motor_config.h` 的 PWM 开环验证分组。

结果码：0空闲，1运行，2限时结束，3手动停止；-1参数错误，-2 ADC未就绪，-3过流/非有限电流，-4驱动错误/输出被外部关闭，-5采样或心跳过旧，-6运行中重复启动。

VOFA 命令回调在 `motor_vofa.c` 的 `motor_vofa_command()`：ENABLE,1直接调用默认参数的 `pwm_test_start()`，ENABLE,0/STOP调用 `pwm_test_stop()` 并取消未消费的调试器请求。运行中重复 ENABLE,1 返回 BUSY，不重启计时；其他速度、位置、电流、PID命令尚未接入。没有文本回执混入JustFloat。语法及执行结果看 `motor_vofa_context.stats.last_command_result` 和接受/拒绝计数，应用回调结果也保存在 `motor_vofa_command_result`。

运行期间不使用断点暂停CPU：软件保护也会停止执行，而PWM可能继续保持输出。

## 验证及下一阶段

主机回归：`tests/run_pmsm_control_tests.ps1` 验证 PI 积分、饱和恢复、六扇区线电压、角度回绕和过调制；`tests/run_pwm_test.ps1` 验证启动、方向、限时、采样心跳、过流和停止；`tests/run_vofa_open_loop_tests.ps1` 使用真实SDK协议解析器和开环状态机验证ENABLE/STOP、启动拒绝不排队、BUSY、保护停机和JustFloat帧；另保留ADC/PWM回归。`tests/check_adc_realtime_placement.py` 扩展检查开环关键路径的ILM位置及调用。

本阶段只构建和软件验证，没有下载、使能实际电机，实际转动和ADC/PWM并行时序需要上板验证。下一阶段再接编码器磁极对齐、Clarke/Park、d/q PI、确定新鲜采样后的20kHz控制入口及PWM更新时刻。
