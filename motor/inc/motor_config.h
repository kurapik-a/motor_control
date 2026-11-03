/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H /* 头文件保护，防止重复包含；无需修改。 */

/* 电机总配置：常用参数统一在此修改，重新编译、下载后生效。
 * PI 初值直接填写 VOFA 整定得到的 Kp、Ki 数值，无需再乘 ADC 比例或控制周期。
 * 每次控制计算：积分 += Ki * 误差，输出 = Kp * 误差 + 积分（随后限幅）。
 * 电流环误差单位 mA、输出单位 PWM 计数；速度环误差单位编码器计数/秒、输出单位 mA。
 * 位置环误差单位编码器计数、输出单位编码器计数/外环周期。
 * SET 只修改 RAM，重启仍加载这里。控制器为 PI，KD 不参与运算。 */

/* ===== 2. 三环 PI：正常/调试共用，不再由调试分支覆盖 ===== */
#ifndef MOTOR_CURRENT_D_KP
#define MOTOR_CURRENT_D_KP 0.00248182f /* d 轴电流环比例增益：电流误差每增加 1 mA，比例输出增加的 PWM 计数。 */
#endif
#ifndef MOTOR_CURRENT_D_KI
#define MOTOR_CURRENT_D_KI 0.000248182f /* d 轴电流环每拍积分增益；按电流环周期累加，即使 Id 目标为 0 也参与控制。 */
#endif
#ifndef MOTOR_CURRENT_Q_KP
#define MOTOR_CURRENT_Q_KP 0.00248182f /* q 轴电流环比例增益，单位 PWM 计数/mA；影响转矩电流响应。 */
#endif
#ifndef MOTOR_CURRENT_Q_KI
#define MOTOR_CURRENT_Q_KI 0.000248182f /* q 轴电流环每拍积分增益；与电流环频率配套，无需再次乘周期。 */
#endif
#ifndef MOTOR_SPEED_KP
#define MOTOR_SPEED_KP 0.8058608f /* 速度环比例增益，单位 mA/(编码器计数/秒)；内部速度误差不是 rpm。 */
#endif
#ifndef MOTOR_SPEED_KI
#define MOTOR_SPEED_KI 0.008058608f /* 速度环每拍积分增益；按外环周期累加，积分及比例输出共同生成 Iq 目标。 */
#endif
#ifndef MOTOR_POSITION_KP
#define MOTOR_POSITION_KP 0.0205f /* 位置环比例增益：编码器计数误差转换为每个外环周期的速度指令。 */
#endif
#ifndef MOTOR_POSITION_KI
#define MOTOR_POSITION_KI 0.0f /* 位置环每拍积分增益；0=关闭位置积分，位置比例控制仍有效。 */
#endif

/* ===== 3. 电机、编码器与机械换算：保留当前用户值 ===== */
#define MOTOR_RESISTANCE_OHM 0.225f                  /* 电机相电阻，单位 Ω；填写相值。 */
#define MOTOR_INDUCTANCE_H 0.00026f                  /* 相电感，H，暂按 Ld=Lq */
#define MOTOR_NOMINAL_BUS_V 24.0f                    /* 标称直流母线电压，单位 V；不是过压保护门限。 */
#define MOTOR_RATED_PHASE_CURRENT_A 4.6f             /* 电机额定相电流参数，单位 A；不会自动替代下方命令/保护限值。 */
#define MOTOR_MAX_SPEED_RPS 20.0f                    /* 电机轴 r/s，电机参数而非自动保护 */
#define MOTOR_POLE_PAIRS 14u                         /* 14 极对=28 极 */
/* 编码器方向同时作用于累计位置、电角度和速度，-1 沿用原调通工程。 */
#ifndef MOTOR_ENCODER_DIRECTION
#define MOTOR_ENCODER_DIRECTION (-1) /* -1=反向，1=硬件计数方向。 */
#endif
#define MOTOR_ENCODER_USE_SPI 0 /* 当前仅移植 QEI A/B，保持 0；SPI 分支尚未接入。 */
#define MOTOR_ABZ_COUNTS_PER_REV 4000u               /* ABZ 编码器轴一圈的实际计数，已含四倍频；不是输出轴一圈计数。 */
#define MOTOR_SPI_COUNTS_PER_REV 16384u              /* SPI 编码器轴一圈计数，14 位对应 16384；仅选择 SPI 时使用。 */
#define MOTOR_SPI_ELECTRICAL_OFFSET_DEG 200.6f       /* SPI 编码器电角度零位补偿，单位电角度 °；仅选择 SPI 时使用。 */
#define MOTOR_SPEED_REDUCTION_RATIO 7.75f            /* 电机轴/输出轴转速比 */
#define MOTOR_POSITION_COMMAND_COUNTS_PER_REV 10000u /* 原位置命令接口每编码器轴一圈的内部刻度，不是 ABZ 硬件脉冲数。 */

/* ===== 4. 周期、滤波：改频率后需重新核对 Ki 与采样时序 ===== */
#define MOTOR_PWM_FREQUENCY_HZ 20000u   /* PWM 及电流环频率，单位 Hz；20000 Hz 对应每拍 50 μs。 */
#define MOTOR_SPEED_LOOP_HZ 1000u       /* 速度/位置外环频率，单位 Hz；1000 Hz 对应每拍 1 ms。 */
#define MOTOR_SPEED_ESTIMATE_HZ 1000u /* 主循环标称测速频率，Hz；速度按实际经过时间计算。 */
#define MOTOR_CURRENT_FILTER_ALPHA 0.2f /* 电流一阶低通系数，范围 (0,1]；越小越平滑、延迟越大，1=不滤波。 */
#define MOTOR_SPEED_FILTER_ALPHA 0.1f /* 标称测速周期下的低通系数，范围 (0,1]；1=不滤波，延迟读取时按实际时间修正。 */
#ifndef MOTOR_NORMAL_FILTERED_FEEDBACK
#define MOTOR_NORMAL_FILTERED_FEEDBACK 0 /* 正常：0 原始测速，1 滤波测速 */
#endif

/* ===== 5. 正常模式限幅：保留原行为，单位请勿混淆 ===== */
#define MOTOR_NORMAL_SPEED_COMMAND_MAX_RPM 500.0f                                  /* 输出轴 rpm，仅命令范围 */
#define MOTOR_NORMAL_CURRENT_COMMAND_MAX_MA 5000.0f                                /* 正常电流模式命令绝对值上限，单位 mA，即允许正负 5 A。 */
#define MOTOR_NORMAL_SPEED_IQ_LIMIT_MA 30000.0f                                    /* 正常速度环 Iq 输出绝对值上限，单位 mA；位置串级也受此限幅。 */
#define MOTOR_NORMAL_POSITION_OUTPUT_MAX 10.0f                                     /* 编码器计数/外环周期 */
#define MOTOR_NORMAL_D_VOLTAGE_LIMIT_TICKS 4000.0f                                 /* 电压输出的 PWM 计数，不是 mA */
#define MOTOR_NORMAL_Q_VOLTAGE_LIMIT_TICKS 7000.0f                                 /* 正常 q 轴 PI 电压输出绝对值上限，单位 PWM 计数。 */
#define MOTOR_NORMAL_ALIGN_MA 8000u                                                /* 正常模式 ABZ 上电电角度对齐的 d 轴电流目标，单位 mA。 */
#define MOTOR_NORMAL_CURRENT_TRIP_MA 30000u                                        /* 原正常保护只检查正 Iq */
#define MOTOR_NORMAL_SPEED_ERROR_LIMIT_COUNTS_S (10.0f * MOTOR_SPI_COUNTS_PER_REV) /* 正常速度误差保护门限，单位计数/秒；保留原来按 SPI 刻度计算的值。 */

/* ===== 7. ADC 电流比例、零偏与 PWM 时序 ===== */
#define MOTOR_ADC_SAMPLE_HZ MOTOR_PWM_FREQUENCY_HZ /* PWM硬件触发频率；不再由主循环调度采样。 */
#define MOTOR_ADC_CONVERSION_TIMEOUT_US 10u /* 预计PWM触发到双路DMA完整的允许时间，微秒；必须小于PWM周期。 */
#define MOTOR_ADC_TRIGGER_GUARD_TICKS 1000u /* 距ADC比较点过近时延后重臂，单位PWM时钟计数。 */
#define MOTOR_ADC_REFERENCE_V 3.3f             /* ADC 参考电压，单位 V；参与电流码值到 mA 的换算。 */
#define MOTOR_ADC_EFFECTIVE_FULL_SCALE 65535.0f /* 当前保留完整16位ADC码；换算比例与65535配套。 */
#define MOTOR_CURRENT_AMPLIFIER_GAIN 10.0f     /* 电流采样放大倍数，无量纲；需与板上采样电路一致。 */
#define MOTOR_SHUNT_RESISTANCE_OHM 0.001f      /* 电流采样电阻，单位 Ω；0.001 Ω=1 mΩ。 */
#define MOTOR_ADC_CURRENT_SAMPLE_CYCLES 15u    /* 电流 ADC 的采样时间设置，单位 ADC 时钟周期。 */
#define MOTOR_ADC_AUX_SAMPLE_CYCLES 20u        /* 母线/温度辅助 ADC 的采样时间设置，单位 ADC 时钟周期。 */
#define MOTOR_ADC_CLOCK_DIVIDER 4u             /* 实际分频值，需核对 ADC 时钟范围 */
#define MOTOR_CURRENT_OFFSET_SETTLE_MS 1000u   /* 上电关桥后，开始电流零偏标定前的等待时间，单位 ms。 */
#define MOTOR_CURRENT_OFFSET_SAMPLES 256u      /* 每路电流零偏求平均的样本数；每次上电重新标定。 */
#define MOTOR_CURRENT_OFFSET_INTERVAL_MS 1u    /* 零偏标定相邻样本的等待时间，单位 ms。 */
#define MOTOR_PWM_DEADTIME_HALF_TICKS 200u     /* 驱动的半周期计数单位 */
#define MOTOR_PWM_CURRENT_ISR_DELAY_TICKS 200u /* 不是 ADC/DMA 完成保证 */
#define MOTOR_ADC_AUX_PERIOD_PRESCALE 22u      /* 辅助 ADC 周期采样硬件预分频字段；按芯片寄存器定义解释，不是 Hz。 */
#define MOTOR_ADC_AUX_PERIOD_COUNT 5u          /* 辅助 ADC 周期采样硬件周期计数字段；配合预分频及 ADC 时钟决定周期。 */

/* ===== PWM 开环验证：上电不自动运行，参数均为试验上限 ===== */
#define MOTOR_PWM_TEST_ELECTRICAL_HZ 1.0f      /* 默认电角频率 Hz，正负改变相序；不是输出轴 rpm。 */
#define MOTOR_PWM_TEST_AMPLITUDE 0.01f        /* 占空比偏离 0.5 的峰值：0.01 为 49%~51%。 */
#define MOTOR_PWM_TEST_DURATION_MS 5000u      /* 默认测试时长，结束自动关桥。 */
#define MOTOR_PWM_TEST_RAMP_MS 500u           /* 幅值及之后电角频率的缓升时间，ms，必须大于0。 */
#define MOTOR_PWM_TEST_CURRENT_LIMIT_MA 3000.0f /* 独立采样阶段 U/V/W 绝对电流软件门限，mA。 */
#define MOTOR_PWM_TEST_ADC_TIMEOUT_MS 10u     /* 采样过旧或主循环未喂新采样心跳时，关断输出。 */
#define MOTOR_PWM_TEST_MAX_ELECTRICAL_HZ 5.0f /* 测试接口允许的最大电角频率绝对值，Hz。 */
#define MOTOR_PWM_TEST_MAX_AMPLITUDE 0.02f    /* 测试接口允许的最大调制幅值，0.02=48%~52%。 */
#define MOTOR_PWM_TEST_MAX_DURATION_MS 10000u /* 单次测试允许的最长时间，ms。 */
#if MOTOR_PWM_TEST_RAMP_MS == 0 || MOTOR_PWM_TEST_ADC_TIMEOUT_MS == 0
#error "PWM test ramp and ADC timeout must be positive"
#endif

/* ===== 8. 对齐过程和保护检查间隔 ===== */
#define MOTOR_ALIGN_START_ELECTRICAL_DEG 90.0f /* ABZ 对齐初始电角度，单位电角度 °，不是机械位置命令角度。 */
#define MOTOR_ALIGN_STEP_DEG_PER_TICK 0.1f     /* 对齐时每个外环周期向零角度移动的电角度步长，单位 °。 */
#define MOTOR_ALIGN_RISE_STEP_MA 1600.0f       /* 对齐电流上升阶段每个外环周期增加的 mA，最终钳位到模式对应的对齐目标。 */
#define MOTOR_ALIGN_FALL_STEP_MA 800.0f        /* 对齐结束时每个外环周期减少的 mA，最终降至 0。 */
#define MOTOR_ALIGN_RISE_TICKS 5u              /* 对齐电流上升阶段最多占用的外环周期数；当前 1000 Hz 时为 5 ms。 */
#define MOTOR_PROTECT_CURRENT_TICKS 1u         /* 外环电流保护每隔多少次调用检查一次，必须大于 0；调试快速保护仍逐拍检查。 */
#define MOTOR_PROTECT_SPEED_TICKS 2u           /* 正常速度误差保护的调用间隔，单位外环周期，必须大于 0；调试实际速度保护不使用此间隔。 */
#define MOTOR_PROTECT_ENCODER_TICKS 1u         /* 正常编码器保护的调用间隔，单位外环周期，必须大于 0；调试不使用原位置为零判据。 */
#define MOTOR_PROTECT_VOLTAGE_TICKS 2u         /* 母线过压/欠压保护检查间隔，单位外环周期，必须大于 0。 */
#define MOTOR_PROTECT_TEMP_TICKS 5u            /* 温度 ADC 门限保护检查间隔，单位外环周期，必须大于 0。 */
#define MOTOR_BUS_OVERVOLTAGE_ADC 34044u       /* ADC 原始码，非直接电压 */
#define MOTOR_BUS_UNDERVOLTAGE_ADC 0u          /* 目前无有效欠压门限 */
#define MOTOR_TEMP_MAX_ADC 55893u              /* 温度 ADC 原始码上限；采样码大于此值置故障，不是直接摄氏温度。 */
#define MOTOR_TEMP_MIN_ADC 0u                  /* 温度 ADC 原始码下限；采样码小于此值置故障，当前 0 无有效下限。 */

/* ===== 9. VOFA 参数：通道数见 motor_vofa.h，改波特率后上位机也需同步 ===== */
#ifndef MOTOR_VOFA_BAUDRATE
#define MOTOR_VOFA_BAUDRATE 115200u /* VOFA 串口波特率，单位 bit/s；上位机必须选择相同数值。 */
#endif
#ifndef MOTOR_VOFA_UPLOAD_HZ
#define MOTOR_VOFA_UPLOAD_HZ 100u /* VOFA 上传频率，单位 Hz；100 Hz 即每 10 ms 一帧，不是控制环频率。 */
#endif

/* ===== 10. 原轨迹/示波对象初值：直接 SET 运动时绕过轨迹规划 ===== */
#define MOTOR_TRAJECTORY_ACCEL_REF 100.0f /* 原轨迹对象加速度初值，单位编码器轴转/秒²；直接 SET 运动不使用。 */
#define MOTOR_TRAJECTORY_JERK_REF 1000.0f /* 原轨迹对象加加速度初值，单位编码器轴转/秒³；直接 SET 运动不使用。 */
#define MOTOR_TRAJECTORY_CONSTANT_MS 1u   /* 原轨迹对象匀速段时间初值，单位 ms；不是外环周期。 */
#define MOTOR_POSITION_DWELL_MS 500u      /* 原位置对象停留时间初值，单位 ms；不影响直接 SET 位置目标。 */
#define MOTOR_POSITION_CYCLE_COUNT 1u     /* 原位置对象循环次数初值，单位次；不是直接 SET 命令的执行次数。 */
#define MOTOR_LEGACY_SCOPE_SAMPLES 200u   /* 原示波对象采样点数初值；不改变 VOFA 通道数量。 */
#define MOTOR_LEGACY_SCOPE_GAP 10u        /* 原示波对象采样间隔字段初值；不控制 VOFA 上传，上传频率见 MOTOR_VOFA_UPLOAD_HZ。 */

/* 基本约束校验；当前前台测速独立于 PWM，不要求两者频率整除。 */
#if MOTOR_PWM_FREQUENCY_HZ == 0 || MOTOR_SPEED_LOOP_HZ == 0 || MOTOR_SPEED_ESTIMATE_HZ == 0
#error "Motor control frequencies must be positive"

#endif
#if MOTOR_CURRENT_OFFSET_SAMPLES == 0 || MOTOR_CURRENT_OFFSET_SAMPLES > 1000000u
#error "Invalid ADC offset averaging count"
#endif
#if MOTOR_ENCODER_USE_SPI != 0 && MOTOR_ENCODER_USE_SPI != 1
#error "Encoder selection must be 0 or 1"
#endif
#if MOTOR_ABZ_COUNTS_PER_REV == 0 || MOTOR_SPI_COUNTS_PER_REV == 0
#error "Encoder counts per revolution must be positive"
#endif
/* 帧带宽由 motor_vofa.c 按实际通道数校验，不沿用旧的 16 通道帧长。 */
#if MOTOR_VOFA_UPLOAD_HZ == 0 || MOTOR_VOFA_BAUDRATE == 0
#error "VOFA frequency and baudrate must be positive"
#endif

#endif /* MOTOR_CONFIG_H：总配置头文件保护结束，必须保留。 */
