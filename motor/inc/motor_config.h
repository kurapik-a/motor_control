/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H /* 头文件保护，防止重复包含；无需修改。 */

/* 电机总配置：常用参数统一在此修改，重新编译、下载后生效。
 * PI 初值直接填写 VOFA 整定得到的 Kp、Ki 数值，无需再乘 ADC 比例或控制周期。
 * 每次控制计算：积分 += Ki * 误差，输出 = Kp * 误差 + 积分（随后限幅）。
 * 电流环误差单位 mA、输出单位 PWM 计数；Ki为20kHz每拍积分增益。
 * 当前仅接入电流闭环，不保留未生效的速度/位置环和开环测试参数。 */

/* ===== 1. 电流环PI及各轴输出限幅 ===== */
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
#define MOTOR_CURRENT_D_VOLTAGE_LIMIT_TICKS 4000.0f /* d轴PI电压输出绝对值上限，PWM计数。 */
#define MOTOR_CURRENT_Q_VOLTAGE_LIMIT_TICKS 7000.0f /* q轴PI电压输出绝对值上限，PWM计数。 */

/* ===== 2. 电机、QEI编码器与机械换算 ===== */
#define MOTOR_MAX_SPEED_RPS 20.0f                    /* 电机轴实际速度保护上限，r/s；下方保护门限引用。 */
#define MOTOR_POLE_PAIRS 14u                         /* 14 极对=28 极 */
/* 编码器方向同时作用于累计位置、电角度和速度，-1 沿用原调通工程。 */
#ifndef MOTOR_ENCODER_DIRECTION
#define MOTOR_ENCODER_DIRECTION (-1) /* -1=反向，1=硬件计数方向。 */
#endif
#define MOTOR_ABZ_COUNTS_PER_REV 4000u               /* ABZ 编码器轴一圈的实际计数，已含四倍频；不是输出轴一圈计数。 */
#define MOTOR_SPEED_REDUCTION_RATIO 7.75f            /* 电机轴/输出轴转速比 */

/* ===== 3. 电流闭环调度、给定及滤波 ===== */
#define MOTOR_CURRENT_CTRL_DEFAULT_IQ_MA 0.0f /* 对齐后默认零转矩；SET,CURRENT_MA设置Iq。 */
#define MOTOR_CURRENT_CTRL_MAX_COMMAND_MA 1000.0f /* 初次闭环调试的正负Iq命令限幅。 */
#define MOTOR_CURRENT_CTRL_SLEW_MA_S 1600.0f /* Id退出对齐及Iq目标缓升速率，mA/s。 */
#define MOTOR_PWM_FREQUENCY_HZ 20000u   /* PWM及电流环频率，20000Hz对应每拍50us。 */
#define MOTOR_SPEED_ESTIMATE_HZ 1000u /* 主循环标称测速频率，Hz；速度按实际经过时间计算。 */
#define MOTOR_CURRENT_FILTER_ALPHA 0.2f /* 电流一阶低通系数，范围(0,1]，1=不滤波。 */
#define MOTOR_SPEED_FILTER_ALPHA 0.1f /* 标称测速周期的低通系数，范围(0,1]。 */

/* ===== 4. 对齐专用：进入RUNNING后不再执行 ===== */
#define MOTOR_ALIGN_START_ELECTRICAL_DEG 90.0f /* 对齐初始电角度，单位电角度。 */
#define MOTOR_CURRENT_ALIGN_VOLTAGE_LIMIT_TICKS 500.0f /* 对齐期独立总限幅，单位为原SVPWM电压计数。 */
#define MOTOR_CURRENT_ALIGN_CURRENT_MA 1000.0f /* 上电对齐d轴电流，mA。 */
#define MOTOR_CURRENT_ALIGN_RELEASE_TIMEOUT_MS 2000u /* 合格后降流并等待实际电流回零的最长时间。 */
#define MOTOR_CURRENT_ALIGN_RAMP_MS 500u
#define MOTOR_CURRENT_ALIGN_SWEEP_MS 500u /* 对齐角由90度缓移到0度。 */
#define MOTOR_CURRENT_ALIGN_STABLE_MS 200u /* 电流合格比例与位置静止判定的窗口时长。 */
#define MOTOR_CURRENT_ALIGN_WINDOW_GOOD_PERCENT 99u /* 稳定窗口内电流合格比例；过流保护仍逐拍执行。 */
#define MOTOR_CURRENT_ALIGN_FINAL_GOOD_MS 10u /* 完成对齐前须连续电流合格的末段时长。 */
#define MOTOR_CURRENT_ALIGN_POSITION_TOLERANCE_COUNTS 2u /* 稳定窗口允许的编码器计数摆动，含量化边界±1计数。 */
#define MOTOR_CURRENT_ALIGN_TIMEOUT_MS 3000u
#define MOTOR_CURRENT_ALIGN_ERROR_MA 200.0f /* Id误差及|Iq|门限；不是相电流过流门限。 */
#define MOTOR_CURRENT_ALIGN_MAX_SPEED_COUNTS_S (0.02f * MOTOR_ABZ_COUNTS_PER_REV) /* 稳定窗口平均速度上限，不用单拍差分判静止。 */

/* ===== 5. ADC电流比例、零偏与PWM时序 ===== */
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
#define MOTOR_ADC_AUX_PERIOD_PRESCALE 22u      /* 母线/温度独立周期采样，沿用原工程：周期时基预分频2^22，不是Hz。 */
#define MOTOR_ADC_AUX_PERIOD_COUNT 5u          /* 母线/温度周期计数5；200MHz时基下约104.86ms一次，不随20kHz电流采样更新。 */
/* ===== 6. 运行保护：每个新ADC帧检查，故障关桥锁存 ===== */
#define MOTOR_CURRENT_CTRL_BUDGET_US 25u /* 新帧控制计算到PWM提交的最大预算。 */
#define MOTOR_CURRENT_CTRL_WATCHDOG_US 100u /* PWM独立中断检查ADC控制心跳。 */
#define MOTOR_BUS_OVERVOLTAGE_ADC 34044u       /* ADC 原始码，非直接电压 */
#ifndef MOTOR_BUS_UNDERVOLTAGE_ADC
#define MOTOR_BUS_UNDERVOLTAGE_ADC 0u          /* 默认0：无有效欠压门限；仍检查辅助数据有效性。 */
#endif
#define MOTOR_TEMP_MAX_ADC 55893u              /* 温度 ADC 原始码上限；采样码大于此值置故障，不是直接摄氏温度。 */
#ifndef MOTOR_TEMP_MIN_ADC
#define MOTOR_TEMP_MIN_ADC 0u                  /* 温度 ADC 原始码下限；默认0：无有效下限。 */
#endif

/* pmsm_protect.c已接入电流闭环，ADC驱动仍保留自身错误检查。 */
#ifndef MOTOR_PROTECT_PHASE_ENABLE
#define MOTOR_PROTECT_PHASE_ENABLE 1
#endif
#ifndef MOTOR_PROTECT_DQ_ENABLE
#define MOTOR_PROTECT_DQ_ENABLE 1 /* 正式闭环已提供Id/Iq有效反馈。 */
#endif
#ifndef MOTOR_PROTECT_ENCODER_ENABLE
#define MOTOR_PROTECT_ENCODER_ENABLE 1 /* 有效性/新鲜度及正负实际超速；位置为0不是断线。 */
#endif
#ifndef MOTOR_PROTECT_BUS_ENABLE
#define MOTOR_PROTECT_BUS_ENABLE 1 /* 阈值使用上方MOTOR_BUS_*，输入必须有独立valid和年龄。 */
#endif
#ifndef MOTOR_PROTECT_TEMP_ENABLE
#define MOTOR_PROTECT_TEMP_ENABLE 1 /* 阈值使用上方MOTOR_TEMP_*，仍为ADC码，非摄氏度。 */
#endif
#define MOTOR_PROTECT_PHASE_LIMIT_MA 3000.0f /* 原始U/V/W绝对值过流门限，mA。 */
#define MOTOR_PROTECT_DQ_LIMIT_MA 3000.0f /* Id/Iq绝对值门限。 */
#define MOTOR_PROTECT_SPEED_LIMIT_COUNTS_S (MOTOR_MAX_SPEED_RPS * MOTOR_ABZ_COUNTS_PER_REV) /* 电机轴20r/s，当前AB每圈4000计数。 */
#define MOTOR_PROTECT_CURRENT_MAX_AGE_US 100u /* 电流数据最大年龄，微秒。 */
#define MOTOR_PROTECT_ENCODER_MAX_AGE_MS 5u /* 编码器数据最大年龄，毫秒。 */
#define MOTOR_PROTECT_AUX_MAX_AGE_MS 500u /* 母线/温度最大年龄，毫秒；应大于独立采样周期。 */
#if (MOTOR_PROTECT_PHASE_ENABLE != 0 && MOTOR_PROTECT_PHASE_ENABLE != 1) || \
    (MOTOR_PROTECT_DQ_ENABLE != 0 && MOTOR_PROTECT_DQ_ENABLE != 1) || \
    (MOTOR_PROTECT_ENCODER_ENABLE != 0 && MOTOR_PROTECT_ENCODER_ENABLE != 1) || \
    (MOTOR_PROTECT_BUS_ENABLE != 0 && MOTOR_PROTECT_BUS_ENABLE != 1) || \
    (MOTOR_PROTECT_TEMP_ENABLE != 0 && MOTOR_PROTECT_TEMP_ENABLE != 1)
#error "Protection enable switches must be 0 or 1"
#endif
#if MOTOR_CURRENT_ALIGN_RAMP_MS == 0 || MOTOR_CURRENT_ALIGN_SWEEP_MS == 0 || MOTOR_CURRENT_ALIGN_STABLE_MS == 0 || MOTOR_CURRENT_ALIGN_TIMEOUT_MS == 0
#error "Current alignment times must be positive"
#endif
#if MOTOR_POLE_PAIRS > (0xffffffffu / MOTOR_ABZ_COUNTS_PER_REV)
#error "Electrical phase product must fit uint32_t"
#endif
#if MOTOR_CURRENT_ALIGN_POSITION_TOLERANCE_COUNTS >= MOTOR_ABZ_COUNTS_PER_REV / 2u
#error "Alignment position tolerance must be less than half a revolution"
#endif
#if MOTOR_CURRENT_ALIGN_WINDOW_GOOD_PERCENT < 1u || MOTOR_CURRENT_ALIGN_WINDOW_GOOD_PERCENT > 100u || \
    MOTOR_CURRENT_ALIGN_FINAL_GOOD_MS == 0u || MOTOR_CURRENT_ALIGN_FINAL_GOOD_MS > MOTOR_CURRENT_ALIGN_STABLE_MS
#error "Invalid alignment current qualification window"
#endif

/* ===== 7. VOFA参数：固定8通道，改波特率后上位机也需同步 ===== */
#ifndef MOTOR_VOFA_BAUDRATE
#define MOTOR_VOFA_BAUDRATE 115200u /* VOFA 串口波特率，单位 bit/s；上位机必须选择相同数值。 */
#endif
#ifndef MOTOR_VOFA_UPLOAD_HZ
#define MOTOR_VOFA_UPLOAD_HZ 100u /* VOFA 上传频率，单位 Hz；100 Hz 即每 10 ms 一帧，不是控制环频率。 */
#endif

/* 基本约束校验；当前前台测速独立于 PWM，不要求两者频率整除。 */
#if MOTOR_PWM_FREQUENCY_HZ == 0 || MOTOR_SPEED_ESTIMATE_HZ == 0
#error "Motor control frequencies must be positive"

#endif
#if MOTOR_CURRENT_OFFSET_SAMPLES == 0 || MOTOR_CURRENT_OFFSET_SAMPLES > 1000000u
#error "Invalid ADC offset averaging count"
#endif
#if MOTOR_ABZ_COUNTS_PER_REV == 0
#error "Encoder counts per revolution must be positive"
#endif
/* 帧带宽由 motor_vofa.c 按实际通道数校验，不沿用旧的 16 通道帧长。 */
#if MOTOR_VOFA_UPLOAD_HZ == 0 || MOTOR_VOFA_BAUDRATE == 0
#error "VOFA frequency and baudrate must be positive"
#endif

#endif /* MOTOR_CONFIG_H：总配置头文件保护结束，必须保留。 */
