# PWM 同步 ADC 采样

2026-11-03：在新工程 motor_control 中接入。未下载固件、未使能桥臂、未执行实机时序验证。

## 当前链路

```text
pwm_init：PWM0计数器运行，PE00~PE05仍强制低电平
 → adc_init：准备ADC0/1、DMA、完成中断和零偏标定
 → adc.c重臂：清DMA字有效位和完成标志，接通TRGM路由
 → PWM0比较器16，在周期计数reload-1处产生TRGO0
 → TRGM0 ADCX_PTRGI0A同时触发ADC0/ADC1的TRIG0A
 → ADC0队列：[U、母线、温度]；ADC1队列：[V]
 → 两路分别完成并进入ADC中断，软件记录本轮完成位
 → 两路都完成后关闭触发路由，检查截止时间及四个DMA字的标识
 → 发布一致的软件快照、电流换算和零偏状态
 → 清本轮DMA字，重新接通下一次PWM触发
```

正常情况下每个20kHz PWM周期得到一组采样。ADC源于同一个硬件事件，发布必须等待两路本轮数据齐全；同一触发不代表两个模拟通道的孔径误差为零。
若临近下一采样边沿，驱动延后重臂并增加rearm_deferred_count；不伪造漏掉的帧，实际采样率需上板确认。

## 职责和文件

- app/src/pwm.c：保留原三相互补PWM，新增pwm_set_adc_trigger_enabled控制共享触发路由，pwm_get_adc_trigger_timing提供下一边沿的估计CPU时刻及PWM周期。
- app/src/adc.c：完全取消软件触发，使用ADC0/ADC1完成中断。两路中断同为优先级5；PWM电流环回调保留给以后FOC，当前没有绑定它。
- adc_poll：前台健康检查和必要的重试，不再按主循环频率发起采样。
- adc_read：短临界区复制已发布快照，不读取正在更新的DMA；主循环仍通过adc_sample观察数据。
- adc_start_offset_calibration：拒绝在桥臂使能时标定；接受请求后旧零偏立即失效，旧的在途帧不会计入新的稳定时间窗口。
- motor/inc/motor_config.h：MOTOR_ADC_SAMPLE_HZ映射PWM频率，转换/DMA窗口默认10us，必须小于PWM周期；重臂前边沿保护距离为1000个PWM时钟计数。

## 完整性及故障处理

DMA放在非缓存区；发布前逐字检查cycle_bit、trig_ch、seq_num、adc_ch。cycle_bit在每次重臂前清零，不作为逐帧翻转计数使用。
两路完成时先关闭路由，避免复制期间下一个PWM边沿覆盖DMA。若完成标志已经到但DMA字尚未落地，保持路由关闭，随后重试；不忙等，不发布半组。
从预计采样边沿开始超过10us仍无法取得本帧，就锁存-7并关闭路由和桥臂，拒绝迟到帧。该窗口小于一个PWM周期，避免CPU处理延迟跨周期后把混合数据发布为本帧。
ADC冲突/溢出锁存-8，DMA标识错误锁存-6，标定饱和锁存-5。故障后需在桥臂关闭时重新adc_init或重新上电；不会自动重启功率输出。
未重臂状态超过两个PWM周期仍无进展，前台健康检查也会锁存-7，避免一直返回旧的有效采样。
capture_cycles是根据PWM计数器估计的采样时刻，不是硬件捕获时间戳；publish_cycles是软件完成发布的CPU时刻。

## 上板观察

1. pwm_result=0；pwm_get_reload在当前200MHz PWM时钟、20kHz频率下为10000。
2. adc_result开始可能为1，第一组数据后应为0；持续错误时查看adc_sample.fault_code。
3. adc_sample.sample_count和irq_count持续增长；正常sample_count约每秒增长20000，实际以测量为准。
4. 上电约1秒稳定等待，加256组间隔至少1ms的零偏样本后，offset_state=ADC_OFFSET_READY、offset_samples=256、current_valid=true。
5. raw_u/raw_v和offset_u/offset_v为完整16位码（0~65535）；桥臂关闭且无相电流时，换算电流应接近0。
6. timeout_count正常为0，fault_code为0。dma_wait_count表示完成中断后等待DMA落地的次数；rearm_deferred_count表示靠近边沿而延后重臂的次数，增长本身不一定是故障。

暂停CPU、断点和单步可能扰动采样时序，恢复后出现超时需重新初始化；判断采样率应在连续运行时进行。
本阶段没有接入FOC、对齐、速度/位置环，也未恢复ENABLE开环测试入口。桥臂默认仍关闭。

## 验证

主机回归使用实际adc.c、模拟硬件状态，覆盖双路先后完成、DMA迟到、标识错误、超时、硬件冲突、完整16位电流换算、零偏等待与256样本平均、在途帧重标定、桥臂使能拒绝标定、边沿延期和初始化失败。脚本为tests/run_adc_pwm_sync_tests.ps1。
固件构建和主机回归只能确认代码路径与接口，不能替代实机触发时序、DMA完成时限及模拟电流测量验证。

本次执行结果：同步ADC主机回归、保留的PWM测试状态机回归、GCC固件构建、SES Debug构建均通过。保留用户现有的开环测试3000mA参数，未恢复测试入口。
