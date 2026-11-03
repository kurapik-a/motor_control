# PWM 同步 ADC 采样

2026-11-03：在新工程 motor_control 中接入。未下载固件、未使能桥臂、未执行实机时序验证。

## 当前链路

```text
pwm_init：PWM0计数器运行，PE00~PE05仍强制低电平
 → adc_init：准备ADC0/1、DMA、完成中断和零偏标定
 → adc.c重臂：清DMA字有效位和完成标志，接通TRGM路由
 → PWM0比较器16，在周期计数reload-1处产生TRGO0
 → TRGM0 ADCX_PTRGI0A同时触发ADC0/ADC1的TRIG0A
 → ADC0抢占队列：[U]；ADC1抢占队列：[V]
 → 两路分别完成并进入ADC中断，软件记录本轮完成位
 → 两路都完成后关闭触发路由，检查截止时间及U/V两个DMA字的标识
 → 发布一致的软件快照、电流换算和零偏状态
 → 清本轮DMA字，重新接通下一次PWM触发
```

正常情况下每个20kHz PWM周期得到一组采样。ADC源于同一个硬件事件，发布必须等待两路本轮数据齐全；同一触发不代表两个模拟通道的孔径误差为零。
若临近下一采样边沿，驱动延后重臂并增加rearm_deferred_count；不伪造漏掉的帧，实际采样率需上板确认。

## 母线、温度独立周期采样

按原工程配置从PWM抢占队列移出，仍共用ADC0：CH15/PF31母线、CH1/PF26温度，采样周期数20，周期预分频字段22（2^22）、周期计数5；200MHz时基下约104.86ms一次。参数位于motor/inc/motor_config.h的MOTOR_ADC_AUX_*。
ADC0转换器只初始化一次，电流队列具有抢占优先级；辅助通道用adc16_set_prd_config配置硬件周期采样，不另外切换ADC0时钟、重新初始化或软件触发，也不产生额外的完成中断。
前台adc_poll通过adc16_get_prd_result读取最近的周期结果，继续放入adc_sample.bus_raw和temperature_raw。这两项不再与U/V属于同一帧，不受电流sample_count/capture_cycles表示；sample_valid/current_valid仅表示电流有效。
初始化后首个辅助周期到来前，辅助字段可能仍为0；持续读取相同数值不表示每次都发生了新转换。当前未增加辅助数据的新鲜度检测、V/℃换算或母线/温度保护。
重复adc_init先关闭两路旧周期采样，再配置转换器、通道与周期参数，避免带着上次的周期任务重新校准ADC。

## 职责和文件

- app/src/pwm.c：保留原三相互补PWM，新增pwm_set_adc_trigger_enabled控制共享触发路由，pwm_get_adc_trigger_timing提供下一边沿的估计CPU时刻及PWM周期。
- app/src/adc.c：完全取消软件触发，使用ADC0/ADC1完成中断。两路中断同为优先级5；PWM电流环回调保留给以后FOC，当前没有绑定它。
- adc_poll：前台电流采样健康检查和必要的重试，并读取最近的母线/温度周期结果，不按主循环频率发起采样。
- adc_read：短临界区复制已发布快照，不读取正在更新的DMA；主循环仍通过adc_sample观察数据。
- adc_start_offset_calibration：拒绝在桥臂使能时标定；接受请求后旧零偏立即失效，旧的在途帧不会计入新的稳定时间窗口。
- motor/inc/motor_config.h：MOTOR_ADC_SAMPLE_HZ映射PWM频率，转换/DMA窗口默认10us，必须小于PWM周期；重臂前边沿保护距离为1000个PWM时钟计数。

## 完整性及故障处理

DMA放在非缓存区；发布前逐字检查cycle_bit、trig_ch、seq_num、adc_ch。cycle_bit在每次重臂前清零，不作为逐帧翻转计数使用。
两路完成时先关闭路由，避免复制期间下一个PWM边沿覆盖DMA。若完成标志已经到但DMA字尚未落地，保持路由关闭，随后重试；不忙等，不发布半组。
从预计采样边沿开始超过10us仍无法取得本帧，就锁存-7并关闭路由和桥臂，拒绝迟到帧。该窗口小于一个PWM周期，避免CPU处理延迟跨周期后把混合数据发布为本帧。
ADC冲突/溢出锁存-8，DMA标识错误锁存-6，标定饱和锁存-5。故障后需在桥臂关闭时重新adc_init或重新上电；不会自动重启功率输出。
未重臂状态超过两个PWM周期仍无进展，前台健康检查也会锁存-7，避免一直返回旧的有效采样。
首次adc_init如果因靠近触发边沿而延期，重臂等待计时从首次adc_poll接管时开始；尚未接通的首次采样不把其它外设初始化时间计入停滞。首次成功接通后，转换截止时间立即生效，运行中延期重臂仍从最近发布时刻计时。
main先初始化串口，再初始化ADC；循环先adc_poll，再处理编码器和上传。
timeout_stage进一步区分-7：ADC_TIMEOUT_REARM（1）为尚未及时重臂，ADC_TIMEOUT_FRAME（2）为预计触发后未及时取得完整数据。
timeout_completion_mask仅记录转换超时当刻的软件完成位：0=两队列均未记录完成，1=仅U，2=仅V，3=双路均完成但DMA尚未通过检查或处理已迟到；不能单凭此值认定模拟硬件故障。
capture_cycles是根据PWM计数器估计的采样时刻，不是硬件捕获时间戳；publish_cycles是软件完成发布的CPU时刻。

## PWM计数寄存器格式修正

HPM6E00的CNT_VAL使用24位整数和8位小数格式，SDK的pwmv2_get_counter_value直接返回寄存器原值；而reload、采样比较点及MOTOR_ADC_TRIGGER_GUARD_TICKS均为普通PWM时钟计数。
旧实现混用单位，例如计数5000对应原始值1280000，拿它直接与8999比较会误认为已经进入边沿保护窗口，导致多次重臂延期、ADC中断与采样计数都为0，最终ADC_TIMEOUT_REARM/-7。
现在先用count_raw>>8进行窗口判断，剩余时间按(point*256-count_raw)/(PWM时钟*256)换算，保留小数精度。未放宽保护时限或修改采样点。
tests/run_pwm_adc_timing_tests.ps1从实际pwm.c和SDK提取当前时序函数及计数读取函数，结合SDK寄存器头进行回归，覆盖中间计数、低8位小数、保护窗口边界与无效配置。旧实现在5000计数用例中失败，修正后应通过；此项补充了原ADC状态机测试中模拟PWM时序接口而遗漏的寄存器格式检查。
格式依据：[先楫HPM6E00用户手册Rev0.7，42.4.3时间基准](https://www.hpmicro.com/Public/Uploads/uploadfile/files/20250414/HPM6E00UMV07.pdf#page=809)；SDK源码drivers/inc/hpm_pwmv2_drv.h中的pwmv2_set_shadow_val、pwmv2_get_counter_value。

## 上板观察

1. pwm_result=0；pwm_get_reload在当前200MHz PWM时钟、20kHz频率下为10000。
2. adc_result开始可能为1，第一组数据后应为0；持续错误时查看adc_sample.fault_code。
3. adc_sample.sample_count和irq_count持续增长；正常sample_count约每秒增长20000，实际以测量为准。
4. 上电约1秒稳定等待，加256组间隔至少1ms的零偏样本后，offset_state=ADC_OFFSET_READY、offset_samples=256、current_valid=true。
5. raw_u/raw_v和offset_u/offset_v为完整16位码（0~65535）；桥臂关闭且无相电流时，换算电流应接近0。
6. timeout_count正常为0，fault_code为0。dma_wait_count表示完成中断后等待DMA落地的次数；rearm_deferred_count表示靠近边沿而延后重臂的次数，增长本身不一定是故障。
7. bus_raw/temperature_raw由独立周期采样提供，前台读取后更新；首次可能需等待约105ms。更改输入时应观察到慢速变化，电流U/V组数与两路完成中断仍应持续增长。

暂停CPU、断点和单步可能扰动采样时序，恢复后出现超时需重新初始化；判断采样率应在连续运行时进行。
本阶段没有接入FOC、对齐、速度/位置环，也未恢复ENABLE开环测试入口。桥臂默认仍关闭。

## 测量结束后的正式配置

已移除临时adc_timing_t、逐帧分阶段打点及16通道时序上传，VOFA恢复JustFloat三通道：CH0输出轴累计角度（°）、CH1原始速度（rpm）、CH2滤波速度（rpm）；115200 bit/s、100Hz。母线和温度仍独立周期采样。

测量过程中发现并保留的正式修复：

- ADC/PWM实时文件局部O2，ADC实时链路及关中断快照复制从.fast/ILM执行；CPU周期计数、ADC状态寄存器访问强制内联。VOFA的快照复制临界区同样从ILM执行。
- 母线/温度SDK结果读取放在开中断区间，仅提交结果时短暂关中断。
- DMA和状态清理在取得PWM时刻之前完成；接通触发前复查实际计数和CPU时间，窗口失效时延期。
- PWM频率在初始化缓存，时基配对记录CPU时间/PWM计数/CPU时间，拒绝读取跨度过大的基准；常用整数时钟比用乘法移位换算。
- 保留10us确认时限，读取DMA后再次确认未越过截止时间；capture_cycles、publish_cycles仍为原有健康诊断字段，不进行临时时序上报。

移除打点前，实机上传31帧中触发到软件确认双路DMA有效为1.592~1.917us（平均1.814us），均无故障；这是100Hz抽取样本，不是所有20kHz周期的最坏值，也不是纯硬件转换时间。移除打点后的真实延迟不沿用该数值。

tests/check_adc_realtime_placement.py保留为固件布局检查：GCC直接检查ELF，SES结合ELF地址和同次构建目标文件检查，防止确认路径再次跳转Flash。临时VOFA时序专用测试已移除。

## 验证

主机回归使用实际adc.c、模拟硬件状态，覆盖双路先后完成、DMA迟到、标识错误、超时、硬件冲突、完整16位电流换算、零偏等待与256样本平均、在途帧重标定、桥臂使能拒绝标定、边沿延期和初始化失败。启动回归另外覆盖首次延期后隔5ms才进入前台、首次接管后持续不能重臂、运行中延期仍按原时限关断，以及超时阶段诊断。脚本为tests/run_adc_pwm_sync_tests.ps1。
固件构建和主机回归只能确认代码路径与接口，不能替代实机触发时序、DMA完成时限及模拟电流测量验证。
辅助采样拆分回归覆盖：U/V队列均只有1项、原辅助通道与22/5/20配置、重初始化先停周期采样、辅助值不参与电流DMA发布、前台独立读取、U/V任一路先完成、辅助配置失败。

本次执行结果：同步ADC主机回归、保留的PWM测试状态机回归、GCC固件构建、SES Debug构建均通过。保留用户现有的开环测试3000mA参数，未恢复测试入口。
