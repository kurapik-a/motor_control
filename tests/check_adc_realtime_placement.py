"""检查实际固件：ADC实时链在ILM、时间戳已内联，防止源代码属性未生效。"""
import argparse
import re
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("elf")
parser.add_argument("--objdump", required=True)
args = parser.parse_args()


def dump(*options):
    return subprocess.check_output([args.objdump, *options, args.elf], text=True, errors="replace")


symbols = {}
for line in dump("-t").splitlines():
    fields = line.split()
    if len(fields) >= 6 and "F" in fields:
        symbols[fields[-1]] = int(fields[0], 16)

placed = ["adc_u_isr", "adc_v_isr", "service_frame", "arm_frame", "read_dma_word",
          "update_sample", "adc_poll", "adc_read", "adc_init", "read_aux_results",
          "pwm_set_adc_trigger_enabled", "pwm_get_adc_trigger_timing", "motor_vofa_set_encoder", "motor_vofa_poll"]
placed += ["pwm_set_counts", "pwm_enable", "pwm_disable",
           "pwm_set_current_loop_callback", "pwm_current_loop_isr", "pwm_get_reload", "pwm_is_enabled", "capture_aux_flags"]
current_chain = ["current_sample", "current_trip", "current_watchdog", "current_begin", "collect_input", "prepare_feedback", "qualify_alignment",
                 "pmsm_current_ctrl_start", "pmsm_current_ctrl_stop", "pmsm_current_ctrl_set_target",
                 "pmsm_current_ctrl_clear_fault", "pmsm_current_ctrl_poll", "pmsm_current_ctrl_get_status",
                 "pmsm_current_ctrl_align", "pmsm_current_ctrl_finish_alignment",
                 "pmsm_current_ctrl_set_speed_counts_s", "pmsm_speed_ctrl_observe", "pmsm_speed_ctrl_reset_feedback",
                 "pmsm_speed_ctrl_step", "pmsm_speed_ctrl_set_target", "pmsm_speed_ctrl_select_current",
                 "pmsm_speed_ctrl_start", "pmsm_speed_ctrl_stop", "pmsm_speed_ctrl_get_status",
                 "pmsm_current_ctrl_set_position_counts", "pmsm_position_ctrl_observe", "pmsm_position_ctrl_reset_feedback",
                 "pmsm_position_ctrl_set_target", "pmsm_position_ctrl_step", "pmsm_position_ctrl_start",
                 "pmsm_position_ctrl_stop", "pmsm_position_ctrl_select_direct", "pmsm_position_ctrl_get_status",
                 "pmsm_clarke", "pmsm_park", "pmsm_inv_park", "pmsm_sincos", "pmsm_svpwm",
                 "pmsm_pi_step", "pmsm_pi_reset", "pmsm_protect_check", "pmsm_protect_can_enable",
                 "pmsm_protect_clear", "evaluate", "encoder_read_phase", "encoder_read_position", "encoder_reset", "encoder_get_generation"]
placed += current_chain
assert "foc_sine_quarter" in dump("-t"), "Missing FOC sine table"
table = next(line for line in dump("-t").splitlines() if line.split() and line.split()[-1] == "foc_sine_quarter")
assert 0x1200000 <= int(table.split()[0], 16) < 0x1240000, "FOC table must be in DLM"
for name in placed:
    assert name in symbols, f"Missing realtime symbol: {name}"
    assert 0 <= symbols[name] < 0x40000, f"Not in local ILM: {name} at {symbols[name]:#x}"

# 这些函数在IRQ入口、10us确认窗口或下一次触发接通前执行，不能直接调用Flash。
# 辅助ADC读取仅在开中断区调用SDK；初始化和非整数时钟比的通用除法不在此列表。
checked = ["adc_u_isr", "adc_v_isr", "service_frame", "arm_frame", "read_dma_word",
           "update_sample", "adc_read", "pwm_set_adc_trigger_enabled"]
checked += ["pwm_set_counts", "pwm_enable", "pwm_disable",
            "pwm_set_current_loop_callback", "pwm_current_loop_isr", "pwm_get_reload", "pwm_is_enabled", "capture_aux_flags"]
checked += current_chain
for name in checked:
    assembly = dump("-d", f"--disassemble={name}")
    if not re.search(r"^\s*[0-9a-fA-F]+:\s", assembly, re.M):
        # SES将ILM内容打包到启动初始化数据中，ELF的RAM段没有可直接反汇编的CONTENTS。
        # 地址取最终ELF，指令及调用重定位取同次构建的目标文件。
        object_root = Path(args.elf).resolve().parent.parent / "Obj"
        if name.startswith("pmsm_current_ctrl_") or name in {"current_sample", "current_trip", "current_watchdog", "current_begin", "collect_input", "prepare_feedback", "qualify_alignment"}:
            filename = "pmsm_current_ctrl.c.o"
        elif name.startswith("pmsm_protect_") or name == "evaluate":
            filename = "pmsm_protect.c.o"
        elif name.startswith("pmsm_pi_"):
            filename = "pid.c.o"
        elif name.startswith("pmsm_speed_ctrl_"):
            filename = "pmsm_speed_ctrl.c.o"
        elif name.startswith("pmsm_position_ctrl_"):
            filename = "pmsm_position_ctrl.c.o"
        elif name.startswith("pmsm_"):
            filename = "pmsm_foc.c.o"
        elif name in {"encoder_read_phase", "encoder_read_position", "encoder_reset", "encoder_get_generation"}:
            filename = "encoder.c.o"
        elif name.startswith("pwm_"):
            filename = "pwm.c.o"
        else:
            filename = "adc.c.o"
        objects = list(object_root.rglob(filename))
        assert len(objects) == 1, f"Cannot find unique SES object for {name}: {objects}"
        assembly = subprocess.check_output([args.objdump, "-dr", f"--disassemble={name}", str(objects[0])],
                                           text=True, errors="replace")
        assert re.search(r"^\s*[0-9a-fA-F]+:\s", assembly, re.M), f"No instructions for {name}"
        for target in re.findall(r"R_RISCV_CALL(?:_PLT)?\s+([^\s+]+)", assembly):
            assert target in symbols and 0 <= symbols[target] < 0x40000, \
                f"Non-ILM call from {name}: {target}"
    for line in assembly.splitlines():
        if re.search(r"\b(?:jal|jalr|jr|j)\s", line):
            target = re.search(r"\b([0-9a-fA-F]+)\s+<([^>]+)>", line)
            if target:
                address = int(target.group(1), 16)
                assert 0 <= address < 0x40000, f"Flash branch from {name}: {line.strip()}"
    if name == "service_frame":
        assert "rdcycle" in assembly, f"Timestamp not inlined in {name}"

print("PASS realtime image: ADC/control ILM placement, DLM FOC table, inline cycle reads, no direct Flash calls in critical paths")
