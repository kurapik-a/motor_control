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
for name in placed:
    assert name in symbols, f"Missing realtime symbol: {name}"
    assert 0 <= symbols[name] < 0x40000, f"Not in local ILM: {name} at {symbols[name]:#x}"

# 这些函数在IRQ入口、10us确认窗口或下一次触发接通前执行，不能直接调用Flash。
# 辅助ADC读取仅在开中断区调用SDK；初始化和非整数时钟比的通用除法不在此列表。
checked = ["adc_u_isr", "adc_v_isr", "service_frame", "arm_frame", "read_dma_word",
           "update_sample", "adc_read", "pwm_set_adc_trigger_enabled"]
for name in checked:
    assembly = dump("-d", f"--disassemble={name}")
    if not re.search(r"^\s*[0-9a-fA-F]+:\s", assembly, re.M):
        # SES将ILM内容打包到启动初始化数据中，ELF的RAM段没有可直接反汇编的CONTENTS。
        # 地址取最终ELF，指令及调用重定位取同次构建的目标文件。
        object_root = Path(args.elf).resolve().parent.parent / "Obj"
        filename = "pwm.c.o" if name == "pwm_set_adc_trigger_enabled" else "adc.c.o"
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

print("PASS realtime image: ILM placement, inline cycle reads, no direct Flash calls in ADC timing chain")
