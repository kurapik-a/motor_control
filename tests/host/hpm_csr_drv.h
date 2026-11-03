#include <stdint.h>
#define CSR_MSTATUS_MIE_MASK 1u
uint64_t hpm_csr_get_core_cycle(void);
uint32_t disable_global_irq(uint32_t);
void restore_global_irq(uint32_t);
