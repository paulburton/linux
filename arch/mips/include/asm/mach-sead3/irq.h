#ifndef __ASM_MACH_MIPS_IRQ_H
#define __ASM_MACH_MIPS_IRQ_H

#define MIPS_GIC_IRQ_BASE 0
#define GIC_NUM_INTRS (24 + NR_CPUS * 2)
#define NR_IRQS 256


#include_next <irq.h>

#endif /* __ASM_MACH_MIPS_IRQ_H */
