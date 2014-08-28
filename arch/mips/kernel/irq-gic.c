/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Ralf Baechle (ralf@linux-mips.org)
 * Copyright (C) 2012 MIPS Technologies, Inc.  All rights reserved.
 */
#include <linux/bitmap.h>
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>

#include <asm/io.h>
#include <asm/gic.h>
#include <asm/setup.h>
#include <asm/traps.h>
#include <linux/hardirq.h>
#include <asm-generic/bitops/find.h>

unsigned int gic_frequency;
unsigned int gic_present;
unsigned long _gic_base;
unsigned int gic_irq_base;
unsigned int gic_irq_flags[GIC_NUM_INTRS];

/* The index into this array is the vector # of the interrupt. */
struct gic_shared_intr_map gic_shared_intr_map[GIC_NUM_INTRS];

struct gic_pcpu_mask {
	DECLARE_BITMAP(pcpu_mask, GIC_NUM_INTRS);
};

struct gic_pending_regs {
	DECLARE_BITMAP(pending, GIC_NUM_INTRS);
};

struct gic_intrmask_regs {
	DECLARE_BITMAP(intrmask, GIC_NUM_INTRS);
};

static struct gic_pcpu_mask pcpu_masks[NR_CPUS];
static struct gic_pending_regs pending_regs[NR_CPUS];
static struct gic_intrmask_regs intrmask_regs[NR_CPUS];

static struct irq_chip gic_irq_controller;

static inline bool gic_is_local_irq(unsigned int hwirq)
{
	return hwirq >= GIC_NUM_INTRS;
}

static inline unsigned int gic_hw_to_local_irq(unsigned int hwirq)
{
	return hwirq - GIC_NUM_INTRS;
}

static inline unsigned int gic_local_to_hw_irq(unsigned int irq)
{
	return irq + GIC_NUM_INTRS;
}

#if defined(CONFIG_CSRC_GIC) || defined(CONFIG_CEVT_GIC)
cycle_t gic_read_count(void)
{
	unsigned int hi, hi2, lo;

	do {
		GICREAD(GIC_REG(SHARED, GIC_SH_COUNTER_63_32), hi);
		GICREAD(GIC_REG(SHARED, GIC_SH_COUNTER_31_00), lo);
		GICREAD(GIC_REG(SHARED, GIC_SH_COUNTER_63_32), hi2);
	} while (hi2 != hi);

	return (((cycle_t) hi) << 32) + lo;
}

void gic_write_compare(cycle_t cnt)
{
	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_COMPARE_HI),
				(int)(cnt >> 32));
	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_COMPARE_LO),
				(int)(cnt & 0xffffffff));
}

void gic_write_cpu_compare(cycle_t cnt, int cpu)
{
	unsigned long flags;

	local_irq_save(flags);

	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_OTHER_ADDR), cpu);
	GICWRITE(GIC_REG(VPE_OTHER, GIC_VPE_COMPARE_HI),
				(int)(cnt >> 32));
	GICWRITE(GIC_REG(VPE_OTHER, GIC_VPE_COMPARE_LO),
				(int)(cnt & 0xffffffff));

	local_irq_restore(flags);
}

cycle_t gic_read_compare(void)
{
	unsigned int hi, lo;

	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_COMPARE_HI), hi);
	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_COMPARE_LO), lo);

	return (((cycle_t) hi) << 32) + lo;
}
#endif

unsigned int gic_get_timer_pending(void)
{
	unsigned int vpe_pending;

	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_OTHER_ADDR), 0);
	GICREAD(GIC_REG(VPE_OTHER, GIC_VPE_PEND), vpe_pending);
	return (vpe_pending & GIC_VPE_PEND_TIMER_MSK);
}

void gic_bind_eic_interrupt(int irq, int set)
{
	/* Convert irq vector # to hw int # */
	irq -= GIC_PIN_TO_VEC_OFFSET;

	/* Set irq to use shadow set */
	GICWRITE(GIC_REG_ADDR(VPE_LOCAL, GIC_VPE_EIC_SS(irq)), set);
}

void gic_send_ipi(unsigned int intr)
{
	GICWRITE(GIC_REG(SHARED, GIC_SH_WEDGE), 0x80000000 | intr);
}

static void gic_eic_irq_dispatch(void)
{
	unsigned int cause = read_c0_cause();
	int irq;

	irq = (cause & ST0_IM) >> STATUSB_IP2;
	if (irq == 0)
		irq = -1;

	if (irq >= 0)
		do_IRQ(gic_irq_base + irq);
	else
		spurious_interrupt();
}

static void __init vpe_local_setup(unsigned int numvpes)
{
	unsigned long timer_intr = GIC_INT_TMR;
	unsigned long perf_intr = GIC_INT_PERFCTR;
	unsigned int vpe_ctl;
	int i;

	if (cpu_has_veic) {
		/*
		 * GIC timer interrupt -> CPU HW Int X (vector X+2) ->
		 * map to pin X+2-1 (since GIC adds 1)
		 */
		timer_intr += (GIC_CPU_TO_VEC_OFFSET - GIC_PIN_TO_VEC_OFFSET);
		/*
		 * GIC perfcnt interrupt -> CPU HW Int X (vector X+2) ->
		 * map to pin X+2-1 (since GIC adds 1)
		 */
		perf_intr += (GIC_CPU_TO_VEC_OFFSET - GIC_PIN_TO_VEC_OFFSET);
	}

	/*
	 * Setup the default performance counter timer interrupts
	 * for all VPEs
	 */
	for (i = 0; i < numvpes; i++) {
		GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_OTHER_ADDR), i);

		/* Are Interrupts locally routable? */
		GICREAD(GIC_REG(VPE_OTHER, GIC_VPE_CTL), vpe_ctl);
		if (vpe_ctl & GIC_VPE_CTL_TIMER_RTBL_MSK)
			GICWRITE(GIC_REG(VPE_OTHER, GIC_VPE_TIMER_MAP),
				 GIC_MAP_TO_PIN_MSK | timer_intr);
		if (cpu_has_veic) {
			set_vi_handler(timer_intr + GIC_PIN_TO_VEC_OFFSET,
				gic_eic_irq_dispatch);
			gic_shared_intr_map[timer_intr + GIC_PIN_TO_VEC_OFFSET].local_intr_mask |= GIC_VPE_RMASK_TIMER_MSK;
		}

		if (vpe_ctl & GIC_VPE_CTL_PERFCNT_RTBL_MSK)
			GICWRITE(GIC_REG(VPE_OTHER, GIC_VPE_PERFCTR_MAP),
				 GIC_MAP_TO_PIN_MSK | perf_intr);
		if (cpu_has_veic) {
			set_vi_handler(perf_intr + GIC_PIN_TO_VEC_OFFSET, gic_eic_irq_dispatch);
			gic_shared_intr_map[perf_intr + GIC_PIN_TO_VEC_OFFSET].local_intr_mask |= GIC_VPE_RMASK_PERFCNT_MSK;
		}
	}
}

unsigned int gic_compare_int(void)
{
	unsigned int pending;

	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_PEND), pending);
	if (pending & GIC_VPE_PEND_CMP_MSK)
		return 1;
	else
		return 0;
}

void gic_get_int_mask(unsigned long *dst, const unsigned long *src)
{
	unsigned int i;
	unsigned long *pending, *intrmask, *pcpu_mask;
	unsigned long *pending_abs, *intrmask_abs;

	/* Get per-cpu bitmaps */
	pending = pending_regs[smp_processor_id()].pending;
	intrmask = intrmask_regs[smp_processor_id()].intrmask;
	pcpu_mask = pcpu_masks[smp_processor_id()].pcpu_mask;

	pending_abs = (unsigned long *) GIC_REG_ABS_ADDR(SHARED,
							 GIC_SH_PEND_31_0_OFS);
	intrmask_abs = (unsigned long *) GIC_REG_ABS_ADDR(SHARED,
							  GIC_SH_MASK_31_0_OFS);

	for (i = 0; i < BITS_TO_LONGS(GIC_NUM_INTRS); i++) {
		GICREAD(*pending_abs, pending[i]);
		GICREAD(*intrmask_abs, intrmask[i]);
		pending_abs++;
		intrmask_abs++;
	}

	bitmap_and(pending, pending, intrmask, GIC_NUM_INTRS);
	bitmap_and(pending, pending, pcpu_mask, GIC_NUM_INTRS);
	bitmap_and(dst, src, pending, GIC_NUM_INTRS);
}

unsigned int gic_get_int(void)
{
	DECLARE_BITMAP(interrupts, GIC_NUM_INTRS);

	bitmap_fill(interrupts, GIC_NUM_INTRS);
	gic_get_int_mask(interrupts, interrupts);

	return find_first_bit(interrupts, GIC_NUM_INTRS);
}

void gic_get_local_int_mask(unsigned long *dst, const unsigned long *src)
{
	unsigned long pending, intrmask;

	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_PEND), pending);
	GICREAD(GIC_REG(VPE_LOCAL, GIC_VPE_MASK), intrmask);

	bitmap_and(&pending, &pending, &intrmask, GIC_NUM_LOCAL_INTRS);
	bitmap_and(dst, src, &pending, GIC_NUM_LOCAL_INTRS);
}

unsigned int gic_get_local_int(void)
{
	unsigned long interrupts;

	bitmap_fill(&interrupts, GIC_NUM_LOCAL_INTRS);
	gic_get_local_int_mask(&interrupts, &interrupts);

	return find_first_bit(&interrupts, GIC_NUM_LOCAL_INTRS);
}

static void gic_mask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq - gic_irq_base;

	if (gic_is_local_irq(irq)) {
		GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_RMASK),
			 1 << GIC_INTR_BIT(gic_hw_to_local_irq(irq)));
	} else {
		GIC_CLR_INTR_MASK(irq);
	}
}

static void gic_unmask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq - gic_irq_base;

	if (gic_is_local_irq(irq)) {
		GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_SMASK),
			 1 << GIC_INTR_BIT(gic_hw_to_local_irq(irq)));
	} else {
		GIC_SET_INTR_MASK(irq);
	}
}

void __weak gic_irq_ack(struct irq_data *d)
{
	unsigned int irq = d->irq - gic_irq_base;

	if (gic_is_local_irq(irq)) {
		GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_RMASK),
			 1 << GIC_INTR_BIT(gic_hw_to_local_irq(irq)));
	} else {
		GIC_CLR_INTR_MASK(irq);

		/* Clear edge detector */
		if (gic_irq_flags[irq] & GIC_TRIG_EDGE)
			GICWRITE(GIC_REG(SHARED, GIC_SH_WEDGE), irq);
	}
}

void __weak gic_finish_irq(struct irq_data *d)
{
	unsigned int irq = d->irq - gic_irq_base;

	if (gic_is_local_irq(irq)) {
		GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_SMASK),
			 1 << GIC_INTR_BIT(gic_hw_to_local_irq(irq)));
	} else {
		GIC_SET_INTR_MASK(irq);
	}
}

static int gic_set_type(struct irq_data *d, unsigned int type)
{
	unsigned int irq = d->irq - gic_irq_base;
	bool is_edge;

	if (gic_is_local_irq(irq))
		return -EINVAL;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_FALLING:
		GIC_SET_POLARITY(irq, GIC_POL_POS);
		GIC_SET_TRIGGER(irq, GIC_TRIG_EDGE);
		GIC_SET_DUAL(irq, GIC_TRIG_DUAL_DISABLE);
		is_edge = true;
		break;
	case IRQ_TYPE_EDGE_RISING:
		GIC_SET_POLARITY(irq, GIC_POL_NEG);
		GIC_SET_TRIGGER(irq, GIC_TRIG_EDGE);
		GIC_SET_DUAL(irq, GIC_TRIG_DUAL_DISABLE);
		is_edge = true;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		/* polarity is irrelevant in this case */
		GIC_SET_TRIGGER(irq, GIC_TRIG_EDGE);
		GIC_SET_DUAL(irq, GIC_TRIG_DUAL_ENABLE);
		is_edge = true;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		GIC_SET_POLARITY(irq, GIC_POL_NEG);
		GIC_SET_TRIGGER(irq, GIC_TRIG_LEVEL);
		GIC_SET_DUAL(irq, GIC_TRIG_DUAL_DISABLE);
		is_edge = false;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
	default:
		GIC_SET_POLARITY(irq, GIC_POL_POS);
		GIC_SET_TRIGGER(irq, GIC_TRIG_LEVEL);
		GIC_SET_DUAL(irq, GIC_TRIG_DUAL_DISABLE);
		is_edge = false;
		break;
	}

	if (is_edge) {
		gic_irq_flags[irq] |= GIC_TRIG_EDGE;
		__irq_set_handler_locked(d->irq, handle_edge_irq);
	} else {
		gic_irq_flags[irq] &= ~GIC_TRIG_EDGE;
		__irq_set_handler_locked(d->irq, handle_level_irq);
	}

	return 0;
}

#ifdef CONFIG_SMP
static DEFINE_SPINLOCK(gic_lock);

static int gic_set_affinity(struct irq_data *d, const struct cpumask *cpumask,
			    bool force)
{
	unsigned int irq = d->irq - gic_irq_base;
	cpumask_t	tmp = CPU_MASK_NONE;
	unsigned long	flags;
	int		i;

	if (gic_is_local_irq(irq))
		return -EINVAL;

	cpumask_and(&tmp, cpumask, cpu_online_mask);
	if (cpus_empty(tmp))
		return -EINVAL;

	/* Assumption : cpumask refers to a single CPU */
	spin_lock_irqsave(&gic_lock, flags);

	/* Re-route this IRQ */
	GIC_SH_MAP_TO_VPE_SMASK(irq, first_cpu(tmp));

	/* Update the pcpu_masks */
	for (i = 0; i < NR_CPUS; i++)
		clear_bit(irq, pcpu_masks[i].pcpu_mask);
	set_bit(irq, pcpu_masks[first_cpu(tmp)].pcpu_mask);

	cpumask_copy(d->affinity, cpumask);
	spin_unlock_irqrestore(&gic_lock, flags);

	return IRQ_SET_MASK_OK_NOCOPY;
}
#endif

static struct irq_chip gic_irq_controller = {
	.name			=	"MIPS GIC",
	.irq_ack		=	gic_irq_ack,
	.irq_mask		=	gic_mask_irq,
	.irq_mask_ack		=	gic_mask_irq,
	.irq_unmask		=	gic_unmask_irq,
	.irq_eoi		=	gic_finish_irq,
	.irq_set_type		=	gic_set_type,
#ifdef CONFIG_SMP
	.irq_set_affinity	=	gic_set_affinity,
#endif
};

static void __init gic_setup_intr(unsigned int intr, unsigned int cpu,
	unsigned int pin, unsigned int polarity, unsigned int trigtype,
	unsigned int flags)
{
	struct gic_shared_intr_map *map_ptr;

	/* Setup Intr to Pin mapping */
	if (pin & GIC_MAP_TO_NMI_MSK) {
		int i;

		GICWRITE(GIC_REG_ADDR(SHARED, GIC_SH_MAP_TO_PIN(intr)), pin);
		/* FIXME: hack to route NMI to all cpu's */
		for (i = 0; i < NR_CPUS; i += 32) {
			GICWRITE(GIC_REG_ADDR(SHARED,
					  GIC_SH_MAP_TO_VPE_REG_OFF(intr, i)),
				 0xffffffff);
		}
	} else {
		GICWRITE(GIC_REG_ADDR(SHARED, GIC_SH_MAP_TO_PIN(intr)),
			 GIC_MAP_TO_PIN_MSK | pin);
		/* Setup Intr to CPU mapping */
		GIC_SH_MAP_TO_VPE_SMASK(intr, cpu);
		if (cpu_has_veic) {
			set_vi_handler(pin + GIC_PIN_TO_VEC_OFFSET,
				gic_eic_irq_dispatch);
			map_ptr = &gic_shared_intr_map[pin + GIC_PIN_TO_VEC_OFFSET];
			if (map_ptr->num_shared_intr >= GIC_MAX_SHARED_INTR)
				BUG();
			map_ptr->intr_list[map_ptr->num_shared_intr++] = intr;
		}
	}

	/* Setup Intr Polarity */
	GIC_SET_POLARITY(intr, polarity);

	/* Setup Intr Trigger Type */
	GIC_SET_TRIGGER(intr, trigtype);

	/* Init Intr Masks */
	GIC_CLR_INTR_MASK(intr);

	/* Initialise per-cpu Interrupt software masks */
	set_bit(intr, pcpu_masks[cpu].pcpu_mask);

	if ((flags & GIC_FLAG_TRANSPARENT) && (cpu_has_veic == 0))
		GIC_SET_INTR_MASK(intr);
	if (trigtype == GIC_TRIG_EDGE)
		gic_irq_flags[intr] |= GIC_TRIG_EDGE;
}

static void __init gic_setup_local_intr(unsigned int intr, unsigned int pin,
				unsigned int flags)
{
	struct gic_shared_intr_map *map_ptr;
	unsigned int local_irq = gic_hw_to_local_irq(intr);
	int i;

	/* Setup Intr to Pin mapping */
	for (i = 0; i < nr_cpu_ids; i++) {
		GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_OTHER_ADDR), i);
		if (pin & GIC_MAP_TO_NMI_MSK) {
			GICWRITE(GIC_REG_ADDR(VPE_OTHER,
					GIC_VPE_MAP_TO_PIN(local_irq)), pin);
		} else {
			GICWRITE(GIC_REG_ADDR(VPE_OTHER,
					GIC_VPE_MAP_TO_PIN(local_irq)),
				 GIC_MAP_TO_PIN_MSK | pin);
		}
	}

	if (!(pin & GIC_MAP_TO_NMI_MSK) && cpu_has_veic) {
		set_vi_handler(pin + GIC_PIN_TO_VEC_OFFSET,
			       gic_eic_irq_dispatch);
		map_ptr = &gic_shared_intr_map[pin + GIC_PIN_TO_VEC_OFFSET];
		if (map_ptr->num_shared_intr >= GIC_MAX_SHARED_INTR)
			BUG();
		map_ptr->intr_list[map_ptr->num_shared_intr++] = intr;
	}

	/* Init Intr Masks */
	GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_RMASK),
		 1 << GIC_INTR_BIT(local_irq));

	irq_set_percpu_devid(gic_irq_base + intr);
}

static void __init gic_basic_init(int numintrs, int numvpes,
			struct gic_intr_map *intrmap, int mapsize)
{
	unsigned int i, cpu;
	unsigned int pin_offset = 0;

	board_bind_eic_interrupt = &gic_bind_eic_interrupt;

	/* Setup defaults */
	for (i = 0; i < numintrs; i++) {
		GIC_SET_POLARITY(i, GIC_POL_POS);
		GIC_SET_TRIGGER(i, GIC_TRIG_LEVEL);
		GIC_CLR_INTR_MASK(i);
		if (i < GIC_NUM_INTRS) {
			gic_irq_flags[i] = 0;
			gic_shared_intr_map[i].num_shared_intr = 0;
			gic_shared_intr_map[i].local_intr_mask = 0;
		}
	}

	/*
	 * In EIC mode, the HW_INT# is offset by (2-1). Need to subtract
	 * one because the GIC will add one (since 0=no intr).
	 */
	if (cpu_has_veic)
		pin_offset = (GIC_CPU_TO_VEC_OFFSET - GIC_PIN_TO_VEC_OFFSET);

	/* Setup specifics */
	for (i = 0; i < mapsize; i++) {
		cpu = intrmap[i].cpunum;
		if (cpu == GIC_UNUSED)
			continue;
		if (gic_is_local_irq(i))
			gic_setup_local_intr(i,
				intrmap[i].pin + pin_offset,
				intrmap[i].flags);
		else
			gic_setup_intr(i,
				intrmap[i].cpunum,
				intrmap[i].pin + pin_offset,
				intrmap[i].polarity,
				intrmap[i].trigtype,
				intrmap[i].flags);
	}

	vpe_local_setup(numvpes);
}

void __init gic_init(unsigned long gic_base_addr,
		     unsigned long gic_addrspace_size,
		     struct gic_intr_map *intr_map, unsigned int intr_map_size,
		     unsigned int irqbase)
{
	unsigned int gicconfig;
	int numvpes, numintrs;

	_gic_base = (unsigned long) ioremap_nocache(gic_base_addr,
						    gic_addrspace_size);
	gic_irq_base = irqbase;

	GICREAD(GIC_REG(SHARED, GIC_SH_CONFIG), gicconfig);
	numintrs = (gicconfig & GIC_SH_CONFIG_NUMINTRS_MSK) >>
		   GIC_SH_CONFIG_NUMINTRS_SHF;
	numintrs = ((numintrs + 1) * 8);

	numvpes = (gicconfig & GIC_SH_CONFIG_NUMVPES_MSK) >>
		  GIC_SH_CONFIG_NUMVPES_SHF;
	numvpes = numvpes + 1;

	gic_basic_init(numintrs, numvpes, intr_map, intr_map_size);

	gic_platform_init(GIC_NUM_INTRS + GIC_NUM_LOCAL_INTRS,
			  &gic_irq_controller);
}

#ifdef CONFIG_IRQ_DOMAIN
/* CPU core IRQs used by GIC */
static int gic_cpu_pin[GIC_NUM_CPU_INT];
static int num_gic_cpu_pins;

/* Index of core IRQ used by a particular GIC IRQ */
static int gic_irq_pin[GIC_NUM_INTRS];

static inline int gic_irq_to_cpu_pin(unsigned int hwirq)
{
	if (gic_is_local_irq(hwirq))
		return gic_cpu_pin[0] - MIPS_CPU_IRQ_BASE - GIC_CPU_PIN_OFFSET;
	return gic_cpu_pin[gic_irq_pin[hwirq]] - MIPS_CPU_IRQ_BASE - GIC_CPU_PIN_OFFSET;
}

#ifdef CONFIG_MIPS_GIC_IPI
static int gic_resched_int_base;
static int gic_call_int_base;

unsigned int plat_ipi_resched_int_xlate(unsigned int cpu)
{
	return gic_resched_int_base + cpu;
}

unsigned int plat_ipi_call_int_xlate(unsigned int cpu)
{
	return gic_call_int_base + cpu;
}

static irqreturn_t ipi_resched_interrupt(int irq, void *dev_id)
{
	scheduler_ipi();

	return IRQ_HANDLED;
}

static irqreturn_t ipi_call_interrupt(int irq, void *dev_id)
{
	smp_call_function_interrupt();

	return IRQ_HANDLED;
}

static struct irqaction irq_resched = {
	.handler	= ipi_resched_interrupt,
	.flags		= IRQF_PERCPU,
	.name		= "IPI resched"
};

static struct irqaction irq_call = {
	.handler	= ipi_call_interrupt,
	.flags		= IRQF_PERCPU,
	.name		= "IPI call"
};

static __init void gic_ipi_init_one(struct irq_domain *domain,
				    unsigned int hwirq, int cpu,
				    struct irqaction *action)
{
	int irq = irq_create_mapping(domain, hwirq);
	int i;

	GIC_SET_POLARITY(hwirq, GIC_POL_POS);
	GIC_SET_TRIGGER(hwirq, GIC_TRIG_EDGE);
	GIC_SH_MAP_TO_VPE_SMASK(hwirq, cpu);
	GICWRITE(GIC_REG_ADDR(SHARED, GIC_SH_MAP_TO_PIN(hwirq)),
		 GIC_MAP_TO_PIN_MSK | gic_irq_to_cpu_pin(hwirq));
	GIC_CLR_INTR_MASK(hwirq);
	gic_irq_flags[hwirq] |= GIC_TRIG_EDGE;

	for (i = 0; i < ARRAY_SIZE(pcpu_masks); i++)
		clear_bit(hwirq, pcpu_masks[i].pcpu_mask);
	set_bit(hwirq, pcpu_masks[cpu].pcpu_mask);

	irq_set_chip_and_handler(irq, &gic_irq_controller, handle_percpu_irq);
	setup_irq(irq, action);
}

static __init void gic_ipi_init(struct irq_domain *domain)
{
	int i;

	/* Use last 2 * NR_CPUS interrupts as IPIs */
	gic_resched_int_base = GIC_NUM_INTRS - nr_cpu_ids;
	gic_call_int_base = gic_resched_int_base - nr_cpu_ids;

	for (i = 0; i < nr_cpu_ids; i++) {
		gic_ipi_init_one(domain, gic_call_int_base + i, i, &irq_call);
		gic_ipi_init_one(domain, gic_resched_int_base + i, i,
				 &irq_resched);
	}
}
#else
static inline void gic_ipi_init(struct irq_domain *domain)
{
}
#endif

static int gic_irq_domain_map(struct irq_domain *d, unsigned int irq,
			      irq_hw_number_t hw)
{
	int i;

	if (gic_is_local_irq(hw)) {
		int local_irq = gic_hw_to_local_irq(hw);

		irq_set_chip_and_handler(irq, &gic_irq_controller,
					 handle_percpu_irq);
		irq_set_percpu_devid(irq);

		for (i = 0; i < nr_cpu_ids; i++) {
			GICWRITE(GIC_REG(VPE_LOCAL, GIC_VPE_OTHER_ADDR), i);
			GICWRITE(GIC_REG_ADDR(VPE_OTHER,
					      GIC_VPE_MAP_TO_PIN(local_irq)),
				 GIC_MAP_TO_PIN_MSK | gic_irq_to_cpu_pin(hw));
		}
	} else {
		irq_set_chip_and_handler(irq, &gic_irq_controller,
					 handle_level_irq);

		GICWRITE(GIC_REG_ADDR(SHARED, GIC_SH_MAP_TO_PIN(hw)),
			 GIC_MAP_TO_PIN_MSK | gic_irq_to_cpu_pin(hw));
		/* Map to VPE 0 by default */
		GIC_SH_MAP_TO_VPE_SMASK(hw, 0);
		set_bit(hw, pcpu_masks[0].pcpu_mask);
	}

	return 0;
}

static int gic_irq_domain_xlate(struct irq_domain *d, struct device_node *ctrlr,
				const u32 *intspec, unsigned int intsize,
				unsigned long *out_hwirq,
				unsigned int *out_type)
{
	if (intsize != 2 && intsize != 3)
		return -EINVAL;

	if (intspec[0] >= GIC_NUM_INTRS)
		return -EINVAL;
	*out_hwirq = intspec[0];

	*out_type = intspec[1] & IRQ_TYPE_SENSE_MASK;

	if (intsize == 3) {
		if (intspec[2] >= num_gic_cpu_pins)
			return -EINVAL;
		gic_irq_pin[intspec[0]] = intspec[2];
	}

	return 0;
}

static const struct irq_domain_ops gic_irq_domain_ops = {
	.map = gic_irq_domain_map,
	.xlate = gic_irq_domain_xlate,
};

static void gic_irq_dispatch(unsigned int irq, struct irq_desc *desc)
{
	struct irq_domain *domain = irq_get_handler_data(irq);
	unsigned int hwirq;

	while ((hwirq = gic_get_local_int()) != GIC_NUM_LOCAL_INTRS) {
		irq = irq_linear_revmap(domain, gic_local_to_hw_irq(hwirq));
		generic_handle_irq(irq);
	}

	while ((hwirq = gic_get_int()) != GIC_NUM_INTRS) {
		irq = irq_linear_revmap(domain, hwirq);
		generic_handle_irq(irq);
	}
}

void __init gic_platform_init(int irqs, struct irq_chip *irq_controller)
{
}

int __init gic_of_init(struct device_node *node, struct device_node *parent)
{
	struct irq_domain *domain;
	struct resource res;
	int i;

	if (cpu_has_veic) {
		pr_err("GIC EIC mode not supported with DT yet\n");
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(gic_cpu_pin); i++) {
		gic_cpu_pin[i] = irq_of_parse_and_map(node, i);
		if (!gic_cpu_pin[i])
			break;
		num_gic_cpu_pins++;
	}
	if (!num_gic_cpu_pins) {
		pr_err("No GIC to CPU interrupts specified\n");
		return -ENODEV;
	}

	if (of_address_to_resource(node, 0, &res)) {
		pr_err("Failed to get GIC memory range\n");
		return -ENODEV;
	}

	gic_init(res.start, resource_size(&res), NULL, 0, MIPS_GIC_IRQ_BASE);

	domain = irq_domain_add_legacy(node,
				       GIC_NUM_INTRS + GIC_NUM_LOCAL_INTRS,
				       MIPS_GIC_IRQ_BASE, 0,
				       &gic_irq_domain_ops, NULL);
	if (!domain) {
		pr_err("Failed to add GIC IRQ domain\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_gic_cpu_pins; i++) {
		irq_set_chained_handler(gic_cpu_pin[i], gic_irq_dispatch);
		irq_set_handler_data(gic_cpu_pin[i], domain);
	}

	gic_ipi_init(domain);

	return 0;
}
#endif
