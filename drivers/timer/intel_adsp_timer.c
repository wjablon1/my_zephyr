/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/interrupt_controller/dw_ace.h>

#include <cavs-idc.h>
#include <adsp_shim.h>
#include <adsp_interrupt.h>
#include <zephyr/irq.h>

#define DT_DRV_COMPAT intel_adsp_timer

/**
 * @file
 * @brief Intel Audio DSP Wall Clock Timer driver
 *
 * The Audio DSP on Intel SoC has a timer with one counter and two compare
 * registers that is external to the CPUs. This timer is accessible from
 * all available CPU cores and provides a synchronized timer under SMP.
 */

#define COMPARATOR_IDX  0 /* 0 or 1 */

#ifdef CONFIG_SOC_SERIES_INTEL_ADSP_ACE
#define TIMER_IRQ ACE_IRQ_TO_ZEPHYR(ACE_INTL_TTS)
#else
#define TIMER_IRQ DSP_WCT_IRQ(COMPARATOR_IDX)
#endif

#define CYC_PER_TICK	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC	\
			/ CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_CYC		0xFFFFFFFFUL
#define MAX_TICKS	((MAX_CYC - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 100000)

BUILD_ASSERT(MIN_DELAY < CYC_PER_TICK);
BUILD_ASSERT(COMPARATOR_IDX >= 0 && COMPARATOR_IDX <= 1);

#define DSP_WCT_CS_TT(x)                     BIT(4 + x)

static uint64_t last_count;

/* Not using current syscon driver due to overhead due to MMU support */
#define SYSCON_REG_ADDR	DT_REG_ADDR(DT_INST_PHANDLE(0, syscon))

#define DSPWCTCS_ADDR (SYSCON_REG_ADDR + ADSP_DSPWCTCS_OFFSET)
#define DSPWCT0C_LO_ADDR (SYSCON_REG_ADDR + ADSP_DSPWCT0C_OFFSET)
#define DSPWCT0C_HI_ADDR (SYSCON_REG_ADDR + ADSP_DSPWCT0C_OFFSET + 4)
#define DSPWC_LO_ADDR (SYSCON_REG_ADDR + ADSP_DSPWC_OFFSET)
#define DSPWC_HI_ADDR (SYSCON_REG_ADDR + ADSP_DSPWC_OFFSET + 4)

#if defined(CONFIG_TEST)
const int32_t z_sys_timer_irq_for_test = TIMER_IRQ; /* See tests/kernel/context */
#endif

static uint64_t count(void);

/* DSPWCTCS.TxA is write-1-to-set and is cleared by hardware only on a compare
 * match - it cannot be disarmed in software, so the compare value is always
 * reprogrammed while still armed. Per erratum HSD 18038820785 that is unsafe
 * on ACE: DSPWCTxC and the comparison logic sit in different asynchronous
 * clock domains, so a partially latched value can produce a spurious,
 * premature match. Guard against it the way the reference ACE firmware does
 * (wallclock_ace.c: adsphal_wallclock_compare_set()): keep the high dword
 * parked at a value the counter cannot reach across every intermediate
 * write, letting each one settle in the wall clock domain, and only commit
 * the real high word last.
 */
#define DSPWCT_PARK_HI 0xF0000000UL

/* DSPWCTCS bits 7:0 hold the write-1-to-set arm bits (TxA) and the
 * write-1-to-clear triggered bits (TxT). Zero them on every read-modify-write
 * so no bit is armed or cleared as a side effect of updating another one
 * (e.g. a foreign comparator's still-set TxT read back and blindly written
 * would silently clear that comparator's trigger).
 */
#define DSPWCTCS_RMW_MASK 0xFFFFFF00UL

static void dspwctcs_update(uint32_t bits)
{
	sys_write32((sys_read32(DSPWCTCS_ADDR) & DSPWCTCS_RMW_MASK) | bits, DSPWCTCS_ADDR);
}

static void wait_wallclock_cycles(uint32_t cycles)
{
	uint64_t start = count();

	while ((count() - start) <= cycles) {
		;
	}
}

/* Some platforms/conditions (observed after a SOFT_OFF/D3 restore) have MMIO
 * round-trip latency for the disarm/write/arm sequence below that exceeds a
 * full CYC_PER_TICK, so a statically-computed "next" compare can already be
 * in the past by the time the arm write takes hardware effect. Verify the
 * arm actually landed in the future and retry from a fresh reading if not,
 * rather than trusting a fixed margin - otherwise the comparator fires
 * immediately, and each retriggered ISR can repeat the same mistake forever
 * (a livelock where the timer preempts everything else, indefinitely).
 */
static void set_compare(uint64_t time)
{
	bool expired;

	do {
		sys_write32(sys_read32(DSPWCT0C_HI_ADDR) | DSPWCT_PARK_HI, DSPWCT0C_HI_ADDR);
		wait_wallclock_cycles(2);
		sys_write32((uint32_t)time, DSPWCT0C_LO_ADDR);
		sys_write32(((uint32_t)(time >> 32)) | DSPWCT_PARK_HI, DSPWCT0C_HI_ADDR);
		wait_wallclock_cycles(2);
		sys_write32((uint32_t)(time >> 32), DSPWCT0C_HI_ADDR);

		/* Arm the timer */
		dspwctcs_update(DSP_WCT_CS_TA(COMPARATOR_IDX));

		uint64_t now = count();

		expired = now >= time;
		if (expired) {
			time = now + CYC_PER_TICK;
		}
	} while (expired);
}

static uint64_t count(void)
{
	/* The count register is 64 bits, but we're a 32 bit CPU that
	 * can only read four bytes at a time, so a bit of care is
	 * needed to prevent racing against a wraparound of the low
	 * word.  Wrap the low read between two reads of the high word
	 * and make sure it didn't change.
	 */
	uint32_t hi0, hi1, lo;

	do {
		hi0 = sys_read32(DSPWC_HI_ADDR);
		lo = sys_read32(DSPWC_LO_ADDR);
		hi1 = sys_read32(DSPWC_HI_ADDR);
	} while (hi0 != hi1);
	return (((uint64_t)hi0) << 32) | lo;
}

static uint32_t count32(void)
{
	uint32_t counter_lo;

	counter_lo = sys_read32(DSPWC_LO_ADDR);
	return counter_lo;
}

static void compare_isr(const void *arg)
{
	ARG_UNUSED(arg);
	uint64_t curr;
	uint64_t dticks;

	k_spinlock_key_t key = sys_clock_lock();

	curr = count();
	dticks = (curr - last_count) / CYC_PER_TICK;

	/* Clear the triggered bit */
	dspwctcs_update(DSP_WCT_CS_TT(COMPARATOR_IDX));

	last_count += dticks * CYC_PER_TICK;

#ifndef CONFIG_TICKLESS_KERNEL
	uint64_t next = last_count + CYC_PER_TICK;

	if ((int64_t)(next - curr) < MIN_DELAY) {
		next += CYC_PER_TICK;
	}
	set_compare(next);
#endif

	sys_clock_announce_locked(dticks, key);
}

void sys_clock_set_timeout(uint32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	__ASSERT(sys_clock_is_locked(), "system clock lock not held");

#ifdef CONFIG_TICKLESS_KERNEL
	ticks = CLAMP(ticks, 1, MAX_TICKS) - 1;

	uint64_t curr = count();
	uint64_t next;
	uint32_t adj, cyc = ticks * CYC_PER_TICK;

	/* Round up to next tick boundary */
	adj = (uint32_t)(curr - last_count) + (CYC_PER_TICK - 1);
	if (cyc <= MAX_CYC - adj) {
		cyc += adj;
	} else {
		cyc = MAX_CYC;
	}

	if (cyc > MAX_CYC - (uint32_t)last_count) {
		cyc = MAX_CYC - (uint32_t)last_count;
	}

	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;
	next = last_count + cyc;

	if (((uint32_t)next - (uint32_t)curr) < MIN_DELAY) {
		next += CYC_PER_TICK;
	}

	set_compare(next);
#endif
}

uint32_t sys_clock_elapsed(void)
{
	__ASSERT(sys_clock_is_locked(), "system clock lock not held");

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}
	uint64_t ret = (count() - last_count) / CYC_PER_TICK;

	return (uint32_t)ret;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return count32();
}

uint64_t sys_clock_cycle_get_64(void)
{
	return count();
}

/* Interrupt setup is partially-cpu-local state, so needs to be
 * repeated for each core when it starts.  Note that this conforms to
 * the Zephyr convention of sending timer interrupts to all cpus (for
 * the benefit of timeslicing).
 */
static void irq_init(void)
{
	int cpu = arch_curr_cpu()->id;

	/* These platforms have an extra layer of interrupt masking
	 * (for per-core control) above the interrupt controller.
	 * Drivers need to do that part.
	 */
#ifdef CONFIG_SOC_SERIES_INTEL_ADSP_ACE
	ACE_DINT[cpu].ie[ACE_INTL_TTS] |= BIT(COMPARATOR_IDX + 1);
	/* Discard any trigger latched before this core was (re)started, e.g.
	 * a stale one surviving a D3 power-down, before unmasking the source.
	 */
	dspwctcs_update(DSP_WCT_CS_TT(COMPARATOR_IDX));
	dspwctcs_update(ADSP_SHIM_DSPWCTCS_TTIE(COMPARATOR_IDX));
#else
	CAVS_INTCTRL[cpu].l2.clear = CAVS_L2_DWCT0;
#endif
	irq_enable(TIMER_IRQ);
}

void smp_timer_init(void)
{
}

static int sys_clock_driver_init(void)
{
	uint64_t curr = count();

	IRQ_CONNECT(TIMER_IRQ, 0, compare_isr, 0, 0);
	set_compare(curr + CYC_PER_TICK);
	last_count = curr;
	irq_init();
	return 0;
}

/* Runs on core 0 only */
void intel_adsp_clock_soft_off_exit(void)
{
	(void)sys_clock_driver_init();
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
