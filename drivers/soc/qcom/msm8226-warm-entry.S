/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/linkage.h>
#include <soc/qcom/msm8226-pm.h>
#include <asm/assembler.h>

#include "msm8226-hotplug-probe.h"

	.arm
	.text
ENTRY(msm8226_warm_probe_entry)
	ARM_BE8(setend be)
	mrc	p15, 0, r0, c0, c0, 5	@ MPIDR
	ubfx	r1, r0, #0, #24
	cmp	r1, #QCOM_MSM8226_HOTPLUG_PROBE_CPU	@ Cluster 0
	bne	cpu_resume_arm

	/* The MMU is off. Resolve the probe word relative to this entry. */
	adr	r1, .Lprobe_offset
	ldr	r2, [r1]
	add	r1, r1, r2
	ldr	r2, [r1]
	cmp	r2, #0
	beq	cpu_resume_arm

	/*
	 * CPU3 has no saved suspend context or valid late startup data.
	 * Leave it parked after recording that firmware reached this vector.
	 */
	cpsid	aif
	mov	r2, #MSM8226_PROBE_WARM_ENTRY
	str	r2, [r1]
	dsb	sy
1:	wfi
	b	1b
ENDPROC(msm8226_warm_probe_entry)

	.align	2
.Lprobe_offset:
	.word	msm8226_warm_probe_state - .Lprobe_offset
