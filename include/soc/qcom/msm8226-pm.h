/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_QCOM_MSM8226_PM_H__
#define __SOC_QCOM_MSM8226_PM_H__

#define QCOM_MSM8226_HOTPLUG_PROBE_CPU	3

#ifndef __ASSEMBLY__
#include <linux/kconfig.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/firmware/qcom/qcom_scm.h>

#if IS_ENABLED(CONFIG_QCOM_MSM8226_PM)
bool qcom_msm8226_pm_active(void);
int qcom_msm8226_cpu_collapse(unsigned int cpu);
int qcom_msm8226_cpu_kill(unsigned int cpu);
int qcom_msm8226_set_warm_boot_addr(void *resume);
bool qcom_msm8226_hotplug_probe_active(void);
void __noreturn qcom_msm8226_hotplug_probe(bool quiesced);
#else
static inline bool qcom_msm8226_pm_active(void) { return false; }
static inline int qcom_msm8226_cpu_collapse(unsigned int cpu) { return -ENODEV; }
static inline int qcom_msm8226_cpu_kill(unsigned int cpu) { return 1; }
static inline int qcom_msm8226_set_warm_boot_addr(void *resume)
{
	return qcom_scm_set_warm_boot_addr(resume);
}
static inline bool qcom_msm8226_hotplug_probe_active(void) { return false; }
static inline void __noreturn qcom_msm8226_hotplug_probe(bool quiesced)
{
	unreachable();
}
#endif

#endif /* __ASSEMBLY__ */
#endif
