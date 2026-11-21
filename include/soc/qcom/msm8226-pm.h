/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_QCOM_MSM8226_PM_H__
#define __SOC_QCOM_MSM8226_PM_H__

#include <linux/kconfig.h>
#include <linux/types.h>
#include <linux/errno.h>

#if IS_ENABLED(CONFIG_QCOM_MSM8226_PM)
bool qcom_msm8226_pm_active(void);
int qcom_msm8226_cpu_collapse(unsigned int cpu);
bool qcom_msm8226_cpu_power_lost(unsigned int cpu);
int qcom_msm8226_cpu_kill(unsigned int cpu);
#else
static inline bool qcom_msm8226_pm_active(void) { return false; }
static inline int qcom_msm8226_cpu_collapse(unsigned int cpu) { return -ENODEV; }
static inline bool qcom_msm8226_cpu_power_lost(unsigned int cpu) { return false; }
static inline int qcom_msm8226_cpu_kill(unsigned int cpu) { return 1; }
#endif

#endif
