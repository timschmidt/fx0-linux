/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_QCOM_CPR_H__
#define __LINUX_SOC_QCOM_CPR_H__

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;

#if IS_ENABLED(CONFIG_QCOM_CPR_MSM8926)
int qcom_cpr_msm8926_get_supported_hw(struct device *cpu, u32 *mask);

/* The owner is a PM supplier of CPR and retains both bound devices. */
int qcom_cpr_msm8926_suspend_attach(struct device *dev, struct device *owner);
int qcom_cpr_msm8926_suspend_detach(struct device *dev, struct device *owner);
int qcom_cpr_msm8926_suspend_prepare(struct device *dev, struct device *owner);
int qcom_cpr_msm8926_suspend_restore(struct device *dev, struct device *owner);
bool qcom_cpr_msm8926_suspend_ready(struct device *dev, struct device *owner);
#else
static inline int qcom_cpr_msm8926_get_supported_hw(struct device *cpu, u32 *mask)
{
	return -EOPNOTSUPP;
}
#endif

#endif
