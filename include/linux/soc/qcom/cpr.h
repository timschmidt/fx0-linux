/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_QCOM_CPR_H__
#define __LINUX_SOC_QCOM_CPR_H__

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;

#if IS_ENABLED(CONFIG_QCOM_CPR_MSM8926)
int qcom_cpr_msm8926_get_supported_hw(struct device *cpu, u32 *mask);
#else
static inline int qcom_cpr_msm8926_get_supported_hw(struct device *cpu, u32 *mask)
{
	return -EOPNOTSUPP;
}
#endif

#endif
