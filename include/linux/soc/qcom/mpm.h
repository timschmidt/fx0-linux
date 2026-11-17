/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_QCOM_MPM_H__
#define __LINUX_SOC_QCOM_MPM_H__

#include <linux/types.h>
struct device;

int qcom_mpm_register_sleep(struct device *dev, struct device *owner);
int qcom_mpm_unregister_sleep(struct device *dev, struct device *owner);
int qcom_mpm_prepare_sleep(struct device *dev, struct device *owner);
bool qcom_mpm_sleep_ready(struct device *dev, struct device *owner);
int qcom_mpm_finish_sleep(struct device *dev, struct device *owner);

#endif
