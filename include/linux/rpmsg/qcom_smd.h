/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LINUX_RPMSG_QCOM_SMD_H
#define _LINUX_RPMSG_QCOM_SMD_H

#include <linux/device.h>

struct qcom_smd_edge;
struct rpmsg_endpoint;

#if IS_ENABLED(CONFIG_RPMSG_QCOM_SMD)

struct qcom_smd_edge *qcom_smd_register_edge(struct device *parent,
					     struct device_node *node);
void qcom_smd_unregister_edge(struct qcom_smd_edge *edge);

#else

static inline struct qcom_smd_edge *
qcom_smd_register_edge(struct device *parent,
		       struct device_node *node)
{
	return NULL;
}

static inline void qcom_smd_unregister_edge(struct qcom_smd_edge *edge)
{
}

#endif

#if IS_REACHABLE(CONFIG_RPMSG_QCOM_SMD)
int qcom_smd_rpm_get_irq(struct rpmsg_endpoint *ept);
#else
static inline int qcom_smd_rpm_get_irq(struct rpmsg_endpoint *ept)
{
	return -EOPNOTSUPP;
}
#endif

#endif
