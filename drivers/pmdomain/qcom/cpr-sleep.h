/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_CPR_SLEEP_H__
#define __QCOM_CPR_SLEEP_H__

#include <linux/device.h>
#include <linux/of.h>
#include <linux/panic.h>
#include <linux/regulator/consumer.h>

static inline void cpr_sleep_vote_disable(void *data)
{
	struct regulator *mx = data;
	int ret;

	ret = regulator_disable(mx);
	if (ret)
		ret = regulator_disable(mx);
	if (ret)
		panic("CPR sleep vote release failed: %d", ret);
}

/* Only a complete system entry coordinator may later lower this live floor. */
static inline int cpr_sleep_vote_init(struct device *dev, int max_uV,
				      struct regulator **sleep_mx)
{
	struct regulator *mx;
	int ret;

	*sleep_mx = NULL;
	if (!of_property_present(dev->of_node, "vdd-mx-sleep-supply"))
		return 0;
	if (max_uV <= 0)
		return -EINVAL;
	mx = devm_regulator_get(dev, "vdd-mx-sleep");
	if (IS_ERR(mx))
		return PTR_ERR(mx);
	ret = regulator_set_voltage(mx, max_uV, max_uV);
	if (ret)
		return ret;
	ret = regulator_enable(mx);
	if (ret)
		return ret;
	/* Disable before the managed put; a failed probe must not leak a vote. */
	ret = devm_add_action_or_reset(dev, cpr_sleep_vote_disable, mx);
	if (ret)
		return ret;
	*sleep_mx = mx;
	return 0;
}

#endif
