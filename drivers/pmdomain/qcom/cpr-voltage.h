/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_CPR_VOLTAGE_H__
#define __QCOM_CPR_VOLTAGE_H__

#include <linux/errno.h>
#include <linux/panic.h>
#include <linux/regulator/consumer.h>

/* The caller owns locking and must retain MX while APC can remain live. */
struct cpr_voltage {
	struct regulator *apc;
	struct regulator *mx;
	int mx_max_uV;
	int mx_min_uV;
	int mx_lower_error;
	bool mx_enabled;
};

static inline int cpr_voltage_vote_mx(struct cpr_voltage *rails, int min_uV)
{
	int ret, recovery;

	if (!rails->mx || min_uV <= 0 || min_uV > rails->mx_max_uV)
		return -EINVAL;

	ret = regulator_set_voltage(rails->mx, min_uV, rails->mx_max_uV);
	if (!ret) {
		rails->mx_min_uV = min_uV;
	} else if (rails->mx_enabled && rails->mx_min_uV > 0) {
		/*
		 * A timed-out RPM request may already have changed the vote.
		 * Core restored the old consumer request on failure; force it
		 * through the provider instead of accepting a cached same vote.
		 * RPM must correlate replies before this can confirm restoration.
		 */
		recovery = regulator_sync_voltage(rails->mx);
		if (recovery)
			recovery = regulator_sync_voltage(rails->mx);
		if (recovery)
			panic("CPR MX vote recovery failed: %d", recovery);
	}

	return ret;
}

/*
 * Establish and activate a floor covering every possible boot corner before
 * any APC transition. Repeating handoff never lowers an existing MX vote or
 * takes another enable reference. A failed enable retains the requested floor
 * for retry, but cannot authorize APC transitions. There is deliberately no
 * automatic disable: an always-on APC rail can outlive CPR domain power-off.
 */
static inline int cpr_voltage_bootstrap(struct cpr_voltage *rails, int boot_min_uV)
{
	int ret;

	if (!rails->mx)
		return 0;
	if (boot_min_uV <= 0 || boot_min_uV > rails->mx_max_uV ||
	    rails->mx_min_uV < 0 || rails->mx_min_uV > rails->mx_max_uV)
		return -EINVAL;
	if (rails->mx_min_uV < boot_min_uV) {
		ret = cpr_voltage_vote_mx(rails, boot_min_uV);
		if (ret)
			return ret;
	}
	if (rails->mx_enabled)
		return 0;

	ret = regulator_enable(rails->mx);
	if (!ret)
		rails->mx_enabled = true;

	return ret;
}

static inline int cpr_voltage_set(struct cpr_voltage *rails, int apc_uV,
				 int mx_min_uV)
{
	int ret;

	rails->mx_lower_error = 0;
	if (!rails->apc || apc_uV <= 0)
		return -EINVAL;
	if (!rails->mx)
		return regulator_set_voltage(rails->apc, apc_uV, apc_uV);

	/* Never infer an initial vote from a rounded-down boot clock corner. */
	if (!rails->mx_enabled || rails->mx_min_uV <= 0 || rails->mx_min_uV > rails->mx_max_uV ||
	    mx_min_uV <= 0 || mx_min_uV > rails->mx_max_uV ||
	    apc_uV > mx_min_uV)
		return -EINVAL;

	if (mx_min_uV > rails->mx_min_uV) {
		ret = cpr_voltage_vote_mx(rails, mx_min_uV);
		if (ret)
			return ret;
	}

	ret = regulator_set_voltage(rails->apc, apc_uV, apc_uV);
	if (ret)
		/* Keep any raised MX vote: dropping it on an APC error is unsafe. */
		return ret;

	if (mx_min_uV < rails->mx_min_uV) {
		ret = cpr_voltage_vote_mx(rails, mx_min_uV);
		if (ret)
			/* APC succeeded; keep MX high without provoking OPP rollback. */
			rails->mx_lower_error = ret;
	}

	return 0;
}

#endif
