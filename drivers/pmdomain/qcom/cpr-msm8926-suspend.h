/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_CPR_MSM8926_SUSPEND_H__
#define __QCOM_CPR_MSM8926_SUSPEND_H__

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include "cpr-voltage.h"

#define CPR_SUSPEND_SAFE_RATE	300000000UL
#define CPR_SUSPEND_CFG_MASK	(GENMASK(10, 8) | GENMASK(4, 0))
#define CPR_SUSPEND_CMD_BUSY	(BIT(0) | BIT(4) | BIT(31))

/* Protected by CPR's mutex; feedback and its IRQ are quiescent throughout. */
struct cpr_msm8926_suspend {
	struct cpr_voltage *rails;
	struct regulator *sleep_mx;
	struct clk *clock;
	struct clk *aux;
	struct clk *pll;
	struct regmap *apcs;
	int cover_apc_uv;
	unsigned long saved_rate;
	u32 saved_cfg;
	int saved_apc_uv;
	int saved_mx_uv;
	bool active;
	bool exclusive;
};

static inline int cpr_suspend_check_parents(struct cpr_msm8926_suspend *sleep)
{
	struct clk *gpll0 = clk_get_parent(sleep->aux);

	if (!gpll0 || clk_get_rate(gpll0) != 600000000 ||
	    clk_get_rate(sleep->aux) != 600000000)
		return -EIO;
	/* Refresh the PLL's noncached rate before CCF decides how to restore it. */
	clk_get_rate(sleep->pll);
	return 0;
}

static inline int cpr_suspend_check_clock(struct cpr_msm8926_suspend *sleep,
					  unsigned long rate, u32 cfg)
{
	u32 cmd, cmd_after, value, value_after;
	int ret;

	ret = cpr_suspend_check_parents(sleep);
	if (ret)
		return ret;
	if ((cfg & GENMASK(10, 8)) == (5 << 8) && !clk_get_rate(sleep->pll))
		return -EIO;
	ret = regmap_read(sleep->apcs, 0x50, &cmd);
	if (ret)
		return ret;
	ret = regmap_read(sleep->apcs, 0x54, &value);
	if (ret)
		return ret;
	ret = regmap_read(sleep->apcs, 0x50, &cmd_after);
	if (ret)
		return ret;
	ret = regmap_read(sleep->apcs, 0x54, &value_after);
	if (ret)
		return ret;
	if ((cmd | cmd_after) & CPR_SUSPEND_CMD_BUSY || value != value_after ||
	    (value & CPR_SUSPEND_CFG_MASK) != (cfg & CPR_SUSPEND_CFG_MASK) ||
	    clk_get_rate(sleep->clock) != rate)
		return -EIO;

	return 0;
}

/* Force the current consumer request through a possibly cached provider. */
static inline int cpr_suspend_sync(struct regulator *rail)
{
	int ret;

	ret = regulator_sync_voltage(rail);
	if (ret)
		ret = regulator_sync_voltage(rail);
	return ret;
}

static inline int cpr_suspend_cover(struct cpr_msm8926_suspend *sleep)
{
	struct cpr_voltage *rails = sleep->rails;
	int ret;

	ret = cpr_voltage_vote_mx(rails, rails->mx_max_uV);
	if (ret)
		return ret;
	ret = cpr_suspend_sync(rails->mx);
	if (ret)
		return ret;
	/* Retain the covering sleep ceiling; lowering it needs a separate proof. */
	ret = cpr_suspend_sync(sleep->sleep_mx);
	if (ret)
		return ret;
	ret = regulator_set_voltage(rails->apc, sleep->cover_apc_uv, sleep->cover_apc_uv);
	if (ret)
		return ret;
	return cpr_suspend_sync(rails->apc);
}

static inline void cpr_suspend_restore(struct cpr_msm8926_suspend *sleep)
{
	struct cpr_voltage *rails = sleep->rails;
	int ret, attempt;

	if (!sleep->active)
		return;

	/* Establish a physical ceiling before any possible clock increase. */
	ret = cpr_suspend_cover(sleep);
	if (ret)
		ret = cpr_suspend_cover(sleep);
	if (ret)
		panic("CPR suspend ceiling restore failed: %d", ret);

	for (attempt = 0; attempt < 2; attempt++) {
		ret = cpr_suspend_check_parents(sleep);
		if (ret)
			continue;
		clk_set_rate(sleep->clock, sleep->saved_rate);
		ret = cpr_suspend_check_clock(sleep, sleep->saved_rate, sleep->saved_cfg);
		if (!ret)
			break;
	}
	if (ret)
		panic("CPR suspend clock restore could not be verified: %d", ret);

	/* The restored clock is now covered by its original observed APC point. */
	ret = cpr_voltage_set(rails, sleep->saved_apc_uv, sleep->saved_mx_uv);
	if (ret)
		ret = cpr_voltage_set(rails, sleep->saved_apc_uv, sleep->saved_mx_uv);
	if (ret)
		panic("CPR suspend APC restore failed: %d", ret);

	/* A failed MX reduction remains high and is safe after APC restoration. */
	sleep->active = false;
	clk_rate_exclusive_put(sleep->clock);
	sleep->exclusive = false;
}

static inline int cpr_suspend_prepare(struct cpr_msm8926_suspend *sleep,
				      unsigned long rate, int min_apc_uV,
				      int max_apc_uV)
{
	struct cpr_voltage *rails = sleep->rails;
	int ret;

	if (sleep->active || sleep->exclusive || !rails || !rails->apc || !rails->mx ||
	    !rails->mx_enabled || !sleep->sleep_mx || !sleep->clock ||
	    !sleep->aux || !sleep->pll || !sleep->apcs ||
	    rails->mx_min_uV <= 0 || rails->mx_min_uV > rails->mx_max_uV ||
	    min_apc_uV <= 0 || max_apc_uV < min_apc_uV ||
	    sleep->cover_apc_uv < max_apc_uV || sleep->cover_apc_uv > rails->mx_max_uV ||
	    rate < CPR_SUSPEND_SAFE_RATE)
		return -EINVAL;
	ret = regulator_is_supported_voltage(rails->apc, sleep->cover_apc_uv,
					     sleep->cover_apc_uv);
	if (ret <= 0)
		return ret ?: -ERANGE;
	ret = clk_rate_exclusive_get(sleep->clock);
	if (ret)
		return ret;
	sleep->exclusive = true;
	sleep->saved_rate = rate;
	sleep->saved_mx_uv = rails->mx_min_uV;
	ret = regmap_read(sleep->apcs, 0x54, &sleep->saved_cfg);
	if (ret)
		goto release;
	ret = cpr_suspend_check_clock(sleep, rate, sleep->saved_cfg);
	if (ret)
		goto release;
	ret = regulator_get_voltage(rails->apc);
	if (ret < 0)
		goto release;
	if (ret < min_apc_uV || ret > max_apc_uV || ret > sleep->saved_mx_uv) {
		ret = -ERANGE;
		goto release;
	}
	sleep->saved_apc_uv = ret;
	if (regulator_get_voltage(sleep->sleep_mx) != rails->mx_max_uV) {
		ret = -ERANGE;
		goto release;
	}

	/* Mark ownership before the first fallible state change. */
	sleep->active = true;
	ret = cpr_suspend_cover(sleep);
	if (ret)
		goto restore;
	ret = cpr_suspend_check_parents(sleep);
	if (ret)
		goto restore;
	ret = clk_set_rate(sleep->clock, CPR_SUSPEND_SAFE_RATE);
	if (ret)
		goto restore;
	ret = cpr_suspend_check_clock(sleep, CPR_SUSPEND_SAFE_RATE, 4 << 8 | 3);
	if (ret)
		goto restore;
	/* CCF must re-enable the PLL on reparent, even if its context was lost. */
	if (__clk_get_enable_count(sleep->pll) ||
	    clk_hw_is_prepared(__clk_get_hw(sleep->pll))) {
		ret = -EBUSY;
		goto restore;
	}
	return 0;

restore:
	cpr_suspend_restore(sleep);
	return ret;
release:
	clk_rate_exclusive_put(sleep->clock);
	sleep->exclusive = false;
	return ret;
}

#endif
