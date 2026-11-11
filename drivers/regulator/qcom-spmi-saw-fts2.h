/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_SPMI_SAW_FTS2_H__
#define __QCOM_SPMI_SAW_FTS2_H__

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/panic.h>
#include <linux/regmap.h>

struct spmi_saw_fts2 {
	struct regmap *pmic;
	struct regmap *saw;
	unsigned int base;
	unsigned int range;
	unsigned int step_uV;
	unsigned int step_rate;
	unsigned int restore_mode;
	bool mode_pending;
	int mode_restore_error;
};

static inline int spmi_saw_fts2_init(struct spmi_saw_fts2 *state)
{
	unsigned int type, subtype, range, mode, step, avs, version, id;
	unsigned int cfg, data0, data1;
	int ret;

	ret = regmap_read(state->saw, 0xfd0, &version);
	if (ret)
		return ret;
	if ((version >> 28) != 2)
		return -ENODEV;
	ret = regmap_read(state->saw, 4, &id);
	if (ret)
		return ret;
	if (!(id & 4))
		return -ENODEV;
	ret = regmap_read(state->saw, 8, &cfg);
	if (ret)
		return ret;
	ret = regmap_read(state->saw, 0x40, &data0);
	if (ret)
		return ret;
	ret = regmap_read(state->saw, 0x44, &data1);
	if (ret)
		return ret;
	/* Madai's L2 master must be initialized by its SPM owner first. */
	if (!cfg && !data0 && !data1)
		return -EPROBE_DEFER;
	if (cfg != 0x14 || data0 != 0x02030080 || data1 != 0x00030000)
		return -EOPNOTSUPP;
	ret = regmap_read(state->pmic, state->base + 4, &type);
	if (ret)
		return ret;
	ret = regmap_read(state->pmic, state->base + 5, &subtype);
	if (ret)
		return ret;
	if (type != 0x1c || subtype != 8)
		return -ENODEV;
	ret = regmap_read(state->pmic, state->base + 0x40, &range);
	if (ret)
		return ret;
	if (range > 1)
		return -EINVAL;
	ret = regmap_read(state->pmic, state->base + 0x45, &mode);
	if (ret)
		return ret;
	if ((mode & 0xc0) != 0x80 && (mode & 0xc0) != 0x40)
		return -EOPNOTSUPP;
	ret = regmap_read(state->pmic, state->base + 0x61, &step);
	if (ret)
		return ret;
	ret = regmap_read(state->saw, 0x20, &avs);
	if (ret)
		return ret;
	/* Software CPR must own the rail; do not race active hardware AVS. */
	if (avs & 1)
		return -EOPNOTSUPP;

	state->range = range;
	state->step_uV = range ? 10000 : 5000;
	state->step_rate = 19200 * state->step_uV * (1 << ((step & 0x18) >> 3));
	state->step_rate /= 1000 * (8 << (step & 7));
	state->step_rate = max(state->step_rate * 4 / 5, 1U);
	state->mode_pending = false;
	state->mode_restore_error = 0;
	return 0;
}

static inline int
spmi_saw_fts2_command(struct spmi_saw_fts2 *state, unsigned int vctl,
		      unsigned int selector, bool *issued)
{
	unsigned int status;
	int ret, poll_ret;

	*issued = false;
	ret = regmap_write(state->saw, 0x18, 1);
	if (ret)
		return ret;
	/* A failed VCTL write may already have changed the physical rail. */
	*issued = true;
	ret = regmap_write(state->saw, 0x1c, (vctl & ~0x700ff) | selector);
	if (ret)
		return ret;
	/* SAW2 v2: acknowledged selector[7:0] and PMIC state[17:16] idle. */
	poll_ret = read_poll_timeout_atomic(regmap_read, ret,
					    ret || (status & 0x300ff) == selector, 1, 50, false,
					    state->saw, 0x14, &status);
	return ret ? ret : poll_ret;
}

static inline int
spmi_saw_fts2_verify(struct spmi_saw_fts2 *state, unsigned int selector)
{
	unsigned int range, actual;
	int ret;

	ret = regmap_read(state->pmic, state->base + 0x40, &range);
	if (ret)
		return ret;
	ret = regmap_read(state->pmic, state->base + 0x41, &actual);
	if (ret)
		return ret;
	return range == state->range && actual == selector ? 0 : -EIO;
}

static inline void spmi_saw_fts2_restore_mode(struct spmi_saw_fts2 *state)
{
	int ret;

	if (!state->mode_pending)
		return;
	/* CPU0 IPI context cannot sleep during PMIC mode settling. */
	udelay(42);
	ret = regmap_write(state->pmic, state->base + 0x45, state->restore_mode);
	state->mode_restore_error = ret;
	if (!ret)
		state->mode_pending = false;
}

/*
 * Restore the previous physical point before an error can reach OPP/genpd.
 * Their cached performance state does not change on failure. Submit directly
 * through SAW: a same-value regulator request can be skipped by core caching.
 */
static inline int
spmi_saw_fts2_recover(struct spmi_saw_fts2 *state, unsigned int vctl,
		      unsigned int mode, unsigned int old_selector,
		      unsigned int delta_uV)
{
	bool issued;
	int ret;

	if (!state->mode_pending)
		state->restore_mode = mode;
	state->mode_pending = true;
	/* Recovery may raise the rail even when the original request lowered it. */
	ret = regmap_write(state->pmic, state->base + 0x45, (mode & ~0xc0) | 0x80);
	if (ret)
		return ret;
	/* Reset cancels the uncertain command before resubmitting the old point. */
	ret = spmi_saw_fts2_command(state, vctl, old_selector, &issued);
	if (ret)
		return ret;
	udelay(DIV_ROUND_UP(delta_uV, state->step_rate));
	ret = spmi_saw_fts2_verify(state, old_selector);
	if (ret)
		return ret;
	/* Failed AUTO restoration leaves the confirmed rail in safe PWM mode. */
	spmi_saw_fts2_restore_mode(state);
	return 0;
}

/* Called synchronously on CPU0 with the regulator core's lock held. */
static inline int spmi_saw_fts2_set(struct spmi_saw_fts2 *state,
				    unsigned int range, unsigned int selector)
{
	unsigned int current_range, mode, old_selector, avs, vctl;
	unsigned int target_uV, old_uV, delta_uV;
	bool issued;
	int ret, recovery;

	if (range != state->range || selector > 255 || !state->step_rate)
		return -EINVAL;
	target_uV = selector * state->step_uV;
	if (target_uV < (range ? 700000 : 350000) ||
	    target_uV > (range ? 2040000 : 1275000))
		return -EINVAL;

	ret = regmap_read(state->pmic, state->base + 0x40, &current_range);
	if (ret)
		return ret;
	if (current_range != state->range)
		return -EIO;
	ret = regmap_read(state->pmic, state->base + 0x41, &old_selector);
	if (ret)
		return ret;
	old_uV = old_selector * state->step_uV;
	if (old_uV < (range ? 700000 : 350000) ||
	    old_uV > (range ? 2040000 : 1275000))
		return -EIO;
	delta_uV = target_uV > old_uV ? target_uV - old_uV : old_uV - target_uV;
	ret = regmap_read(state->pmic, state->base + 0x45, &mode);
	if (ret)
		return ret;
	if ((mode & 0xc0) != 0x80 && (mode & 0xc0) != 0x40)
		return -EOPNOTSUPP;
	ret = regmap_read(state->saw, 0x20, &avs);
	if (ret)
		return ret;
	if (avs & 1)
		return -EOPNOTSUPP;
	ret = regmap_read(state->saw, 0x1c, &vctl);
	if (ret)
		return ret;

	/* Force PWM for a rising transition, as in the vendor FTS2 driver. */
	if (target_uV > old_uV && (mode & 0xc0) == 0x40) {
		if (!state->mode_pending)
			state->restore_mode = mode;
		state->mode_pending = true;
		ret = regmap_write(state->pmic, state->base + 0x45,
				   (mode & ~0xc0) | 0x80);
		if (ret)
			return ret;
	}

	/* Use acknowledged port0; leave boot PMIC data slots unchanged. */
	ret = spmi_saw_fts2_command(state, vctl, selector, &issued);
	if (ret)
		goto failed;
	if (target_uV > old_uV)
		udelay(DIV_ROUND_UP(target_uV - old_uV, state->step_rate));
	ret = spmi_saw_fts2_verify(state, selector);
	if (ret)
		goto failed;
	/* Voltage succeeded; retain/retry safe PWM if AUTO restoration fails. */
	spmi_saw_fts2_restore_mode(state);
	return 0;

failed:
	if (!issued)
		return ret;
	recovery = spmi_saw_fts2_recover(state, vctl, mode, old_selector,
					 delta_uV);
	if (recovery)
		recovery = spmi_saw_fts2_recover(state, vctl, mode, old_selector,
						 delta_uV);
	if (recovery)
		/* Unknown CPU voltage cannot authorize any later clock transition. */
		panic("PM8226 APC voltage recovery failed: %d", recovery);
	return ret;
}

#endif
