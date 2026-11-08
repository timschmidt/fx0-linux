/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_CPR_MSM8926_CORNER_H__
#define __QCOM_CPR_MSM8926_CORNER_H__

#include <linux/array_size.h>

#include "cpr-msm8926-voltage.h"

struct cpr_msm8926_corner {
	u32 virtual_corner;
	u32 fuse_corner;
	u32 quotient;
	u32 ring_osc;
	int floor_uV;
	int ceiling_uV;
	int initial_uV;
	int mx_min_uV;
};

static const unsigned long cpr_msm8926_rates[] = {
	300000000, 384000000, 600000000, 787200000, 998400000,
	1094400000, 1190400000, 1305600000, 1344000000,
	1401600000, 1497600000, 1593600000, 1689600000, 1785600000,
};

/*
 * Reproduce clock-a7's first covering fmax entry, including its conservative
 * speed0/v0 fallback. Clock and CPR speed-bin selectors are intentionally
 * independent. This maps metadata only; the caller must validate the clock's
 * supported rates and activate the rail plan before installing an OPP.
 */
static inline int
cpr_msm8926_map_corner(const struct cpr_msm8926_calibration *calibration,
		       const struct cpr_msm8926_voltage_plan *plan,
		       unsigned long rate, struct cpr_msm8926_corner *corner)
{
	const unsigned long *frequency = cpr_msm8926_rates;
	struct cpr_msm8926_corner result = {};
	unsigned int maximum = 7, selected, fuse, i;
	bool extended = false;

	if (!calibration || !plan || !corner || !rate)
		return -EINVAL;
	if (calibration->clock_speed_bin > 7 || calibration->pvs_version > 3)
		return -EINVAL;
	if (rate > 787200000 &&
	    (!calibration->clock_bin_valid || !calibration->fuse_allows_loop))
		return -ERANGE;
	if (calibration->pvs_version == 1) {
		switch (calibration->clock_speed_bin) {
		case 0:
			extended = true;
			break;
		case 1:
			maximum = 12;
			extended = true;
			break;
		case 2:
			maximum = 10;
			extended = true;
			break;
		case 5:
			maximum = 14;
			extended = true;
			break;
		}
	}
	if (rate > frequency[maximum - 1])
		return -ERANGE;
	if (rate <= frequency[1])
		selected = 2;
	else if (rate <= frequency[3])
		selected = 4;
	else if (!extended)
		selected = 7;
	else {
		for (i = 4; i < maximum; i++)
			if (rate <= frequency[i])
				break;
		selected = i + 1;
	}
	fuse = selected <= 2 ? 0 : selected <= 4 ? 1 : 2;
	if (plan->floor_uV[fuse] <= 0 ||
	    plan->floor_uV[fuse] > plan->initial_uV[fuse] ||
	    plan->initial_uV[fuse] > plan->ceiling_uV[fuse] ||
	    plan->ceiling_uV[fuse] > plan->mx_min_uV[fuse])
		return -EINVAL;
	if (calibration->fuse_allows_loop &&
	    (calibration->quotient_adjust[selected - 1] >= calibration->quotient[fuse] ||
	     calibration->quotient[fuse] > 4095 || calibration->ring_osc[fuse] > 7))
		return -EINVAL;
	result.virtual_corner = selected;
	result.fuse_corner = fuse + 1;
	if (calibration->fuse_allows_loop) {
		result.quotient = calibration->quotient[fuse] -
				  calibration->quotient_adjust[selected - 1];
		result.ring_osc = calibration->ring_osc[fuse];
	}
	result.floor_uV = plan->floor_uV[fuse];
	result.ceiling_uV = plan->ceiling_uV[fuse];
	result.initial_uV = plan->initial_uV[fuse];
	result.mx_min_uV = plan->mx_min_uV[fuse];
	*corner = result;
	return 0;
}

/* One bit per canonical rate; CPU and CPR tables must use the same mask. */
static inline int
cpr_msm8926_supported_hw(const struct cpr_msm8926_calibration *calibration,
			 const struct cpr_msm8926_voltage_plan *plan, u32 *mask)
{
	struct cpr_msm8926_corner corner;
	u32 result = 0;
	unsigned int i;
	int ret;

	if (!mask)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(cpr_msm8926_rates); i++) {
		ret = cpr_msm8926_map_corner(calibration, plan, cpr_msm8926_rates[i],
					     &corner);
		if (ret == -ERANGE)
			break;
		if (ret)
			return ret;
		result |= 1U << i;
	}
	if (!result)
		return -ERANGE;
	*mask = result;
	return 0;
}

#endif
