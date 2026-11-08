/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_CPR_MSM8926_VOLTAGE_H__
#define __QCOM_CPR_MSM8926_VOLTAGE_H__

#include <linux/kernel.h>
#include <linux/regulator/consumer.h>

#include "cpr-msm8926.h"

struct cpr_msm8926_voltage_plan {
	int floor_uV[3];
	int ceiling_uV[3];
	int initial_uV[3];
	int mx_min_uV[3];
	int bootstrap_mx_uV;
};

/* Read-only provider metadata: never substitute a higher point for a hole. */
static inline int cpr_msm8926_check_voltage(struct regulator *regulator,
					  int count, int wanted_uV)
{
	bool found = false;
	int selector, voltage;

	for (selector = 0; selector < count; selector++) {
		voltage = regulator_list_voltage(regulator, selector);
		if (voltage < 0)
			return voltage;
		found |= voltage == wanted_uV;
	}
	return found ? 0 : -ERANGE;
}

/*
 * Map nominal Madai limits to the fixed FTS2 boot range and PM8226 L3 grid.
 * This computes a plan only: rail activation, votes and OPP eligibility belong
 * to the caller. Preserve the nominal global APC ceiling, and include actual
 * boot APC voltage in the initial MX floor instead of inferring it from Hz.
 */
static inline int
cpr_msm8926_plan_voltages(const struct cpr_msm8926_calibration *calibration,
			 struct regulator *apc, struct regulator *mx,
			 int boot_apc_uV, int mx_max_uV,
			 struct cpr_msm8926_voltage_plan *plan)
{
	struct cpr_msm8926_voltage_plan result = {};
	unsigned int global_max_uV = 0, step_uV;
	int apc_count, mx_count, bootstrap_apc_uV, ret;
	unsigned int i;

	if (!calibration || !apc || !mx || !plan)
		return -EINVAL;
	if (boot_apc_uV < 0)
		return boot_apc_uV;
	if (!boot_apc_uV || boot_apc_uV > 1350000 ||
	    mx_max_uV < 750000 || mx_max_uV > 1337500)
		return -EINVAL;
	for (i = 0; i < 3; i++) {
		if (!calibration->floor_uV[i] ||
		    calibration->floor_uV[i] > calibration->initial_uV[i] ||
		    calibration->initial_uV[i] > calibration->ceiling_uV[i] ||
		    calibration->ceiling_uV[i] > 1350000)
			return -EINVAL;
		global_max_uV = max(global_max_uV, calibration->ceiling_uV[i]);
	}
	step_uV = regulator_get_linear_step(apc);
	if (step_uV != 5000 && step_uV != 10000)
		return -EOPNOTSUPP;
	apc_count = regulator_count_voltages(apc);
	if (apc_count <= 0)
		return apc_count ? apc_count : -ENODEV;
	mx_count = regulator_count_voltages(mx);
	if (mx_count <= 0)
		return mx_count ? mx_count : -ENODEV;
	ret = cpr_msm8926_check_voltage(apc, apc_count, boot_apc_uV);
	if (ret)
		return ret;

	bootstrap_apc_uV = boot_apc_uV;
	for (i = 0; i < 3; i++) {
		result.floor_uV[i] = DIV_ROUND_UP(calibration->floor_uV[i], step_uV) *
				    step_uV;
		result.ceiling_uV[i] = DIV_ROUND_UP(calibration->ceiling_uV[i], step_uV) *
				      step_uV;
		result.initial_uV[i] = DIV_ROUND_UP(calibration->initial_uV[i], step_uV) *
				      step_uV;
		if ((unsigned int)result.ceiling_uV[i] > global_max_uV)
			return -ERANGE;
		ret = cpr_msm8926_check_voltage(apc, apc_count, result.floor_uV[i]);
		if (ret)
			return ret;
		ret = cpr_msm8926_check_voltage(apc, apc_count, result.ceiling_uV[i]);
		if (ret)
			return ret;
		ret = cpr_msm8926_check_voltage(apc, apc_count, result.initial_uV[i]);
		if (ret)
			return ret;
		/* L3's verified 750mV + n*12.5mV grid covers actual APC ceiling. */
		result.mx_min_uV[i] = DIV_ROUND_UP(result.ceiling_uV[i], 12500) * 12500;
		if (result.mx_min_uV[i] > mx_max_uV)
			return -ERANGE;
		ret = cpr_msm8926_check_voltage(mx, mx_count, result.mx_min_uV[i]);
		if (ret)
			return ret;
		bootstrap_apc_uV = max(bootstrap_apc_uV, result.ceiling_uV[i]);
	}
	result.bootstrap_mx_uV = DIV_ROUND_UP(bootstrap_apc_uV, 12500) * 12500;
	if (result.bootstrap_mx_uV > mx_max_uV)
		return -ERANGE;
	ret = cpr_msm8926_check_voltage(mx, mx_count, result.bootstrap_mx_uV);
	if (ret)
		return ret;
	*plan = result;
	return 0;
}

#endif
