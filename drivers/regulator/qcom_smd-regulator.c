// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, Sony Mobile Communications AB.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/soc/qcom/smd-rpm.h>

static struct qcom_smd_rpm *smd_vreg_rpm;

struct rpm_reg_vote {
	bool enabled;
	int uV;
	u32 load;
	bool load_valid;
};

/* Owned before either regulator is registered; no peer rdev is dereferenced. */
struct rpm_reg_pair {
	/* Serializes the cached votes and their synchronous RPM writes. */
	struct mutex lock;
	struct rpm_reg_vote votes[3];
	bool has_sleep_only;
	int boot_uV;
	int boot_sleep_uV;
	int boot_max_uV;
	int boot_min_uV;
	bool sleep_initialized;
	bool sleep_enable_valid;
	bool sleep_enabled;
	bool sleep_voltage_valid;
	u32 sleep_uV;
};

struct qcom_rpm_reg {
	struct device *dev;
	u32 type;
	u32 id;
	struct rpm_reg_pair *pair;
	bool active_only;
	bool sleep_only;

	struct regulator_desc desc;

	int is_enabled;
	int uV;
	u32 load;

	unsigned int enabled_updated:1;
	unsigned int uv_updated:1;
	unsigned int load_updated:1;
};

struct rpm_regulator_req {
	__le32 key;
	__le32 nbytes;
	__le32 value;
};

#define RPM_KEY_SWEN	0x6e657773 /* "swen" */
#define RPM_KEY_UV	0x00007675 /* "uv" */
#define RPM_KEY_MA	0x0000616d /* "ma" */

static int rpm_reg_send_vote(struct qcom_rpm_reg *vreg, int state,
			     const struct rpm_reg_vote *vote)
{
	struct rpm_regulator_req req[3];
	bool trace = vreg->pair && vreg->type == QCOM_SMD_RPM_LDOA && vreg->id == 3;
	int count = 1, ret;

	req[0].key = cpu_to_le32(RPM_KEY_SWEN);
	req[0].nbytes = cpu_to_le32(sizeof(u32));
	req[0].value = cpu_to_le32(vote->enabled);
	if (vote->uV > 0) {
		req[count].key = cpu_to_le32(RPM_KEY_UV);
		req[count].nbytes = cpu_to_le32(sizeof(u32));
		req[count++].value = cpu_to_le32(vote->uV);
	}
	if (vote->load_valid) {
		req[count].key = cpu_to_le32(RPM_KEY_MA);
		req[count].nbytes = cpu_to_le32(sizeof(u32));
		req[count++].value = cpu_to_le32(vote->load / 1000);
	}

	if (trace)
		dev_info(vreg->dev, "FX0_MX_VOTE_BEGIN set=%d enabled=%u uV=%d\n",
			 state, vote->enabled, vote->uV);
	ret = qcom_rpm_smd_write(smd_vreg_rpm, state, vreg->type, vreg->id,
				 req, sizeof(req[0]) * count);
	if (trace)
		dev_info(vreg->dev, "FX0_MX_VOTE_END set=%d ret=%d\n", state, ret);
	return ret;
}

static struct rpm_reg_vote rpm_reg_pair_sleep_vote(struct rpm_reg_pair *pair,
						   bool handoff)
{
	struct rpm_reg_vote vote = pair->votes[0];
	const struct rpm_reg_vote *sleep_only = &pair->votes[2];
	u64 load = (u64)vote.load + sleep_only->load;

	if (pair->sleep_enable_valid)
		vote.enabled = pair->sleep_enabled;
	if (pair->sleep_voltage_valid)
		vote.uV = pair->sleep_uV;
	vote.enabled |= sleep_only->enabled || pair->boot_sleep_uV;
	vote.uV = max(vote.uV, sleep_only->uV);
	if (!handoff)
		vote.uV = max(vote.uV, pair->boot_sleep_uV);
	vote.load = min_t(u64, load, U32_MAX);
	vote.load_valid |= sleep_only->load_valid;
	return vote;
}

static int rpm_reg_write_triple(struct qcom_rpm_reg *vreg)
{
	struct rpm_reg_pair *pair = vreg->pair;
	struct rpm_reg_vote *normal = &pair->votes[0];
	struct rpm_reg_vote *ao = &pair->votes[1];
	struct rpm_reg_vote *so = &pair->votes[2];
	unsigned int index = vreg->sleep_only ? 2 : vreg->active_only;
	struct rpm_reg_vote previous, active, sleep;
	int previous_boot_sleep, ret = 0;
	bool initialize_sleep, active_handoff, sleep_handoff;
	u64 load;

	mutex_lock(&pair->lock);
	previous = pair->votes[index];
	previous_boot_sleep = pair->boot_sleep_uV;
	pair->votes[index] = (struct rpm_reg_vote) {
		.enabled = vreg->is_enabled,
		.uV = vreg->uV,
		.load = vreg->load,
		.load_valid = previous.load_valid || vreg->load_updated,
	};
	/* Releasing a CPU sleep voter must not remove its live-APC protection. */
	if (vreg->sleep_only && !so->enabled)
		pair->boot_sleep_uV = pair->boot_max_uV;
	if (!normal->enabled && !ao->enabled && !so->enabled &&
	    !vreg->enabled_updated && !pair->boot_uV && !pair->boot_sleep_uV) {
		if (!vreg->active_only)
			pair->sleep_initialized = false;
		goto unlock;
	}

	sleep_handoff = vreg->sleep_only && so->enabled && so->uV >= pair->boot_min_uV;
	sleep = rpm_reg_pair_sleep_vote(pair, sleep_handoff);
	if (vreg->sleep_only) {
		ret = rpm_reg_send_vote(vreg, QCOM_SMD_RPM_SLEEP_STATE, &sleep);
		if (ret)
			goto restore;
		pair->sleep_initialized = true;
		if (sleep_handoff)
			pair->boot_sleep_uV = 0;
		goto committed;
	}

	initialize_sleep = !pair->sleep_initialized;
	if (initialize_sleep) {
		ret = rpm_reg_send_vote(vreg, QCOM_SMD_RPM_SLEEP_STATE, &sleep);
		if (ret)
			goto restore;
		pair->sleep_initialized = true;
	}
	load = (u64)normal->load + ao->load;
	active_handoff = ao->enabled && ao->uV >= pair->boot_min_uV;
	active = (struct rpm_reg_vote) {
		.enabled = normal->enabled || ao->enabled || pair->boot_uV,
		.uV = max(normal->uV, ao->uV),
		.load = min_t(u64, load, U32_MAX),
		.load_valid = normal->load_valid || ao->load_valid,
	};
	if (!active_handoff)
		active.uV = max(active.uV, pair->boot_uV);
	ret = rpm_reg_send_vote(vreg, QCOM_SMD_RPM_ACTIVE_STATE, &active);
	if (ret)
		goto restore;
	if (!vreg->active_only && !initialize_sleep) {
		ret = rpm_reg_send_vote(vreg, QCOM_SMD_RPM_SLEEP_STATE, &sleep);
		if (ret)
			goto restore;
	}
	if (active_handoff)
		pair->boot_uV = 0;
committed:
	vreg->enabled_updated = 0;
	vreg->uv_updated = 0;
	vreg->load_updated = 0;
	goto unlock;
restore:
	pair->votes[index] = previous;
	/* Keep a rearmed boot hold even if its recovery request was uncertain. */
	pair->boot_sleep_uV = max(previous_boot_sleep, pair->boot_sleep_uV);
	pair->sleep_initialized = false;
unlock:
	mutex_unlock(&pair->lock);
	return ret;
}

static int rpm_reg_write_pair(struct qcom_rpm_reg *vreg)
{
	struct rpm_reg_pair *pair = vreg->pair;
	struct rpm_reg_vote *normal = &pair->votes[0];
	struct rpm_reg_vote *active_only = &pair->votes[1];
	struct rpm_reg_vote previous, active, sleep;
	bool initialize_sleep;
	bool boot_handoff;
	u64 load;
	int ret = 0;

	if (pair->has_sleep_only)
		return rpm_reg_write_triple(vreg);

	mutex_lock(&pair->lock);
	previous = pair->votes[vreg->active_only];
	pair->votes[vreg->active_only] = (struct rpm_reg_vote) {
		.enabled = vreg->is_enabled,
		.uV = vreg->uV,
		.load = vreg->load,
		.load_valid = previous.load_valid || vreg->load_updated,
	};
	/* Keep disabled voltage requests for the next enable, as the vendor does. */
	if (!normal->enabled && !active_only->enabled && !vreg->enabled_updated &&
	    !pair->boot_uV) {
		if (!vreg->active_only)
			pair->sleep_initialized = false;
		goto unlock;
	}

	sleep = *normal;
	if (pair->sleep_enable_valid)
		sleep.enabled = pair->sleep_enabled;
	if (pair->sleep_voltage_valid)
		sleep.uV = pair->sleep_uV;
	initialize_sleep = !pair->sleep_initialized;
	if (initialize_sleep) {
		/* RPM otherwise inherits ACTIVE, including the active-only vote. */
		ret = rpm_reg_send_vote(vreg, QCOM_SMD_RPM_SLEEP_STATE, &sleep);
		if (ret)
			goto restore;
		pair->sleep_initialized = true;
	}

	load = (u64)normal->load + active_only->load;
	boot_handoff = active_only->enabled &&
		       active_only->uV >= pair->boot_min_uV;
	active = (struct rpm_reg_vote) {
		.enabled = normal->enabled || active_only->enabled || pair->boot_uV,
		.uV = max(normal->uV, active_only->uV),
		.load = min_t(u64, load, U32_MAX),
		.load_valid = normal->load_valid || active_only->load_valid,
	};
	if (!boot_handoff)
		active.uV = max(active.uV, pair->boot_uV);
	ret = rpm_reg_send_vote(vreg, QCOM_SMD_RPM_ACTIVE_STATE, &active);
	if (ret)
		goto restore;

	/* Active-only requests never change the normal voter's sleep policy. */
	if (!vreg->active_only && !initialize_sleep) {
		ret = rpm_reg_send_vote(vreg, QCOM_SMD_RPM_SLEEP_STATE, &sleep);
		if (ret)
			goto restore;
	}
	vreg->enabled_updated = 0;
	vreg->uv_updated = 0;
	vreg->load_updated = 0;
	if (boot_handoff)
		pair->boot_uV = 0;
	goto unlock;

restore:
	/* Callers restore their cache and can force a fresh acknowledged vote. */
	pair->votes[vreg->active_only] = previous;
	/* A partial or timed-out write may have changed the firmware sleep set. */
	pair->sleep_initialized = false;
unlock:
	mutex_unlock(&pair->lock);
	return ret;
}

static int rpm_reg_write_active(struct qcom_rpm_reg *vreg)
{
	struct rpm_regulator_req req[3];
	int reqlen = 0;
	int ret;

	if (vreg->pair)
		return rpm_reg_write_pair(vreg);

	if (vreg->enabled_updated) {
		req[reqlen].key = cpu_to_le32(RPM_KEY_SWEN);
		req[reqlen].nbytes = cpu_to_le32(sizeof(u32));
		req[reqlen].value = cpu_to_le32(vreg->is_enabled);
		reqlen++;
	}

	if (vreg->uv_updated && vreg->is_enabled) {
		req[reqlen].key = cpu_to_le32(RPM_KEY_UV);
		req[reqlen].nbytes = cpu_to_le32(sizeof(u32));
		req[reqlen].value = cpu_to_le32(vreg->uV);
		reqlen++;
	}

	if (vreg->load_updated && vreg->is_enabled) {
		req[reqlen].key = cpu_to_le32(RPM_KEY_MA);
		req[reqlen].nbytes = cpu_to_le32(sizeof(u32));
		req[reqlen].value = cpu_to_le32(vreg->load / 1000);
		reqlen++;
	}

	if (!reqlen)
		return 0;

	ret = qcom_rpm_smd_write(smd_vreg_rpm, QCOM_SMD_RPM_ACTIVE_STATE,
				 vreg->type, vreg->id,
				 req, sizeof(req[0]) * reqlen);
	if (!ret) {
		vreg->enabled_updated = 0;
		vreg->uv_updated = 0;
		vreg->load_updated = 0;
	}

	return ret;
}

static int rpm_reg_enable(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	int old_enabled = vreg->is_enabled;
	int ret;

	vreg->is_enabled = 1;
	vreg->enabled_updated = 1;

	ret = rpm_reg_write_active(vreg);
	if (ret)
		vreg->is_enabled = old_enabled;

	return ret;
}

static int rpm_reg_is_enabled(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);

	return vreg->is_enabled;
}

static int rpm_reg_disable(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	int old_enabled = vreg->is_enabled;
	int ret;

	vreg->is_enabled = 0;
	vreg->enabled_updated = 1;

	ret = rpm_reg_write_active(vreg);
	if (ret)
		vreg->is_enabled = old_enabled;

	return ret;
}

static int rpm_reg_get_voltage(struct regulator_dev *rdev)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);

	return vreg->uV;
}

static int rpm_reg_set_voltage(struct regulator_dev *rdev,
			       int min_uV,
			       int max_uV,
			       unsigned *selector)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	int ret;
	int old_uV = vreg->uV;

	vreg->uV = min_uV;
	vreg->uv_updated = 1;

	ret = rpm_reg_write_active(vreg);
	if (ret)
		vreg->uV = old_uV;

	return ret;
}

static int rpm_reg_set_load(struct regulator_dev *rdev, int load_uA)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	u32 old_load = vreg->load;
	int ret;

	vreg->load = load_uA;
	vreg->load_updated = 1;
	ret = rpm_reg_write_active(vreg);
	if (ret)
		vreg->load = old_load;

	return ret;
}

static int rpm_reg_write_suspend_triple(struct qcom_rpm_reg *vreg, u32 key,
					u32 value)
{
	struct rpm_reg_pair *pair = vreg->pair;
	struct rpm_reg_vote sleep;
	bool enable_valid, enabled, voltage_valid;
	u32 uV;
	int ret;

	mutex_lock(&pair->lock);
	enable_valid = pair->sleep_enable_valid;
	enabled = pair->sleep_enabled;
	voltage_valid = pair->sleep_voltage_valid;
	uV = pair->sleep_uV;
	if (key == RPM_KEY_SWEN) {
		pair->sleep_enable_valid = true;
		pair->sleep_enabled = value;
	} else if (key == RPM_KEY_UV) {
		pair->sleep_voltage_valid = true;
		pair->sleep_uV = value;
	}
	sleep = rpm_reg_pair_sleep_vote(pair, false);
	ret = rpm_reg_send_vote(vreg, QCOM_SMD_RPM_SLEEP_STATE, &sleep);
	if (ret) {
		pair->sleep_enable_valid = enable_valid;
		pair->sleep_enabled = enabled;
		pair->sleep_voltage_valid = voltage_valid;
		pair->sleep_uV = uV;
	}
	pair->sleep_initialized = !ret;
	mutex_unlock(&pair->lock);
	return ret;
}

static int rpm_reg_write_suspend(struct regulator_dev *rdev, u32 key, u32 value)
{
	struct qcom_rpm_reg *vreg = rdev_get_drvdata(rdev);
	struct rpm_regulator_req req = {
		.key = cpu_to_le32(key),
		.nbytes = cpu_to_le32(sizeof(u32)),
		.value = cpu_to_le32(value),
	};

	int ret;

	if (!vreg->pair)
		return qcom_rpm_smd_write(smd_vreg_rpm, QCOM_SMD_RPM_SLEEP_STATE,
					 vreg->type, vreg->id, &req, sizeof(req));
	if (vreg->active_only || vreg->sleep_only)
		return -EOPNOTSUPP;
	if (vreg->pair->has_sleep_only)
		return rpm_reg_write_suspend_triple(vreg, key, value);

	mutex_lock(&vreg->pair->lock);
	ret = qcom_rpm_smd_write(smd_vreg_rpm, QCOM_SMD_RPM_SLEEP_STATE,
				 vreg->type, vreg->id, &req, sizeof(req));
	if (!ret && key == RPM_KEY_SWEN) {
		vreg->pair->sleep_enable_valid = true;
		vreg->pair->sleep_enabled = value;
	} else if (!ret && key == RPM_KEY_UV) {
		vreg->pair->sleep_voltage_valid = true;
		vreg->pair->sleep_uV = value;
	}
	if (ret)
		vreg->pair->sleep_initialized = false;
	mutex_unlock(&vreg->pair->lock);
	return ret;
}

static int rpm_reg_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	return rpm_reg_write_suspend(rdev, RPM_KEY_UV, uV);
}

static int rpm_reg_set_suspend_enable(struct regulator_dev *rdev)
{
	return rpm_reg_write_suspend(rdev, RPM_KEY_SWEN, 1);
}

static int rpm_reg_set_suspend_disable(struct regulator_dev *rdev)
{
	return rpm_reg_write_suspend(rdev, RPM_KEY_SWEN, 0);
}

static const struct regulator_ops rpm_smps_ldo_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,
	.list_voltage = regulator_list_voltage_linear_range,

	.get_voltage = rpm_reg_get_voltage,
	.set_voltage = rpm_reg_set_voltage,

	.set_load = rpm_reg_set_load,
	.set_suspend_voltage = rpm_reg_set_suspend_voltage,
	.set_suspend_enable = rpm_reg_set_suspend_enable,
	.set_suspend_disable = rpm_reg_set_suspend_disable,
};

static const struct regulator_ops rpm_smps_ldo_ops_fixed = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,

	.get_voltage = rpm_reg_get_voltage,
	.set_voltage = rpm_reg_set_voltage,

	.set_load = rpm_reg_set_load,
	.set_suspend_voltage = rpm_reg_set_suspend_voltage,
	.set_suspend_enable = rpm_reg_set_suspend_enable,
	.set_suspend_disable = rpm_reg_set_suspend_disable,
};

static const struct regulator_ops rpm_switch_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,
	.set_suspend_enable = rpm_reg_set_suspend_enable,
	.set_suspend_disable = rpm_reg_set_suspend_disable,
};

static const struct regulator_ops rpm_bob_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,

	.get_voltage = rpm_reg_get_voltage,
	.set_voltage = rpm_reg_set_voltage,
	.set_suspend_voltage = rpm_reg_set_suspend_voltage,
	.set_suspend_enable = rpm_reg_set_suspend_enable,
	.set_suspend_disable = rpm_reg_set_suspend_disable,
};

static const struct regulator_ops rpm_mp5496_ops = {
	.enable = rpm_reg_enable,
	.disable = rpm_reg_disable,
	.is_enabled = rpm_reg_is_enabled,
	.list_voltage = regulator_list_voltage_linear_range,

	.get_voltage = rpm_reg_get_voltage,
	.set_voltage = rpm_reg_set_voltage,
	.set_suspend_voltage = rpm_reg_set_suspend_voltage,
	.set_suspend_enable = rpm_reg_set_suspend_enable,
	.set_suspend_disable = rpm_reg_set_suspend_disable,
};

static const struct regulator_desc pma8084_hfsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(375000,  0,  95, 12500),
		REGULATOR_LINEAR_RANGE(1550000, 96, 158, 25000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 159,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pma8084_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(350000,  0, 184, 5000),
		REGULATOR_LINEAR_RANGE(1280000, 185, 261, 10000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 262,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pma8084_pldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE( 750000,  0,  63, 12500),
		REGULATOR_LINEAR_RANGE(1550000, 64, 126, 25000),
		REGULATOR_LINEAR_RANGE(3100000, 127, 163, 50000),
	},
	.n_linear_ranges = 3,
	.n_voltages = 164,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pma8084_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(750000, 0, 63, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 64,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pma8084_switch = {
	.ops = &rpm_switch_ops,
};

static const struct regulator_desc pm8226_hfsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(375000,   0,  95, 12500),
		REGULATOR_LINEAR_RANGE(1575000, 96, 158, 25000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 159,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8226_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(350000,    0, 184,  5000),
		REGULATOR_LINEAR_RANGE(1280000, 185, 261, 10000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 262,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8226_pldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(750000,    0,  63, 12500),
		REGULATOR_LINEAR_RANGE(1550000,  64, 126, 25000),
		REGULATOR_LINEAR_RANGE(3100000, 127, 163, 50000),
	},
	.n_linear_ranges = 3,
	.n_voltages = 164,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8226_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(750000, 0, 63, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 64,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8226_switch = {
	.ops = &rpm_switch_ops,
};

static const struct regulator_desc pm8x41_hfsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE( 375000,  0,  95, 12500),
		REGULATOR_LINEAR_RANGE(1575000, 96, 158, 25000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 159,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8841_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(350000,  0, 184, 5000),
		REGULATOR_LINEAR_RANGE(1280000, 185, 261, 10000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 262,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8941_boost = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(4000000, 0, 30, 50000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 31,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8941_pldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE( 750000,  0,  63, 12500),
		REGULATOR_LINEAR_RANGE(1550000, 64, 126, 25000),
		REGULATOR_LINEAR_RANGE(3100000, 127, 163, 50000),
	},
	.n_linear_ranges = 3,
	.n_voltages = 164,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8941_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(750000, 0, 63, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 64,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8941_lnldo = {
	.fixed_uV = 1740000,
	.n_voltages = 1,
	.ops = &rpm_smps_ldo_ops_fixed,
};

static const struct regulator_desc pm8941_switch = {
	.ops = &rpm_switch_ops,
};

static const struct regulator_desc pm8916_pldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1750000, 0, 127, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 128,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8916_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(375000, 0, 93, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 94,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8916_buck_lvo_smps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(375000, 0, 95, 12500),
		REGULATOR_LINEAR_RANGE(750000, 96, 127, 25000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 128,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8916_buck_hvo_smps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1550000, 0, 31, 25000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 32,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8950_hfsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(375000, 0, 95, 12500),
		REGULATOR_LINEAR_RANGE(1550000, 96, 127, 25000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 128,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8950_ftsmps2p5 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(80000, 0, 255, 5000),
		REGULATOR_LINEAR_RANGE(160000, 256, 460, 10000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 461,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8950_ult_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(375000, 0, 202, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 203,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8950_ult_pldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1750000, 0, 127, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 128,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8950_pldo_lv = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1500000, 0, 16, 25000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 17,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8950_pldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(975000, 0, 164, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 165,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8953_lnldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(690000, 0, 7, 60000),
		REGULATOR_LINEAR_RANGE(1380000, 8, 15, 120000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 16,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8953_ult_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(375000, 0, 93, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 94,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8994_hfsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE( 375000,  0,  95, 12500),
		REGULATOR_LINEAR_RANGE(1550000, 96, 158, 25000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 159,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8994_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(350000,  0, 199, 5000),
		REGULATOR_LINEAR_RANGE(700000, 200, 349, 10000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 350,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8994_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(750000, 0, 63, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 64,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8994_pldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE( 750000,  0,  63, 12500),
		REGULATOR_LINEAR_RANGE(1550000, 64, 126, 25000),
		REGULATOR_LINEAR_RANGE(3100000, 127, 163, 50000),
	},
	.n_linear_ranges = 3,
	.n_voltages = 164,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8994_switch = {
	.ops = &rpm_switch_ops,
};

static const struct regulator_desc pm8994_lnldo = {
	.fixed_uV = 1740000,
	.n_voltages = 1,
	.ops = &rpm_smps_ldo_ops_fixed,
};

static const struct regulator_desc pmi8994_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(350000,  0, 199, 5000),
		REGULATOR_LINEAR_RANGE(700000, 200, 349, 10000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 350,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pmi8994_hfsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(350000,  0,  80, 12500),
		REGULATOR_LINEAR_RANGE(700000, 81, 141, 25000),
	},
	.n_linear_ranges = 2,
	.n_voltages = 142,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pmi8994_bby = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(3000000, 0, 44, 50000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 45,
	.ops = &rpm_bob_ops,
};

static const struct regulator_desc pm8998_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(320000, 0, 258, 4000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 259,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8998_hfsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(320000, 0, 215, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 216,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8998_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(312000, 0, 127, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 128,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8998_pldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1664000, 0, 255, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 256,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8998_pldo_lv = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1256000, 0, 127, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 128,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm8998_switch = {
	.ops = &rpm_switch_ops,
};

static const struct regulator_desc pmi8998_bob = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1824000, 0, 83, 32000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 84,
	.ops = &rpm_bob_ops,
};

static const struct regulator_desc pm660_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(355000, 0, 199, 5000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 200,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm660_hfsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(320000, 0, 216, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 217,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm660_ht_nldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(312000, 0, 124, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 125,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm660_ht_lvpldo = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1504000, 0, 62, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 63,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm660_nldo660 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(320000, 0, 123, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 124,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm660_pldo660 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1504000, 0, 255, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 256,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm660l_bob = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1800000, 0, 84, 32000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 85,
	.ops = &rpm_bob_ops,
};

static const struct regulator_desc pm6125_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(300000, 0, 268, 4000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 269,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pmic5_ftsmps520 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(300000, 0, 263, 4000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 264,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pmic5_hfsmps515 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(320000, 0, 235, 16000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 236,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pms405_hfsmps3 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(320000, 0, 215, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 216,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pms405_nldo300 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(312000, 0, 127, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 128,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pms405_nldo1200 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(312000, 0, 127, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 128,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pms405_pldo50 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1664000, 0, 128, 16000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 129,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pms405_pldo150 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1664000, 0, 128, 16000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 129,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pms405_pldo600 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(1256000, 0, 98, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 99,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc mp5496_smps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(600000, 0, 127, 12500),
	},
	.n_linear_ranges = 1,
	.n_voltages = 128,
	.ops = &rpm_mp5496_ops,
};

static const struct regulator_desc mp5496_ldoa2 = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(800000, 0, 127, 25000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 128,
	.ops = &rpm_mp5496_ops,
};

static const struct regulator_desc pm2250_lvftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(320000, 0, 269, 4000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 270,
	.ops = &rpm_smps_ldo_ops,
};

static const struct regulator_desc pm2250_ftsmps = {
	.linear_ranges = (struct linear_range[]) {
		REGULATOR_LINEAR_RANGE(640000, 0, 269, 8000),
	},
	.n_linear_ranges = 1,
	.n_voltages = 270,
	.ops = &rpm_smps_ldo_ops,
};

struct rpm_regulator_data {
	const char *name;
	u32 type;
	u32 id;
	const struct regulator_desc *desc;
	const char *supply;
	bool active_only;
	bool sleep_only;
};

static const struct rpm_regulator_data rpm_mp5496_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &mp5496_smps, "s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &mp5496_smps, "s2" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &mp5496_ldoa2, "l2" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &mp5496_ldoa2, "l5" },
	{}
};

static const struct rpm_regulator_data rpm_pm2250_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm2250_lvftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm2250_lvftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm2250_lvftsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm2250_ftsmps, "vdd_s4" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm660_pldo660, "vdd_l4_l17_l18_l19_l20_l21_l22" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm660_nldo660, "vdd_l1_l2_l3_l5_l6_l7_l8_l9_l10_l11_l12" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm660_ht_lvpldo, "vdd_l13_l14_l15_l16" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm660_ht_lvpldo, "vdd_l13_l14_l15_l16" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm660_ht_lvpldo, "vdd_l13_l14_l15_l16" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm660_ht_lvpldo, "vdd_l13_l14_l15_l16" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm660_pldo660, "vdd_l4_l17_l18_l19_l20_l21_l22" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm660_pldo660, "vdd_l4_l17_l18_l19_l20_l21_l22" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm660_pldo660, "vdd_l4_l17_l18_l19_l20_l21_l22" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm660_pldo660, "vdd_l4_l17_l18_l19_l20_l21_l22" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm660_pldo660, "vdd_l4_l17_l18_l19_l20_l21_l22" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm660_pldo660, "vdd_l4_l17_l18_l19_l20_l21_l22" },
	{}
};

static const struct rpm_regulator_data rpm_pm6125_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm6125_ftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm6125_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm6125_ftsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm6125_ftsmps, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPA, 5, &pm8998_hfsmps, "vdd_s5" },
	{ "s6", QCOM_SMD_RPM_SMPA, 6, &pm8998_hfsmps, "vdd_s6" },
	{ "s7", QCOM_SMD_RPM_SMPA, 7, &pm8998_hfsmps, "vdd_s7" },
	{ "s8", QCOM_SMD_RPM_SMPA, 8, &pm6125_ftsmps, "vdd_s8" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm660_nldo660, "vdd_l1_l7_l17_l18" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm660_nldo660, "vdd_l2_l3_l4" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm660_nldo660, "vdd_l2_l3_l4" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm660_nldo660, "vdd_l2_l3_l4" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm660_nldo660, "vdd_l6_l8" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm660_nldo660, "vdd_l1_l7_l17_l18" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm660_nldo660, "vdd_l6_l8" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm660_ht_lvpldo, "vdd_l9_l11" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm660_ht_lvpldo, "vdd_l10_l13_l14" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm660_ht_lvpldo, "vdd_l9_l11" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm660_ht_lvpldo, "vdd_l12_l16" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm660_ht_lvpldo, "vdd_l10_l13_l14" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm660_ht_lvpldo, "vdd_l10_l13_l14" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm660_ht_lvpldo, "vdd_l12_l16" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm660_nldo660, "vdd_l1_l7_l17_l18" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm660_nldo660, "vdd_l1_l7_l17_l18" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm660_pldo660, "vdd_l5_l15_l19_l20_l21_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm660_pldo660, "vdd_l23_l24" },
	{ "l24", QCOM_SMD_RPM_LDOA, 24, &pm660_pldo660, "vdd_l23_l24" },
	{ }
};

static const struct rpm_regulator_data rpm_pm660_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm660_ftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm660_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm660_ftsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm660_hfsmps, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPA, 5, &pm660_hfsmps, "vdd_s5" },
	{ "s6", QCOM_SMD_RPM_SMPA, 6, &pm660_hfsmps, "vdd_s6" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm660_nldo660, "vdd_l1_l6_l7" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm660_ht_nldo, "vdd_l2_l3" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm660_nldo660, "vdd_l2_l3" },
	/* l4 is unaccessible on PM660 */
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm660_ht_nldo, "vdd_l5" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm660_ht_nldo, "vdd_l1_l6_l7" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm660_ht_nldo, "vdd_l1_l6_l7" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm660_ht_lvpldo, "vdd_l8_l9_l10_l11_l12_l13_l14" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm660_ht_lvpldo, "vdd_l8_l9_l10_l11_l12_l13_l14" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm660_ht_lvpldo, "vdd_l8_l9_l10_l11_l12_l13_l14" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm660_ht_lvpldo, "vdd_l8_l9_l10_l11_l12_l13_l14" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm660_ht_lvpldo, "vdd_l8_l9_l10_l11_l12_l13_l14" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm660_ht_lvpldo, "vdd_l8_l9_l10_l11_l12_l13_l14" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm660_ht_lvpldo, "vdd_l8_l9_l10_l11_l12_l13_l14" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm660_pldo660, "vdd_l15_l16_l17_l18_l19" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm660_pldo660, "vdd_l15_l16_l17_l18_l19" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm660_pldo660, "vdd_l15_l16_l17_l18_l19" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm660_pldo660, "vdd_l15_l16_l17_l18_l19" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm660_pldo660, "vdd_l15_l16_l17_l18_l19" },
	{ }
};

static const struct rpm_regulator_data rpm_pm660l_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPB, 1, &pm660_ftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPB, 2, &pm660_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_RWCX, 0, &pm660_ftsmps, "vdd_s3_s4" },
	{ "s5", QCOM_SMD_RPM_RWMX, 0, &pm660_ftsmps, "vdd_s5" },
	{ "l1", QCOM_SMD_RPM_LDOB, 1, &pm660_nldo660, "vdd_l1_l9_l10" },
	{ "l2", QCOM_SMD_RPM_LDOB, 2, &pm660_pldo660, "vdd_l2" },
	{ "l3", QCOM_SMD_RPM_LDOB, 3, &pm660_pldo660, "vdd_l3_l5_l7_l8" },
	{ "l4", QCOM_SMD_RPM_LDOB, 4, &pm660_pldo660, "vdd_l4_l6" },
	{ "l5", QCOM_SMD_RPM_LDOB, 5, &pm660_pldo660, "vdd_l3_l5_l7_l8" },
	{ "l6", QCOM_SMD_RPM_LDOB, 6, &pm660_pldo660, "vdd_l4_l6" },
	{ "l7", QCOM_SMD_RPM_LDOB, 7, &pm660_pldo660, "vdd_l3_l5_l7_l8" },
	{ "l8", QCOM_SMD_RPM_LDOB, 8, &pm660_pldo660, "vdd_l3_l5_l7_l8" },
	{ "l9", QCOM_SMD_RPM_RWLC, 0, &pm660_ht_nldo, "vdd_l1_l9_l10" },
	{ "l10", QCOM_SMD_RPM_RWLM, 0, &pm660_ht_nldo, "vdd_l1_l9_l10" },
	{ "bob", QCOM_SMD_RPM_BOBB, 1, &pm660l_bob, "vdd_bob", },
	{ }
};

static const struct rpm_regulator_data rpm_pm8226_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm8226_hfsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm8226_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm8226_hfsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm8226_hfsmps, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPA, 5, &pm8226_hfsmps, "vdd_s5" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm8226_nldo, "vdd_l1_l2_l4_l5" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm8226_nldo, "vdd_l1_l2_l4_l5" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm8226_nldo, "vdd_l3_l24_l26" },
	{ "l3-ao", QCOM_SMD_RPM_LDOA, 3, &pm8226_nldo, "vdd_l3_l24_l26", true },
	{ "l3-so", QCOM_SMD_RPM_LDOA, 3, &pm8226_nldo, "vdd_l3_l24_l26", false, true },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm8226_nldo, "vdd_l1_l2_l4_l5" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm8226_nldo, "vdd_l1_l2_l4_l5" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm8226_pldo, "vdd_l6_l7_l8_l9_l27" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm8226_pldo, "vdd_l6_l7_l8_l9_l27" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm8226_pldo, "vdd_l6_l7_l8_l9_l27" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm8226_pldo, "vdd_l6_l7_l8_l9_l27" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8226_pldo, "vdd_l10_l11_l13" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8226_pldo, "vdd_l10_l11_l13" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8226_pldo, "vdd_l12_l14" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8226_pldo, "vdd_l10_l11_l13" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8226_pldo, "vdd_l12_l14" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8226_pldo, "vdd_l15_l16_l17_l18" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm8226_pldo, "vdd_l15_l16_l17_l18" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8226_pldo, "vdd_l15_l16_l17_l18" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm8226_pldo, "vdd_l15_l16_l17_l18" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm8226_pldo, "vdd_l19_l20_l21_l22_l23_l28" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm8226_pldo, "vdd_l19_l20_l21_l22_l23_l28" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm8226_pldo, "vdd_l19_l20_l21_l22_l23_l28" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm8226_pldo, "vdd_l19_l20_l21_l22_l23_l28" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm8226_pldo, "vdd_l19_l20_l21_l22_l23_l28" },
	{ "l24", QCOM_SMD_RPM_LDOA, 24, &pm8226_nldo, "vdd_l3_l24_l26" },
	{ "l25", QCOM_SMD_RPM_LDOA, 25, &pm8226_pldo, "vdd_l25" },
	{ "l26", QCOM_SMD_RPM_LDOA, 26, &pm8226_nldo, "vdd_l3_l24_l26" },
	{ "l27", QCOM_SMD_RPM_LDOA, 27, &pm8226_pldo, "vdd_l6_l7_l8_l9_l27" },
	{ "l28", QCOM_SMD_RPM_LDOA, 28, &pm8226_pldo, "vdd_l19_l20_l21_l22_l23_l28" },
	{ "lvs1", QCOM_SMD_RPM_VSA, 1, &pm8226_switch, "vdd_lvs1" },
	{}
};

static const struct rpm_regulator_data rpm_pm8841_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPB, 1, &pm8x41_hfsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPB, 2, &pm8841_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPB, 3, &pm8x41_hfsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPB, 4, &pm8841_ftsmps, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPB, 5, &pm8841_ftsmps, "vdd_s5" },
	{ "s6", QCOM_SMD_RPM_SMPB, 6, &pm8841_ftsmps, "vdd_s6" },
	{ "s7", QCOM_SMD_RPM_SMPB, 7, &pm8841_ftsmps, "vdd_s7" },
	{ "s8", QCOM_SMD_RPM_SMPB, 8, &pm8841_ftsmps, "vdd_s8" },
	{}
};

static const struct rpm_regulator_data rpm_pm8909_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm8916_buck_lvo_smps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm8916_buck_hvo_smps, "vdd_s2" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm8916_nldo, "vdd_l1" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm8916_nldo, "vdd_l2_l5" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm8916_nldo, "vdd_l3_l6_l10" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm8916_pldo, "vdd_l4_l7" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm8226_pldo, "vdd_l2_l5" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm8226_pldo, "vdd_l3_l6_l10" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm8226_pldo, "vdd_l4_l7" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm8916_pldo, "vdd_l8_l11_l15_l18" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm8916_pldo, "vdd_l9_l12_l14_l17" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8916_nldo, "vdd_l3_l6_l10" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8226_pldo, "vdd_l8_l11_l15_l18" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8916_pldo, "vdd_l9_l12_l14_l17" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8916_pldo, "vdd_l13" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8916_pldo, "vdd_l9_l12_l14_l17" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8916_pldo, "vdd_l8_l11_l15_l18" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8916_pldo, "vdd_l9_l12_l14_l17" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm8916_pldo, "vdd_l8_l11_l15_l18" },
	{}
};

static const struct rpm_regulator_data rpm_pm8916_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm8916_buck_lvo_smps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm8916_buck_lvo_smps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm8916_buck_lvo_smps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm8916_buck_hvo_smps, "vdd_s4" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm8916_nldo, "vdd_l1_l2_l3" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm8916_nldo, "vdd_l1_l2_l3" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm8916_nldo, "vdd_l1_l2_l3" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm8916_pldo, "vdd_l4_l5_l6" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm8916_pldo, "vdd_l4_l5_l6" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm8916_pldo, "vdd_l4_l5_l6" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm8916_pldo, "vdd_l7" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm8916_pldo, "vdd_l8_l9_l10_l11_l12_l13_l14_l15_l16_l17_l18"},
	{}
};

static const struct rpm_regulator_data rpm_pm8937_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm8994_hfsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm8994_hfsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm8994_hfsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm8994_hfsmps, "vdd_s4" },
	/* S5 - S6 are managed by SPMI */

	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm8953_ult_nldo, "vdd_l1_l19" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm8953_ult_nldo, "vdd_l2_l23" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm8953_ult_nldo, "vdd_l3" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18"},
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm8953_ult_nldo, "vdd_l1_l19" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm8953_lnldo, "vdd_l20_l21" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm8953_lnldo, "vdd_l20_l21" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm8994_nldo, "vdd_l2_l23" },
	{}
};

static const struct rpm_regulator_data rpm_pm8941_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm8x41_hfsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm8x41_hfsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm8x41_hfsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_BOOST, 1, &pm8941_boost },

	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm8941_nldo, "vdd_l1_l3" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm8941_nldo, "vdd_l2_lvs1_2_3" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm8941_nldo, "vdd_l1_l3" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm8941_nldo, "vdd_l4_l11" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm8941_lnldo, "vdd_l5_l7" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm8941_pldo, "vdd_l6_l12_l14_l15" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm8941_lnldo, "vdd_l5_l7" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm8941_pldo, "vdd_l8_l16_l18_l19" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm8941_pldo, "vdd_l9_l10_l17_l22" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8941_pldo, "vdd_l9_l10_l17_l22" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8941_nldo, "vdd_l4_l11" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8941_pldo, "vdd_l6_l12_l14_l15" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8941_pldo, "vdd_l13_l20_l23_l24" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8941_pldo, "vdd_l6_l12_l14_l15" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8941_pldo, "vdd_l6_l12_l14_l15" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm8941_pldo, "vdd_l8_l16_l18_l19" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8941_pldo, "vdd_l9_l10_l17_l22" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm8941_pldo, "vdd_l8_l16_l18_l19" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm8941_pldo, "vdd_l8_l16_l18_l19" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm8941_pldo, "vdd_l13_l20_l23_l24" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm8941_pldo, "vdd_l21" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm8941_pldo, "vdd_l9_l10_l17_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm8941_pldo, "vdd_l13_l20_l23_l24" },
	{ "l24", QCOM_SMD_RPM_LDOA, 24, &pm8941_pldo, "vdd_l13_l20_l23_l24" },

	{ "lvs1", QCOM_SMD_RPM_VSA, 1, &pm8941_switch, "vdd_l2_lvs1_2_3" },
	{ "lvs2", QCOM_SMD_RPM_VSA, 2, &pm8941_switch, "vdd_l2_lvs1_2_3" },
	{ "lvs3", QCOM_SMD_RPM_VSA, 3, &pm8941_switch, "vdd_l2_lvs1_2_3" },

	{ "5vs1", QCOM_SMD_RPM_VSA, 4, &pm8941_switch, "vin_5vs" },
	{ "5vs2", QCOM_SMD_RPM_VSA, 5, &pm8941_switch, "vin_5vs" },

	{}
};

static const struct rpm_regulator_data rpm_pm8950_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm8950_hfsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm8950_hfsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm8950_hfsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm8950_hfsmps, "vdd_s4" },
	/* S5 is managed via SPMI. */
	{ "s6", QCOM_SMD_RPM_SMPA, 6, &pm8950_hfsmps, "vdd_s6" },

	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm8950_ult_nldo, "vdd_l1_l19" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm8950_ult_nldo, "vdd_l2_l23" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm8950_ult_nldo, "vdd_l3" },
	/* L4 seems not to exist. */
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm8950_pldo_lv, "vdd_l5_l6_l7_l16" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm8950_pldo_lv, "vdd_l5_l6_l7_l16" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm8950_pldo_lv, "vdd_l5_l6_l7_l16" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8950_ult_nldo, "vdd_l9_l10_l13_l14_l15_l18"},
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8950_ult_pldo, "vdd_l9_l10_l13_l14_l15_l18" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm8950_ult_pldo, "vdd_l5_l6_l7_l16" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8950_ult_pldo, "vdd_l8_l11_l12_l17_l22" },
	/* L18 seems not to exist. */
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm8950_pldo, "vdd_l1_l19" },
	/* L20 & L21 seem not to exist. */
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm8950_pldo, "vdd_l8_l11_l12_l17_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm8950_pldo, "vdd_l2_l23" },
	{}
};

static const struct rpm_regulator_data rpm_pm8953_regulators[] = {
	{  "s1", QCOM_SMD_RPM_SMPA,  1, &pm8998_hfsmps, "vdd_s1" },
	{  "s2", QCOM_SMD_RPM_SMPA,  2, &pm8998_hfsmps, "vdd_s2" },
	{  "s3", QCOM_SMD_RPM_SMPA,  3, &pm8998_hfsmps, "vdd_s3" },
	{  "s4", QCOM_SMD_RPM_SMPA,  4, &pm8998_hfsmps, "vdd_s4" },
	{  "s5", QCOM_SMD_RPM_SMPA,  5, &pm8950_ftsmps2p5, "vdd_s5" },
	{  "s6", QCOM_SMD_RPM_SMPA,  6, &pm8950_ftsmps2p5, "vdd_s6" },
	{  "s7", QCOM_SMD_RPM_SMPA,  7, &pm8998_hfsmps, "vdd_s7" },

	{  "l1", QCOM_SMD_RPM_LDOA,  1, &pm8953_ult_nldo, "vdd_l1" },
	{  "l2", QCOM_SMD_RPM_LDOA,  2, &pm8953_ult_nldo, "vdd_l2_l3" },
	{  "l3", QCOM_SMD_RPM_LDOA,  3, &pm8953_ult_nldo, "vdd_l2_l3" },
	{  "l4", QCOM_SMD_RPM_LDOA,  4, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16_l19" },
	{  "l5", QCOM_SMD_RPM_LDOA,  5, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16_l19" },
	{  "l6", QCOM_SMD_RPM_LDOA,  6, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16_l19" },
	{  "l7", QCOM_SMD_RPM_LDOA,  7, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16_l19" },
	{  "l8", QCOM_SMD_RPM_LDOA,  8, &pm8950_ult_pldo, "vdd_l8_l11_l12_l13_l14_l15" },
	{  "l9", QCOM_SMD_RPM_LDOA,  9, &pm8950_ult_pldo, "vdd_l9_l10_l17_l18_l22" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8950_ult_pldo, "vdd_l9_l10_l17_l18_l22" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8950_ult_pldo, "vdd_l8_l11_l12_l13_l14_l15" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8950_ult_pldo, "vdd_l8_l11_l12_l13_l14_l15" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8950_ult_pldo, "vdd_l8_l11_l12_l13_l14_l15" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8950_ult_pldo, "vdd_l8_l11_l12_l13_l14_l15" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8950_ult_pldo, "vdd_l8_l11_l12_l13_l14_l15" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm8950_ult_pldo, "vdd_l4_l5_l6_l7_l16_l19" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8950_ult_pldo, "vdd_l9_l10_l17_l18_l22" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm8950_ult_pldo, "vdd_l9_l10_l17_l18_l22" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm8953_ult_nldo, "vdd_l4_l5_l6_l7_l16_l19" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm8953_lnldo,    "vdd_l20" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm8953_lnldo,    "vdd_l21" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm8950_ult_pldo, "vdd_l9_l10_l17_l18_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm8953_ult_nldo, "vdd_l23" },
	{}
};

static const struct rpm_regulator_data rpm_pm8994_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm8994_ftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm8994_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm8994_hfsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm8994_hfsmps, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPA, 5, &pm8994_hfsmps, "vdd_s5" },
	{ "s6", QCOM_SMD_RPM_SMPA, 6, &pm8994_ftsmps, "vdd_s6" },
	{ "s7", QCOM_SMD_RPM_SMPA, 7, &pm8994_hfsmps, "vdd_s7" },
	{ "s8", QCOM_SMD_RPM_SMPA, 8, &pm8994_ftsmps, "vdd_s8" },
	{ "s9", QCOM_SMD_RPM_SMPA, 9, &pm8994_ftsmps, "vdd_s9" },
	{ "s10", QCOM_SMD_RPM_SMPA, 10, &pm8994_ftsmps, "vdd_s10" },
	{ "s11", QCOM_SMD_RPM_SMPA, 11, &pm8994_ftsmps, "vdd_s11" },
	{ "s12", QCOM_SMD_RPM_SMPA, 12, &pm8994_ftsmps, "vdd_s12" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm8994_nldo, "vdd_l1" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm8994_nldo, "vdd_l2_l26_l28" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm8994_nldo, "vdd_l3_l11" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm8994_nldo, "vdd_l4_l27_l31" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm8994_lnldo, "vdd_l5_l7" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm8994_pldo, "vdd_l6_l12_l32" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm8994_lnldo, "vdd_l5_l7" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm8994_pldo, "vdd_l8_l16_l30" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm8994_pldo, "vdd_l9_l10_l18_l22" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8994_pldo, "vdd_l9_l10_l18_l22" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8994_nldo, "vdd_l3_l11" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8994_pldo, "vdd_l6_l12_l32" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8994_pldo, "vdd_l13_l19_l23_l24" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8994_pldo, "vdd_l14_l15" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8994_pldo, "vdd_l14_l15" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm8994_pldo, "vdd_l8_l16_l30" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8994_pldo, "vdd_l17_l29" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm8994_pldo, "vdd_l9_l10_l18_l22" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm8994_pldo, "vdd_l13_l19_l23_l24" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm8994_pldo, "vdd_l20_l21" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm8994_pldo, "vdd_l20_l21" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm8994_pldo, "vdd_l9_l10_l18_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm8994_pldo, "vdd_l13_l19_l23_l24" },
	{ "l24", QCOM_SMD_RPM_LDOA, 24, &pm8994_pldo, "vdd_l13_l19_l23_l24" },
	{ "l25", QCOM_SMD_RPM_LDOA, 25, &pm8994_pldo, "vdd_l25" },
	{ "l26", QCOM_SMD_RPM_LDOA, 26, &pm8994_nldo, "vdd_l2_l26_l28" },
	{ "l27", QCOM_SMD_RPM_LDOA, 27, &pm8994_nldo, "vdd_l4_l27_l31" },
	{ "l28", QCOM_SMD_RPM_LDOA, 28, &pm8994_nldo, "vdd_l2_l26_l28" },
	{ "l29", QCOM_SMD_RPM_LDOA, 29, &pm8994_pldo, "vdd_l17_l29" },
	{ "l30", QCOM_SMD_RPM_LDOA, 30, &pm8994_pldo, "vdd_l8_l16_l30" },
	{ "l31", QCOM_SMD_RPM_LDOA, 31, &pm8994_nldo, "vdd_l4_l27_l31" },
	{ "l32", QCOM_SMD_RPM_LDOA, 32, &pm8994_pldo, "vdd_l6_l12_l32" },
	{ "lvs1", QCOM_SMD_RPM_VSA, 1, &pm8994_switch, "vdd_lvs1_2" },
	{ "lvs2", QCOM_SMD_RPM_VSA, 2, &pm8994_switch, "vdd_lvs1_2" },

	{}
};

static const struct rpm_regulator_data rpm_pm8998_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pm8998_ftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pm8998_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pm8998_hfsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pm8998_hfsmps, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPA, 5, &pm8998_hfsmps, "vdd_s5" },
	{ "s6", QCOM_SMD_RPM_SMPA, 6, &pm8998_ftsmps, "vdd_s6" },
	{ "s7", QCOM_SMD_RPM_SMPA, 7, &pm8998_ftsmps, "vdd_s7" },
	{ "s8", QCOM_SMD_RPM_SMPA, 8, &pm8998_ftsmps, "vdd_s8" },
	{ "s9", QCOM_SMD_RPM_SMPA, 9, &pm8998_ftsmps, "vdd_s9" },
	{ "s10", QCOM_SMD_RPM_SMPA, 10, &pm8998_ftsmps, "vdd_s10" },
	{ "s11", QCOM_SMD_RPM_SMPA, 11, &pm8998_ftsmps, "vdd_s11" },
	{ "s12", QCOM_SMD_RPM_SMPA, 12, &pm8998_ftsmps, "vdd_s12" },
	{ "s13", QCOM_SMD_RPM_SMPA, 13, &pm8998_ftsmps, "vdd_s13" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pm8998_nldo, "vdd_l1_l27" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pm8998_nldo, "vdd_l2_l8_l17" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pm8998_nldo, "vdd_l3_l11" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pm8998_nldo, "vdd_l4_l5" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pm8998_nldo, "vdd_l4_l5" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pm8998_pldo, "vdd_l6" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pm8998_pldo_lv, "vdd_l7_l12_l14_l15" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pm8998_nldo, "vdd_l2_l8_l17" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pm8998_pldo, "vdd_l9" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pm8998_pldo, "vdd_l10_l23_l25" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pm8998_nldo, "vdd_l3_l11" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pm8998_pldo_lv, "vdd_l7_l12_l14_l15" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pm8998_pldo, "vdd_l13_l19_l21" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pm8998_pldo_lv, "vdd_l7_l12_l14_l15" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pm8998_pldo_lv, "vdd_l7_l12_l14_l15" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pm8998_pldo, "vdd_l16_l28" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pm8998_nldo, "vdd_l2_l8_l17" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pm8998_pldo, "vdd_l18_l22" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pm8998_pldo, "vdd_l13_l19_l21" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pm8998_pldo, "vdd_l20_l24" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pm8998_pldo, "vdd_l13_l19_l21" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pm8998_pldo, "vdd_l18_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pm8998_pldo, "vdd_l10_l23_l25" },
	{ "l24", QCOM_SMD_RPM_LDOA, 24, &pm8998_pldo, "vdd_l20_l24" },
	{ "l25", QCOM_SMD_RPM_LDOA, 25, &pm8998_pldo, "vdd_l10_l23_l25" },
	{ "l26", QCOM_SMD_RPM_LDOA, 26, &pm8998_nldo, "vdd_l26" },
	{ "l27", QCOM_SMD_RPM_LDOA, 27, &pm8998_nldo, "vdd_l1_l27" },
	{ "l28", QCOM_SMD_RPM_LDOA, 28, &pm8998_pldo, "vdd_l16_l28" },
	{ "lvs1", QCOM_SMD_RPM_VSA, 1, &pm8998_switch, "vdd_lvs1_lvs2" },
	{ "lvs2", QCOM_SMD_RPM_VSA, 2, &pm8998_switch, "vdd_lvs1_lvs2" },
	{}
};

static const struct rpm_regulator_data rpm_pma8084_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pma8084_ftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pma8084_ftsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pma8084_hfsmps, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pma8084_hfsmps, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPA, 5, &pma8084_hfsmps, "vdd_s5" },
	{ "s6", QCOM_SMD_RPM_SMPA, 6, &pma8084_ftsmps, "vdd_s6" },
	{ "s7", QCOM_SMD_RPM_SMPA, 7, &pma8084_ftsmps, "vdd_s7" },
	{ "s8", QCOM_SMD_RPM_SMPA, 8, &pma8084_ftsmps, "vdd_s8" },
	{ "s9", QCOM_SMD_RPM_SMPA, 9, &pma8084_ftsmps, "vdd_s9" },
	{ "s10", QCOM_SMD_RPM_SMPA, 10, &pma8084_ftsmps, "vdd_s10" },
	{ "s11", QCOM_SMD_RPM_SMPA, 11, &pma8084_ftsmps, "vdd_s11" },
	{ "s12", QCOM_SMD_RPM_SMPA, 12, &pma8084_ftsmps, "vdd_s12" },

	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pma8084_nldo, "vdd_l1_l11" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pma8084_nldo, "vdd_l2_l3_l4_l27" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pma8084_nldo, "vdd_l2_l3_l4_l27" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pma8084_nldo, "vdd_l2_l3_l4_l27" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pma8084_pldo, "vdd_l5_l7" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pma8084_pldo, "vdd_l6_l12_l14_l15_l26" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pma8084_pldo, "vdd_l5_l7" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pma8084_pldo, "vdd_l8" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pma8084_pldo, "vdd_l9_l10_l13_l20_l23_l24" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pma8084_pldo, "vdd_l9_l10_l13_l20_l23_l24" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pma8084_nldo, "vdd_l1_l11" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pma8084_pldo, "vdd_l6_l12_l14_l15_l26" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pma8084_pldo, "vdd_l9_l10_l13_l20_l23_l24" },
	{ "l14", QCOM_SMD_RPM_LDOA, 14, &pma8084_pldo, "vdd_l6_l12_l14_l15_l26" },
	{ "l15", QCOM_SMD_RPM_LDOA, 15, &pma8084_pldo, "vdd_l6_l12_l14_l15_l26" },
	{ "l16", QCOM_SMD_RPM_LDOA, 16, &pma8084_pldo, "vdd_l16_l25" },
	{ "l17", QCOM_SMD_RPM_LDOA, 17, &pma8084_pldo, "vdd_l17" },
	{ "l18", QCOM_SMD_RPM_LDOA, 18, &pma8084_pldo, "vdd_l18" },
	{ "l19", QCOM_SMD_RPM_LDOA, 19, &pma8084_pldo, "vdd_l19" },
	{ "l20", QCOM_SMD_RPM_LDOA, 20, &pma8084_pldo, "vdd_l9_l10_l13_l20_l23_l24" },
	{ "l21", QCOM_SMD_RPM_LDOA, 21, &pma8084_pldo, "vdd_l21" },
	{ "l22", QCOM_SMD_RPM_LDOA, 22, &pma8084_pldo, "vdd_l22" },
	{ "l23", QCOM_SMD_RPM_LDOA, 23, &pma8084_pldo, "vdd_l9_l10_l13_l20_l23_l24" },
	{ "l24", QCOM_SMD_RPM_LDOA, 24, &pma8084_pldo, "vdd_l9_l10_l13_l20_l23_l24" },
	{ "l25", QCOM_SMD_RPM_LDOA, 25, &pma8084_pldo, "vdd_l16_l25" },
	{ "l26", QCOM_SMD_RPM_LDOA, 26, &pma8084_pldo, "vdd_l6_l12_l14_l15_l26" },
	{ "l27", QCOM_SMD_RPM_LDOA, 27, &pma8084_nldo, "vdd_l2_l3_l4_l27" },

	{ "lvs1", QCOM_SMD_RPM_VSA, 1, &pma8084_switch },
	{ "lvs2", QCOM_SMD_RPM_VSA, 2, &pma8084_switch },
	{ "lvs3", QCOM_SMD_RPM_VSA, 3, &pma8084_switch },
	{ "lvs4", QCOM_SMD_RPM_VSA, 4, &pma8084_switch },
	{ "5vs1", QCOM_SMD_RPM_VSA, 5, &pma8084_switch },

	{}
};

static const struct rpm_regulator_data rpm_pmi8994_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPB, 1, &pmi8994_ftsmps, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPB, 2, &pmi8994_hfsmps, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPB, 3, &pmi8994_hfsmps, "vdd_s3" },
	{ "boost-bypass", QCOM_SMD_RPM_BBYB, 1, &pmi8994_bby, "vdd_bst_byp" },
	{}
};

static const struct rpm_regulator_data rpm_pmi8998_regulators[] = {
	{ "bob", QCOM_SMD_RPM_BOBB, 1, &pmi8998_bob, "vdd_bob" },
	{}
};

static const struct rpm_regulator_data rpm_pmr735a_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPE, 1, &pmic5_ftsmps520, "vdd_s1"},
	{ "s2", QCOM_SMD_RPM_SMPE, 2, &pmic5_ftsmps520, "vdd_s2"},
	{ "s3", QCOM_SMD_RPM_SMPE, 3, &pmic5_hfsmps515, "vdd_s3"},
	{ "l1", QCOM_SMD_RPM_LDOE, 1, &pm660_nldo660, "vdd_l1_l2"},
	{ "l2", QCOM_SMD_RPM_LDOE, 2, &pm660_nldo660, "vdd_l1_l2"},
	{ "l3", QCOM_SMD_RPM_LDOE, 3, &pm660_nldo660, "vdd_l3"},
	{ "l4", QCOM_SMD_RPM_LDOE, 4, &pm660_ht_lvpldo, "vdd_l4"},
	{ "l5", QCOM_SMD_RPM_LDOE, 5, &pm660_nldo660, "vdd_l5_l6"},
	{ "l6", QCOM_SMD_RPM_LDOE, 6, &pm660_nldo660, "vdd_l5_l6"},
	{ "l7", QCOM_SMD_RPM_LDOE, 7, &pm660_pldo660, "vdd_l7_bob"},
	{}
};

static const struct rpm_regulator_data rpm_pms405_regulators[] = {
	{ "s1", QCOM_SMD_RPM_SMPA, 1, &pms405_hfsmps3, "vdd_s1" },
	{ "s2", QCOM_SMD_RPM_SMPA, 2, &pms405_hfsmps3, "vdd_s2" },
	{ "s3", QCOM_SMD_RPM_SMPA, 3, &pms405_hfsmps3, "vdd_s3" },
	{ "s4", QCOM_SMD_RPM_SMPA, 4, &pms405_hfsmps3, "vdd_s4" },
	{ "s5", QCOM_SMD_RPM_SMPA, 5, &pms405_hfsmps3, "vdd_s5" },
	{ "l1", QCOM_SMD_RPM_LDOA, 1, &pms405_nldo1200, "vdd_l1_l2" },
	{ "l2", QCOM_SMD_RPM_LDOA, 2, &pms405_nldo1200, "vdd_l1_l2" },
	{ "l3", QCOM_SMD_RPM_LDOA, 3, &pms405_nldo1200, "vdd_l3_l8" },
	{ "l4", QCOM_SMD_RPM_LDOA, 4, &pms405_nldo300, "vdd_l4" },
	{ "l5", QCOM_SMD_RPM_LDOA, 5, &pms405_pldo600, "vdd_l5_l6" },
	{ "l6", QCOM_SMD_RPM_LDOA, 6, &pms405_pldo600, "vdd_l5_l6" },
	{ "l7", QCOM_SMD_RPM_LDOA, 7, &pms405_pldo150, "vdd_l7" },
	{ "l8", QCOM_SMD_RPM_LDOA, 8, &pms405_nldo1200, "vdd_l3_l8" },
	{ "l9", QCOM_SMD_RPM_LDOA, 9, &pms405_nldo1200, "vdd_l9" },
	{ "l10", QCOM_SMD_RPM_LDOA, 10, &pms405_pldo50, "vdd_l10_l11_l12_l13" },
	{ "l11", QCOM_SMD_RPM_LDOA, 11, &pms405_pldo150, "vdd_l10_l11_l12_l13" },
	{ "l12", QCOM_SMD_RPM_LDOA, 12, &pms405_pldo150, "vdd_l10_l11_l12_l13" },
	{ "l13", QCOM_SMD_RPM_LDOA, 13, &pms405_pldo150, "vdd_l10_l11_l12_l13" },
	{}
};

static const struct of_device_id rpm_of_match[] = {
	{ .compatible = "qcom,rpm-mp5496-regulators", .data = &rpm_mp5496_regulators },
	{ .compatible = "qcom,rpm-pm2250-regulators", .data = &rpm_pm2250_regulators },
	{ .compatible = "qcom,rpm-pm6125-regulators", .data = &rpm_pm6125_regulators },
	{ .compatible = "qcom,rpm-pm660-regulators", .data = &rpm_pm660_regulators },
	{ .compatible = "qcom,rpm-pm660l-regulators", .data = &rpm_pm660l_regulators },
	{ .compatible = "qcom,rpm-pm8226-regulators", .data = &rpm_pm8226_regulators },
	{ .compatible = "qcom,rpm-pm8841-regulators", .data = &rpm_pm8841_regulators },
	{ .compatible = "qcom,rpm-pm8909-regulators", .data = &rpm_pm8909_regulators },
	{ .compatible = "qcom,rpm-pm8916-regulators", .data = &rpm_pm8916_regulators },
	{ .compatible = "qcom,rpm-pm8937-regulators", .data = &rpm_pm8937_regulators },
	{ .compatible = "qcom,rpm-pm8941-regulators", .data = &rpm_pm8941_regulators },
	{ .compatible = "qcom,rpm-pm8950-regulators", .data = &rpm_pm8950_regulators },
	{ .compatible = "qcom,rpm-pm8953-regulators", .data = &rpm_pm8953_regulators },
	{ .compatible = "qcom,rpm-pm8994-regulators", .data = &rpm_pm8994_regulators },
	{ .compatible = "qcom,rpm-pm8998-regulators", .data = &rpm_pm8998_regulators },
	{ .compatible = "qcom,rpm-pma8084-regulators", .data = &rpm_pma8084_regulators },
	{ .compatible = "qcom,rpm-pmi8994-regulators", .data = &rpm_pmi8994_regulators },
	{ .compatible = "qcom,rpm-pmi8998-regulators", .data = &rpm_pmi8998_regulators },
	{ .compatible = "qcom,rpm-pmr735a-regulators", .data = &rpm_pmr735a_regulators },
	{ .compatible = "qcom,rpm-pms405-regulators", .data = &rpm_pms405_regulators },
	{}
};
MODULE_DEVICE_TABLE(of, rpm_of_match);

/**
 * rpm_regulator_init_vreg() - initialize all attributes of a qcom_smd-regulator
 * @vreg:		Pointer to the individual qcom_smd-regulator resource
 * @dev:		Pointer to the top level qcom_smd-regulator PMIC device
 * @node:		Pointer to the individual qcom_smd-regulator resource
 *			device node
 * @pmic_rpm_data:	Pointer to a null-terminated array of qcom_smd-regulator
 *			resources defined for the top level PMIC device
 * @pair:		Optional parent-owned PM8226 L3 voter pair
 *
 * Return: 0 on success, or a negative error number on failure
 */
static int rpm_regulator_init_vreg(struct qcom_rpm_reg *vreg, struct device *dev,
				   struct device_node *node,
				   const struct rpm_regulator_data *pmic_rpm_data,
				   struct rpm_reg_pair *pair)
{
	struct regulator_config config = {};
	const struct rpm_regulator_data *rpm_data;
	struct regulator_dev *rdev;
	int ret;

	for (rpm_data = pmic_rpm_data; rpm_data->name; rpm_data++)
		if (of_node_name_eq(node, rpm_data->name))
			break;

	if (!rpm_data->name) {
		dev_err(dev, "Unknown regulator %pOFn\n", node);
		return -EINVAL;
	}

	vreg->dev	= dev;
	vreg->type	= rpm_data->type;
	vreg->id	= rpm_data->id;
	vreg->active_only = rpm_data->active_only;
	vreg->sleep_only = rpm_data->sleep_only;
	if (pair && vreg->type == QCOM_SMD_RPM_LDOA && vreg->id == 3)
		vreg->pair = pair;
	if ((vreg->active_only || vreg->sleep_only) && !vreg->pair)
		return -EINVAL;

	memcpy(&vreg->desc, rpm_data->desc, sizeof(vreg->desc));
	vreg->desc.name = rpm_data->name;
	vreg->desc.supply_name = rpm_data->supply;
	vreg->desc.owner = THIS_MODULE;
	vreg->desc.type = REGULATOR_VOLTAGE;
	vreg->desc.of_match = rpm_data->name;

	config.dev		= dev;
	config.of_node		= node;
	config.driver_data	= vreg;

	rdev = devm_regulator_register(dev, &vreg->desc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(dev, "%pOFn: devm_regulator_register() failed, ret=%d\n", node, ret);
		return ret;
	}

	return 0;
}

static int rpm_reg_pair_check_sleep_node(struct device *dev, u32 active_max_uV)
{
	struct device_node *node;
	u32 min_uV, max_uV;
	int ret = 0;

	node = of_get_available_child_by_name(dev->of_node, "l3-so");
	if (!node)
		return 0;
	if (of_property_read_bool(node, "regulator-boot-on") ||
	    of_property_read_bool(node, "regulator-always-on") ||
	    of_property_read_u32(node, "regulator-min-microvolt", &min_uV) ||
	    of_property_read_u32(node, "regulator-max-microvolt", &max_uV) ||
	    min_uV < 750000 || min_uV > active_max_uV ||
	    max_uV < active_max_uV || max_uV > 1537500 ||
	    (min_uV - 750000) % 12500 || (max_uV - 750000) % 12500)
		ret = -EINVAL;
	of_node_put(node);
	return ret;
}

static int rpm_reg_pair_init_boot(struct device *dev, struct rpm_reg_pair *pair,
				  struct device_node *ao_node)
{
	struct regulator *apc;
	u32 min_uV, max_uV;
	int boot_uV, ret;

	if (!of_property_present(dev->of_node, "qcom,boot-apc-supply"))
		return 0;
	if (!of_node_name_eq(ao_node, "l3-ao"))
		return -EINVAL;
	if (of_property_read_bool(ao_node, "regulator-boot-on") ||
	    of_property_read_bool(ao_node, "regulator-always-on"))
		return -EINVAL;
	if (of_property_read_u32(ao_node, "regulator-min-microvolt", &min_uV) ||
	    of_property_read_u32(ao_node, "regulator-max-microvolt", &max_uV) ||
	    min_uV < 750000 || max_uV > 1537500 || min_uV > max_uV ||
	    (min_uV - 750000) % 12500 || (max_uV - 750000) % 12500)
		return -EINVAL;
	ret = rpm_reg_pair_check_sleep_node(dev, max_uV);
	if (ret)
		return ret;

	/* Probe has no regulator callback locks; never nest this in vote writes. */
	apc = devm_regulator_get(dev, "qcom,boot-apc");
	if (IS_ERR(apc))
		return PTR_ERR(apc);
	boot_uV = regulator_get_voltage(apc);
	if (boot_uV <= 0)
		return boot_uV ?: -EINVAL;
	if (boot_uV > max_uV)
		return -ERANGE;

	/* Hold the validated ceiling until CPR has a covering, enabled vote. */
	pair->boot_min_uV = DIV_ROUND_UP(max_t(int, boot_uV, min_uV), 12500) * 12500;
	pair->boot_uV = max_uV;
	pair->boot_max_uV = max_uV;
	if (pair->has_sleep_only)
		pair->boot_sleep_uV = max_uV;
	return 0;
}

static int rpm_reg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct rpm_regulator_data *vreg_data;
	struct qcom_rpm_reg *vreg;
	struct qcom_smd_rpm *rpm;
	struct rpm_reg_pair *pair = NULL;
	struct device_node *pair_node;
	struct device_node *normal_node;
	struct device_node *sleep_node;
	int ret;

	rpm = dev_get_drvdata(pdev->dev.parent);
	if (!rpm) {
		dev_err(&pdev->dev, "Unable to retrieve handle to rpm\n");
		return -ENODEV;
	}

	if (smd_vreg_rpm && rpm != smd_vreg_rpm)
		return dev_err_probe(dev, -EINVAL, "RPM mismatch\n");

	smd_vreg_rpm = rpm;

	vreg_data = of_device_get_match_data(dev);
	if (!vreg_data)
		return -ENODEV;

	pair_node = of_get_available_child_by_name(dev->of_node, "l3-ao");
	if (!pair_node)
		pair_node = of_get_available_child_by_name(dev->of_node, "l3-so");
	if (pair_node) {
		if (vreg_data != rpm_pm8226_regulators) {
			of_node_put(pair_node);
			return dev_err_probe(dev, -EINVAL, "L3 voters require PM8226\n");
		}
		normal_node = of_get_available_child_by_name(dev->of_node, "l3");
		if (!normal_node) {
			of_node_put(pair_node);
			return dev_err_probe(dev, -EINVAL, "L3 voters require l3\n");
		}
		of_node_put(normal_node);
		pair = devm_kzalloc(dev, sizeof(*pair), GFP_KERNEL);
		if (!pair) {
			of_node_put(pair_node);
			return -ENOMEM;
		}
		mutex_init(&pair->lock);
		sleep_node = of_get_available_child_by_name(dev->of_node, "l3-so");
		pair->has_sleep_only = !!sleep_node;
		of_node_put(sleep_node);
		ret = rpm_reg_pair_init_boot(dev, pair, pair_node);
		of_node_put(pair_node);
		if (ret)
			return dev_err_probe(dev, ret, "L3 boot APC handoff unavailable\n");
	} else if (of_property_present(dev->of_node, "qcom,boot-apc-supply")) {
		return dev_err_probe(dev, -EINVAL, "boot APC requires l3-ao\n");
	}

	for_each_available_child_of_node_scoped(dev->of_node, node) {
		vreg = devm_kzalloc(&pdev->dev, sizeof(*vreg), GFP_KERNEL);
		if (!vreg)
			return -ENOMEM;

		ret = rpm_regulator_init_vreg(vreg, dev, node, vreg_data, pair);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct platform_driver rpm_reg_driver = {
	.probe = rpm_reg_probe,
	.driver = {
		.name  = "qcom_rpm_smd_regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = rpm_of_match,
	},
};

static int __init rpm_reg_init(void)
{
	return platform_driver_register(&rpm_reg_driver);
}
subsys_initcall(rpm_reg_init);

static void __exit rpm_reg_exit(void)
{
	platform_driver_unregister(&rpm_reg_driver);
}
module_exit(rpm_reg_exit)

MODULE_DESCRIPTION("Qualcomm RPM regulator driver");
MODULE_LICENSE("GPL v2");
