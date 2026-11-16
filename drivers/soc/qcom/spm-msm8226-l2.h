/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_SPM_MSM8226_L2_H__
#define __QCOM_SPM_MSM8226_L2_H__

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/panic.h>
#include <linux/regmap.h>

#include <soc/qcom/spm.h>

/*
 * MSM8226 v2 L2 RET, GDHS, PC without RPM and PC with RPM programs.
 * From msm8226-v2-pm.dtsi, Copyright (c) 2013-2014 The Linux Foundation.
 * The final two bytes are END padding; the hardware has 32 sequence words.
 */
static const u32 spm_8226_l2_sequence[] = {
	0x0f000300, 0x6b322000, 0x42d0e0c0, 0x024e5003,
	0xc0e0d002, 0x32026b22, 0x32000f50, 0xd0e010b0,
	0xf042c06b, 0xb0010311, 0x02024e50, 0xe012d0c0,
	0x5032026b, 0x32000ff0, 0xd0e010b0, 0xf042c06b,
	0xb0010711, 0x02024e50, 0xe012d0c0, 0x5032026b,
	0x0f0f0ff0,
};

#define SPM_8226_L2_CTL		0x30
#define SPM_8226_L2_SEQ		0x80

static inline int spm_8226_l2_sequence_verify(struct regmap *map)
{
	unsigned int i, value;
	int ret;

	for (i = 0; i < ARRAY_SIZE(spm_8226_l2_sequence); i++) {
		ret = regmap_read(map, SPM_8226_L2_SEQ + 4 * i, &value);
		if (ret)
			return ret;
		if (value != spm_8226_l2_sequence[i])
			return -EIO;
	}
	return 0;
}

static inline int spm_8226_l2_sequence_init(struct regmap *map)
{
	unsigned int control, avs, i;
	int ret;

	ret = regmap_read(map, SPM_8226_L2_CTL, &control);
	if (ret)
		return ret;
	if (control)
		return -EBUSY;
	ret = regmap_read(map, 0x20, &avs);
	if (ret)
		return ret;
	if (avs & 1)
		return -EBUSY;
	/* Load and verify every program while the sequencer remains disabled. */
	for (i = 0; i < ARRAY_SIZE(spm_8226_l2_sequence); i++) {
		ret = regmap_write(map, SPM_8226_L2_SEQ + 4 * i,
				   spm_8226_l2_sequence[i]);
		if (ret)
			return ret;
	}
	ret = spm_8226_l2_sequence_verify(map);
	if (ret)
		return ret;
	ret = regmap_read(map, SPM_8226_L2_CTL, &control);
	if (ret)
		return ret;
	if (control)
		return -EBUSY;
	ret = regmap_read(map, 0x20, &avs);
	if (ret)
		return ret;
	return avs & 1 ? -EBUSY : 0;
}

static inline int spm_8226_l2_disarm(struct regmap *map)
{
	unsigned int control, attempt;
	int ret = -EIO, read_ret;

	for (attempt = 0; attempt < 2; attempt++) {
		ret = regmap_write(map, SPM_8226_L2_CTL, 0);
		read_ret = regmap_read(map, SPM_8226_L2_CTL, &control);
		/* Readback also resolves an applied-but-failed write. */
		if (!read_ret && !control)
			return 0;
		if (!ret)
			ret = read_ret ?: -EIO;
	}
	return ret;
}

/* No permission for collapse is implied: the system coordinator owns it. */
static inline int spm_8226_l2_set_mode(struct regmap *map,
				       enum spm_l2_sleep_mode mode)
{
	static const unsigned int starts[] = { 0, 0, 4, 26, 54 };
	static const unsigned int registers[] = {
		0xfd0, 4, 8, 0x34, 0x40, 0x44, 0x20, 0x14,
	};
	static const unsigned int expected[] = {
		0x20010000, 4, 0x14, 0x3c102800, 0x02030080, 0x00030000, 0, 0,
	};
	static const unsigned int masks[] = {
		~0U, 4, ~0U, ~0U, ~0U, ~0U, 1, 0x30000,
	};
	unsigned int value, control, i;
	int ret;

	ret = spm_8226_l2_disarm(map);
	if (ret)
		panic("MSM8226 L2 disarm failed: %d", ret);
	if ((unsigned int)mode >= SPM_L2_MODE_NR)
		return -EINVAL;
	if (mode == SPM_L2_MODE_DISABLED)
		return 0;
	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		ret = regmap_read(map, registers[i], &value);
		if (ret)
			return ret;
		if ((value & masks[i]) != expected[i])
			return -EOPNOTSUPP;
	}
	ret = spm_8226_l2_sequence_verify(map);
	if (ret)
		return ret;
	control = starts[mode] << 4 | 1;
	ret = regmap_write(map, SPM_8226_L2_CTL, control);
	if (!ret) {
		ret = regmap_read(map, SPM_8226_L2_CTL, &value);
		if (!ret && value != control)
			ret = -EIO;
	}
	if (!ret)
		return 0;
	/* A failed write may have armed collapse: never return with unknown EN. */
	if (spm_8226_l2_disarm(map))
		panic("MSM8226 L2 entry recovery failed\n");
	return ret;
}

/* Compiled Madai L2 master configuration; never issue VCTL or run a sequence. */
static inline int spm_8226_l2_master_init(struct regmap *map)
{
	static const unsigned int registers[] = { 8, 0x34, 0x40, 0x44 };
	static const unsigned int expected[] = {
		0x14, 0x3c102800, 0x02030080, 0x00030000,
	};
	unsigned int values[4], version, id, avs, control, value;
	bool complete = true;
	unsigned int i;
	int ret;

	ret = regmap_read(map, 0xfd0, &version);
	if (ret)
		return ret;
	if (version != 0x20010000)
		return -ENODEV;
	ret = regmap_read(map, 4, &id);
	if (ret)
		return ret;
	if (!(id & 4))
		return -ENODEV;
	ret = regmap_read(map, 0x20, &avs);
	if (ret)
		return ret;
	if (avs & 1)
		return -EBUSY;
	ret = regmap_read(map, 0x30, &control);
	if (ret)
		return ret;
	if (control)
		return -EBUSY;

	for (i = 0; i < 4; i++) {
		ret = regmap_read(map, registers[i], &values[i]);
		if (ret)
			return ret;
		if (values[i] && values[i] != expected[i])
			return -EOPNOTSUPP;
		complete &= values[i] == expected[i];
	}
	/* Never repair a live, inconsistently configured arbiter. */
	if (values[0] && !complete)
		return -EOPNOTSUPP;

	/* Retry known partial setup safely while CFG is still disabled. */
	for (i = 1; i < 4; i++) {
		if (values[i] != expected[i]) {
			ret = regmap_write(map, registers[i], expected[i]);
			if (ret)
				return ret;
		}
		ret = regmap_read(map, registers[i], &value);
		if (ret)
			return ret;
		if (value != expected[i])
			return -EIO;
	}
	/* Publish the master configuration only after its data is verified. */
	if (!values[0]) {
		ret = regmap_write(map, registers[0], expected[0]);
		if (ret)
			return ret;
	}
	ret = regmap_read(map, registers[0], &value);
	if (ret)
		return ret;
	if (value != expected[0])
		return -EIO;
	ret = regmap_read(map, 0x30, &control);
	if (ret)
		return ret;
	if (control)
		return -EBUSY;
	ret = regmap_read(map, 0x20, &avs);
	if (ret)
		return ret;
	return avs & 1 ? -EBUSY : 0;
}

#endif
