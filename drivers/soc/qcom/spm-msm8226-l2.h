/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_SPM_MSM8226_L2_H__
#define __QCOM_SPM_MSM8226_L2_H__

#include <linux/errno.h>
#include <linux/regmap.h>

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
