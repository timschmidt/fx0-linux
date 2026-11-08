/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_CPR_MSM8926_FUSES_H__
#define __QCOM_CPR_MSM8926_FUSES_H__

#include <linux/err.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/nvmem-consumer.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "cpr-msm8926.h"

/* Both halves of row22 are needed: the uplift selector is bit53. */
static inline int cpr_msm8926_read_row(struct device *dev, const char *name,
				       u64 *row)
{
	struct nvmem_cell *cell;
	size_t length;
	void *data;
	int ret = 0;

	cell = nvmem_cell_get(dev, name);
	if (IS_ERR(cell))
		return PTR_ERR(cell);
	data = nvmem_cell_read(cell, &length);
	nvmem_cell_put(cell);
	if (IS_ERR(data))
		return PTR_ERR(data);
	if (length != sizeof(*row))
		ret = -EINVAL;
	else
		*row = get_unaligned_le64(data);
	kfree_sensitive(data);

	return ret;
}

/*
 * Sleepable acquisition for the MSM8926 CPR consumer. NVMEM cells must name
 * full, unshifted raw rows22/138/139. Protected row54 is read only through the
 * legacy SCM service; a failed read must never select a voltage rule. Publish
 * calibration only after all reads and the nominal decoder have succeeded.
 * This does not enable CPR, activate regulators or authorize any CPU OPP.
 */
static inline int
cpr_msm8926_read_calibration(struct device *dev,
			     struct cpr_msm8926_calibration *calibration)
{
	struct cpr_msm8926_fuses fuses = {};
	u64 conditional;
	int ret;

	if (!dev || !calibration)
		return -EINVAL;

	ret = qcom_scm_legacy_fuse_read(0xfc4b81b0, &conditional);
	if (ret)
		return ret < 0 ? ret : -EIO;
	fuses.conditional_matches = ((conditional >> 42) & 63) == 7;
	ret = cpr_msm8926_read_row(dev, "cpu_calibration", &fuses.row22);
	if (ret)
		return ret;
	ret = cpr_msm8926_read_row(dev, "cpr_primary", &fuses.row138);
	if (ret)
		return ret;
	ret = cpr_msm8926_read_row(dev, "cpr_redundant", &fuses.row139);
	if (ret)
		return ret;

	return cpr_msm8926_decode(&fuses, calibration);
}

#endif
