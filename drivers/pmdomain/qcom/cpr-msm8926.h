/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_CPR_MSM8926_H__
#define __QCOM_CPR_MSM8926_H__

#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/types.h>

/*
 * Madai nominal calibration; no hardware access or regulator activation.
 * Quantize voltages to the fixed PMIC boot range before forming rail votes.
 */
struct cpr_msm8926_fuses {
	u64 row22;
	u64 row138;
	u64 row139;
	int read_status;
	int conditional_read_status;
	int conditional_matches;
};

struct cpr_msm8926_calibration {
	u32 pvs_bin;
	u32 pvs_version;
	u32 clock_speed_bin;
	u32 cpr_speed_bin;
	u32 floor_uV[3];
	u32 ceiling_uV[3];
	u32 initial_uV[3];
	u32 quotient[3];
	u32 ring_osc[3];
	u32 quotient_adjust[14];
	u32 quotient_scaling;
	bool clock_bin_valid;
	bool pvs_redundant;
	bool cpr_redundant;
	bool local_scheme;
	bool minimum_override;
	bool uplift;
	bool fuse_allows_loop;
	bool mx_ceiling_supported;
	bool interpolation_applied;
};

static inline int
cpr_msm8926_decode(const struct cpr_msm8926_fuses *fuses,
		   struct cpr_msm8926_calibration *calibration)
{
	struct cpr_msm8926_calibration result = {};
	static const u32 quotient_shift[] = { 24, 12, 0 };
	static const u32 primary_ro_shift[] = { 54, 38, 41 };
	static const u32 redundant_ro_shift[] = { 46, 36, 39 };
	static const u32 base_floor[] = { 1075000, 1075000, 1125000 };
	static const u32 base_ceiling[] = { 1075000, 1175000, 1280000 };
	static const u32 frequency_mhz[] = {
		300, 384, 600, 787, 998, 1094, 1190,
		1305, 1344, 1401, 1497, 1593, 1689, 1785,
	};
	static const u32 max_corners[][3] = {
		{ 0, 1, 7 }, { 1, 1, 12 }, { 2, 1, 10 }, { 5, 1, 14 },
	};
	const u32 *ro_shift;
	u64 row;
	u32 selector, turbo_corner = 0;
	unsigned int i;

	if (!fuses || !calibration)
		return -EINVAL;
	if (fuses->read_status)
		return fuses->read_status < 0 ? fuses->read_status : -EIO;
	if (fuses->conditional_read_status)
		return fuses->conditional_read_status < 0 ?
			fuses->conditional_read_status : -EIO;
	if (fuses->conditional_matches != 0 && fuses->conditional_matches != 1)
		return -EINVAL;

	selector = (fuses->row22 >> 24) & 7;
	result.clock_bin_valid = !!(fuses->row22 & 8);
	result.clock_speed_bin = selector == 1 ?
		(fuses->row22 >> 27) & 7 : fuses->row22 & 7;
	if (!result.clock_bin_valid)
		result.clock_speed_bin = 0;
	/* The vendor CPR selector uses primary bits, unlike clock-a7. */
	result.cpr_speed_bin = fuses->row22 & 7;
	result.pvs_version = (fuses->row22 >> 4) & 3;
	result.pvs_redundant = selector == 2;
	result.pvs_bin = (fuses->row22 >> (result.pvs_redundant ? 27 : 6)) & 31;
	result.cpr_redundant = !!(fuses->row138 & (1ULL << 57));
	row = result.cpr_redundant ? fuses->row139 : fuses->row138;
	ro_shift = result.cpr_redundant ? redundant_ro_shift : primary_ro_shift;
	/* Madai has no redundant override for these two control bits. */
	result.local_scheme = !!(fuses->row138 & (1ULL << 37));
	result.minimum_override = !fuses->conditional_matches;
	result.uplift = !result.minimum_override && result.cpr_speed_bin == 1 &&
		!(fuses->row22 & (1ULL << 53));

	for (i = 0; i < 3; i++) {
		result.floor_uV[i] = base_floor[i];
		result.ceiling_uV[i] = base_ceiling[i];
		if (result.minimum_override) {
			result.floor_uV[i] = max(result.floor_uV[i], 1140000U);
			result.ceiling_uV[i] = max(result.ceiling_uV[i], 1140000U);
		}
		result.quotient[i] = (row >> quotient_shift[i]) & 4095;
		if (i)
			result.quotient[i] += 72;
		result.ring_osc[i] = (row >> ro_shift[i]) & 7;
	}
	/* Exact 32-bin compiled PVS table, expressed as its three sequences. */
	result.initial_uV[0] = 1050000;
	result.initial_uV[1] = result.pvs_bin <= 8 ? 1150000 :
		max(1050000, 1230000 - (int)result.pvs_bin * 10000);
	result.initial_uV[2] = max(1140000, 1350000 - (int)result.pvs_bin * 10000);
	if (result.uplift) {
		result.initial_uV[2] = min(result.initial_uV[2] + 50000, 1330000U);
		result.quotient[2] += 120;
	}
	result.ceiling_uV[2] = max(result.ceiling_uV[2], result.initial_uV[2]);
	for (i = 0; i < 2; i++)
		result.initial_uV[i] = clamp(result.initial_uV[i],
					   result.floor_uV[i], result.ceiling_uV[i]);
	/* Vendor leaves the turbo PVS value unclamped, including below its floor. */
	result.fuse_allows_loop = row && !(fuses->row138 & (1ULL << 36)) &&
		result.quotient[2] > result.quotient[1] + 50 &&
		result.quotient[1] > result.quotient[0];
	for (i = 0; i < 3; i++)
		if (result.quotient[i] > 4095)
			result.fuse_allows_loop = false;
	result.mx_ceiling_supported = result.ceiling_uV[2] <= 1337500;

	for (i = 0; i < 4; i++)
		if (result.cpr_speed_bin == max_corners[i][0] &&
		    result.pvs_version == max_corners[i][1])
			turbo_corner = max_corners[i][2];
	/* Invalid quotients never enter unsigned interpolation arithmetic. */
	if (turbo_corner && result.fuse_allows_loop) {
		result.quotient_scaling = min(650U,
			1000 * (result.quotient[2] - result.quotient[1]) /
			(frequency_mhz[turbo_corner - 1] - frequency_mhz[3]));
		for (i = 4; i < turbo_corner; i++)
			result.quotient_adjust[i] = result.quotient_scaling *
				(frequency_mhz[turbo_corner - 1] - frequency_mhz[i]) / 1000;
		result.interpolation_applied = true;
	}
	*calibration = result;
	return 0;
}

#endif
