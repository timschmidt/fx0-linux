/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 */

#ifndef __QCOM_CLK_PLL_H__
#define __QCOM_CLK_PLL_H__

#include <linux/clk-provider.h>
#include "clk-regmap.h"

/**
 * struct pll_freq_tbl - PLL frequency table
 * @l: L value
 * @m: M value
 * @n: N value
 * @ibits: internal values
 */
struct pll_freq_tbl {
	unsigned long freq;
	u16 l;
	u16 m;
	u16 n;
	u32 ibits;
};

/**
 * struct clk_pll - phase locked loop (PLL)
 * @l_reg: L register
 * @m_reg: M register
 * @n_reg: N register
 * @config_reg: config register
 * @mode_reg: mode register
 * @status_reg: status register
 * @status_bit: ANDed with @status_reg to determine if PLL is enabled
 * @freq_tbl: PLL frequency table
 * @critical_rate: Stop on uncertain CPU PLL programming; CCF can discard errors
 * @suspend_valid: MSM8226 configuration snapshot is available
 * @suspend_armed: CCF unprepared the PLL after its last verified disable
 * @suspend_config: original static configuration register
 * @suspend_l: latest successfully programmed L value
 * @suspend_m: latest successfully programmed M value
 * @suspend_n: latest successfully programmed N value
 * @hw: handle between common and hardware-specific interfaces
 */
struct clk_pll {
	u32	l_reg;
	u32	m_reg;
	u32	n_reg;
	u32	config_reg;
	u32	mode_reg;
	u32	status_reg;
	u8	status_bit;
	u8	post_div_width;
	u8	post_div_shift;
	bool	critical_rate;
	bool	suspend_valid;
	bool	suspend_armed;
	u32	suspend_config;
	u32	suspend_l;
	u32	suspend_m;
	u32	suspend_n;

	const struct pll_freq_tbl *freq_tbl;

	struct clk_regmap clkr;
};

extern const struct clk_ops clk_pll_ops;
extern const struct clk_ops clk_pll_vote_ops;
extern const struct clk_ops clk_pll_sr2_ops;

#define to_clk_pll(_hw) container_of(to_clk_regmap(_hw), struct clk_pll, clkr)

struct pll_config {
	u16 l;
	u32 m;
	u32 n;
	u32 vco_val;
	u32 vco_mask;
	u32 pre_div_val;
	u32 pre_div_mask;
	u32 post_div_val;
	u32 post_div_mask;
	u32 mn_ena_mask;
	u32 main_output_mask;
	u32 aux_output_mask;
};

void clk_pll_configure_sr(struct clk_pll *pll, struct regmap *regmap,
		const struct pll_config *config, bool fsm_mode);
void clk_pll_configure_sr_hpm_lp(struct clk_pll *pll, struct regmap *regmap,
		const struct pll_config *config, bool fsm_mode);

#endif
