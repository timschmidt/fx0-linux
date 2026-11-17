/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_CLK_PLL_SUSPEND_H__
#define __QCOM_CLK_PLL_SUSPEND_H__

/* Called under CCF's prepare lock, before output enable or mux selection. */
static int clk_pll_suspend_prepare(struct clk_hw *hw)
{
	struct clk_pll *pll = to_clk_pll(hw);
	u32 offsets[4], expected[4], masks[4];
	u32 value, mode;
	bool different = false;
	int i, ret;

	if (!pll->suspend_valid)
		return 0;
	if (!pll->suspend_l)
		return clk_pll_rate_error(pll, -EINVAL);
	offsets[0] = pll->config_reg;
	offsets[1] = pll->l_reg;
	offsets[2] = pll->m_reg;
	offsets[3] = pll->n_reg;
	expected[0] = pll->suspend_config;
	expected[1] = pll->suspend_l;
	expected[2] = pll->suspend_m;
	expected[3] = pll->suspend_n;
	masks[0] = U32_MAX;
	masks[1] = 0x3ff;
	masks[2] = 0x7ffff;
	masks[3] = 0x7ffff;
	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		ret = regmap_read(pll->clkr.regmap, offsets[i], &value);
		if (ret)
			return clk_pll_rate_error(pll, ret);
		if ((value & masks[i]) != expected[i])
			different = true;
	}
	ret = regmap_read(pll->clkr.regmap, pll->mode_reg, &mode);
	if (ret)
		return clk_pll_rate_error(pll, ret);
	if (mode & PLL_VOTE_FSM_ENA)
		return clk_pll_rate_error(pll, -EOPNOTSUPP);
	if (!different) {
		pll->suspend_armed = false;
		return 0;
	}
	if (!pll->suspend_armed && mode & (PLL_OUTCTRL | PLL_BYPASSNL | PLL_RESET_N))
		return clk_pll_rate_error(pll, -EBUSY);
	ret = clk_pll_disable_checked(hw);
	if (ret)
		return clk_pll_rate_error(pll, ret);
	ret = regmap_read(pll->clkr.regmap, pll->mode_reg, &mode);
	if (!ret && mode & (PLL_OUTCTRL | PLL_BYPASSNL | PLL_RESET_N | PLL_VOTE_FSM_ENA))
		ret = -EIO;
	if (ret)
		return clk_pll_rate_error(pll, ret);
	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		ret = regmap_update_bits(pll->clkr.regmap, offsets[i], masks[i], expected[i]);
		if (ret)
			return clk_pll_rate_error(pll, ret);
		ret = regmap_read(pll->clkr.regmap, offsets[i], &value);
		if (ret)
			return clk_pll_rate_error(pll, ret);
		if ((value & masks[i]) != expected[i])
			return clk_pll_rate_error(pll, -EIO);
	}
	pll->suspend_armed = false;
	return 0;
}

static void clk_pll_suspend_unprepare(struct clk_hw *hw)
{
	struct clk_pll *pll = to_clk_pll(hw);
	u32 mode;
	int ret;

	if (!pll->suspend_valid)
		return;
	ret = regmap_read(pll->clkr.regmap, pll->mode_reg, &mode);
	if (!ret && mode & (PLL_OUTCTRL | PLL_BYPASSNL | PLL_RESET_N | PLL_VOTE_FSM_ENA))
		ret = -EIO;
	if (clk_pll_rate_error(pll, ret))
		return;
	pll->suspend_armed = true;
}

#endif
