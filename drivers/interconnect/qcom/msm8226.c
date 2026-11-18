// SPDX-License-Identifier: GPL-2.0-only
/*
 * MSM8226 multimedia-to-memory topology from the CAF MSM8226 bus tables.
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/interconnect/qcom,msm8226.h>

#include "icc-rpm.h"

enum {
	MSM8226_MASTER_VIDEO_P0 = 8226,
	MSM8226_SLAVE_MMSS_BIMC,
	MSM8226_MASTER_BIMC_MMSS,
	MSM8226_SLAVE_EBI_CH0,
};

static const u16 video_p0_links[] = { MSM8226_SLAVE_MMSS_BIMC };
static const u16 mmss_bimc_links[] = { MSM8226_MASTER_BIMC_MMSS };
static const u16 bimc_mmss_links[] = { MSM8226_SLAVE_EBI_CH0 };

static struct qcom_icc_node mas_video_p0 = {
	.name = "mas_video_p0",
	.id = MSM8226_MASTER_VIDEO_P0,
	.links = video_p0_links,
	.num_links = ARRAY_SIZE(video_p0_links),
	/* Use the width of the downstream MMSS-to-BIMC hop. */
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos = {
		.ap_owned = true,
		.qos_mode = NOC_QOS_MODE_BYPASS,
		.qos_port = 4,
	},
};

static struct qcom_icc_node slv_mmss_bimc = {
	.name = "slv_mmss_bimc",
	.id = MSM8226_SLAVE_MMSS_BIMC,
	.links = mmss_bimc_links,
	.num_links = ARRAY_SIZE(mmss_bimc_links),
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
};

static struct qcom_icc_node mas_bimc_mmss = {
	.name = "mas_bimc_mmss",
	.id = MSM8226_MASTER_BIMC_MMSS,
	.links = bimc_mmss_links,
	.num_links = ARRAY_SIZE(bimc_mmss_links),
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos = {
		.ap_owned = true,
		.qos_mode = NOC_QOS_MODE_BYPASS,
		.qos_port = 2,
	},
};

static struct qcom_icc_node slv_ebi_ch0 = {
	.name = "slv_ebi_ch0",
	.id = MSM8226_SLAVE_EBI_CH0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 0,
};

static struct qcom_icc_node * const msm8226_mmss_nodes[] = {
	[MSM8226_MMSS_MASTER_VIDEO_P0] = &mas_video_p0,
	[MSM8226_MMSS_SLAVE_BIMC] = &slv_mmss_bimc,
};

static const char * const msm8226_mmss_intf_clocks[] = { "iface" };

static int msm8226_get_bw(struct icc_node *node, u32 *avg, u32 *peak)
{
	/* These endpoints only account for video requests made after probe. */
	*avg = 0;
	*peak = 0;

	return 0;
}

static int msm8226_set_mmss_rate(struct clk *clk, unsigned long rate)
{
	long actual;

	if (rate) {
		actual = clk_round_rate(clk, rate);
		if (actual < 0)
			return actual;
		if ((unsigned long)actual < rate)
			return -ERANGE;
	}

	/* Display and camera clocks can also propagate rates to this root. */
	return clk_set_min_rate(clk, rate);
}

static const struct regmap_config msm8226_mmss_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x3ffc,
	.fast_io = true,
};

static const struct qcom_icc_desc msm8226_mmss = {
	.type = QCOM_ICC_NOC,
	.nodes = msm8226_mmss_nodes,
	.num_nodes = ARRAY_SIZE(msm8226_mmss_nodes),
	.get_bw = msm8226_get_bw,
	.set_bus_clk_rate = msm8226_set_mmss_rate,
	.intf_clocks = msm8226_mmss_intf_clocks,
	.num_intf_clocks = ARRAY_SIZE(msm8226_mmss_intf_clocks),
	.regmap_cfg = &msm8226_mmss_regmap_config,
	.qos_offset = 0x3000,
	.noc_qos_stride = 0x80,
};

static struct qcom_icc_node * const msm8226_bimc_nodes[] = {
	[MSM8226_BIMC_MASTER_MMSS] = &mas_bimc_mmss,
	[MSM8226_BIMC_SLAVE_EBI_CH0] = &slv_ebi_ch0,
};

static const struct regmap_config msm8226_bimc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x69ffc,
	.fast_io = true,
};

static const struct qcom_icc_desc msm8226_bimc = {
	.type = QCOM_ICC_BIMC,
	.nodes = msm8226_bimc_nodes,
	.num_nodes = ARRAY_SIZE(msm8226_bimc_nodes),
	.get_bw = msm8226_get_bw,
	/* Keep the RPM clock controller's vote until all DDR users are modelled. */
	.regmap_cfg = &msm8226_bimc_regmap_config,
	.qos_offset = 0x8000,
};

static const struct of_device_id msm8226_icc_of_match[] = {
	{ .compatible = "qcom,msm8226-bimc", .data = &msm8226_bimc },
	{ .compatible = "qcom,msm8226-mmssnoc", .data = &msm8226_mmss },
	{ }
};
MODULE_DEVICE_TABLE(of, msm8226_icc_of_match);

static bool fx0_hold_mmss_iface;
module_param(fx0_hold_mmss_iface, bool, 0400);
MODULE_PARM_DESC(fx0_hold_mmss_iface,
		 "Keep an S0 AXI clock reference for the Fx0 MMSS probe diagnostic");

static int msm8226_icc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk *iface;
	int ret;

	if (!fx0_hold_mmss_iface || !of_machine_is_compatible("lg,madai") ||
	    !of_device_is_compatible(dev->of_node, "qcom,msm8226-mmssnoc"))
		return qnoc_probe(pdev);

	/*
	 * The installed kernel sees S0 enabled with no CCF references.
	 * Keep a reference while qnoc_probe() releases its QoS setup vote.
	 */
	iface = devm_clk_get_enabled(dev, "iface");
	if (IS_ERR(iface))
		return dev_err_probe(dev, PTR_ERR(iface), "cannot hold S0 AXI clock\n");

	dev_info(dev, "Fx0 S0 AXI hold acquired before provider probe\n");
	ret = qnoc_probe(pdev);
	if (!ret)
		dev_info(dev, "Fx0 S0 AXI hold retained after provider probe\n");

	return ret;
}

static struct platform_driver msm8226_icc_driver = {
	.probe = msm8226_icc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-msm8226",
		.of_match_table = msm8226_icc_of_match,
	},
};
module_platform_driver(msm8226_icc_driver);

MODULE_DESCRIPTION("Qualcomm MSM8226 multimedia interconnect preview");
MODULE_LICENSE("GPL");
