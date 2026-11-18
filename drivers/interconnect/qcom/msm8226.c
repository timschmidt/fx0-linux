// SPDX-License-Identifier: GPL-2.0-only
/*
 * MSM8226 multimedia-to-memory topology from the CAF MSM8226 bus tables.
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 */

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

static bool fx0_trace;
module_param(fx0_trace, bool, 0400);
MODULE_PARM_DESC(fx0_trace, "Trace MMSS provider registration and AXI votes on Fx0");

static bool msm8226_fx0_trace(void)
{
	return fx0_trace && of_machine_is_compatible("lg,madai");
}

static int msm8226_set_mmss_rate(struct clk *clk, unsigned long rate)
{
	bool trace = msm8226_fx0_trace();
	long actual;
	int ret;

	if (trace)
		pr_info("fx0-mmss-trace: AXI vote begin rate=%lu\n", rate);

	if (rate) {
		actual = clk_round_rate(clk, rate);
		if (actual < 0)
			return actual;
		if ((unsigned long)actual < rate)
			return -ERANGE;
	}

	/* Display and camera clocks can also propagate rates to this root. */
	ret = clk_set_min_rate(clk, rate);
	if (trace)
		pr_info("fx0-mmss-trace: AXI vote end ret=%d\n", ret);

	return ret;
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

static struct platform_driver msm8226_icc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-msm8226",
		.of_match_table = msm8226_icc_of_match,
	},
};

static int __init msm8226_icc_driver_init(void)
{
	bool trace = msm8226_fx0_trace();
	int ret;

	if (trace)
		pr_info("fx0-mmss-trace: platform driver registration begin\n");
	ret = platform_driver_register(&msm8226_icc_driver);
	if (trace)
		pr_info("fx0-mmss-trace: platform driver registration end ret=%d\n", ret);

	return ret;
}
module_init(msm8226_icc_driver_init);

static void __exit msm8226_icc_driver_exit(void)
{
	platform_driver_unregister(&msm8226_icc_driver);
}
module_exit(msm8226_icc_driver_exit);

MODULE_DESCRIPTION("Qualcomm MSM8226 multimedia interconnect preview");
MODULE_LICENSE("GPL");
