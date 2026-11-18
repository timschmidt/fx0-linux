// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017-18 Linaro Limited

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reboot-mode.h>
#include <linux/regmap.h>

#define PON_SOFT_RB_SPARE		0x8f
#define PON_REVISION			0x01
#define PON_PERIPHERAL_TYPE		0x04
#define PON_POWER_ON_REASON		0x08
#define PON_WARM_RESET_REASON1		0x0a
#define PON_WATCHDOG_S1_TIMER		0x54
#define PON_S3_SOURCE			0x74

static bool fx0_watchdog_inventory;
module_param(fx0_watchdog_inventory, bool, 0400);
MODULE_PARM_DESC(fx0_watchdog_inventory, "Expose read-only PM8226 reset and watchdog registers on the Fx0");

#define GEN1_REASON_SHIFT		2
#define GEN2_REASON_SHIFT		1

#define NO_REASON_SHIFT			0

struct qcom_pon {
	struct device *dev;
	struct regmap *regmap;
	u32 baseaddr;
	struct reboot_mode_driver reboot_mode;
	long reason_shift;
};

static ssize_t watchdog_inventory_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct qcom_pon *pon = dev_get_drvdata(dev);
	unsigned int revision;
	u8 type[2], watchdog[4];
	int ret;

	ret = regmap_read(pon->regmap, pon->baseaddr + PON_REVISION, &revision);
	if (ret)
		return ret;
	ret = regmap_bulk_read(pon->regmap, pon->baseaddr + PON_PERIPHERAL_TYPE,
			       type, ARRAY_SIZE(type));
	if (ret)
		return ret;
	/* S1, S2, reset type and enable. Leave the adjacent pet register alone. */
	ret = regmap_bulk_read(pon->regmap, pon->baseaddr + PON_WATCHDOG_S1_TIMER,
			       watchdog, ARRAY_SIZE(watchdog));
	if (ret)
		return ret;

	return sysfs_emit(buf, "%02x %02x %02x %02x %02x %02x %02x\n",
			  revision, type[0], type[1], watchdog[0],
			  watchdog[1], watchdog[2], watchdog[3]);
}
static DEVICE_ATTR_RO(watchdog_inventory);

static ssize_t s3_inventory_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct qcom_pon *pon = dev_get_drvdata(dev);
	u8 s3[2];
	int ret;

	/* Read only the S3 source and the adjacent debounce setting. */
	ret = regmap_bulk_read(pon->regmap, pon->baseaddr + PON_S3_SOURCE,
			       s3, ARRAY_SIZE(s3));
	if (ret)
		return ret;

	return sysfs_emit(buf, "%02x %02x\n", s3[0], s3[1]);
}
static DEVICE_ATTR_RO(s3_inventory);

static ssize_t reset_reason_inventory_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct qcom_pon *pon = dev_get_drvdata(dev);
	unsigned int power_on;
	u8 reset[4];
	int ret;

	ret = regmap_read(pon->regmap, pon->baseaddr + PON_POWER_ON_REASON,
			  &power_on);
	if (ret)
		return ret;

	/* Two warm-reset bytes followed by two power-off bytes. */
	ret = regmap_bulk_read(pon->regmap, pon->baseaddr + PON_WARM_RESET_REASON1,
			       reset, ARRAY_SIZE(reset));
	if (ret)
		return ret;

	return sysfs_emit(buf, "%02x %02x %02x %02x %02x\n",
			  power_on, reset[0], reset[1], reset[2], reset[3]);
}
static DEVICE_ATTR_RO(reset_reason_inventory);

static struct attribute *inventory_attrs[] = {
	&dev_attr_watchdog_inventory.attr,
	&dev_attr_s3_inventory.attr,
	&dev_attr_reset_reason_inventory.attr,
	NULL,
};
static const struct attribute_group inventory_group = {
	.attrs = inventory_attrs,
};

static int qcom_pon_reboot_mode_write(struct reboot_mode_driver *reboot,
				    unsigned int magic)
{
	struct qcom_pon *pon = container_of
			(reboot, struct qcom_pon, reboot_mode);
	int ret;

	ret = regmap_update_bits(pon->regmap,
				 pon->baseaddr + PON_SOFT_RB_SPARE,
				 GENMASK(7, pon->reason_shift),
				 magic << pon->reason_shift);
	if (ret < 0)
		dev_err(pon->dev, "update reboot mode bits failed\n");

	return ret;
}

static int qcom_pon_probe(struct platform_device *pdev)
{
	struct qcom_pon *pon;
	long reason_shift;
	int error;

	pon = devm_kzalloc(&pdev->dev, sizeof(*pon), GFP_KERNEL);
	if (!pon)
		return -ENOMEM;

	pon->dev = &pdev->dev;

	pon->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pon->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	error = of_property_read_u32(pdev->dev.of_node, "reg",
				     &pon->baseaddr);
	if (error)
		return error;

	reason_shift = (long)of_device_get_match_data(&pdev->dev);

	if (reason_shift != NO_REASON_SHIFT) {
		pon->reboot_mode.dev = &pdev->dev;
		pon->reason_shift = reason_shift;
		pon->reboot_mode.write = qcom_pon_reboot_mode_write;
		error = devm_reboot_mode_register(&pdev->dev, &pon->reboot_mode);
		if (error) {
			dev_err(&pdev->dev, "can't register reboot mode\n");
			return error;
		}
	}

	platform_set_drvdata(pdev, pon);

	if (fx0_watchdog_inventory && pon->baseaddr == 0x800 &&
	    of_machine_is_compatible("lg,madai") &&
	    of_device_is_compatible(pdev->dev.of_node, "qcom,pm8916-pon") &&
	    of_device_is_compatible(pdev->dev.parent->of_node, "qcom,pm8226")) {
		error = devm_device_add_group(&pdev->dev, &inventory_group);
		if (error)
			return error;
	}

	return devm_of_platform_populate(&pdev->dev);
}

static const struct of_device_id qcom_pon_id_table[] = {
	{ .compatible = "qcom,pm8916-pon", .data = (void *)GEN1_REASON_SHIFT },
	{ .compatible = "qcom,pm8941-pon", .data = (void *)NO_REASON_SHIFT },
	{ .compatible = "qcom,pms405-pon", .data = (void *)GEN1_REASON_SHIFT },
	{ .compatible = "qcom,pm8998-pon", .data = (void *)GEN2_REASON_SHIFT },
	{ .compatible = "qcom,pmk8350-pon", .data = (void *)GEN2_REASON_SHIFT },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_pon_id_table);

static struct platform_driver qcom_pon_driver = {
	.probe = qcom_pon_probe,
	.driver = {
		.name = "qcom-pon",
		.of_match_table = qcom_pon_id_table,
	},
};
module_platform_driver(qcom_pon_driver);

MODULE_DESCRIPTION("Qualcomm Power On driver");
MODULE_LICENSE("GPL v2");
