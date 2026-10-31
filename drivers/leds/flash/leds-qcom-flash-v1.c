// SPDX-License-Identifier: GPL-2.0-only
/*
 * Torch support for the first-generation flash LED module of Qualcomm
 * PM8226 and PM8941 PMICs (peripheral type 0x18, subtypes 1 and 2).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define FLASH_TYPE			0x04
#define FLASH_TYPE_VAL			0x18
#define FLASH_SUBTYPE			0x05
#define FLASH_SUBTYPE_DUAL		0x01
#define FLASH_SUBTYPE_SINGLE		0x02

#define FLASH_MAX_CURR			0x41
#define FLASH_LED0_CURR			0x42
#define FLASH_LED1_CURR			0x43
#define FLASH_ENABLE_CTRL		0x46
#define FLASH_ENABLE_MASK		GENMASK(7, 5)
#define FLASH_ENABLE_ALL		GENMASK(7, 5)
#define FLASH_STROBE_CTRL		0x47
#define FLASH_STROBE_TORCH		GENMASK(7, 6)
#define FLASH_STROBE_HW			BIT(2)
#define FLASH_SEC_ACCESS		0xd0
#define FLASH_SEC_UNLOCK		0xa5
#define FLASH_TORCH			0xe4
#define FLASH_TORCH_MASK		GENMASK(1, 0)
#define FLASH_TORCH_ENABLE		0x00
#define FLASH_TORCH_DISABLE		0x03

/* Torch current: 16 steps up to 200 mA (12.5 mA per step). */
#define TORCH_MAX_LEVEL			0x0f
#define TORCH_MAX_UA			200000

struct qcom_flash_v1 {
	struct led_classdev cdev;
	struct regmap *regmap;
	struct regulator *vin;
	struct mutex lock;	/* serialises torch register sequences */
	u32 base;
	bool vin_on;
};

static int qcom_flash_v1_torch_mode(struct qcom_flash_v1 *flash, bool on)
{
	int ret;

	/* The torch switch is behind the module's secure access lock. */
	ret = regmap_write(flash->regmap, flash->base + FLASH_SEC_ACCESS,
			   FLASH_SEC_UNLOCK);
	if (ret)
		return ret;

	return regmap_update_bits(flash->regmap, flash->base + FLASH_TORCH,
				  FLASH_TORCH_MASK,
				  on ? FLASH_TORCH_ENABLE : FLASH_TORCH_DISABLE);
}

static int qcom_flash_v1_off(struct qcom_flash_v1 *flash)
{
	int ret;

	ret = regmap_update_bits(flash->regmap, flash->base + FLASH_STROBE_CTRL,
				 FLASH_STROBE_TORCH, 0);
	if (ret)
		return ret;

	ret = qcom_flash_v1_torch_mode(flash, false);
	if (ret)
		return ret;

	/* Let the current ramp down before disabling the module. */
	usleep_range(2200, 2500);
	ret = regmap_update_bits(flash->regmap, flash->base + FLASH_ENABLE_CTRL,
				 FLASH_ENABLE_MASK, 0);
	if (ret)
		return ret;

	if (flash->vin_on) {
		regulator_disable(flash->vin);
		flash->vin_on = false;
	}

	return 0;
}

static int qcom_flash_v1_on(struct qcom_flash_v1 *flash, unsigned int level)
{
	struct regmap *map = flash->regmap;
	u32 base = flash->base;
	int ret;

	if (flash->vin && !flash->vin_on) {
		ret = regulator_enable(flash->vin);
		if (ret)
			return ret;
		flash->vin_on = true;
	}

	ret = qcom_flash_v1_torch_mode(flash, true);
	if (ret)
		return ret;

	/* Torch drives both channel outputs at the same programmed current. */
	ret = regmap_write(map, base + FLASH_LED0_CURR, level);
	if (ret)
		return ret;
	ret = regmap_write(map, base + FLASH_LED1_CURR, level);
	if (ret)
		return ret;
	ret = regmap_write(map, base + FLASH_MAX_CURR, TORCH_MAX_LEVEL);
	if (ret)
		return ret;
	ret = regmap_update_bits(map, base + FLASH_ENABLE_CTRL,
				 FLASH_ENABLE_MASK, FLASH_ENABLE_ALL);
	if (ret)
		return ret;

	return regmap_update_bits(map, base + FLASH_STROBE_CTRL,
				  FLASH_STROBE_TORCH | FLASH_STROBE_HW,
				  FLASH_STROBE_TORCH);
}

static int qcom_flash_v1_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct qcom_flash_v1 *flash = container_of(cdev, struct qcom_flash_v1, cdev);
	int ret;

	mutex_lock(&flash->lock);
	if (brightness)
		ret = qcom_flash_v1_on(flash, brightness - 1);
	else
		ret = qcom_flash_v1_off(flash);
	if (ret && brightness)
		qcom_flash_v1_off(flash);
	mutex_unlock(&flash->lock);

	return ret;
}

static int qcom_flash_v1_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct led_init_data init_data = {};
	struct qcom_flash_v1 *flash;
	struct fwnode_handle *child;
	unsigned int val;
	u32 max_ua = TORCH_MAX_UA;
	int ret;

	flash = devm_kzalloc(dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	flash->regmap = dev_get_regmap(dev->parent, NULL);
	if (!flash->regmap)
		return dev_err_probe(dev, -ENODEV, "No parent regmap\n");

	ret = device_property_read_u32(dev, "reg", &flash->base);
	if (ret)
		return dev_err_probe(dev, ret, "No base address\n");

	ret = regmap_read(flash->regmap, flash->base + FLASH_TYPE, &val);
	if (ret)
		return ret;
	if (val != FLASH_TYPE_VAL)
		return dev_err_probe(dev, -ENODEV, "Unexpected type 0x%02x\n", val);

	ret = regmap_read(flash->regmap, flash->base + FLASH_SUBTYPE, &val);
	if (ret)
		return ret;
	if (val != FLASH_SUBTYPE_DUAL && val != FLASH_SUBTYPE_SINGLE)
		return dev_err_probe(dev, -ENODEV, "Unsupported subtype 0x%02x\n", val);

	flash->vin = devm_regulator_get_optional(dev, "vin");
	if (IS_ERR(flash->vin)) {
		if (PTR_ERR(flash->vin) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(flash->vin),
					     "Failed to get vin supply\n");
		flash->vin = NULL;
	}

	ret = devm_mutex_init(dev, &flash->lock);
	if (ret)
		return ret;

	child = device_get_next_child_node(dev, NULL);
	if (!child)
		return dev_err_probe(dev, -EINVAL, "No LED child node\n");

	fwnode_property_read_u32(child, "led-max-microamp", &max_ua);
	max_ua = clamp(max_ua, TORCH_MAX_UA / (TORCH_MAX_LEVEL + 1), TORCH_MAX_UA);

	flash->cdev.max_brightness = max_ua * (TORCH_MAX_LEVEL + 1) / TORCH_MAX_UA;
	flash->cdev.brightness_set_blocking = qcom_flash_v1_brightness_set;
	init_data.fwnode = child;
	init_data.devicename = "qcom-flash";
	init_data.default_label = "torch";

	/* Start from a known off state. */
	ret = qcom_flash_v1_off(flash);
	if (ret) {
		fwnode_handle_put(child);
		return ret;
	}

	ret = devm_led_classdev_register_ext(dev, &flash->cdev, &init_data);
	fwnode_handle_put(child);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register LED\n");

	platform_set_drvdata(pdev, flash);

	return 0;
}

static void qcom_flash_v1_remove(struct platform_device *pdev)
{
	struct qcom_flash_v1 *flash = platform_get_drvdata(pdev);

	mutex_lock(&flash->lock);
	qcom_flash_v1_off(flash);
	mutex_unlock(&flash->lock);
}

static const struct of_device_id qcom_flash_v1_of_match[] = {
	{ .compatible = "qcom,pm8941-flash-led" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_flash_v1_of_match);

static struct platform_driver qcom_flash_v1_driver = {
	.driver = {
		.name = "leds-qcom-flash-v1",
		.of_match_table = qcom_flash_v1_of_match,
	},
	.probe = qcom_flash_v1_probe,
	.remove = qcom_flash_v1_remove,
};
module_platform_driver(qcom_flash_v1_driver);

MODULE_DESCRIPTION("Qualcomm first-generation PMIC flash LED torch driver");
MODULE_LICENSE("GPL");
