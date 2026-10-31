// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm WCD9306/WCD9302 (Tapan) audio codec.
 *
 * Identification and power-up only: enable the static supplies, reset the
 * codec, read its chip ID over SLIMbus and run the core bring-up sequence.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slimbus.h>

#include "wcd9306.h"

#define SLIM_MANF_ID_QCOM		0x217
#define SLIM_PROD_CODE_WCD9306		0xe0
#define WCD9306_ID_MAJOR		0x0103

static const char * const wcd9306_supplies[] = {
	"vdd-h", "vdd-px", "vdd-cx",
};

struct wcd9306_codec {
	struct device *dev;
	struct slim_device *slim;
	struct slim_device *slim_ifc_dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(wcd9306_supplies)];
};

static bool wcd9306_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= WCD9306_MAX_REGISTER && wcd9306_readable[reg];
}

static bool wcd9306_volatile_reg(struct device *dev, unsigned int reg)
{
	/* Top-level registers are also written by the codec core. */
	if (reg < 0x100 || reg >= WCD9306_CDC_MBHC_EN_CTL)
		return true;

	/* IIR and ANC coefficients go through auto-incrementing ports. */
	if (reg >= WCD9306_CDC_IIR1_COEF_B1_CTL &&
	    reg <= WCD9306_CDC_IIR2_COEF_B2_CTL)
		return true;
	if (reg >= WCD9306_CDC_ANC1_IIR_B1_CTL &&
	    reg <= WCD9306_CDC_ANC1_LPF_B2_CTL)
		return true;
	if (reg >= WCD9306_CDC_ANC2_IIR_B1_CTL &&
	    reg <= WCD9306_CDC_ANC2_LPF_B2_CTL)
		return true;

	switch (reg) {
	/* Digital gains must be rewritten even when unchanged. */
	case WCD9306_CDC_RX1_VOL_CTL_B2_CTL:
	case WCD9306_CDC_RX2_VOL_CTL_B2_CTL:
	case WCD9306_CDC_RX3_VOL_CTL_B2_CTL:
	case WCD9306_CDC_RX4_VOL_CTL_B2_CTL:
	case WCD9306_CDC_TX1_VOL_CTL_GAIN:
	case WCD9306_CDC_TX2_VOL_CTL_GAIN:
	case WCD9306_CDC_TX3_VOL_CTL_GAIN:
	case WCD9306_CDC_TX4_VOL_CTL_GAIN:
	case WCD9306_RX_HPH_L_STATUS:
	case WCD9306_RX_HPH_R_STATUS:
	case WCD9306_MBHC_INSERT_DET_STATUS:
		return true;
	}

	return false;
}

static const struct regmap_config wcd9306_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = WCD9306_MAX_REGISTER,
	.reg_defaults = wcd9306_defaults,
	.num_reg_defaults = ARRAY_SIZE(wcd9306_defaults),
	.readable_reg = wcd9306_readable_reg,
	.volatile_reg = wcd9306_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
	.can_multi_write = true,
};

static int wcd9306_power_on_reset(struct wcd9306_codec *wcd)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(wcd->supplies), wcd->supplies);
	if (ret)
		return dev_err_probe(wcd->dev, ret, "Failed to enable supplies\n");

	/* Same timing as the vendor codec core: 20 ms in and out of reset. */
	gpiod_set_value_cansleep(wcd->reset_gpio, 1);
	msleep(20);
	gpiod_set_value_cansleep(wcd->reset_gpio, 0);
	msleep(20);

	return 0;
}

static int wcd9306_identify(struct wcd9306_codec *wcd)
{
	unsigned int b0, b1, b2, b3, version;
	int ret;

	ret = regmap_read(wcd->regmap, WCD9306_CHIP_ID_BYTE_0, &b0);
	ret = ret ?: regmap_read(wcd->regmap, WCD9306_CHIP_ID_BYTE_1, &b1);
	ret = ret ?: regmap_read(wcd->regmap, WCD9306_CHIP_ID_BYTE_2, &b2);
	ret = ret ?: regmap_read(wcd->regmap, WCD9306_CHIP_ID_BYTE_3, &b3);
	ret = ret ?: regmap_read(wcd->regmap, WCD9306_CHIP_VERSION, &version);
	if (ret)
		return dev_err_probe(wcd->dev, ret, "Failed to read chip ID\n");

	if ((b3 << 8 | b2) != WCD9306_ID_MAJOR)
		return dev_err_probe(wcd->dev, -ENODEV,
				     "Unexpected chip ID %02x %02x %02x %02x\n",
				     b0, b1, b2, b3);

	dev_info(wcd->dev, "WCD9306 family codec, minor ID 0x%04x, version 0x%02x\n",
		 b1 << 8 | b0, version);

	return 0;
}

static void wcd9306_bring_up(struct wcd9306_codec *wcd)
{
	regmap_write(wcd->regmap, WCD9306_LEAKAGE_CTL, 0x04);
	regmap_write(wcd->regmap, WCD9306_CDC_CTL, 0x00);
	usleep_range(5000, 5100);
	regmap_write(wcd->regmap, WCD9306_CDC_CTL, 0x03);
	regmap_write(wcd->regmap, WCD9306_LEAKAGE_CTL, 0x03);
}

static int wcd9306_slim_probe(struct slim_device *slim)
{
	struct device *dev = &slim->dev;
	struct wcd9306_codec *wcd;
	int i, ret;

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return -ENOMEM;

	wcd->dev = dev;

	wcd->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(wcd->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(wcd->reset_gpio),
				     "Failed to get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(wcd9306_supplies); i++)
		wcd->supplies[i].supply = wcd9306_supplies[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(wcd->supplies),
				      wcd->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get supplies\n");

	ret = wcd9306_power_on_reset(wcd);
	if (ret)
		return ret;

	dev_set_drvdata(dev, wcd);

	return 0;
}

static int wcd9306_slim_status(struct slim_device *sdev,
			       enum slim_device_status status)
{
	struct device *dev = &sdev->dev;
	struct wcd9306_codec *wcd = dev_get_drvdata(dev);
	struct device_node *ifc_dev_np;
	int ret;

	if (status != SLIM_DEVICE_STATUS_UP || wcd->regmap)
		return 0;

	ifc_dev_np = of_parse_phandle(dev->of_node, "slim-ifc-dev", 0);
	if (!ifc_dev_np)
		return dev_err_probe(dev, -EINVAL, "No interface device\n");

	wcd->slim = sdev;
	wcd->slim_ifc_dev = of_slim_get_device(sdev->ctrl, ifc_dev_np);
	of_node_put(ifc_dev_np);
	if (!wcd->slim_ifc_dev)
		return dev_err_probe(dev, -EINVAL,
				     "Unable to get interface device\n");

	slim_get_logical_addr(wcd->slim_ifc_dev);

	wcd->regmap = devm_regmap_init_slimbus(sdev, &wcd9306_regmap_config);
	if (IS_ERR(wcd->regmap))
		return dev_err_probe(dev, PTR_ERR(wcd->regmap),
				     "Failed to allocate register map\n");

	ret = wcd9306_identify(wcd);
	if (ret)
		return ret;

	wcd9306_bring_up(wcd);

	return 0;
}

static void wcd9306_slim_remove(struct slim_device *slim)
{
	struct wcd9306_codec *wcd = dev_get_drvdata(&slim->dev);

	gpiod_set_value_cansleep(wcd->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(wcd->supplies), wcd->supplies);
}

static const struct slim_device_id wcd9306_slim_id[] = {
	{ SLIM_MANF_ID_QCOM, SLIM_PROD_CODE_WCD9306, 0x1, 0x0 },
	{}
};
MODULE_DEVICE_TABLE(slim, wcd9306_slim_id);

static struct slim_driver wcd9306_slim_driver = {
	.driver = {
		.name = "wcd9306-slim",
	},
	.probe = wcd9306_slim_probe,
	.remove = wcd9306_slim_remove,
	.device_status = wcd9306_slim_status,
	.id_table = wcd9306_slim_id,
};
module_slim_driver(wcd9306_slim_driver);

MODULE_DESCRIPTION("Qualcomm WCD9306 (Tapan) audio codec");
MODULE_LICENSE("GPL");
