// SPDX-License-Identifier: GPL-2.0-only
/*
 * LG Fx0 JDI in-cell panel, production board revisions.
 * Initialization data and power sequencing derived from the LG madai kernel.
 * Copyright (c) 2013, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2026 Timothy Schmidt
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct lg_fx0 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *vdd;
	struct gpio_desc *reset;
	struct gpio_desc *io_enable;
	struct gpio_desc *positive_enable;
	struct gpio_desc *negative_enable;
};

static inline struct lg_fx0 *to_lg_fx0(struct drm_panel *panel)
{
	return container_of(panel, struct lg_fx0, panel);
}

static int lg_fx0_on(struct lg_fx0 *priv)
{
	struct mipi_dsi_multi_context ctx = { .dsi = priv->dsi };

	priv->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* Production revisions use the stock touch-countermeasure sequence. */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x51, 0xff);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x53, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x55, 0x00);
	mipi_dsi_generic_write_seq_multi(&ctx, 0xb0, 0x04);
	mipi_dsi_generic_write_seq_multi(&ctx,
		0xec, 0x29, 0x00, 0x2c, 0x2b, 0x01, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x09, 0x00);
	mipi_dsi_generic_write_seq_multi(&ctx,
		0xee, 0x1c, 0x02, 0x02, 0x03, 0x00, 0x10, 0x00, 0x02, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x03);
	mipi_dsi_generic_write_seq_multi(&ctx, 0xc1, 0x40, 0x61, 0x01);
	mipi_dsi_generic_write_seq_multi(&ctx,
		0xc7, 0x00, 0x0a, 0x16, 0x20, 0x2c, 0x39, 0x43, 0x52, 0x36,
		0x3e, 0x4b, 0x58, 0x5a, 0x5f, 0x67, 0x00, 0x0a, 0x16, 0x20,
		0x2c, 0x39, 0x43, 0x52, 0x36, 0x3e, 0x4b, 0x58, 0x5a, 0x5f,
		0x67);
	mipi_dsi_generic_write_seq_multi(&ctx,
		0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00);
	mipi_dsi_generic_write_seq_multi(&ctx, 0xd6, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x36, 0x00);
	mipi_dsi_generic_write_seq_multi(&ctx,
		0xca, 0x01, 0x70, 0x9a, 0x95, 0xa2, 0x98, 0x95, 0x8f, 0x3f,
		0x3f, 0x80, 0x80, 0x08, 0x80, 0x08, 0x3f, 0x08, 0x90, 0x0c,
		0x0c, 0x0a, 0x06, 0x04, 0x04, 0x00, 0xc8, 0x10, 0x10, 0x3f,
		0x3f, 0x3f, 0x3f);
	/* The production firmware sends display-on before sleep-out. */
	mipi_dsi_dcs_set_display_on_multi(&ctx);
	mipi_dsi_dcs_exit_sleep_mode_multi(&ctx);
	mipi_dsi_msleep(&ctx, 120);

	return ctx.accum_err;
}

static void lg_fx0_power_off(struct lg_fx0 *priv)
{
	gpiod_direction_output(priv->reset, 1);
	gpiod_direction_output(priv->negative_enable, 0);
	usleep_range(10000, 11000);
	gpiod_direction_output(priv->positive_enable, 0);
	usleep_range(10000, 11000);
	gpiod_direction_output(priv->io_enable, 0);
	regulator_disable(priv->vdd);
}

static int lg_fx0_prepare(struct drm_panel *panel)
{
	struct lg_fx0 *priv = to_lg_fx0(panel);
	int ret;

	ret = regulator_enable(priv->vdd);
	if (ret)
		return ret;

	ret = gpiod_direction_output(priv->reset, 1);
	if (ret)
		goto power_off;
	ret = gpiod_direction_output(priv->io_enable, 1);
	if (ret)
		goto power_off;
	usleep_range(10000, 11000);
	ret = gpiod_direction_output(priv->positive_enable, 1);
	if (ret)
		goto power_off;
	usleep_range(10000, 11000);
	ret = gpiod_direction_output(priv->negative_enable, 1);
	if (ret)
		goto power_off;
	usleep_range(10000, 11000);

	gpiod_set_value_cansleep(priv->reset, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(priv->reset, 0);
	msleep(20);
	gpiod_set_value_cansleep(priv->reset, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(priv->reset, 0);
	msleep(20);

	ret = lg_fx0_on(priv);
	if (!ret)
		return 0;

power_off:
	lg_fx0_power_off(priv);
	return ret;
}

static int lg_fx0_disable(struct drm_panel *panel)
{
	struct lg_fx0 *priv = to_lg_fx0(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = priv->dsi };

	priv->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&ctx);
	mipi_dsi_msleep(&ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&ctx);
	mipi_dsi_msleep(&ctx, 80);

	return ctx.accum_err;
}

static int lg_fx0_unprepare(struct drm_panel *panel)
{
	lg_fx0_power_off(to_lg_fx0(panel));

	return 0;
}

static const struct drm_display_mode lg_fx0_mode = {
	.clock = 70170,
	.hdisplay = 720,
	.hsync_start = 720 + 140,
	.hsync_end = 720 + 140 + 8,
	.htotal = 720 + 140 + 8 + 40,
	.vdisplay = 1280,
	.vsync_start = 1280 + 4,
	.vsync_end = 1280 + 4 + 1,
	.vtotal = 1280 + 4 + 1 + 3,
	.width_mm = 58,
	.height_mm = 103,
};

static int lg_fx0_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &lg_fx0_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs lg_fx0_funcs = {
	.prepare = lg_fx0_prepare,
	.disable = lg_fx0_disable,
	.unprepare = lg_fx0_unprepare,
	.get_modes = lg_fx0_get_modes,
};

static int lg_fx0_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lg_fx0 *priv;
	int ret;

	priv = devm_drm_panel_alloc(dev, struct lg_fx0, panel,
				    &lg_fx0_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	priv->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(priv->vdd))
		return dev_err_probe(dev, PTR_ERR(priv->vdd), "Failed to get vdd\n");

	priv->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset), "Failed to get reset\n");

	priv->io_enable = devm_gpiod_get(dev, "io-enable", GPIOD_ASIS);
	if (IS_ERR(priv->io_enable))
		return dev_err_probe(dev, PTR_ERR(priv->io_enable), "Failed to get IO enable\n");

	priv->positive_enable = devm_gpiod_get(dev, "positive-enable", GPIOD_ASIS);
	if (IS_ERR(priv->positive_enable))
		return dev_err_probe(dev, PTR_ERR(priv->positive_enable),
				     "Failed to get positive bias enable\n");

	priv->negative_enable = devm_gpiod_get(dev, "negative-enable", GPIOD_ASIS);
	if (IS_ERR(priv->negative_enable))
		return dev_err_probe(dev, PTR_ERR(priv->negative_enable),
				     "Failed to get negative bias enable\n");

	priv->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, priv);
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_NO_EOT_PACKET;

	ret = drm_panel_of_backlight(&priv->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	priv->panel.prepare_prev_first = true;
	drm_panel_add(&priv->panel);
	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&priv->panel);
		return dev_err_probe(dev, ret, "Failed to attach DSI\n");
	}

	return 0;
}

static void lg_fx0_remove(struct mipi_dsi_device *dsi)
{
	struct lg_fx0 *priv = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&priv->panel);
}

static const struct of_device_id lg_fx0_of_match[] = {
	{ .compatible = "lg,fx0-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg_fx0_of_match);

static struct mipi_dsi_driver lg_fx0_driver = {
	.probe = lg_fx0_probe,
	.remove = lg_fx0_remove,
	.driver = {
		.name = "panel-lg-fx0",
		.of_match_table = lg_fx0_of_match,
	},
};
module_mipi_dsi_driver(lg_fx0_driver);

MODULE_AUTHOR("Timothy Schmidt");
MODULE_DESCRIPTION("LG Fx0 JDI in-cell DSI panel");
MODULE_LICENSE("GPL");
