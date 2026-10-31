// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX179 8 MP CMOS image sensor.
 *
 * The full-resolution register sequence follows NVIDIA's GPLv2 Tegra
 * imx179.c (Copyright (c) 2014, NVIDIA CORPORATION).
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX179_REG_CHIP_ID		CCI_REG16(0x0002)
#define IMX179_CHIP_ID			0x0179
#define IMX179_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX179_MODE_STANDBY		0x00
#define IMX179_MODE_STREAMING		0x01
#define IMX179_REG_HOLD			CCI_REG8(0x0104)
#define IMX179_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX179_REG_ANALOG_GAIN		CCI_REG8(0x0205)
#define IMX179_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX179_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define IMX179_XCLK_FREQ		24000000
#define IMX179_LINK_FREQ		324000000ULL
#define IMX179_NUM_LANES		4
/* 4 lanes x 648 Mbit/s / 10 bits per pixel */
#define IMX179_PIXEL_RATE		259200000

#define IMX179_WIDTH			3280
#define IMX179_HEIGHT			2464
#define IMX179_LINE_LENGTH		3440
#define IMX179_FRAME_LENGTH_DEF		2507
#define IMX179_FRAME_LENGTH_MAX		0xffff
#define IMX179_EXPOSURE_MARGIN		4
#define IMX179_EXPOSURE_MIN		1
#define IMX179_AGAIN_MIN		0
#define IMX179_AGAIN_MAX		224
#define IMX179_AGAIN_DEF		0

static const char * const imx179_supply_names[] = {
	"dovdd",	/* digital I/O, 1.8 V */
	"avdd",		/* analogue, 2.8 V */
	"dvdd",		/* digital core, 1.2 V */
};

static const s64 imx179_link_freqs[] = { IMX179_LINK_FREQ };

static const char * const imx179_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Colour Bars",
	"Fade to Grey Colour Bars",
	"PN9",
};

/* 3280x2464 RAW10, 4 lanes, 24 MHz input, PLL 648 MHz: about 30 frames/s. */
static const struct cci_reg_sequence imx179_mode_3280x2464[] = {
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0202), 0x09 },
	{ CCI_REG8(0x0203), 0xc7 },
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x01 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0309), 0x05 },
	{ CCI_REG8(0x030b), 0x01 },
	{ CCI_REG8(0x030c), 0x00 },
	{ CCI_REG8(0x030d), 0xa2 },
	{ CCI_REG8(0x0340), 0x09 },
	{ CCI_REG8(0x0341), 0xcb },
	{ CCI_REG8(0x0342), 0x0d },
	{ CCI_REG8(0x0343), 0x70 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x0c },
	{ CCI_REG8(0x0349), 0xcf },
	{ CCI_REG8(0x034a), 0x09 },
	{ CCI_REG8(0x034b), 0x9f },
	{ CCI_REG8(0x034c), 0x0c },
	{ CCI_REG8(0x034d), 0xd0 },
	{ CCI_REG8(0x034e), 0x09 },
	{ CCI_REG8(0x034f), 0xa0 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0390), 0x00 },
	{ CCI_REG8(0x0401), 0x00 },
	{ CCI_REG8(0x0405), 0x10 },
	{ CCI_REG8(0x3020), 0x10 },
	{ CCI_REG8(0x3041), 0x15 },
	{ CCI_REG8(0x3042), 0x87 },
	{ CCI_REG8(0x3089), 0x4f },
	{ CCI_REG8(0x3302), 0x01 },
	{ CCI_REG8(0x3309), 0x9a },
	{ CCI_REG8(0x3344), 0x57 },
	{ CCI_REG8(0x3345), 0x1f },
	{ CCI_REG8(0x3362), 0x0a },
	{ CCI_REG8(0x3363), 0x0a },
	{ CCI_REG8(0x3364), 0x00 },
	{ CCI_REG8(0x3368), 0x18 },
	{ CCI_REG8(0x3369), 0x00 },
	{ CCI_REG8(0x3370), 0x77 },
	{ CCI_REG8(0x3371), 0x2f },
	{ CCI_REG8(0x3372), 0x4f },
	{ CCI_REG8(0x3373), 0x2f },
	{ CCI_REG8(0x3374), 0x2f },
	{ CCI_REG8(0x3375), 0x37 },
	{ CCI_REG8(0x3376), 0x9f },
	{ CCI_REG8(0x3377), 0x37 },
	{ CCI_REG8(0x33c8), 0x00 },
	{ CCI_REG8(0x33d4), 0x0c },
	{ CCI_REG8(0x33d5), 0xd0 },
	{ CCI_REG8(0x33d6), 0x09 },
	{ CCI_REG8(0x33d7), 0xa0 },
	{ CCI_REG8(0x4100), 0x0e },
	{ CCI_REG8(0x4108), 0x01 },
	{ CCI_REG8(0x4109), 0x7c },
};

struct imx179 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx179_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
};

static inline struct imx179 *to_imx179(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx179, sd);
}

static int imx179_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx179 *imx179 = container_of(ctrl->handler, struct imx179,
					     ctrl_handler);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int max = IMX179_HEIGHT + ctrl->val - IMX179_EXPOSURE_MARGIN;

		__v4l2_ctrl_modify_range(imx179->exposure,
					 imx179->exposure->minimum, max,
					 imx179->exposure->step,
					 min(imx179->exposure->val, max));
	}

	if (!pm_runtime_get_if_in_use(imx179->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(imx179->regmap, IMX179_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx179->regmap, IMX179_REG_ANALOG_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(imx179->regmap, IMX179_REG_FRAME_LENGTH,
			  IMX179_HEIGHT + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(imx179->regmap, IMX179_REG_TEST_PATTERN, ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx179->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx179_ctrl_ops = {
	.s_ctrl = imx179_set_ctrl,
};

static int imx179_init_controls(struct imx179 *imx179)
{
	struct v4l2_ctrl_handler *hdl = &imx179->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int vblank = IMX179_FRAME_LENGTH_DEF - IMX179_HEIGHT;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &imx179_ctrl_ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(imx179_link_freqs) - 1, 0,
				      imx179_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &imx179_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  IMX179_PIXEL_RATE, IMX179_PIXEL_RATE, 1,
			  IMX179_PIXEL_RATE);

	ctrl = v4l2_ctrl_new_std(hdl, &imx179_ctrl_ops, V4L2_CID_HBLANK,
				 IMX179_LINE_LENGTH - IMX179_WIDTH,
				 IMX179_LINE_LENGTH - IMX179_WIDTH, 1,
				 IMX179_LINE_LENGTH - IMX179_WIDTH);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx179->vblank = v4l2_ctrl_new_std(hdl, &imx179_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   IMX179_FRAME_LENGTH_MAX - IMX179_HEIGHT,
					   1, vblank);

	imx179->exposure = v4l2_ctrl_new_std(hdl, &imx179_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX179_EXPOSURE_MIN,
					     IMX179_FRAME_LENGTH_DEF -
					     IMX179_EXPOSURE_MARGIN, 1,
					     0x09c7);

	v4l2_ctrl_new_std(hdl, &imx179_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX179_AGAIN_MIN, IMX179_AGAIN_MAX, 1,
			  IMX179_AGAIN_DEF);

	v4l2_ctrl_new_std_menu_items(hdl, &imx179_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx179_test_pattern_menu) - 1,
				     0, 0, imx179_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(imx179->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(hdl, &imx179_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx179->sd.ctrl_handler = hdl;

	return 0;
}

static void imx179_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = IMX179_WIDTH;
	fmt->height = IMX179_HEIGHT;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx179_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	imx179_fill_format(v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static int imx179_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int imx179_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = IMX179_WIDTH;
	fse->max_width = IMX179_WIDTH;
	fse->min_height = IMX179_HEIGHT;
	fse->max_height = IMX179_HEIGHT;

	return 0;
}

static int imx179_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	imx179_fill_format(&fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int imx179_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx179 *imx179 = to_imx179(sd);
	int ret;

	ret = pm_runtime_resume_and_get(imx179->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(imx179->regmap, imx179_mode_3280x2464,
				  ARRAY_SIZE(imx179_mode_3280x2464), NULL);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(&imx179->ctrl_handler);
	if (ret)
		goto err_put;

	ret = cci_write(imx179->regmap, IMX179_REG_MODE_SELECT,
			IMX179_MODE_STREAMING, NULL);
	if (ret)
		goto err_put;

	return 0;

err_put:
	pm_runtime_put(imx179->dev);
	return ret;
}

static int imx179_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx179 *imx179 = to_imx179(sd);
	int ret;

	ret = cci_write(imx179->regmap, IMX179_REG_MODE_SELECT,
			IMX179_MODE_STANDBY, NULL);
	pm_runtime_put(imx179->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops imx179_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx179_pad_ops = {
	.enum_mbus_code = imx179_enum_mbus_code,
	.enum_frame_size = imx179_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx179_set_format,
	.enable_streams = imx179_enable_streams,
	.disable_streams = imx179_disable_streams,
};

static const struct v4l2_subdev_ops imx179_subdev_ops = {
	.video = &imx179_video_ops,
	.pad = &imx179_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx179_internal_ops = {
	.init_state = imx179_init_state,
};

static int imx179_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx179 *imx179 = to_imx179(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx179->supplies),
				    imx179->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(imx179->xclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(imx179->supplies),
				       imx179->supplies);
		return ret;
	}

	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(imx179->reset_gpio, 0);
	/* Wait for the sensor to leave reset before the first access. */
	usleep_range(10000, 11000);

	return 0;
}

static int imx179_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx179 *imx179 = to_imx179(sd);

	gpiod_set_value_cansleep(imx179->reset_gpio, 1);
	clk_disable_unprepare(imx179->xclk);
	regulator_bulk_disable(ARRAY_SIZE(imx179->supplies), imx179->supplies);

	return 0;
}

static int imx179_identify(struct imx179 *imx179)
{
	u64 id;
	int ret;

	ret = cci_read(imx179->regmap, IMX179_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(imx179->dev, ret, "Failed to read chip ID\n");

	if (id != IMX179_CHIP_ID)
		return dev_err_probe(imx179->dev, -ENODEV,
				     "Unexpected chip ID 0x%04llx\n", id);

	return 0;
}

static int imx179_check_endpoint(struct imx179 *imx179)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *fwnode;
	unsigned int i;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(imx179->dev), NULL);
	if (!fwnode)
		return dev_err_probe(imx179->dev, -EINVAL, "No endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return dev_err_probe(imx179->dev, ret, "Invalid endpoint\n");

	if (ep.bus.mipi_csi2.num_data_lanes != IMX179_NUM_LANES) {
		ret = dev_err_probe(imx179->dev, -EINVAL,
				    "Only %u data lanes are supported\n",
				    IMX179_NUM_LANES);
		goto out;
	}

	ret = -EINVAL;
	for (i = 0; i < ep.nr_of_link_frequencies; i++)
		if (ep.link_frequencies[i] == IMX179_LINK_FREQ)
			ret = 0;
	if (ret)
		dev_err_probe(imx179->dev, ret, "Link frequency %llu not listed\n",
			      IMX179_LINK_FREQ);
out:
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int imx179_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx179 *imx179;
	unsigned int i;
	int ret;

	imx179 = devm_kzalloc(dev, sizeof(*imx179), GFP_KERNEL);
	if (!imx179)
		return -ENOMEM;

	imx179->dev = dev;
	v4l2_i2c_subdev_init(&imx179->sd, client, &imx179_subdev_ops);
	imx179->sd.internal_ops = &imx179_internal_ops;

	ret = imx179_check_endpoint(imx179);
	if (ret)
		return ret;

	imx179->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx179->regmap))
		return dev_err_probe(dev, PTR_ERR(imx179->regmap),
				     "Failed to initialise CCI\n");

	imx179->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx179->xclk))
		return dev_err_probe(dev, PTR_ERR(imx179->xclk),
				     "Failed to get clock\n");
	if (clk_get_rate(imx179->xclk) != IMX179_XCLK_FREQ) {
		ret = clk_set_rate(imx179->xclk, IMX179_XCLK_FREQ);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to set clock rate\n");
	}
	if (clk_get_rate(imx179->xclk) != IMX179_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "Clock is %lu Hz, need %u Hz\n",
				     clk_get_rate(imx179->xclk),
				     IMX179_XCLK_FREQ);

	for (i = 0; i < ARRAY_SIZE(imx179_supply_names); i++)
		imx179->supplies[i].supply = imx179_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx179->supplies),
				      imx179->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get supplies\n");

	imx179->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx179->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx179->reset_gpio),
				     "Failed to get reset GPIO\n");

	ret = imx179_power_on(dev);
	if (ret)
		return ret;

	ret = imx179_identify(imx179);
	if (ret)
		goto err_power_off;

	ret = imx179_init_controls(imx179);
	if (ret)
		goto err_power_off;

	imx179->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx179->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx179->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx179->sd.entity, 1, &imx179->pad);
	if (ret)
		goto err_ctrls;

	imx179->sd.state_lock = imx179->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx179->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx179->sd);
	if (ret)
		goto err_pm;

	pm_runtime_idle(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx179->sd);
err_entity:
	media_entity_cleanup(&imx179->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx179->ctrl_handler);
err_power_off:
	imx179_power_off(dev);
	return ret;
}

static void imx179_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx179 *imx179 = to_imx179(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx179->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx179_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx179_pm_ops, imx179_power_off,
				 imx179_power_on, NULL);

static const struct of_device_id imx179_of_match[] = {
	{ .compatible = "sony,imx179" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx179_of_match);

static struct i2c_driver imx179_i2c_driver = {
	.driver = {
		.name = "imx179",
		.of_match_table = imx179_of_match,
		.pm = pm_ptr(&imx179_pm_ops),
	},
	.probe = imx179_probe,
	.remove = imx179_remove,
};
module_i2c_driver(imx179_i2c_driver);

MODULE_DESCRIPTION("Sony IMX179 image sensor driver");
MODULE_LICENSE("GPL");
