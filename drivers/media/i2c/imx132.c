// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX132 2.4 MP CMOS image sensor.
 *
 * The register sequence follows NVIDIA's GPLv2 Tegra imx132.c (Copyright (c)
 * 2012-2014, NVIDIA CORPORATION). The lane selection and the PLL relationship
 * follow the historical AtomISP imx driver.
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

#define IMX132_REG_CHIP_ID		CCI_REG16(0x0000)
#define IMX132_CHIP_ID			0x0132
#define IMX132_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX132_MODE_STANDBY		0x00
#define IMX132_MODE_STREAMING		0x01
#define IMX132_REG_HOLD			CCI_REG8(0x0104)
#define IMX132_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX132_REG_ANALOG_GAIN		CCI_REG8(0x0205)
#define IMX132_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX132_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define IMX132_XCLK_FREQ		24000000
#define IMX132_LINK_FREQ		198000000ULL
#define IMX132_NUM_LANES		1
/* 1 lane x 396 Mbit/s / 10 bits per pixel */
#define IMX132_PIXEL_RATE		39600000

#define IMX132_WIDTH			1976
#define IMX132_HEIGHT			1200
#define IMX132_LINE_LENGTH		2248
#define IMX132_FRAME_LENGTH_DEF		1226
#define IMX132_FRAME_LENGTH_MAX		0xffff
#define IMX132_EXPOSURE_MARGIN		4
#define IMX132_EXPOSURE_MIN		1
#define IMX132_AGAIN_MIN		0
#define IMX132_AGAIN_MAX		224
#define IMX132_AGAIN_DEF		0

static const char * const imx132_supply_names[] = {
	"dovdd",	/* digital I/O, 1.8 V */
	"avdd",		/* analogue, 2.8 V */
	"dvdd",		/* digital core, 1.2 V */
};

static const s64 imx132_link_freqs[] = { IMX132_LINK_FREQ };

static const char * const imx132_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Colour Bars",
	"Fade to Grey Colour Bars",
	"PN9",
};

/*
 * 1976x1200 RAW10 on one lane, 24 MHz input. The PLL runs at
 * 24 MHz / 2 * 33 = 396 Mbit/s per lane; with one lane the pixel rate is
 * 396 / 10 = 39.6 Mpixel/s, about 14 frames/s at 2248x1226.
 */
static const struct cci_reg_sequence imx132_common_regs[] = {
	/* Global settings */
	{ CCI_REG8(0x3087), 0x53 },
	{ CCI_REG8(0x308b), 0x5a },
	{ CCI_REG8(0x3094), 0x11 },
	{ CCI_REG8(0x309d), 0xa4 },
	{ CCI_REG8(0x30aa), 0x01 },
	{ CCI_REG8(0x30c6), 0x00 },
	{ CCI_REG8(0x30c7), 0x00 },
	{ CCI_REG8(0x3118), 0x2f },
	{ CCI_REG8(0x312a), 0x00 },
	{ CCI_REG8(0x312b), 0x0b },
	{ CCI_REG8(0x312c), 0x0b },
	{ CCI_REG8(0x312d), 0x13 },
	/* PLL */
	{ CCI_REG8(0x0305), 0x02 },
	{ CCI_REG8(0x0307), 0x21 },
	{ CCI_REG8(0x30a4), 0x02 },	/* VT divider /1 for one lane */
	{ CCI_REG8(0x303c), 0x4b },
	/* Timing and readout */
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0342), 0x08 },
	{ CCI_REG8(0x0343), 0xc8 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x07 },
	{ CCI_REG8(0x0349), 0xb7 },
	{ CCI_REG8(0x034a), 0x04 },
	{ CCI_REG8(0x034b), 0xaf },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x303d), 0x10 },
	{ CCI_REG8(0x303e), 0x4a },
	{ CCI_REG8(0x3040), 0x08 },
	{ CCI_REG8(0x3041), 0x97 },
	{ CCI_REG8(0x3048), 0x00 },
	{ CCI_REG8(0x304c), 0x2f },
	{ CCI_REG8(0x304d), 0x02 },
	{ CCI_REG8(0x3064), 0x92 },
	{ CCI_REG8(0x306a), 0x10 },
	{ CCI_REG8(0x309b), 0x00 },
	{ CCI_REG8(0x309e), 0x41 },
	{ CCI_REG8(0x30a0), 0x10 },
	{ CCI_REG8(0x30a1), 0x0b },
	{ CCI_REG8(0x30b2), 0x00 },
	{ CCI_REG8(0x30d5), 0x00 },
	{ CCI_REG8(0x30d6), 0x00 },
	{ CCI_REG8(0x30d7), 0x00 },
	{ CCI_REG8(0x30d8), 0x00 },
	{ CCI_REG8(0x30d9), 0x00 },
	{ CCI_REG8(0x30da), 0x00 },
	{ CCI_REG8(0x30db), 0x00 },
	{ CCI_REG8(0x30dc), 0x00 },
	{ CCI_REG8(0x30dd), 0x00 },
	{ CCI_REG8(0x30de), 0x00 },
	{ CCI_REG8(0x3102), 0x0c },
	{ CCI_REG8(0x3103), 0x33 },
	{ CCI_REG8(0x3104), 0x30 },
	{ CCI_REG8(0x3105), 0x00 },
	{ CCI_REG8(0x3106), 0xca },
	{ CCI_REG8(0x3107), 0x00 },
	{ CCI_REG8(0x3108), 0x06 },
	{ CCI_REG8(0x3109), 0x04 },
	{ CCI_REG8(0x310a), 0x04 },
	{ CCI_REG8(0x315c), 0x3d },
	{ CCI_REG8(0x315d), 0x3c },
	{ CCI_REG8(0x316e), 0x3e },
	{ CCI_REG8(0x316f), 0x3d },
	{ CCI_REG8(0x3301), 0x01 },	/* one MIPI lane */
	{ CCI_REG8(0x3304), 0x07 },
	{ CCI_REG8(0x3305), 0x06 },
	{ CCI_REG8(0x3306), 0x19 },
	{ CCI_REG8(0x3307), 0x03 },
	{ CCI_REG8(0x3308), 0x0f },
	{ CCI_REG8(0x3309), 0x07 },
	{ CCI_REG8(0x330a), 0x0c },
	{ CCI_REG8(0x330b), 0x06 },
	{ CCI_REG8(0x330c), 0x0b },
	{ CCI_REG8(0x330d), 0x07 },
	{ CCI_REG8(0x330e), 0x03 },
	{ CCI_REG8(0x3318), 0x67 },
	{ CCI_REG8(0x3322), 0x09 },
	{ CCI_REG8(0x3342), 0x00 },
	{ CCI_REG8(0x3348), 0xe0 },
};

static const struct cci_reg_sequence imx132_mode_1976x1200[] = {
	{ CCI_REG16(0x034c), IMX132_WIDTH },
	{ CCI_REG16(0x034e), IMX132_HEIGHT },
};

struct imx132_mode {
	u32 width;
	u32 height;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const struct imx132_mode imx132_modes[] = {
	{
		.width = IMX132_WIDTH,
		.height = IMX132_HEIGHT,
		.regs = imx132_mode_1976x1200,
		.num_regs = ARRAY_SIZE(imx132_mode_1976x1200),
	},
};

struct imx132 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx132_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;

	/* Protected by the control handler lock, which is the state lock. */
	const struct imx132_mode *mode;
};

static inline struct imx132 *to_imx132(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx132, sd);
}

static int imx132_update_exposure_range(struct imx132 *imx132, int vblank)
{
	int max = imx132->mode->height + vblank - IMX132_EXPOSURE_MARGIN;

	return __v4l2_ctrl_modify_range(imx132->exposure,
					imx132->exposure->minimum, max,
					imx132->exposure->step,
					min(imx132->exposure->val, max));
}

static int imx132_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx132 *imx132 = container_of(ctrl->handler, struct imx132,
					     ctrl_handler);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		ret = imx132_update_exposure_range(imx132, ctrl->val);
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_in_use(imx132->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(imx132->regmap, IMX132_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx132->regmap, IMX132_REG_ANALOG_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(imx132->regmap, IMX132_REG_FRAME_LENGTH,
			  imx132->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(imx132->regmap, IMX132_REG_TEST_PATTERN, ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx132->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx132_ctrl_ops = {
	.s_ctrl = imx132_set_ctrl,
};

static int imx132_init_controls(struct imx132 *imx132)
{
	struct v4l2_ctrl_handler *hdl = &imx132->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int vblank = IMX132_FRAME_LENGTH_DEF - IMX132_HEIGHT;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &imx132_ctrl_ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(imx132_link_freqs) - 1, 0,
				      imx132_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &imx132_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  IMX132_PIXEL_RATE, IMX132_PIXEL_RATE, 1,
			  IMX132_PIXEL_RATE);

	ctrl = v4l2_ctrl_new_std(hdl, &imx132_ctrl_ops, V4L2_CID_HBLANK,
				 IMX132_LINE_LENGTH - IMX132_WIDTH,
				 IMX132_LINE_LENGTH - IMX132_WIDTH, 1,
				 IMX132_LINE_LENGTH - IMX132_WIDTH);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx132->vblank = v4l2_ctrl_new_std(hdl, &imx132_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   IMX132_FRAME_LENGTH_MAX - IMX132_HEIGHT,
					   1, vblank);

	imx132->exposure = v4l2_ctrl_new_std(hdl, &imx132_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX132_EXPOSURE_MIN,
					     IMX132_FRAME_LENGTH_DEF -
					     IMX132_EXPOSURE_MARGIN, 1,
					     0x0433);

	v4l2_ctrl_new_std(hdl, &imx132_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX132_AGAIN_MIN, IMX132_AGAIN_MAX, 1,
			  IMX132_AGAIN_DEF);

	v4l2_ctrl_new_std_menu_items(hdl, &imx132_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx132_test_pattern_menu) - 1,
				     0, 0, imx132_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(imx132->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(hdl, &imx132_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx132->sd.ctrl_handler = hdl;

	return 0;
}

static void imx132_fill_format(struct v4l2_mbus_framefmt *fmt,
			       const struct imx132_mode *mode)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx132_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	imx132_fill_format(v4l2_subdev_state_get_format(state, 0),
			   &imx132_modes[0]);

	return 0;
}

static int imx132_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int imx132_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(imx132_modes) ||
	    fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = imx132_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx132_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx132_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct imx132 *imx132 = to_imx132(sd);
	const struct imx132_mode *mode;
	int vblank, ret;

	mode = v4l2_find_nearest_size(imx132_modes, ARRAY_SIZE(imx132_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);
	imx132_fill_format(&fmt->format, mode);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	if (fmt->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	imx132->mode = mode;
	vblank = IMX132_FRAME_LENGTH_DEF - mode->height;
	ret = __v4l2_ctrl_modify_range(imx132->vblank, vblank,
				       IMX132_FRAME_LENGTH_MAX - mode->height,
				       1, vblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx132->vblank, vblank);
	if (ret)
		return ret;

	/* The vblank value may be unchanged while the height changed. */
	return imx132_update_exposure_range(imx132, imx132->vblank->val);
}

static int imx132_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx132 *imx132 = to_imx132(sd);
	int ret;

	ret = pm_runtime_resume_and_get(imx132->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(imx132->regmap, imx132_common_regs,
				  ARRAY_SIZE(imx132_common_regs), NULL);
	if (ret)
		goto err_put;

	ret = cci_multi_reg_write(imx132->regmap, imx132->mode->regs,
				  imx132->mode->num_regs, NULL);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(&imx132->ctrl_handler);
	if (ret)
		goto err_put;

	ret = cci_write(imx132->regmap, IMX132_REG_MODE_SELECT,
			IMX132_MODE_STREAMING, NULL);
	if (ret)
		goto err_put;

	return 0;

err_put:
	pm_runtime_put(imx132->dev);
	return ret;
}

static int imx132_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx132 *imx132 = to_imx132(sd);
	int ret;

	ret = cci_write(imx132->regmap, IMX132_REG_MODE_SELECT,
			IMX132_MODE_STANDBY, NULL);
	pm_runtime_put(imx132->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops imx132_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static int imx132_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		/* The mode reads out the whole active pixel array. */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX132_WIDTH;
		sel->r.height = IMX132_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_pad_ops imx132_pad_ops = {
	.enum_mbus_code = imx132_enum_mbus_code,
	.enum_frame_size = imx132_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx132_set_format,
	.get_selection = imx132_get_selection,
	.enable_streams = imx132_enable_streams,
	.disable_streams = imx132_disable_streams,
};

static const struct v4l2_subdev_ops imx132_subdev_ops = {
	.video = &imx132_video_ops,
	.pad = &imx132_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx132_internal_ops = {
	.init_state = imx132_init_state,
};

static int imx132_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx132 *imx132 = to_imx132(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx132->supplies),
				    imx132->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(imx132->xclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(imx132->supplies),
				       imx132->supplies);
		return ret;
	}

	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(imx132->reset_gpio, 0);
	/* Wait for the sensor to leave reset before the first access. */
	usleep_range(10000, 11000);

	return 0;
}

static int imx132_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx132 *imx132 = to_imx132(sd);

	gpiod_set_value_cansleep(imx132->reset_gpio, 1);
	clk_disable_unprepare(imx132->xclk);
	regulator_bulk_disable(ARRAY_SIZE(imx132->supplies), imx132->supplies);

	return 0;
}

static int imx132_identify(struct imx132 *imx132)
{
	u64 id;
	int ret;

	ret = cci_read(imx132->regmap, IMX132_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(imx132->dev, ret, "Failed to read chip ID\n");

	if (id != IMX132_CHIP_ID)
		return dev_err_probe(imx132->dev, -ENODEV,
				     "Unexpected chip ID 0x%04llx\n", id);

	return 0;
}

static int imx132_check_endpoint(struct imx132 *imx132)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *fwnode;
	unsigned int i;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(imx132->dev), NULL);
	if (!fwnode)
		return dev_err_probe(imx132->dev, -EINVAL, "No endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return dev_err_probe(imx132->dev, ret, "Invalid endpoint\n");

	if (ep.bus.mipi_csi2.num_data_lanes != IMX132_NUM_LANES) {
		ret = dev_err_probe(imx132->dev, -EINVAL,
				    "Only %u data lanes are supported\n",
				    IMX132_NUM_LANES);
		goto out;
	}

	ret = -EINVAL;
	for (i = 0; i < ep.nr_of_link_frequencies; i++)
		if (ep.link_frequencies[i] == IMX132_LINK_FREQ)
			ret = 0;
	if (ret)
		dev_err_probe(imx132->dev, ret, "Link frequency %llu not listed\n",
			      IMX132_LINK_FREQ);
out:
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int imx132_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx132 *imx132;
	unsigned int i;
	int ret;

	imx132 = devm_kzalloc(dev, sizeof(*imx132), GFP_KERNEL);
	if (!imx132)
		return -ENOMEM;

	imx132->dev = dev;
	imx132->mode = &imx132_modes[0];
	v4l2_i2c_subdev_init(&imx132->sd, client, &imx132_subdev_ops);
	imx132->sd.internal_ops = &imx132_internal_ops;

	ret = imx132_check_endpoint(imx132);
	if (ret)
		return ret;

	imx132->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx132->regmap))
		return dev_err_probe(dev, PTR_ERR(imx132->regmap),
				     "Failed to initialise CCI\n");

	imx132->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx132->xclk))
		return dev_err_probe(dev, PTR_ERR(imx132->xclk),
				     "Failed to get clock\n");
	if (clk_get_rate(imx132->xclk) != IMX132_XCLK_FREQ) {
		ret = clk_set_rate(imx132->xclk, IMX132_XCLK_FREQ);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to set clock rate\n");
	}
	if (clk_get_rate(imx132->xclk) != IMX132_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "Clock is %lu Hz, need %u Hz\n",
				     clk_get_rate(imx132->xclk),
				     IMX132_XCLK_FREQ);

	for (i = 0; i < ARRAY_SIZE(imx132_supply_names); i++)
		imx132->supplies[i].supply = imx132_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx132->supplies),
				      imx132->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get supplies\n");

	imx132->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx132->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx132->reset_gpio),
				     "Failed to get reset GPIO\n");

	ret = imx132_power_on(dev);
	if (ret)
		return ret;

	ret = imx132_identify(imx132);
	if (ret)
		goto err_power_off;

	ret = imx132_init_controls(imx132);
	if (ret)
		goto err_power_off;

	imx132->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx132->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx132->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx132->sd.entity, 1, &imx132->pad);
	if (ret)
		goto err_ctrls;

	imx132->sd.state_lock = imx132->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx132->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx132->sd);
	if (ret)
		goto err_pm;

	pm_runtime_idle(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx132->sd);
err_entity:
	media_entity_cleanup(&imx132->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx132->ctrl_handler);
err_power_off:
	imx132_power_off(dev);
	return ret;
}

static void imx132_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx132 *imx132 = to_imx132(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx132->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx132_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx132_pm_ops, imx132_power_off,
				 imx132_power_on, NULL);

static const struct of_device_id imx132_of_match[] = {
	{ .compatible = "sony,imx132" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx132_of_match);

static struct i2c_driver imx132_i2c_driver = {
	.driver = {
		.name = "imx132",
		.of_match_table = imx132_of_match,
		.pm = pm_ptr(&imx132_pm_ops),
	},
	.probe = imx132_probe,
	.remove = imx132_remove,
};
module_i2c_driver(imx132_i2c_driver);

MODULE_DESCRIPTION("Sony IMX132 image sensor driver");
MODULE_LICENSE("GPL");
