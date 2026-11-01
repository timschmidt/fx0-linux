// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm WCD9306/WCD9302 (Tapan) audio codec.
 *
 * Power-up, identification and a first playback path: SLIMbus RX1 through
 * the RX3 interpolator to the class-D speaker driver, as used by the
 * WCD9302 variant.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slimbus.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "wcd9306.h"

#define SLIM_MANF_ID_QCOM		0x217
#define SLIM_PROD_CODE_WCD9306		0xe0
#define WCD9306_ID_MAJOR		0x0103

/* SLIMbus RX ports start at 16; the interface layout matches WCD9335. */
#define WCD9306_RX_PORT_START		16
#define WCD9306_RX_MAX			5
/* AIF1 playback uses RX1 and RX2 (SLIMbus ports 16 and 17). */
#define WCD9306_AIF1_PB_CHANNELS	2
#define WCD9306_SLIM_PGD_RX_PORT_CFG(p)		(0x30 + (p))
#define WCD9306_SLIM_PGD_RX_PORT_MULTI_CHNL_0(p) (0x140 + 4 * (p))
/* 12-byte watermark (value 2), port enabled */
#define WCD9306_SLIM_WATER_MARK_VAL	((2 << 1) | 1)

enum {
	AIF1_PB = 0,
	NUM_CODEC_DAIS,
};

struct wcd9306_slim_ch {
	u32 ch_num;
	u16 port;
	u16 shift;
};

static const char * const wcd9306_supplies[] = {
	"vdd-h", "vdd-px", "vdd-cx",
};

struct wcd9306_codec {
	struct device *dev;
	struct slim_device *slim;
	struct slim_device *slim_ifc_dev;
	struct regmap *regmap;
	struct regmap *if_regmap;
	struct gpio_desc *reset_gpio;
	struct clk *mclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(wcd9306_supplies)];

	struct wcd9306_slim_ch rx_chs[WCD9306_RX_MAX];
	unsigned int num_rx_port;
	struct slim_stream_runtime *sruntime;
	struct slim_stream_config sconfig;
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
	/* In SLIMbus mode the registers start at value element 0x800. */
	.reg_base = 0x800,
	.max_register = WCD9306_MAX_REGISTER,
	.reg_defaults = wcd9306_defaults,
	.num_reg_defaults = ARRAY_SIZE(wcd9306_defaults),
	.readable_reg = wcd9306_readable_reg,
	.volatile_reg = wcd9306_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
	.can_multi_write = true,
};

static const struct regmap_config wcd9306_ifc_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_base = 0x800,
	.max_register = 0x200,
	.cache_type = REGCACHE_NONE,
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

struct wcd9306_reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

/* Register defaults and init values from the vendor codec driver. */
static const struct wcd9306_reg_mask_val wcd9306_reg_defaults[] = {
	{ WCD9306_QFUSE_CTL, 0xff, 0x03 },
	{ WCD9306_BIAS_CURR_CTL_2, 0xff, 0x04 },
	{ WCD9306_CDC_CLK_POWER_CTL, 0xff, 0x03 },
	{ WCD9306_RX_EAR_CMBUFF, 0xff, 0x05 },
	{ WCD9306_CDC_RX1_B6_CTL, 0xff, 0xa0 },
	{ WCD9306_CDC_RX2_B6_CTL, 0xff, 0xa0 },
	{ WCD9306_CDC_CONN_RX2_B2_CTL, 0xff, 0x10 },
	{ WCD9306_RX_HPH_CHOP_CTL, 0xff, 0xf4 },
	{ WCD9306_BIAS_CURR_CTL_2, 0xff, 0x08 },
	{ WCD9306_BUCK_CTRL_CCL_1, 0xff, 0x5b },
	{ WCD9306_BUCK_CTRL_CCL_3, 0xff, 0x6f },
	{ WCD9306_BIAS_CURR_CTL_2, 0xff, 0x04 },
	{ WCD9306_RX_HPH_CHOP_CTL, 0xff, 0x74 },
	{ WCD9306_RX_BUCK_BIAS1, 0xff, 0x62 },
	{ WCD9306_NCP_CLK, 0xff, 0xfc },
	{ WCD9306_BUCK_CTRL_VCL_1, 0xff, 0x08 },
	{ WCD9306_BUCK_MODE_3, 0xff, 0xce },
	{ WCD9306_RX_HPH_BIAS_PA, 0xff, 0x7a },
	{ WCD9306_RX_EAR_BIAS_PA, 0xff, 0x76 },
	{ WCD9306_RX_LINE_BIAS_PA, 0xff, 0x78 },
	{ WCD9306_MICB_2_MBHC, 0xff, 0x41 },
	{ WCD9306_BUCK_MODE_2, 0xff, 0xaa },
};

static const struct wcd9306_reg_mask_val wcd9306_reg_init[] = {
	{ WCD9306_RX_HPH_OCP_CTL, 0xe9, 0x69 },
	{ WCD9306_RX_COM_OCP_COUNT, 0xff, 0xff },
	{ WCD9306_RX_HPH_L_TEST, 0x01, 0x01 },
	{ WCD9306_RX_HPH_R_TEST, 0x01, 0x01 },
	{ WCD9306_RX_HPH_L_GAIN, 0x20, 0x20 },
	{ WCD9306_RX_HPH_R_GAIN, 0x20, 0x20 },
	{ WCD9306_RX_LINE_1_GAIN, 0x20, 0x20 },
	{ WCD9306_RX_LINE_2_GAIN, 0x20, 0x20 },
	{ WCD9306_SPKR_DRV_GAIN, 0x04, 0x04 },
	{ WCD9306_CDC_CONN_MISC, 0x04, 0x04 },
	{ WCD9306_CDC_CONN_CLSH_CTL, 0x3c, 0x14 },
	{ WCD9306_CDC_CONN_TX_SB_B1_CTL, 0x30, 0x20 },
	{ WCD9306_CDC_CONN_TX_SB_B2_CTL, 0x30, 0x20 },
	{ WCD9306_CDC_CONN_TX_SB_B3_CTL, 0x30, 0x20 },
	{ WCD9306_CDC_CONN_TX_SB_B4_CTL, 0x30, 0x20 },
	{ WCD9306_CDC_CONN_TX_SB_B5_CTL, 0x30, 0x20 },
	{ WCD9306_SPKR_DRV_DAC_CTL, 0x04, 0x00 },
	{ WCD9306_CDC_CONN_RX_SB_B1_CTL, 0xff, 0xaa },
	{ WCD9306_CDC_CONN_RX_SB_B2_CTL, 0xff, 0x2a },
	{ WCD9306_CDC_TX1_MUX_CTL, 0x8, 0x0 },
	{ WCD9306_CDC_TX2_MUX_CTL, 0x8, 0x0 },
	{ WCD9306_CDC_TX3_MUX_CTL, 0x8, 0x0 },
	{ WCD9306_CDC_TX4_MUX_CTL, 0x8, 0x0 },
	{ WCD9306_CDC_COMP0_B4_CTL, 0x3f, 0x37 },
	{ WCD9306_CDC_COMP1_B4_CTL, 0x3f, 0x37 },
	{ WCD9306_CDC_COMP2_B4_CTL, 0x3f, 0x37 },
	{ WCD9306_CDC_COMP0_B5_CTL, 0x7f, 0x7f },
	{ WCD9306_CDC_COMP1_B5_CTL, 0x7f, 0x7f },
	{ WCD9306_CDC_COMP2_B5_CTL, 0x7f, 0x7f },
	{ WCD9306_RX_HPH_CNP_WG_CTL, 0xff, 0xdb },
	{ WCD9306_RX_HPH_CNP_WG_TIME, 0xff, 0x58 },
	{ WCD9306_RX_HPH_BIAS_WG_OCP, 0xff, 0x1a },
	{ WCD9306_RX_HPH_CHOP_CTL, 0xff, 0x24 },
};

static void wcd9306_init_regs(struct wcd9306_codec *wcd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(wcd9306_reg_defaults); i++)
		regmap_write(wcd->regmap, wcd9306_reg_defaults[i].reg,
			     wcd9306_reg_defaults[i].val);
	for (i = 0; i < ARRAY_SIZE(wcd9306_reg_init); i++)
		regmap_update_bits(wcd->regmap, wcd9306_reg_init[i].reg,
				   wcd9306_reg_init[i].mask,
				   wcd9306_reg_init[i].val);
}

static int wcd9306_mclk_enable(struct wcd9306_codec *wcd, bool enable)
{
	struct regmap *map = wcd->regmap;
	int ret;

	if (enable) {
		ret = clk_prepare_enable(wcd->mclk);
		if (ret)
			return ret;

		/* Bandgap in audio mode: slow start with precharge. */
		regmap_update_bits(map, WCD9306_BIAS_CENTRAL_BG_CTL, 0x80, 0x80);
		regmap_update_bits(map, WCD9306_BIAS_CENTRAL_BG_CTL, 0x04, 0x04);
		regmap_update_bits(map, WCD9306_BIAS_CENTRAL_BG_CTL, 0x01, 0x01);
		usleep_range(1000, 1100);
		regmap_update_bits(map, WCD9306_BIAS_CENTRAL_BG_CTL, 0x80, 0x00);

		/* Clock buffer from the external MCLK, referenced to VBG. */
		regmap_update_bits(map, WCD9306_CLK_BUFF_EN1, 0x08, 0x00);
		regmap_update_bits(map, WCD9306_CLK_BUFF_EN1, 0x0c, 0x04);
		regmap_update_bits(map, WCD9306_CLK_BUFF_EN1, 0x01, 0x01);
		usleep_range(1000, 1200);
		regmap_update_bits(map, WCD9306_CLK_BUFF_EN2, 0x02, 0x00);
		regmap_update_bits(map, WCD9306_CLK_BUFF_EN2, 0x04, 0x04);
		regmap_update_bits(map, WCD9306_CDC_CLK_MCLK_CTL, 0x01, 0x01);
		usleep_range(50, 100);
	} else {
		regmap_update_bits(map, WCD9306_CLK_BUFF_EN2, 0x04, 0x00);
		usleep_range(50, 100);
		regmap_update_bits(map, WCD9306_CDC_CLK_MCLK_CTL, 0x01, 0x00);
		regmap_update_bits(map, WCD9306_CLK_BUFF_EN1, 0x01, 0x00);
		regmap_update_bits(map, WCD9306_BIAS_CENTRAL_BG_CTL, 0x05, 0x00);
		clk_disable_unprepare(wcd->mclk);
	}

	return 0;
}

static int wcd9306_codec_enable_mclk(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kc, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct wcd9306_codec *wcd = snd_soc_component_get_drvdata(comp);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return wcd9306_mclk_enable(wcd, true);
	case SND_SOC_DAPM_POST_PMD:
		return wcd9306_mclk_enable(wcd, false);
	}

	return 0;
}

static int wcd9306_codec_enable_interpolator(struct snd_soc_dapm_widget *w,
					     struct snd_kcontrol *kc, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	if (event == SND_SOC_DAPM_PRE_PMU) {
		snd_soc_component_update_bits(comp, WCD9306_CDC_CLK_RX_RESET_CTL,
					      BIT(w->shift), BIT(w->shift));
		snd_soc_component_update_bits(comp, WCD9306_CDC_CLK_RX_RESET_CTL,
					      BIT(w->shift), 0);
	}

	return 0;
}

static int wcd9306_codec_enable_spk_pa(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kc, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, WCD9306_SPKR_DRV_EN, 0x80, 0x80);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(comp, WCD9306_SPKR_DRV_EN, 0x80, 0x00);
		break;
	}

	return 0;
}

static const char * const rx_3_4_mix1_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2", "RX1", "RX2", "RX3", "RX4",
	"RX5", "AUXRX", "AUXTX1", "AUXTX2",
};

static SOC_ENUM_SINGLE_DECL(rx3_mix1_inp1_enum, WCD9306_CDC_CONN_RX3_B1_CTL, 0,
			    rx_3_4_mix1_text);

static const struct snd_kcontrol_new rx3_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP1 Mux", rx3_mix1_inp1_enum);

static const struct snd_kcontrol_new spk_dac_switch[] = {
	SOC_DAPM_SINGLE("Switch", WCD9306_SPKR_DRV_DAC_CTL, 2, 1, 0),
};

static const DECLARE_TLV_DB_SCALE(digital_gain, -8400, 100, -8400);

static const struct snd_kcontrol_new wcd9306_snd_controls[] = {
	SOC_SINGLE_S8_TLV("RX3 Digital Volume", WCD9306_CDC_RX3_VOL_CTL_B2_CTL,
			  -84, 40, digital_gain),
};

static const struct snd_soc_dapm_widget wcd9306_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF1 PB", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("MCLK", SND_SOC_NOPM, 0, 0,
			    wcd9306_codec_enable_mclk,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP1", SND_SOC_NOPM, 0, 0, &rx3_mix1_inp1_mux),
	SND_SOC_DAPM_MIXER_E("RX3 MIX1", WCD9306_CDC_CLK_RX_B1_CTL, 2, 0, NULL, 0,
			     wcd9306_codec_enable_interpolator, SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_MIXER("SPK DAC", SND_SOC_NOPM, 0, 0, spk_dac_switch,
			   ARRAY_SIZE(spk_dac_switch)),
	SND_SOC_DAPM_REGULATOR_SUPPLY("vdd-spkdrv", 0, 0),
	SND_SOC_DAPM_PGA_E("SPK PA", SND_SOC_NOPM, 0, 0, NULL, 0,
			   wcd9306_codec_enable_spk_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("SPK_OUT"),
};

static const struct snd_soc_dapm_route wcd9306_audio_map[] = {
	{ "AIF1 PB", NULL, "MCLK" },
	{ "RX3 MIX1 INP1", "RX1", "AIF1 PB" },
	{ "RX3 MIX1", NULL, "RX3 MIX1 INP1" },
	{ "RX3 MIX1", NULL, "MCLK" },
	{ "SPK DAC", "Switch", "RX3 MIX1" },
	{ "SPK DAC", NULL, "vdd-spkdrv" },
	{ "SPK PA", NULL, "SPK DAC" },
	{ "SPK_OUT", NULL, "SPK PA" },
};

static int wcd9306_set_interpolator_rate(struct wcd9306_codec *wcd,
					 unsigned int rate)
{
	unsigned int val;

	switch (rate) {
	case 8000:
		val = 0x00;
		break;
	case 16000:
		val = 0x20;
		break;
	case 32000:
		val = 0x40;
		break;
	case 48000:
		val = 0x60;
		break;
	case 96000:
		val = 0x80;
		break;
	case 192000:
		val = 0xa0;
		break;
	default:
		return -EINVAL;
	}

	/* Only RX3, which feeds the speaker, is wired up so far. */
	return regmap_update_bits(wcd->regmap, WCD9306_CDC_RX3_B5_CTL, 0xe0, val);
}

static int wcd9306_slim_set_hw_params(struct wcd9306_codec *wcd,
				      unsigned int channels)
{
	struct slim_stream_config *cfg = &wcd->sconfig;
	u16 payload = 0;
	int ret, i;

	if (channels > wcd->num_rx_port)
		return -EINVAL;

	kfree(cfg->chs);
	cfg->chs = kcalloc(channels, sizeof(unsigned int), GFP_KERNEL);
	if (!cfg->chs)
		return -ENOMEM;

	cfg->ch_count = channels;
	cfg->direction = SNDRV_PCM_STREAM_PLAYBACK;
	cfg->port_mask = 0;
	for (i = 0; i < channels; i++) {
		payload |= BIT(wcd->rx_chs[i].shift);
		cfg->port_mask |= BIT(wcd->rx_chs[i].port);
		cfg->chs[i] = wcd->rx_chs[i].ch_num;
	}

	for (i = 0; i < channels; i++) {
		u16 port = wcd->rx_chs[i].port;

		ret = regmap_write(wcd->if_regmap,
				   WCD9306_SLIM_PGD_RX_PORT_MULTI_CHNL_0(port),
				   payload);
		if (ret)
			return ret;
		ret = regmap_write(wcd->if_regmap,
				   WCD9306_SLIM_PGD_RX_PORT_CFG(port),
				   WCD9306_SLIM_WATER_MARK_VAL);
		if (ret)
			return ret;
	}

	if (!wcd->sruntime)
		wcd->sruntime = slim_stream_allocate(wcd->slim, "WCD9306-SLIM");

	return IS_ERR(wcd->sruntime) ? PTR_ERR(wcd->sruntime) : 0;
}

static int wcd9306_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct wcd9306_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	int ret;

	ret = wcd9306_set_interpolator_rate(wcd, params_rate(params));
	if (ret) {
		dev_err(wcd->dev, "Unsupported rate %u\n", params_rate(params));
		return ret;
	}

	switch (params_width(params)) {
	case 16 ... 24:
		wcd->sconfig.bps = params_width(params);
		break;
	default:
		return -EINVAL;
	}
	wcd->sconfig.rate = params_rate(params);

	return wcd9306_slim_set_hw_params(wcd, params_channels(params));
}

static int wcd9306_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct wcd9306_codec *wcd = snd_soc_component_get_drvdata(dai->component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		slim_stream_prepare(wcd->sruntime, &wcd->sconfig);
		slim_stream_enable(wcd->sruntime);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		slim_stream_disable(wcd->sruntime);
		slim_stream_unprepare(wcd->sruntime);
		break;
	}

	return 0;
}

static int wcd9306_set_channel_map(struct snd_soc_dai *dai,
				   unsigned int tx_num, const unsigned int *tx_slot,
				   unsigned int rx_num, const unsigned int *rx_slot)
{
	struct wcd9306_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	int i;

	if (!rx_slot)
		return -EINVAL;

	wcd->num_rx_port = min(rx_num, WCD9306_RX_MAX);
	for (i = 0; i < wcd->num_rx_port; i++) {
		wcd->rx_chs[i].ch_num = rx_slot[i];
		wcd->rx_chs[i].port = WCD9306_RX_PORT_START + i;
		wcd->rx_chs[i].shift = i;
	}

	return 0;
}

static int wcd9306_get_channel_map(const struct snd_soc_dai *dai,
				   unsigned int *tx_num, unsigned int *tx_slot,
				   unsigned int *rx_num, unsigned int *rx_slot)
{
	struct wcd9306_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	int i;

	if (!rx_slot || !rx_num)
		return -EINVAL;

	if (wcd->num_rx_port < WCD9306_AIF1_PB_CHANNELS)
		return -EINVAL;

	/* Report only the channels this DAI streams on. */
	for (i = 0; i < WCD9306_AIF1_PB_CHANNELS; i++)
		rx_slot[i] = wcd->rx_chs[i].ch_num;
	*rx_num = WCD9306_AIF1_PB_CHANNELS;

	return 0;
}

static const struct snd_soc_dai_ops wcd9306_dai_ops = {
	.hw_params = wcd9306_hw_params,
	.trigger = wcd9306_trigger,
	.set_channel_map = wcd9306_set_channel_map,
	.get_channel_map = wcd9306_get_channel_map,
};

static struct snd_soc_dai_driver wcd9306_dais[] = {
	[AIF1_PB] = {
		.name = "wcd9306_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = WCD9306_AIF1_PB_CHANNELS,
			.channels_max = WCD9306_AIF1_PB_CHANNELS,
		},
		.ops = &wcd9306_dai_ops,
	},
};

static int wcd9306_component_probe(struct snd_soc_component *comp)
{
	struct wcd9306_codec *wcd = snd_soc_component_get_drvdata(comp);

	snd_soc_component_init_regmap(comp, wcd->regmap);
	wcd9306_init_regs(wcd);

	/*
	 * Tell the codec its MCLK rate. Boards feed either 12.288 MHz or
	 * 9.6 MHz; the RPM divided clock used for 9.6 MHz does not report
	 * its real rate, so treat anything other than 12.288 MHz as 9.6 MHz.
	 */
	if (clk_get_rate(wcd->mclk) == 12288000) {
		regmap_update_bits(wcd->regmap, WCD9306_CHIP_CTL, 0x06, 0x00);
		regmap_update_bits(wcd->regmap, WCD9306_RX_COM_TIMER_DIV, 0x01, 0x01);
	} else {
		regmap_update_bits(wcd->regmap, WCD9306_CHIP_CTL, 0x06, 0x02);
	}

	return 0;
}

static const struct snd_soc_component_driver wcd9306_component_drv = {
	.probe = wcd9306_component_probe,
	.controls = wcd9306_snd_controls,
	.num_controls = ARRAY_SIZE(wcd9306_snd_controls),
	.dapm_widgets = wcd9306_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wcd9306_dapm_widgets),
	.dapm_routes = wcd9306_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wcd9306_audio_map),
	.endianness = 1,
};

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

	wcd->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(wcd->mclk))
		return dev_err_probe(dev, PTR_ERR(wcd->mclk), "Failed to get MCLK\n");

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

	wcd->if_regmap = devm_regmap_init_slimbus(wcd->slim_ifc_dev,
						  &wcd9306_ifc_regmap_config);
	if (IS_ERR(wcd->if_regmap))
		return dev_err_probe(dev, PTR_ERR(wcd->if_regmap),
				     "Failed to allocate interface register map\n");

	return devm_snd_soc_register_component(dev, &wcd9306_component_drv,
					       wcd9306_dais,
					       ARRAY_SIZE(wcd9306_dais));
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
