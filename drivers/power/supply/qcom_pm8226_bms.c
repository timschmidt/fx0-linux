// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm PM8226 non-VM battery measurement block. */
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define BMS_HOLD BIT(0)
#define BMS_CLEAR_MASK (BIT(7) | BIT(6))
#define BMS_CC_MASK GENMASK_ULL(35, 0)
#define BMS_POLL_MS 5000
#define BMS_CALIBRATE_NS (300ULL * NSEC_PER_SEC)
#define BMS_COUNTER_MAX_AGE_NS (1800ULL * NSEC_PER_SEC)
/* 542535/100000 uV/code, ideal gain 3291, 56 ticks at 32764 Hz. */
#define BMS_CURRENT_NUMERATOR (542535LL * 3291 * 10)
#define BMS_CHARGE_NUMERATOR (542535ULL * 3291 * 56 * 10 / 3600)

struct bms_counter {
	u64 previous;
	s64 charge_uah;
	s64 remainder;
	bool initialized;
};

struct bms_outputs {
	u8 vsense[2];
	u8 counter[5];
};

struct pm8226_bms {
	struct device *dev;
	struct regmap *regmap;
	u16 base, iadc_base, present_reg;
	struct iio_channel *sense_channel, *gain, *offset, *voltage, *temperature;
	struct mutex lock;
	struct delayed_work work;
	struct power_supply *battery;
	struct power_supply_desc desc;
	struct bms_counter counter;
	u32 rsense;
	int gain_delta;
	u64 denominator;
	u64 last_sample_ns, last_counter_ns, last_calibration_ns;
	int voltage_uv, current_ua, temperature_decic;
	int present_error, voltage_error, current_error, temperature_error, charge_error;
	bool present, configured, calibration_valid;
};

/* BEGIN TESTED BMS MATH */
static u64 bms_decode_counter(const u8 *data)
{
	return (get_unaligned_le32(data) | (u64)data[4] << 32) & BMS_CC_MASK;
}

static int bms_current_to_ua(s16 raw, int gain_delta, u32 rsense, int *value)
{
	s64 result;

	if (gain_delta <= 0 || !rsense)
		return -EINVAL;
	/* Hardware reports the opposite sign to positive-charging power_supply. */
	result = div64_s64(-(s64)raw * BMS_CURRENT_NUMERATOR,
			   (s64)gain_delta * rsense);
	if (result < INT_MIN || result > INT_MAX)
		return -ERANGE;
	*value = result;
	return 0;
}

static int bms_counter_step(struct bms_counter *counter, u64 raw, u64 denominator)
{
	s64 delta, fraction, whole, carry, charge;
	u64 numerator, remainder, quotient;

	if (!denominator || denominator > S64_MAX / 2)
		return -ERANGE;
	if (!counter->initialized) {
		counter->previous = raw;
		counter->charge_uah = 0;
		counter->remainder = 0;
		counter->initialized = true;
		return 0;
	}
	delta = sign_extend64((raw - counter->previous) & BMS_CC_MASK, 35);
	/* |delta| <= 2^35: the product fits u64, but not always s64. */
	numerator = (delta < 0 ? -delta : delta) * BMS_CHARGE_NUMERATOR;
	quotient = div64_u64_rem(numerator, denominator, &remainder);
	if (quotient > S64_MAX)
		return -ERANGE;
	whole = delta < 0 ? (s64)quotient : -(s64)quotient;
	fraction = delta < 0 ? (s64)remainder : -(s64)remainder;
	if (check_add_overflow(counter->remainder, fraction, &fraction))
		return -ERANGE;
	carry = div64_s64(fraction, denominator);
	fraction -= carry * (s64)denominator;
	if (check_add_overflow(whole, carry, &whole) ||
	    check_add_overflow(counter->charge_uah, whole, &charge))
		return -ERANGE;
	/* Keep the integer part truncated toward zero when direction changes. */
	if (charge > 0 && fraction < 0) {
		charge--;
		fraction += (s64)denominator;
	} else if (charge < 0 && fraction > 0) {
		charge++;
		fraction -= (s64)denominator;
	}
	counter->previous = raw;
	counter->charge_uah = charge;
	counter->remainder = fraction;
	return 0;
}
/* END TESTED BMS MATH */

static int bms_read_outputs(struct pm8226_bms *bms, struct bms_outputs *out)
{
	int ret;

	ret = regmap_bulk_read(bms->regmap, bms->base + 0x98,
			       out->vsense, sizeof(out->vsense));
	if (ret)
		return ret;
	return regmap_bulk_read(bms->regmap, bms->base + 0xa8,
				out->counter, sizeof(out->counter));
}

static int bms_snapshot(struct pm8226_bms *bms, struct bms_outputs *out)
{
	struct bms_outputs verify;
	unsigned int before, held, after;
	int ret, release_ret = -EIO, attempt;

	ret = regmap_read(bms->regmap, bms->base + 0x42, &before);
	if (ret)
		return ret;
	if (before & BMS_HOLD)
		return -EBUSY;
	ret = regmap_update_bits(bms->regmap, bms->base + 0x42, BMS_HOLD, BMS_HOLD);
	if (ret)
		goto restore;
	ret = regmap_read(bms->regmap, bms->base + 0x42, &held);
	if (ret)
		goto restore;
	if (!(held & BMS_HOLD)) {
		ret = -EIO;
		goto restore;
	}
	ret = bms_read_outputs(bms, out);
	if (ret)
		goto restore;
	ret = bms_read_outputs(bms, &verify);
	if (!ret && memcmp(out, &verify, sizeof(*out)))
		ret = -EIO;
restore:
	for (attempt = 0; attempt < 2; attempt++) {
		release_ret = regmap_update_bits(bms->regmap, bms->base + 0x42, BMS_HOLD, 0);
		if (release_ret)
			continue;
		release_ret = regmap_read(bms->regmap, bms->base + 0x42, &after);
		if (!release_ret && !(after & BMS_HOLD))
			return ret;
		release_ret = -EIO;
	}
	return release_ret;
}

static int bms_restart_counter(struct pm8226_bms *bms)
{
	unsigned int control;
	int ret, release_ret = -EIO, attempt;

	ret = regmap_read(bms->regmap, bms->base + 0x43, &control);
	if (ret)
		return ret;
	if (control & BMS_CLEAR_MASK)
		return -EBUSY;
	ret = regmap_write_bits(bms->regmap, bms->base + 0x43,
				BMS_CLEAR_MASK, BMS_CLEAR_MASK);
	if (!ret)
		udelay(100);
	for (attempt = 0; attempt < 2; attempt++) {
		/* Clear-command readback may be zero while counting remains stopped. */
		release_ret = regmap_write_bits(bms->regmap, bms->base + 0x43,
						BMS_CLEAR_MASK, 0);
		if (release_ret)
			continue;
		release_ret = regmap_read(bms->regmap, bms->base + 0x43, &control);
		if (!release_ret && !(control & BMS_CLEAR_MASK))
			return ret;
		release_ret = -EIO;
	}
	return release_ret;
}

static int bms_refresh_calibration(struct pm8226_bms *bms)
{
	int gain, offset, ret;
	u64 denominator;
	s64 remainder;

	/* Also reject a provider whose trim rollback left its cache invalid. */
	ret = iio_read_channel_raw(bms->sense_channel, &gain);
	if (ret < 0)
		goto invalid;
	ret = iio_read_channel_raw(bms->gain, &gain);
	if (ret < 0)
		goto invalid;
	ret = iio_read_channel_raw(bms->offset, &offset);
	if (ret < 0)
		goto invalid;
	if (offset < 0 || gain > U16_MAX || gain <= offset) {
		ret = -ERANGE;
		goto invalid;
	}
	denominator = 32764ULL * (gain - offset) * bms->rsense;
	if (!denominator || denominator > S64_MAX / 2) {
		ret = -ERANGE;
		goto invalid;
	}
	/* Preserve fractional uAh when recalibration changes the denominator. */
	if (bms->denominator && bms->counter.remainder) {
		remainder = bms->counter.remainder;
		bms->counter.remainder = mul_u64_u64_div_u64(remainder < 0 ? -remainder : remainder,
							   denominator, bms->denominator);
		if (remainder < 0)
			bms->counter.remainder = -bms->counter.remainder;
	}
	bms->denominator = denominator;
	bms->gain_delta = gain - offset;
	bms->calibration_valid = true;
	return 0;
invalid:
	bms->calibration_valid = false;
	bms->counter.initialized = false;
	return ret;
}

static int bms_calibrate(struct pm8226_bms *bms)
{
	int ret;

	ret = iio_write_channel_ext_info(bms->sense_channel, "sys_calibration", "1", 1);
	if (ret != 1) {
		bms->calibration_valid = false;
		bms->counter.initialized = false;
		return ret < 0 ? ret : -EIO;
	}
	ret = bms_refresh_calibration(bms);
	if (!ret)
		bms->last_calibration_ns = ktime_get_boottime_ns();
	return ret;
}

static int bms_restore_sense(struct pm8226_bms *bms, unsigned int channel,
			     unsigned int sense)
{
	unsigned int after;
	int ret = -EIO, attempt;

	for (attempt = 0; attempt < 2; attempt++) {
		ret = regmap_update_bits(bms->regmap, bms->iadc_base + 0x48, 7, channel);
		if (ret)
			continue;
		ret = regmap_read(bms->regmap, bms->iadc_base + 0x48, &after);
		if (ret || (after & 7) != (channel & 7))
			continue;
		ret = regmap_update_bits(bms->regmap, bms->iadc_base + 0x49, 3, sense);
		if (ret)
			continue;
		ret = regmap_read(bms->regmap, bms->iadc_base + 0x49, &after);
		if (!ret && (after & 3) == (sense & 3))
			return 0;
		ret = -EIO;
	}
	return ret ?: -EIO;
}

static int bms_configure(struct pm8226_bms *bms)
{
	unsigned int channel, sense;
	int ret, restore_ret;

	ret = regmap_read(bms->regmap, bms->iadc_base + 0x48, &channel);
	if (ret)
		return ret;
	ret = regmap_read(bms->regmap, bms->iadc_base + 0x49, &sense);
	if (ret)
		return ret;
	ret = bms_calibrate(bms);
	if (ret)
		return ret;
	ret = regmap_update_bits(bms->regmap, bms->iadc_base + 0x49, 3, 2);
	if (ret)
		goto restore;
	ret = regmap_update_bits(bms->regmap, bms->iadc_base + 0x48, 7, 1);
	if (ret)
		goto restore;
	/* A new driver/pack epoch cannot inherit an uncalibrated counter history. */
	ret = bms_restart_counter(bms);
	if (ret)
		goto restore;
	bms->counter.initialized = false;
	bms->configured = true;
	return 0;
restore:
	restore_ret = bms_restore_sense(bms, channel, sense);
	return restore_ret ? restore_ret : ret;
}

static void bms_sample(struct pm8226_bms *bms)
{
	struct bms_outputs output;
	u64 now = ktime_get_boottime_ns();
	unsigned int present;
	bool rebase;
	int ret;

	bms->present_error = regmap_read(bms->regmap, bms->present_reg, &present);
	if (bms->present_error)
		goto absent;
	bms->present = !!(present & BIT(7));
	if (!bms->present) {
		bms->configured = false;
		bms->counter.initialized = false;
		goto absent;
	}
	bms->voltage_error = iio_read_channel_processed_scale(bms->voltage,
							     &bms->voltage_uv, 1000);
	if (bms->temperature) {
		bms->temperature_error = iio_read_channel_processed(bms->temperature, &ret);
		if (!bms->temperature_error)
			bms->temperature_decic = ret / 100;
	}
	if (!bms->configured)
		ret = bms_configure(bms);
	else if (!bms->calibration_valid)
		ret = bms_calibrate(bms);
	else
		ret = 0;
	if (ret)
		goto current_error;
	ret = bms_refresh_calibration(bms);
	if (ret)
		goto current_error;
	ret = bms_snapshot(bms, &output);
	if (ret)
		goto current_error;
	bms->current_error = bms_current_to_ua((s16)get_unaligned_le16(output.vsense),
					     bms->gain_delta, bms->rsense, &bms->current_ua);
	/* Long unobserved intervals cannot establish an unambiguous wrapped delta. */
	rebase = bms->counter.initialized && now - bms->last_counter_ns > BMS_COUNTER_MAX_AGE_NS;
	if (rebase)
		bms->counter.initialized = false;
	bms->charge_error = bms_counter_step(&bms->counter, bms_decode_counter(output.counter),
					    bms->denominator);
	if (rebase && !bms->charge_error)
		bms->charge_error = -ENODATA;
	bms->last_counter_ns = now;
	if (ktime_get_boottime_ns() - bms->last_calibration_ns >= BMS_CALIBRATE_NS) {
		ret = bms_calibrate(bms);
		if (ret)
			goto current_error;
	}
	bms->last_sample_ns = now;
	return;
current_error:
	bms->current_error = ret;
	bms->charge_error = ret;
	bms->last_sample_ns = now;
	return;
absent:
	bms->voltage_error = bms->current_error = bms->charge_error = -ENODATA;
	bms->temperature_error = -ENODATA;
	bms->last_sample_ns = now;
}

static int bms_get_property(struct power_supply *psy, enum power_supply_property prop,
			    union power_supply_propval *value)
{
	struct pm8226_bms *bms = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&bms->lock);
	if (ktime_get_boottime_ns() - bms->last_sample_ns > NSEC_PER_SEC)
		bms_sample(bms);
	if (bms->present_error) {
		ret = bms->present_error;
		goto unlock;
	}
	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		value->intval = bms->present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (bms->present &&
		    !power_supply_get_property_from_supplier(psy, prop, value))
			break;
		if (!bms->present || bms->current_error ||
		    (bms->current_ua > -10000 && bms->current_ua < 10000))
			value->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else
			value->intval = bms->current_ua > 0 ? POWER_SUPPLY_STATUS_CHARGING :
							     POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = power_supply_get_property_from_supplier(psy, prop, value);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bms->voltage_error;
		value->intval = bms->voltage_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bms->current_error;
		value->intval = bms->current_ua;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = bms->charge_error;
		if (!ret && (bms->counter.charge_uah < INT_MIN || bms->counter.charge_uah > INT_MAX))
			ret = -ERANGE;
		if (!ret)
			value->intval = bms->counter.charge_uah;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bms->temperature_error;
		value->intval = bms->temperature_decic;
		break;
	default:
		ret = -EINVAL;
	}
unlock:
	mutex_unlock(&bms->lock);
	return ret;
}

static const enum power_supply_property bms_properties[] = {
	POWER_SUPPLY_PROP_PRESENT, POWER_SUPPLY_PROP_STATUS, POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_CHARGE_COUNTER, POWER_SUPPLY_PROP_TEMP,
};

static void bms_work(struct work_struct *work)
{
	struct pm8226_bms *bms = container_of(to_delayed_work(work), struct pm8226_bms, work);

	mutex_lock(&bms->lock);
	bms_sample(bms);
	mutex_unlock(&bms->lock);
	power_supply_changed(bms->battery);
	queue_delayed_work(system_freezable_wq, &bms->work, msecs_to_jiffies(BMS_POLL_MS));
}

static void bms_stop(void *data)
{
	struct pm8226_bms *bms = data;

	cancel_delayed_work_sync(&bms->work);
}

static int bms_link_channel(struct device *dev, struct iio_channel *channel)
{
	struct device *supplier = channel->indio_dev->dev.parent;

	if (!device_link_add(dev, supplier, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -EINVAL;
	return device_is_bound(supplier) ? 0 : -EPROBE_DEFER;
}

static int bms_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config config = {};
	struct pm8226_bms *bms;
	const struct iio_chan_spec_ext_info *info;
	enum iio_chan_type type;
	u32 address[3];
	u8 identity[6];
	int i, ret;
	bool calibration_found = false;

	bms = devm_kzalloc(dev, sizeof(*bms), GFP_KERNEL);
	if (!bms)
		return -ENOMEM;
	bms->dev = dev;
	mutex_init(&bms->lock);
	bms->regmap = dev_get_regmap(dev->parent, NULL);
	if (!bms->regmap)
		return -ENODEV;
	ret = device_property_read_u32_array(dev, "reg", address, ARRAY_SIZE(address));
	if (ret)
		return ret;
	if (address[0] > 0xff00 || address[1] > 0xff00 || address[2] > U16_MAX)
		return -EINVAL;
	bms->base = address[0];
	bms->iadc_base = address[1];
	bms->present_reg = address[2];
	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms", &bms->rsense);
	if (ret || !bms->rsense)
		return -EINVAL;
	ret = regmap_bulk_read(bms->regmap, bms->base, identity, sizeof(identity));
	if (ret)
		return ret;
	if (identity[4] != 0x0d || identity[5] != 1)
		return -ENODEV;
	ret = regmap_bulk_read(bms->regmap, bms->iadc_base, identity, sizeof(identity));
	if (ret)
		return ret;
	if (identity[1] != 4 || identity[4] != 8 || identity[5] != 5)
		return -ENODEV;
	bms->sense_channel = devm_iio_channel_get(dev, "current");
	if (IS_ERR(bms->sense_channel))
		return dev_err_probe(dev, PTR_ERR(bms->sense_channel), "Current channel unavailable\n");
	bms->gain = devm_iio_channel_get(dev, "gain");
	if (IS_ERR(bms->gain))
		return dev_err_probe(dev, PTR_ERR(bms->gain), "Gain reference unavailable\n");
	bms->offset = devm_iio_channel_get(dev, "offset");
	if (IS_ERR(bms->offset))
		return dev_err_probe(dev, PTR_ERR(bms->offset), "Zero reference unavailable\n");
	bms->voltage = devm_iio_channel_get(dev, "voltage");
	if (IS_ERR(bms->voltage))
		return dev_err_probe(dev, PTR_ERR(bms->voltage), "Voltage channel unavailable\n");
	if (device_property_match_string(dev, "io-channel-names", "temperature") >= 0) {
		bms->temperature = devm_iio_channel_get(dev, "temperature");
		if (IS_ERR(bms->temperature))
			return dev_err_probe(dev, PTR_ERR(bms->temperature), "Temperature unavailable\n");
	}
	/* Managed links also order unbind before callbacks/code in suppliers disappear. */
	ret = bms_link_channel(dev, bms->sense_channel);
	if (ret)
		return ret;
	ret = bms_link_channel(dev, bms->voltage);
	if (ret)
		return ret;
	if (bms->gain->indio_dev != bms->sense_channel->indio_dev ||
	    bms->offset->indio_dev != bms->sense_channel->indio_dev)
		return -EINVAL;
	if (bms->temperature) {
		ret = bms_link_channel(dev, bms->temperature);
		if (ret)
			return ret;
		ret = iio_get_channel_type(bms->temperature, &type);
		if (ret || type != IIO_TEMP)
			return -EINVAL;
	}
	for (i = 0; i < 4; i++) {
		struct iio_channel *channel = i == 0 ? bms->sense_channel : i == 1 ? bms->gain :
							 i == 2 ? bms->offset : bms->voltage;

		ret = iio_get_channel_type(channel, &type);
		if (ret || type != (i ? IIO_VOLTAGE : IIO_CURRENT))
			return -EINVAL;
	}
	info = bms->sense_channel->channel->ext_info;
	for (; info && info->name; info++)
		if (!strcmp(info->name, "sys_calibration") && info->write)
			calibration_found = true;
	if (!calibration_found)
		return dev_err_probe(dev, -EOPNOTSUPP, "IADC calibration unavailable\n");
	bms_sample(bms);
	if (bms->present_error || (bms->present && !bms->configured))
		return dev_err_probe(dev, bms->present_error ?: bms->current_error,
				     "BMS initialization failed\n");
	bms->desc.name = "pm8226-bms";
	bms->desc.type = POWER_SUPPLY_TYPE_BATTERY;
	bms->desc.properties = bms_properties;
	bms->desc.num_properties = ARRAY_SIZE(bms_properties) - !bms->temperature;
	bms->desc.get_property = bms_get_property;
	config.drv_data = bms;
	config.fwnode = dev_fwnode(dev);
	bms->battery = devm_power_supply_register(dev, &bms->desc, &config);
	if (IS_ERR(bms->battery))
		return PTR_ERR(bms->battery);
	platform_set_drvdata(pdev, bms);
	INIT_DELAYED_WORK(&bms->work, bms_work);
	ret = devm_add_action_or_reset(dev, bms_stop, bms);
	if (ret)
		return ret;
	queue_delayed_work(system_freezable_wq, &bms->work, msecs_to_jiffies(BMS_POLL_MS));
	return 0;
}

static void bms_shutdown(struct platform_device *pdev)
{
	bms_stop(platform_get_drvdata(pdev));
}

static const struct of_device_id bms_match[] = {
	{ .compatible = "qcom,pm8226-bms" },
	{ }
};
MODULE_DEVICE_TABLE(of, bms_match);

static struct platform_driver bms_driver = {
	.probe = bms_probe,
	.shutdown = bms_shutdown,
	.driver = {
		.name = "qcom-pm8226-bms",
		.of_match_table = bms_match,
	},
};
module_platform_driver(bms_driver);
MODULE_DESCRIPTION("Qualcomm PM8226 non-VM battery measurements");
MODULE_LICENSE("GPL");
