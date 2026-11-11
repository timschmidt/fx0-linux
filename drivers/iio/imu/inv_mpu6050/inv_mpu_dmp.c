// SPDX-License-Identifier: GPL-2.0-only
/* MPU6515 DMP RAM transport. Firmware and execution lifecycle are separate. */
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/math.h>
#include <linux/firmware.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "inv_mpu_iio.h"

#define INV_DMP_BANK_SEL 0x6d
#define INV_DMP_MEM_START 0x6e
#define INV_DMP_MEM_RW 0x6f
#define INV_DMP_RAM_START 32
#define INV_DMP_RAM_END 3072
#define INV_DMP_CHUNK 16

/*
 * Caller holds st->lock and a runtime-PM reference, with DMP/FIFO stopped.
 * Never select OTP or the reserved first 32 bytes. Re-select the address for
 * every transfer: independent regmap transactions must not rely on a retained
 * cursor. The memory port itself must not increment the register address.
 */
static int inv_mpu6515_dmp_mem_transfer(struct inv_mpu6050_state *st,
				      unsigned int address, u8 *data,
				      size_t len, bool write)
{
	unsigned int ctrl;
	size_t chunk;
	int ret;

	lockdep_assert_held(&st->lock);
	if (st->chip_type != INV_MPU6515)
		return -EOPNOTSUPP;
	if (!len || address < INV_DMP_RAM_START || address >= INV_DMP_RAM_END ||
	    len > INV_DMP_RAM_END - address)
		return -EINVAL;

	ret = regmap_read(st->map, st->reg->user_ctrl, &ctrl);
	if (ret)
		return ret;
	if (ctrl & (INV_MPU6050_BIT_DMP_EN | INV_MPU6050_BIT_FIFO_EN))
		return -EBUSY;

	while (len) {
		chunk = min_t(size_t, len, INV_DMP_CHUNK);
		chunk = min_t(size_t, chunk, 256 - (address & 0xff));
		ret = regmap_write(st->map, INV_DMP_BANK_SEL, address >> 8);
		if (ret)
			return ret;
		ret = regmap_write(st->map, INV_DMP_MEM_START, address & 0xff);
		if (ret)
			return ret;
		if (write)
			ret = regmap_noinc_write(st->map, INV_DMP_MEM_RW, data, chunk);
		else
			ret = regmap_noinc_read(st->map, INV_DMP_MEM_RW, data, chunk);
		if (ret)
			return ret;
		address += chunk;
		data += chunk;
		len -= chunk;
	}

	return 0;
}

int inv_mpu6515_dmp_mem_read(struct inv_mpu6050_state *st,
			   unsigned int address, u8 *data, size_t len)
{
	return inv_mpu6515_dmp_mem_transfer(st, address, data, len, false);
}

/* A failed write/verify leaves execution disabled; caller must invalidate FW. */
int inv_mpu6515_dmp_mem_write(struct inv_mpu6050_state *st,
			    unsigned int address, const u8 *data, size_t len)
{
	u8 verify[INV_DMP_CHUNK];
	size_t chunk, offset = 0;
	int ret;

	/* Validate the complete range before the first hardware write. */
	if (!len || address < INV_DMP_RAM_START || address >= INV_DMP_RAM_END ||
	    len > INV_DMP_RAM_END - address)
		return -EINVAL;
	while (offset < len) {
		chunk = min_t(size_t, len - offset, sizeof(verify));
		ret = inv_mpu6515_dmp_mem_transfer(st, address + offset,
						(u8 *)data + offset, chunk, true);
		if (ret)
			return ret;
		ret = inv_mpu6515_dmp_mem_read(st, address + offset, verify, chunk);
		if (ret)
			return ret;
		if (memcmp(verify, data + offset, chunk))
			return -EIO;
		offset += chunk;
	}

	return 0;
}

#define INV_MPU6515_DMP_FIRMWARE "invensense/mpu6515-dmp.bin"
#define INV_MPU6515_DMP_IMAGE_SIZE 2943
#define INV_MPU6515_DMP_IMAGE_CRC 0xa7e2110d
#define INV_MPU6515_DMP_PROGRAM_START 0x70
#define INV_MPU6515_DMP_ENTRY 0x0400

/*
 * Always reload: no cached RAM-valid state survives reset or power loss.
 * Caller owns lock/runtime PM and has stopped DMP/FIFO and masked interrupts.
 * On failure the image is unusable; this function never enables execution.
 */
int inv_mpu6515_dmp_load(struct inv_mpu6050_state *st)
{
	struct device *dev = regmap_get_device(st->map);
	const struct firmware *fw;
	u8 entry[] = { INV_MPU6515_DMP_ENTRY >> 8,
		       INV_MPU6515_DMP_ENTRY & 0xff };
	unsigned int ctrl, irq, high, low;
	int ret;

	lockdep_assert_held(&st->lock);
	if (st->chip_type != INV_MPU6515)
		return -EOPNOTSUPP;
	ret = regmap_read(st->map, st->reg->user_ctrl, &ctrl);
	if (ret)
		return ret;
	ret = regmap_read(st->map, st->reg->int_enable, &irq);
	if (ret)
		return ret;
	if (irq || (ctrl & (INV_MPU6050_BIT_DMP_EN | INV_MPU6050_BIT_FIFO_EN)))
		return -EBUSY;

	ret = request_firmware(&fw, INV_MPU6515_DMP_FIRMWARE, dev);
	if (ret)
		return ret;
	/* Match the manufacturer MPU6515 image contract before modifying RAM. */
	if (fw->size != INV_MPU6515_DMP_IMAGE_SIZE ||
	    crc32_le(0, fw->data, fw->size) != INV_MPU6515_DMP_IMAGE_CRC) {
		ret = -EINVAL;
		goto release;
	}
	ret = inv_mpu6515_dmp_mem_write(st, INV_DMP_RAM_START, fw->data, fw->size);
	if (ret)
		goto release;
	ret = regmap_bulk_write(st->map, INV_MPU6515_DMP_PROGRAM_START,
				entry, sizeof(entry));
	if (ret)
		goto release;
	ret = regmap_read(st->map, INV_MPU6515_DMP_PROGRAM_START, &high);
	if (ret)
		goto release;
	ret = regmap_read(st->map, INV_MPU6515_DMP_PROGRAM_START + 1, &low);
	if (!ret && ((high << 8) | low) != INV_MPU6515_DMP_ENTRY)
		ret = -EIO;
release:
	release_firmware(fw);
	return ret;
}

MODULE_FIRMWARE(INV_MPU6515_DMP_FIRMWARE);

/*
 * Reference image sensor frame, accel + six-axis quaternion, 25 Hz each.
 * No batching, pedometer, gesture, compass or pressure output is requested.
 * Interrupt generation is configured separately by the execution lifecycle.
 */
int inv_mpu6515_dmp_configure(struct inv_mpu6050_state *st)
{
	static const struct {
		u16 address;
		u8 length;
		u8 data[4];
	} patches[] = {
		{ 1127, 3, { 0x4c, 0xcd, 0x6c } },
		{ 1152, 3, { 0x36, 0x56, 0x76 } },
		{ 1131, 3, { 0x0c, 0xc9, 0x2c } },
		{ 1138, 3, { 0x26, 0x46, 0x66 } },
		{ 1770, 2, { 0xa3, 0xa3 } }, /* accel on */
		{ 1701, 2, { 0xa3, 0xa3 } }, /* six-axis quaternion on */
		{ 1804, 2, { 0xf4, 0x12 } }, /* gyro off */
		{ 1666, 2, { 0xf4, 0x13 } }, /* low-power quaternion off */
		{ 1736, 2, { 0xf4, 0x12 } }, /* pedometer quaternion off */
		{ 1838, 2, { 0xf4, 0x12 } }, /* compass off */
		{ 1872, 2, { 0xf4, 0x12 } }, /* pressure off */
		{ 1636, 2, { 0xf4, 0x0e } }, /* step output off */
		{ 1983, 1, { 0xfe } }, /* continuous FIFO interrupt mode */
		{ 2807, 2, { 0xf4, 0x03 } }, /* step indicator off */
		{ 744, 2, { 0, 7 } }, /* accel ODR */
		{ 736, 2, { 0 } }, /* accel ODR counter */
		{ 730, 2, { 0, 7 } }, /* quaternion ODR */
		{ 722, 2, { 0 } }, /* quaternion ODR counter */
		{ 454, 2, { 0 } }, /* batch enable */
		{ 452, 2, { 0 } }, /* batch word count */
		{ 436, 4, { 0 } }, /* batch counter */
		{ 384, 4, { 0 } }, /* run counter */
		{ 456, 4, { 0 } }, /* step timestamp */
	};
	unsigned int i;
	int ret;

	lockdep_assert_held(&st->lock);
	for (i = 0; i < ARRAY_SIZE(patches); i++) {
		ret = inv_mpu6515_dmp_mem_write(st, patches[i].address,
					      patches[i].data, patches[i].length);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Decode exactly one wire packet, returning bytes consumed. Caller retains an
 * incomplete tail on -EAGAIN and flushes/restarts on any other error. Do not
 * scan for another header after corruption: payload bytes can mimic headers.
 * Quaternion scalar reconstruction/mounting belong to the output ABI layer.
 */
int inv_mpu6515_dmp_decode(const u8 *data, size_t len,
			   struct inv_mpu6515_dmp_packet *packet)
{
	struct inv_mpu6515_dmp_packet decoded = { };
	u16 header;
	unsigned int i;
	size_t size;

	if (len < 2)
		return -EAGAIN;
	header = get_unaligned_be16(data);
	switch (header & 0xfff0) {
	case 0x4000:
		decoded.type = INV_MPU6515_DMP_ACCEL;
		size = 8;
		break;
	case 0x0400:
		decoded.type = INV_MPU6515_DMP_QUATERNION;
		size = 16;
		break;
	default:
		return -EPROTO;
	}
	if (len < size)
		return -EAGAIN;
	decoded.step_flags = header & 0xf;
	for (i = 0; i < ARRAY_SIZE(decoded.xyz); i++) {
		if (decoded.type == INV_MPU6515_DMP_ACCEL)
			decoded.xyz[i] = (s16)get_unaligned_be16(data + 2 + 2 * i);
		else
			decoded.xyz[i] = (s32)get_unaligned_be32(data + 4 + 4 * i);
	}
	*packet = decoded;

	return size;
}

/* Manufacturer SMD defaults; physical threshold units are not exposed. */
static int inv_mpu6515_dmp_configure_smd(struct inv_mpu6050_state *st)
{
	static const struct {
		u16 address;
		u8 length;
		u8 data[4];
	} patches[] = {
		{ 1770, 2, { 0xf4, 0x12 } }, /* no accel FIFO producer */
		{ 1701, 2, { 0xf4, 0x13 } }, /* no quaternion FIFO producer */
		{ 1983, 1, { 0xa3 } }, /* algorithm events only */
		{ 1985, 1, { 0xff } }, /* pedometer disabled */
		{ 2796, 3, { 0xf1, 0xf1, 0xf1 } }, /* pedometer IRQ disabled */
		{ 344, 4, { 0x05, 0xdc, 0, 0 } }, /* threshold 1500 << 16 */
		{ 340, 4, { 0, 0, 0x02, 0x58 } }, /* 3 seconds at 200 Hz */
		{ 348, 4, { 0, 0, 0, 0xc8 } }, /* 1 second at 200 Hz */
		{ 288, 2, { 0, 1 } }, /* SMD enable */
	};
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(patches); i++) {
		ret = inv_mpu6515_dmp_mem_write(st, patches[i].address,
					      patches[i].data, patches[i].length);
		if (ret)
			return ret;
	}
	return 0;
}

/* Registers changed by the DMP mode; restore cached raw settings verbatim. */
static const u8 inv_dmp_saved_regs[] = {
	0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x23, 0x38, 0x6a,
	0x6b, 0x6c, 0x70, 0x71,
};

int inv_mpu6515_dmp_stop(struct inv_mpu6050_state *st)
{
	unsigned int i, value;
	int ret, first = 0;

	lockdep_assert_held(&st->lock);
	st->dmp_active = false;
	st->dmp_overflow = false;
	st->dmp_tail_len = 0;
	st->dmp_last_timestamp = 0;
	if (!st->dmp_saved_valid)
		return 0;
	/* Try every cleanup operation, retaining the first failure for callers. */
	ret = regmap_write(st->map, st->reg->int_enable, 0);
	if (ret)
		first = ret;
	ret = regmap_write(st->map, st->reg->user_ctrl, st->chip_config.user_ctrl);
	if (ret && !first)
		first = ret;
	for (i = 0; i < ARRAY_SIZE(inv_dmp_saved_regs); i++) {
		if (inv_dmp_saved_regs[i] == st->reg->int_enable ||
		    inv_dmp_saved_regs[i] == st->reg->user_ctrl)
			continue;
		ret = regmap_write(st->map, inv_dmp_saved_regs[i], st->dmp_saved[i]);
		if (ret && !first)
			first = ret;
	}
	ret = regmap_write(st->map, st->reg->user_ctrl, st->dmp_saved[7]);
	if (ret && !first)
		first = ret;
	/* A successful bus write alone is insufficient to certify restoration. */
	for (i = 0; i < ARRAY_SIZE(inv_dmp_saved_regs); i++) {
		if (inv_dmp_saved_regs[i] == st->reg->int_enable)
			continue;
		ret = regmap_read(st->map, inv_dmp_saved_regs[i], &value);
		if (!ret && value != st->dmp_saved[i])
			ret = -EIO;
		if (ret && !first)
			first = ret;
	}
	/* Do not rearm an interrupt if functional configuration restoration failed. */
	if (!first) {
		ret = regmap_write(st->map, st->reg->int_enable, st->dmp_saved[6]);
		if (ret)
			first = ret;
	}
	if (!first)
		st->dmp_saved_valid = false;
	return first;
}

/* Caller holds lock and runtime PM before resuming ordinary register access. */
int inv_mpu6515_dmp_recover(struct inv_mpu6050_state *st)
{
	lockdep_assert_held(&st->lock);
	if (st->dmp_active)
		return -EBUSY;
	if (st->dmp_saved_valid)
		return inv_mpu6515_dmp_stop(st);
	return 0;
}

int inv_mpu6515_dmp_start(struct inv_mpu6050_state *st)
{
	static const struct reg_sequence setup[] = {
		{ 0x19, 4 }, { 0x1a, 1 }, { 0x1b, 0x18 }, { 0x1c, 0 },
		{ 0x1d, 0x40 }, { 0x23, 0 }, { 0x6b, 9 }, { 0x6c, 0 },
	};
	unsigned int i, value;
	int ret, cleanup;

	lockdep_assert_held(&st->lock);
	if (st->chip_type != INV_MPU6515)
		return -EOPNOTSUPP;
	if (st->dmp_active || st->chip_config.wom_en)
		return -EBUSY;
	if (st->dmp_saved_valid) {
		ret = inv_mpu6515_dmp_stop(st);
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(inv_dmp_saved_regs); i++) {
		ret = regmap_read(st->map, inv_dmp_saved_regs[i], &value);
		if (ret)
			return ret;
		st->dmp_saved[i] = value;
	}
	if (st->dmp_saved[6] || (st->dmp_saved[7] & 0xc0))
		return -EBUSY;
	st->dmp_saved_valid = true;
	ret = inv_mpu6515_dmp_load(st);
	if (ret)
		goto fail;
	ret = inv_mpu6515_dmp_configure(st);
	if (ret)
		goto fail;
	if (st->dmp_smd_mode) {
		ret = inv_mpu6515_dmp_configure_smd(st);
		if (ret)
			goto fail;
		/* Clear a previous algorithm status before enabling a new run. */
		ret = regmap_read(st->map, 0x39, &value);
		if (ret)
			goto fail;
	}
	ret = regmap_multi_reg_write(st->map, setup, ARRAY_SIZE(setup));
	if (ret)
		goto fail;
	msleep(50);
	ret = regmap_write(st->map, st->reg->user_ctrl,
			   st->chip_config.user_ctrl | INV_MPU6050_BIT_DMP_RST |
			   INV_MPU6050_BIT_FIFO_RST);
	if (ret)
		goto fail;
	usleep_range(10000, 12000);
	st->dmp_overflow = false;
	st->dmp_tail_len = 0;
	st->dmp_last_timestamp = 0;
	ret = regmap_write(st->map, st->reg->user_ctrl,
			   st->chip_config.user_ctrl | INV_MPU6050_BIT_DMP_EN |
			   (st->dmp_smd_mode ? 0 : INV_MPU6050_BIT_FIFO_EN));
	if (ret)
		goto fail;
	st->dmp_active = true;
	if (st->dmp_smd_mode) {
		ret = regmap_read(st->map, st->reg->int_status, &value);
		if (ret)
			goto fail;
	}
	ret = regmap_write(st->map, st->reg->int_enable,
			   INV_MPU6050_BIT_DMP_INT_EN |
			   (st->dmp_smd_mode ? 0 : INV_MPU6050_BIT_FIFO_OVERFLOW_INT));
	if (!ret)
		return 0;
fail:
	cleanup = inv_mpu6515_dmp_stop(st);
	if (cleanup)
		dev_err(regmap_get_device(st->map), "DMP restoration failed: %d\n", cleanup);
	return ret;
}

/* Flush corruption/overflow without retaining a tail or replaying old samples. */
static int inv_mpu6515_dmp_flush(struct inv_mpu6050_state *st)
{
	int ret;

	st->dmp_overflow = false;
	st->dmp_tail_len = 0;
	ret = regmap_write(st->map, st->reg->user_ctrl,
			   st->chip_config.user_ctrl | INV_MPU6050_BIT_FIFO_RST);
	if (ret)
		return ret;
	return regmap_write(st->map, st->reg->user_ctrl,
			    st->chip_config.user_ctrl | INV_MPU6050_BIT_DMP_EN |
			    INV_MPU6050_BIT_FIFO_EN);
}

int inv_mpu6515_dmp_read_fifo(struct iio_dev *indio_dev, s64 timestamp)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	struct inv_mpu6515_dmp_packet packet;
	/* Repeated quaternion storage aligns the complete timestamped scan to 16. */
	struct { s32 q[4]; s64 padding; s64 timestamp; } sample __aligned(16) = { };
	unsigned int hi, lo, check, tries, count, quats = 0;
	size_t len, offset;
	u64 norm;
	s64 t;
	int ret, pushed;

	lockdep_assert_held(&st->lock);
	if (st->dmp_overflow) {
		ret = -EOVERFLOW;
		goto flush;
	}
	for (tries = 0; tries < 4; tries++) {
		ret = regmap_read(st->map, 0x72, &hi);
		if (ret)
			goto flush;
		ret = regmap_read(st->map, 0x73, &lo);
		if (ret)
			goto flush;
		ret = regmap_read(st->map, 0x72, &check);
		if (ret)
			goto flush;
		if (hi == check)
			break;
	}
	if (tries == 4) {
		ret = -EAGAIN;
		goto flush;
	}
	count = (hi << 8) | lo;
	if (count >= 992) {
		ret = -EOVERFLOW;
		goto flush;
	}
	if (!count)
		return 0;
	memcpy(st->dmp_fifo, st->dmp_tail, st->dmp_tail_len);
	ret = regmap_noinc_read(st->map, st->reg->fifo_r_w,
				st->dmp_fifo + st->dmp_tail_len, count);
	if (ret)
		goto flush;
	len = count + st->dmp_tail_len;
	/* Validate the complete batch before publishing any sample from it. */
	for (offset = 0; offset < len; offset += ret) {
		ret = inv_mpu6515_dmp_decode(st->dmp_fifo + offset, len - offset, &packet);
		if (ret == -EAGAIN)
			break;
		if (ret < 0)
			goto flush;
		if (packet.type == INV_MPU6515_DMP_QUATERNION) {
			norm = (u64)((s64)packet.xyz[0] * packet.xyz[0]) +
			       (u64)((s64)packet.xyz[1] * packet.xyz[1]) +
			       (u64)((s64)packet.xyz[2] * packet.xyz[2]);
			if (norm > BIT_ULL(60)) {
				ret = -ERANGE;
				goto flush;
			}
			quats++;
		}
	}
	st->dmp_tail_len = len - offset;
	memcpy(st->dmp_tail, st->dmp_fifo + offset, st->dmp_tail_len);
	len = offset;
	/* IRQ-anchored 25 Hz timestamps; hardware packet timestamps are not used. */
	t = timestamp - (s64)(quats ? quats - 1 : 0) * 40000000;
	for (offset = 0; offset < len; offset += ret) {
		ret = inv_mpu6515_dmp_decode(st->dmp_fifo + offset, len - offset, &packet);
		if (packet.type != INV_MPU6515_DMP_QUATERNION)
			continue;
		norm = (u64)((s64)packet.xyz[0] * packet.xyz[0]) +
		       (u64)((s64)packet.xyz[1] * packet.xyz[1]) +
		       (u64)((s64)packet.xyz[2] * packet.xyz[2]);
		if (norm > BIT_ULL(60)) {
			ret = -ERANGE;
			goto flush;
		}
		sample.q[0] = int_sqrt64(BIT_ULL(60) - norm);
		memcpy(&sample.q[1], packet.xyz, sizeof(packet.xyz));
		if (t > st->dmp_last_timestamp) {
			pushed = iio_push_to_buffers_with_ts(indio_dev, &sample, sizeof(sample), t);
			if (pushed) {
				ret = pushed;
				goto flush;
			}
			st->dmp_last_timestamp = t;
		}
		t += 40000000;
	}
	return 0;
flush:
	dev_warn_ratelimited(regmap_get_device(st->map), "DMP FIFO reset: %d\n", ret);
	if (inv_mpu6515_dmp_flush(st))
		inv_mpu6515_dmp_stop(st);
	return ret;
}

/* Enable owns a direct-mode claim; all callers hold lock. Event owns PM vote. */
int inv_mpu6515_dmp_smd_set(struct inv_mpu6050_state *st, bool enable)
{
	struct device *dev = regmap_get_device(st->map);
	int ret, wake_ret = 0;

	lockdep_assert_held(&st->lock);
	if (st->chip_type != INV_MPU6515 || st->irq <= 0)
		return -EOPNOTSUPP;
	if (st->dmp_smd_suspended)
		return -EBUSY;
	if (!enable) {
		if (st->dmp_smd_irq_wake) {
			wake_ret = disable_irq_wake(st->irq);
			if (!wake_ret)
				st->dmp_smd_irq_wake = false;
		}
		if (!st->dmp_smd_pm)
			return wake_ret;
		ret = inv_mpu6515_dmp_stop(st);
		st->dmp_smd_mode = false;
		st->dmp_smd_armed = false;
		st->dmp_smd_pm = false;
		pm_runtime_put_autosuspend(dev);
		return ret ? ret : wake_ret;
	}
	if (st->dmp_smd_irq_wake)
		return -EBUSY;
	if (st->dmp_smd_armed)
		return 0;
	if (st->dmp_active || st->chip_config.wom_en)
		return -EBUSY;
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;
	st->dmp_smd_mode = true;
	ret = inv_mpu6515_dmp_start(st);
	if (ret) {
		st->dmp_smd_mode = false;
		pm_runtime_put_autosuspend(dev);
		return ret;
	}
	st->dmp_smd_armed = true;
	st->dmp_smd_pm = true;
	return 0;
}

/* One-shot event attribution: only manufacturer status 0x39 bit 0x04 qualifies. */
int inv_mpu6515_dmp_smd_event(struct inv_mpu6050_state *st)
{
	unsigned int status;
	int ret;

	lockdep_assert_held(&st->lock);
	if (!st->dmp_smd_mode || !st->dmp_smd_armed)
		return 0;
	if (st->dmp_smd_suspended)
		return -EBUSY;
	st->dmp_smd_last_status = 0;
	ret = regmap_read(st->map, 0x39, &status);
	if (ret)
		return ret;
	st->dmp_smd_last_status = status;
	if (!(status & BIT(2)))
		return 0;
	pm_wakeup_event(regmap_get_device(st->map), 0);
	st->dmp_smd_armed = false;
	dev_dbg(regmap_get_device(st->map), "DMP significant motion status %#x\n", status);
	ret = inv_mpu6515_dmp_smd_set(st, false);
	if (ret)
		dev_err(regmap_get_device(st->map), "DMP event cleanup failed: %d\n", ret);
	return 1;
}

void inv_mpu6515_dmp_cleanup(void *data)
{
	struct inv_mpu6050_state *st = data;

	guard(mutex)(&st->lock);
	if (st->dmp_smd_pm || st->dmp_smd_irq_wake)
		inv_mpu6515_dmp_smd_set(st, false);
}

/* IRQ synchronization must occur outside st->lock: the IRQ thread takes it. */
int inv_mpu6515_dmp_smd_suspend(struct inv_mpu6050_state *st, bool *event)
{
	struct device *dev = regmap_get_device(st->map);
	int ret;

	*event = false;
	if (!device_may_wakeup(dev))
		return -EBUSY;
	disable_irq(st->irq);
	mutex_lock(&st->lock);
	if (!st->dmp_smd_pm || !st->dmp_smd_armed || !st->dmp_active ||
	    st->dmp_smd_suspended || st->dmp_smd_irq_wake) {
		ret = -EBUSY;
		goto unlock;
	}
	/* Attribute a pre-entry event, then abort rather than silently lose it. */
	ret = inv_mpu6515_dmp_smd_event(st);
	if (ret) {
		*event = ret > 0;
		ret = ret > 0 ? -EBUSY : ret;
		goto unlock;
	}
	ret = enable_irq_wake(st->irq);
	if (ret)
		goto unlock;
	st->dmp_smd_irq_wake = true;
	st->dmp_smd_suspended = true;
	mutex_unlock(&st->lock);
	/* No registers, regulators or firmware are changed while the bus sleeps. */
	return 0;
unlock:
	mutex_unlock(&st->lock);
	enable_irq(st->irq);
	return ret;
}

/* Parent I2C adapter has resumed before this child's system-resume callback. */
int inv_mpu6515_dmp_smd_resume(struct inv_mpu6050_state *st, bool *event)
{
	int ret = 0, status_ret;

	*event = false;
	mutex_lock(&st->lock);
	if (!st->dmp_smd_suspended) {
		mutex_unlock(&st->lock);
		return 0;
	}
	if (st->dmp_smd_irq_wake) {
		ret = disable_irq_wake(st->irq);
		if (!ret)
			st->dmp_smd_irq_wake = false;
	}
	st->dmp_smd_suspended = false;
	status_ret = inv_mpu6515_dmp_smd_event(st);
	*event = status_ret > 0;
	dev_dbg(regmap_get_device(st->map), "DMP resume status %#x, event %d\n",
		st->dmp_smd_last_status, *event);
	if (!ret && status_ret < 0)
		ret = status_ret;
	mutex_unlock(&st->lock);
	enable_irq(st->irq);
	return ret;
}
