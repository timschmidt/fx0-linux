// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, Sony Mobile Communications AB.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#include <linux/rpmsg.h>
#include <linux/soc/qcom/smd-rpm.h>

#define RPM_REQUEST_TIMEOUT     (5 * HZ)

/**
 * struct qcom_smd_rpm - state of the rpm device driver
 * @rpm_channel:	reference to the smd channel
 * @dev:		rpm device
 * @ack:		completion for acks
 * @lock:		mutual exclusion around the send/complete pair
 * @ack_status:		result of the rpm request
 * @ack_lock:		protects response matching and completion retirement
 * @msg_id:		last request identifier allocated on this channel
 * @pending_id:		identifier expected by the current waiter
 * @waiting:		whether the current request can still accept a response
 */
struct qcom_smd_rpm {
	struct rpmsg_endpoint *rpm_channel;
	struct device *dev;

	struct completion ack;
	struct mutex lock;
	int ack_status;
	spinlock_t ack_lock; /* Protects response matching and completion retirement. */
	u32 msg_id;
	u32 pending_id;
	bool waiting;
};

/**
 * struct qcom_rpm_header - header for all rpm requests and responses
 * @service_type:	identifier of the service
 * @length:		length of the payload
 */
struct qcom_rpm_header {
	__le32 service_type;
	__le32 length;
};

/**
 * struct qcom_rpm_request - request message to the rpm
 * @msg_id:	identifier of the outgoing message
 * @flags:	active/sleep state flags
 * @type:	resource type
 * @id:		resource id
 * @data_len:	length of the payload following this header
 */
struct qcom_rpm_request {
	__le32 msg_id;
	__le32 flags;
	__le32 type;
	__le32 id;
	__le32 data_len;
};

/**
 * struct qcom_rpm_message - response message from the rpm
 * @msg_type:	indicator of the type of message
 * @length:	the size of this message's payload
 * @msg_id:	message id
 * @message:	textual message from the rpm
 *
 * Multiple of these messages can be stacked in an rpm message.
 */
struct qcom_rpm_message {
	__le32 msg_type;
	__le32 length;
	union {
		__le32 msg_id;
		DECLARE_FLEX_ARRAY(u8, message);
	};
};

#define RPM_SERVICE_TYPE_REQUEST	0x00716572 /* "req\0" */

#define RPM_MSG_TYPE_ERR		0x00727265 /* "err\0" */
#define RPM_MSG_TYPE_MSG_ID		0x2367736d /* "msg#" */

/**
 * qcom_rpm_smd_write - write @buf to @type:@id
 * @rpm:	rpm handle
 * @state:	active/sleep state flags
 * @type:	resource type
 * @id:		resource identifier
 * @buf:	the data to be written
 * @count:	number of bytes in @buf
 */
int qcom_rpm_smd_write(struct qcom_smd_rpm *rpm,
		       int state,
		       u32 type, u32 id,
		       void *buf,
		       size_t count)
{
	unsigned long flags;
	u32 request_id;
	int left;
	int ret;
	struct {
		struct qcom_rpm_header hdr;
		struct qcom_rpm_request req;
		u8 payload[];
	} *pkt;
	size_t size = sizeof(*pkt) + count;

	/* SMD packets to the RPM may not exceed 256 bytes */
	if (WARN_ON(count >= 256 - sizeof(*pkt)))
		return -EINVAL;

	pkt = kmalloc(size, GFP_ATOMIC);
	if (!pkt)
		return -ENOMEM;

	mutex_lock(&rpm->lock);

	pkt->hdr.service_type = cpu_to_le32(RPM_SERVICE_TYPE_REQUEST);
	pkt->hdr.length = cpu_to_le32(sizeof(struct qcom_rpm_request) + count);

	request_id = ++rpm->msg_id;
	if (!request_id)
		request_id = ++rpm->msg_id;
	pkt->req.msg_id = cpu_to_le32(request_id);
	pkt->req.flags = cpu_to_le32(state);
	pkt->req.type = cpu_to_le32(type);
	pkt->req.id = cpu_to_le32(id);
	pkt->req.data_len = cpu_to_le32(count);
	memcpy(pkt->payload, buf, count);

	spin_lock_irqsave(&rpm->ack_lock, flags);
	reinit_completion(&rpm->ack);
	rpm->pending_id = request_id;
	rpm->ack_status = -ETIMEDOUT;
	rpm->waiting = true;
	spin_unlock_irqrestore(&rpm->ack_lock, flags);

	ret = rpmsg_send(rpm->rpm_channel, pkt, size);
	if (ret)
		goto out;

	left = wait_for_completion_timeout(&rpm->ack, RPM_REQUEST_TIMEOUT);
	if (!left) {
		ret = -ETIMEDOUT;
	} else {
		spin_lock_irqsave(&rpm->ack_lock, flags);
		ret = rpm->ack_status;
		spin_unlock_irqrestore(&rpm->ack_lock, flags);
	}

out:
	spin_lock_irqsave(&rpm->ack_lock, flags);
	rpm->waiting = false;
	spin_unlock_irqrestore(&rpm->ack_lock, flags);
	kfree(pkt);
	mutex_unlock(&rpm->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_rpm_smd_write);

static int qcom_smd_rpm_callback(struct rpmsg_device *rpdev,
				 void *data,
				 int count,
				 void *priv,
				 u32 addr)
{
	struct qcom_smd_rpm *rpm = dev_get_drvdata(&rpdev->dev);
	const u8 *buf = data, *end;
	unsigned long flags;
	u32 length, type, msg_length, response_id = 0;
	size_t remaining;
	bool have_id = false, have_error = false;
	int status = 0;

	if (!rpm || count < (int)sizeof(struct qcom_rpm_header))
		return 0;
	length = get_unaligned_le32(buf + sizeof(u32));
	if (get_unaligned_le32(buf) != RPM_SERVICE_TYPE_REQUEST ||
	    length > count - sizeof(struct qcom_rpm_header))
		return 0;
	buf += sizeof(struct qcom_rpm_header);
	end = buf + length;
	while (buf < end) {
		remaining = end - buf;
		if (remaining < 2 * sizeof(u32))
			return 0;
		type = get_unaligned_le32(buf);
		msg_length = get_unaligned_le32(buf + sizeof(u32));
		buf += 2 * sizeof(u32);
		remaining = end - buf;
		if (msg_length > remaining || ALIGN(msg_length, 4) > remaining)
			return 0;
		switch (type) {
		case RPM_MSG_TYPE_MSG_ID:
			if (have_id || msg_length != sizeof(u32))
				return 0;
			response_id = get_unaligned_le32(buf);
			have_id = true;
			break;
		case RPM_MSG_TYPE_ERR:
			if (have_error || !msg_length)
				return 0;
			have_error = true;
			if (msg_length == sizeof("resource does not exist") &&
			    !memcmp(buf, "resource does not exist", msg_length))
				status = -ENXIO;
			else
				status = -EINVAL;
			break;
		}
		buf += ALIGN(msg_length, 4);
	}
	if (!have_id)
		return 0;

	spin_lock_irqsave(&rpm->ack_lock, flags);
	if (rpm->waiting && rpm->pending_id == response_id) {
		rpm->ack_status = status;
		rpm->waiting = false;
		/* Complete before the waiter can retire/reinitialize this request. */
		complete(&rpm->ack);
	}
	spin_unlock_irqrestore(&rpm->ack_lock, flags);
	return 0;
}

static int qcom_smd_rpm_probe(struct rpmsg_device *rpdev)
{
	struct qcom_smd_rpm *rpm;

	rpm = devm_kzalloc(&rpdev->dev, sizeof(*rpm), GFP_KERNEL);
	if (!rpm)
		return -ENOMEM;

	mutex_init(&rpm->lock);
	spin_lock_init(&rpm->ack_lock);
	init_completion(&rpm->ack);

	rpm->dev = &rpdev->dev;
	rpm->rpm_channel = rpdev->ept;
	dev_set_drvdata(&rpdev->dev, rpm);

	return of_platform_populate(rpdev->dev.of_node, NULL, NULL, &rpdev->dev);
}

static void qcom_smd_rpm_remove(struct rpmsg_device *rpdev)
{
	of_platform_depopulate(&rpdev->dev);
}

static const struct of_device_id qcom_smd_rpm_of_match[] = {
	{ .compatible = "qcom,glink-smd-rpm" },
	{ .compatible = "qcom,smd-rpm" },
	/*
	 * Don't add any more compatibles to the list, two previous entryes
	 * should match all defined devices.
	 */
	{ .compatible = "qcom,rpm-apq8084" },
	{ .compatible = "qcom,rpm-ipq6018" },
	{ .compatible = "qcom,rpm-ipq9574" },
	{ .compatible = "qcom,rpm-msm8226" },
	{ .compatible = "qcom,rpm-msm8909" },
	{ .compatible = "qcom,rpm-msm8916" },
	{ .compatible = "qcom,rpm-msm8936" },
	{ .compatible = "qcom,rpm-msm8953" },
	{ .compatible = "qcom,rpm-msm8974" },
	{ .compatible = "qcom,rpm-msm8976" },
	{ .compatible = "qcom,rpm-msm8994" },
	{ .compatible = "qcom,rpm-msm8996" },
	{ .compatible = "qcom,rpm-msm8998" },
	{ .compatible = "qcom,rpm-sdm660" },
	{ .compatible = "qcom,rpm-sm6115" },
	{ .compatible = "qcom,rpm-sm6125" },
	{ .compatible = "qcom,rpm-sm6375" },
	{ .compatible = "qcom,rpm-qcm2290" },
	{ .compatible = "qcom,rpm-qcs404" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smd_rpm_of_match);

static struct rpmsg_driver qcom_smd_rpm_driver = {
	.probe = qcom_smd_rpm_probe,
	.remove = qcom_smd_rpm_remove,
	.callback = qcom_smd_rpm_callback,
	.drv  = {
		.name  = "qcom_smd_rpm",
		.of_match_table = qcom_smd_rpm_of_match,
	},
};

static int __init qcom_smd_rpm_init(void)
{
	return register_rpmsg_driver(&qcom_smd_rpm_driver);
}
arch_initcall(qcom_smd_rpm_init);

static void __exit qcom_smd_rpm_exit(void)
{
	unregister_rpmsg_driver(&qcom_smd_rpm_driver);
}
module_exit(qcom_smd_rpm_exit);

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Qualcomm SMD backed RPM driver");
MODULE_LICENSE("GPL v2");
