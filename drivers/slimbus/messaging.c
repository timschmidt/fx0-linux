// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2011-2017, The Linux Foundation
 */

#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include "slimbus.h"

/* Each IDR slot owns its response storage, never the sender's stack frame. */
struct slim_pending_msg {
	struct slim_msg_txn *owner;
	struct slim_val_inf *msg_key;
	struct completion *comp;
	u8 *rbuf;
	u8 num_bytes;
	bool prepared;
	bool asynchronous;
};

/**
 * slim_init_transactions() - Initialize transaction state before RX starts
 * @ctrl: Controller whose transaction state is to be initialized
 *
 * Call before publishing a controller to RX or message producers. Registration
 * also calls this helper for controllers that do not need early messaging.
 * Initialization and registration must be serialized by the controller driver.
 */
void slim_init_transactions(struct slim_controller *ctrl)
{
	if (ctrl->transactions_initialized)
		return;

	idr_init(&ctrl->tid_idr);
	spin_lock_init(&ctrl->txn_lock);
	ctrl->transactions_stopped = false;
	ctrl->transactions_initialized = true;
}
EXPORT_SYMBOL_GPL(slim_init_transactions);

/**
 * slim_start_transactions() - Open transaction admission for registration
 * @ctrl: Initialized controller
 *
 * The driver must serialize registration with controller teardown.
 */
void slim_start_transactions(struct slim_controller *ctrl)
{
	unsigned long flags;

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	ctrl->transactions_stopped = false;
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);
}

/**
 * slim_stop_transactions() - Close admission and retire pending responses
 * @ctrl: Initialized controller being unregistered
 *
 * No completion is signalled for cancelled requests. Client remove callbacks
 * must quiesce their callers and may use slim_cancel_msg() to synchronize with
 * response delivery before releasing their buffers. Transport/DMA producers
 * still need their own shutdown and synchronization.
 */
void slim_stop_transactions(struct slim_controller *ctrl)
{
	struct slim_pending_msg *pending;
	unsigned long flags;
	unsigned int refs = 0;
	int id;

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	ctrl->transactions_stopped = true;
	idr_for_each_entry(&ctrl->tid_idr, pending, id) {
		idr_remove(&ctrl->tid_idr, id);
		refs += pending->prepared;
		kfree(pending);
	}
	idr_destroy(&ctrl->tid_idr);
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);

	if (refs)
		pm_runtime_mark_last_busy(ctrl->dev);
	while (refs) {
		refs--;
		pm_runtime_put_autosuspend(ctrl->dev);
	}
}

/**
 * slim_msg_response() - Deliver Message response received from a device to the
 *			framework.
 *
 * @ctrl: Controller handle
 * @reply: Reply received from the device
 * @tid: Transaction ID received with which framework can associate reply.
 * @len: Length of the reply
 *
 * Called by controller to inform framework about the response received.
 * This helps in making the API asynchronous, and controller-driver doesn't need
 * to manage 1 more table other than the one managed by framework mapping TID
 * with buffers
 */
void slim_msg_response(struct slim_controller *ctrl, u8 *reply, u8 tid, u8 len)
{
	struct slim_pending_msg *pending;
	unsigned long flags;

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	pending = idr_find(&ctrl->tid_idr, tid);
	if (!pending || !pending->prepared)
		goto unlock;

	if (!pending->rbuf || len > pending->num_bytes) {
		dev_err(ctrl->dev, "Got response to invalid TID:%d, len:%d\n",
				tid, len);
		goto unlock;
	}

	memcpy(pending->rbuf, reply, len);
	idr_remove(&ctrl->tid_idr, tid);
	if (pending->comp)
		complete(pending->comp);
	kfree(pending);
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);

	pm_runtime_mark_last_busy(ctrl->dev);
	pm_runtime_put_autosuspend(ctrl->dev);
	return;

unlock:
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);
}
EXPORT_SYMBOL_GPL(slim_msg_response);

/**
 * slim_alloc_txn_tid() - Allocate a tid to txn
 *
 * @ctrl: Controller handle
 * @txn: transaction to be allocated with tid.
 *
 * Return: zero on success with valid txn->tid and error code on failures.
 */
int slim_alloc_txn_tid(struct slim_controller *ctrl, struct slim_msg_txn *txn)
{
	struct slim_pending_msg *pending;
	unsigned long flags;
	int ret;

	txn->prepared = false;
	txn->tid = 0;
	pending = kzalloc(sizeof(*pending), GFP_ATOMIC);
	if (!pending)
		return -ENOMEM;
	pending->owner = txn;

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	if (ctrl->transactions_stopped)
		ret = -ESHUTDOWN;
	else
		ret = idr_alloc_cyclic(&ctrl->tid_idr, pending, 1,
				       SLIM_MAX_TIDS, GFP_ATOMIC);
	if (ret >= 0)
		txn->tid = ret;
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);
	if (ret < 0) {
		kfree(pending);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(slim_alloc_txn_tid);

/**
 * slim_prepare_txn() - Publish initialized response pointers for a transaction
 * @ctrl: Controller handle
 * @txn: Transaction with an allocated TID and initialized response pointers
 *
 * The sender must hold a runtime-PM reference. A prepared transaction acquires
 * a separate response reference, released by its response or cancellation.
 *
 * Return: zero on success, -EINVAL if the TID is no longer owned, or
 * -EALREADY if response ownership was already published.
 */
int slim_prepare_txn(struct slim_controller *ctrl, struct slim_msg_txn *txn)
{
	struct slim_pending_msg *pending, *other;
	unsigned long flags;
	int id, ret = 0;

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	pending = idr_find(&ctrl->tid_idr, txn->tid);
	if (!pending || pending->owner != txn) {
		ret = -EINVAL;
	} else if (pending->prepared) {
		ret = -EALREADY;
	} else if (!txn->msg) {
		ret = -EINVAL;
	} else {
		if (txn->msg->comp) {
			idr_for_each_entry(&ctrl->tid_idr, other, id) {
				if (other->asynchronous &&
				    other->msg_key == txn->msg) {
					ret = -EBUSY;
					goto unlock;
				}
			}
		}
		pending->msg_key = txn->msg;
		pending->rbuf = txn->msg->rbuf;
		pending->num_bytes = txn->msg->num_bytes;
		pending->comp = txn->comp;
		pending->asynchronous = !!txn->msg->comp;
		pm_runtime_get_noresume(ctrl->dev);
		pending->prepared = true;
		txn->prepared = true;
	}
unlock:
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(slim_prepare_txn);

/**
 * slim_free_txn_tid() - Free tid of txn
 *
 * @ctrl: Controller handle
 * @txn: transaction whose tid should be freed
 *
 * Return: true if this call retired the transaction, false if it no longer
 * owns its TID. Retirement synchronizes with response copy and completion
 * and releases the response PM reference of a prepared transaction.
 */
bool slim_free_txn_tid(struct slim_controller *ctrl, struct slim_msg_txn *txn)
{
	struct slim_pending_msg *pending;
	unsigned long flags;
	bool released = false, prepared = false;

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	pending = idr_find(&ctrl->tid_idr, txn->tid);
	if (pending && pending->owner == txn) {
		idr_remove(&ctrl->tid_idr, txn->tid);
		prepared = pending->prepared;
		kfree(pending);
		released = true;
	}
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);

	if (prepared) {
		pm_runtime_mark_last_busy(ctrl->dev);
		pm_runtime_put_autosuspend(ctrl->dev);
	}

	return released;
}
EXPORT_SYMBOL_GPL(slim_free_txn_tid);

/**
 * slim_cancel_msg() - Retire an asynchronous elemental request
 * @sbdev: Device used to submit the request
 * @msg: Original request descriptor passed to slim_xfer_msg()
 *
 * Call only after slim_xfer_msg() has returned, and serialize against a new
 * submission using the same descriptor. Cancellation synchronizes with any
 * response copy and completion already in progress. It does not signal the
 * completion. After either return value below, response delivery no longer
 * accesses this descriptor's buffers or completion.
 *
 * Return: 0 if cancelled, -ENOENT if already completed/cancelled, or -EINVAL
 * for a missing controller or descriptor.
 */
int slim_cancel_msg(struct slim_device *sbdev, struct slim_val_inf *msg)
{
	struct slim_controller *ctrl = sbdev->ctrl;
	struct slim_pending_msg *pending;
	unsigned long flags;
	int id, ret = -ENOENT;

	if (!ctrl || !msg)
		return -EINVAL;

	spin_lock_irqsave(&ctrl->txn_lock, flags);
	idr_for_each_entry(&ctrl->tid_idr, pending, id) {
		if (!pending->asynchronous || pending->msg_key != msg)
			continue;
		idr_remove(&ctrl->tid_idr, id);
		kfree(pending);
		ret = 0;
		break;
	}
	spin_unlock_irqrestore(&ctrl->txn_lock, flags);
	if (!ret) {
		pm_runtime_mark_last_busy(ctrl->dev);
		pm_runtime_put_autosuspend(ctrl->dev);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(slim_cancel_msg);

/**
 * slim_do_transfer() - Process a SLIMbus-messaging transaction
 *
 * @ctrl: Controller handle
 * @txn: Transaction to be sent over SLIMbus
 *
 * Called by controller to transmit messaging transactions not dealing with
 * Interface/Value elements. (e.g. transmitting a message to assign logical
 * address to a slave device
 *
 * Return: -ETIMEDOUT: If transmission of this message timed out
 *	(e.g. due to bus lines not being clocked or driven by controller)
 */
int slim_do_transfer(struct slim_controller *ctrl, struct slim_msg_txn *txn)
{
	DECLARE_COMPLETION_ONSTACK(done);
	bool need_tid = false, clk_pause_msg = false;
	int ret;
	unsigned long time_left;

	txn->tid = 0;
	/* Clock-pause messages do not acquire a runtime-PM reference. */
	if (ctrl->sched.clk_state == SLIM_CLK_ENTERING_PAUSE &&
		(txn->mt == SLIM_MSG_MT_CORE &&
		 txn->mc >= SLIM_MSG_MC_BEGIN_RECONFIGURATION &&
		 txn->mc <= SLIM_MSG_MC_RECONFIGURE_NOW))
		clk_pause_msg = true;

	if (!clk_pause_msg) {
		ret = pm_runtime_get_sync(ctrl->dev);
		if (ret < 0 || ctrl->sched.clk_state != SLIM_CLK_ACTIVE) {
			dev_err(ctrl->dev, "ctrl wrong state:%d, ret:%d\n",
				ctrl->sched.clk_state, ret);
			if (ret >= 0)
				ret = -EREMOTEIO;
			goto slim_xfer_err;
		}
	}

	need_tid = slim_tid_txn(txn->mt, txn->mc);
	if (need_tid) {
		txn->comp = txn->msg->comp ?: &done;
		ret = slim_alloc_txn_tid(ctrl, txn);
		if (ret)
			goto slim_xfer_err;
		ret = slim_prepare_txn(ctrl, txn);
		if (ret) {
			slim_free_txn_tid(ctrl, txn);
			goto slim_xfer_err;
		}
	}

	ret = ctrl->xfer_msg(ctrl, txn);
	if (ret) {
		slim_free_txn_tid(ctrl, txn);
	} else if (need_tid && !txn->msg->comp) {
		unsigned long ms = txn->rl + HZ;

		time_left = wait_for_completion_timeout(txn->comp,
							msecs_to_jiffies(ms));
		if (!time_left) {
			ret = -ETIMEDOUT;
			slim_free_txn_tid(ctrl, txn);
		}
	}

	if (ret)
		dev_err(ctrl->dev, "Tx:MT:0x%x, MC:0x%x, LA:0x%x failed:%d\n",
			txn->mt, txn->mc, txn->la, ret);

slim_xfer_err:
	if (!clk_pause_msg) {
		pm_runtime_mark_last_busy(ctrl->dev);
		pm_runtime_put_autosuspend(ctrl->dev);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(slim_do_transfer);

static int slim_val_inf_sanity(struct slim_controller *ctrl,
			       struct slim_val_inf *msg, u8 mc)
{
	if (!msg || msg->num_bytes > 16 ||
	    (msg->start_offset + msg->num_bytes) > 0xC00)
		goto reterr;
	switch (mc) {
	case SLIM_MSG_MC_REQUEST_VALUE:
	case SLIM_MSG_MC_REQUEST_INFORMATION:
		if (msg->rbuf != NULL)
			return 0;
		break;

	case SLIM_MSG_MC_CHANGE_VALUE:
	case SLIM_MSG_MC_CLEAR_INFORMATION:
		if (msg->wbuf != NULL)
			return 0;
		break;

	case SLIM_MSG_MC_REQUEST_CHANGE_VALUE:
	case SLIM_MSG_MC_REQUEST_CLEAR_INFORMATION:
		if (msg->rbuf != NULL && msg->wbuf != NULL)
			return 0;
		break;
	}
reterr:
	if (msg)
		dev_err(ctrl->dev, "Sanity check failed:msg:offset:0x%x, mc:%d\n",
			msg->start_offset, mc);
	return -EINVAL;
}

static u16 slim_slicesize(int code)
{
	static const u8 sizetocode[16] = {
		0, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7
	};

	code = clamp(code, 1, (int)ARRAY_SIZE(sizetocode));

	return sizetocode[code - 1];
}

/**
 * slim_xfer_msg() - Transfer a value info message on slim device
 *
 * @sbdev: slim device to which this msg has to be transferred
 * @msg: value info message pointer
 * @mc: message code of the message
 *
 * Called by drivers which want to transfer a vlaue or info elements.
 *
 * Return: -ETIMEDOUT: If transmission of this message timed out
 */
int slim_xfer_msg(struct slim_device *sbdev, struct slim_val_inf *msg,
		  u8 mc)
{
	DEFINE_SLIM_LDEST_TXN(txn_stack, mc, 6, sbdev->laddr, msg);
	struct slim_msg_txn *txn = &txn_stack;
	struct slim_controller *ctrl = sbdev->ctrl;
	int ret;
	u16 sl;

	if (!ctrl)
		return -EINVAL;

	ret = slim_val_inf_sanity(ctrl, msg, mc);
	if (ret)
		return ret;

	sl = slim_slicesize(msg->num_bytes);

	dev_dbg(ctrl->dev, "SB xfer msg:os:%x, len:%d, MC:%x, sl:%x\n",
		msg->start_offset, msg->num_bytes, mc, sl);

	txn->ec = ((sl | (1 << 3)) | ((msg->start_offset & 0xFFF) << 4));

	switch (mc) {
	case SLIM_MSG_MC_REQUEST_CHANGE_VALUE:
	case SLIM_MSG_MC_CHANGE_VALUE:
	case SLIM_MSG_MC_REQUEST_CLEAR_INFORMATION:
	case SLIM_MSG_MC_CLEAR_INFORMATION:
		txn->rl += msg->num_bytes;
		break;
	default:
		break;
	}

	if (slim_tid_txn(txn->mt, txn->mc))
		txn->rl++;

	return slim_do_transfer(ctrl, txn);
}
EXPORT_SYMBOL_GPL(slim_xfer_msg);

static void slim_fill_msg(struct slim_val_inf *msg, u32 addr,
			 size_t count, u8 *rbuf, u8 *wbuf)
{
	msg->start_offset = addr;
	msg->num_bytes = count;
	msg->rbuf = rbuf;
	msg->wbuf = wbuf;
	msg->comp = NULL;
}

/**
 * slim_read() - Read SLIMbus value element
 *
 * @sdev: client handle.
 * @addr:  address of value element to read.
 * @count: number of bytes to read. Maximum bytes allowed are 16.
 * @val: will return what the value element value was
 *
 * Return: -EINVAL for Invalid parameters, -ETIMEDOUT If transmission of
 * this message timed out (e.g. due to bus lines not being clocked
 * or driven by controller)
 */
int slim_read(struct slim_device *sdev, u32 addr, size_t count, u8 *val)
{
	struct slim_val_inf msg;

	slim_fill_msg(&msg, addr, count, val, NULL);

	return slim_xfer_msg(sdev, &msg, SLIM_MSG_MC_REQUEST_VALUE);
}
EXPORT_SYMBOL_GPL(slim_read);

/**
 * slim_readb() - Read byte from SLIMbus value element
 *
 * @sdev: client handle.
 * @addr:  address in the value element to read.
 *
 * Return: byte value of value element.
 */
int slim_readb(struct slim_device *sdev, u32 addr)
{
	int ret;
	u8 buf;

	ret = slim_read(sdev, addr, 1, &buf);
	if (ret < 0)
		return ret;
	else
		return buf;
}
EXPORT_SYMBOL_GPL(slim_readb);

/**
 * slim_write() - Write SLIMbus value element
 *
 * @sdev: client handle.
 * @addr:  address in the value element to write.
 * @count: number of bytes to write. Maximum bytes allowed are 16.
 * @val: value to write to value element
 *
 * Return: -EINVAL for Invalid parameters, -ETIMEDOUT If transmission of
 * this message timed out (e.g. due to bus lines not being clocked
 * or driven by controller)
 */
int slim_write(struct slim_device *sdev, u32 addr, size_t count, u8 *val)
{
	struct slim_val_inf msg;

	slim_fill_msg(&msg, addr, count,  NULL, val);

	return slim_xfer_msg(sdev, &msg, SLIM_MSG_MC_CHANGE_VALUE);
}
EXPORT_SYMBOL_GPL(slim_write);

/**
 * slim_writeb() - Write byte to SLIMbus value element
 *
 * @sdev: client handle.
 * @addr:  address of value element to write.
 * @value: value to write to value element
 *
 * Return: -EINVAL for Invalid parameters, -ETIMEDOUT If transmission of
 * this message timed out (e.g. due to bus lines not being clocked
 * or driven by controller)
 *
 */
int slim_writeb(struct slim_device *sdev, u32 addr, u8 value)
{
	return slim_write(sdev, addr, 1, &value);
}
EXPORT_SYMBOL_GPL(slim_writeb);
