// SPDX-License-Identifier: GPL-2.0
/*
 * camss-video.c
 *
 * Qualcomm MSM Camera Subsystem - V4L2 device node
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2018 Linaro Ltd.
 */
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-sg.h>

#include "camss-vfe-gen1.h"
#include "camss-video.h"
#include "camss.h"

#define CAMSS_FRAME_MIN_WIDTH		1
#define CAMSS_FRAME_MAX_WIDTH		8191
#define CAMSS_FRAME_MIN_HEIGHT		1
#define CAMSS_FRAME_MAX_HEIGHT_RDI	8191
#define CAMSS_FRAME_MAX_HEIGHT_PIX	4096

/* -----------------------------------------------------------------------------
 * Helper functions
 */

/*
 * video_mbus_to_pix_mp - Convert v4l2_mbus_framefmt to v4l2_pix_format_mplane
 * @mbus: v4l2_mbus_framefmt format (input)
 * @pix: v4l2_pix_format_mplane format (output)
 * @f: a pointer to formats array element to be used for the conversion
 * @alignment: bytesperline alignment value
 *
 * Fill the output pix structure with information from the input mbus format.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int video_mbus_to_pix_mp(const struct v4l2_mbus_framefmt *mbus,
				struct v4l2_pix_format_mplane *pix,
				const struct camss_format_info *f,
				unsigned int alignment)
{
	unsigned int i;
	u32 bytesperline;

	memset(pix, 0, sizeof(*pix));
	v4l2_fill_pix_format_mplane(pix, mbus);
	pix->pixelformat = f->pixelformat;
	pix->num_planes = f->planes;
	for (i = 0; i < pix->num_planes; i++) {
		bytesperline = pix->width / f->hsub[i].numerator *
			f->hsub[i].denominator * f->bpp[i] / 8;
		bytesperline = ALIGN(bytesperline, alignment);
		pix->plane_fmt[i].bytesperline = bytesperline;
		pix->plane_fmt[i].sizeimage = pix->height /
				f->vsub[i].numerator * f->vsub[i].denominator *
				bytesperline;
	}

	return 0;
}

static struct v4l2_subdev *video_remote_subdev(struct camss_video *video,
					       u32 *pad)
{
	struct media_pad *remote;

	remote = media_pad_remote_pad_first(&video->pad);

	if (!remote || !is_media_entity_v4l2_subdev(remote->entity))
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static int video_get_subdev_format(struct camss_video *video,
				   struct v4l2_format *format)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EPIPE;

	fmt.pad = pad;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	ret = camss_format_find_format(fmt.format.code, format->fmt.pix_mp.pixelformat,
				       video->formats, video->nformats);
	if (ret < 0)
		return ret;

	format->type = video->type;

	return video_mbus_to_pix_mp(&fmt.format, &format->fmt.pix_mp,
				    &video->formats[ret], video->bpl_alignment);
}

/* -----------------------------------------------------------------------------
 * Video queue operations
 */

static int video_queue_setup(struct vb2_queue *q,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[], struct device *alloc_devs[])
{
	struct camss_video *video = vb2_get_drv_priv(q);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	unsigned int i;

	if (*num_planes) {
		if (*num_planes != format->num_planes)
			return -EINVAL;

		for (i = 0; i < *num_planes; i++)
			if (sizes[i] < format->plane_fmt[i].sizeimage)
				return -EINVAL;

		return 0;
	}

	*num_planes = format->num_planes;

	for (i = 0; i < *num_planes; i++)
		sizes[i] = format->plane_fmt[i].sizeimage;

	return 0;
}

static int video_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	struct sg_table *sgt;
	struct scatterlist *s;
	dma_addr_t expected;
	unsigned long size;
	unsigned int i, j;

	for (i = 0; i < format->num_planes; i++) {
		sgt = vb2_dma_sg_plane_desc(vb, i);
		if (!sgt)
			return -EFAULT;

		/* Adjacent DMA segments may cover one linear image plane. */
		expected = sg_dma_address(sgt->sgl);
		size = 0;
		for_each_sgtable_dma_sg(sgt, s, j) {
			if (sg_dma_address(s) != expected)
				break;
			size += sg_dma_len(s);
			if (size >= format->plane_fmt[i].sizeimage)
				break;
			expected += sg_dma_len(s);
		}
		if (size < format->plane_fmt[i].sizeimage) {
			dev_err_ratelimited(video->camss->dev,
					    "Buffer plane %u is not contiguous\n",
					    i);
			return -EFAULT;
		}

		buffer->addr[i] = sg_dma_address(sgt->sgl);
	}

	if (format->pixelformat == V4L2_PIX_FMT_NV12 ||
			format->pixelformat == V4L2_PIX_FMT_NV21 ||
			format->pixelformat == V4L2_PIX_FMT_NV16 ||
			format->pixelformat == V4L2_PIX_FMT_NV61)
		buffer->addr[1] = buffer->addr[0] +
				format->plane_fmt[0].bytesperline *
				format->height;

	return 0;
}

static int video_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	unsigned int i;

	for (i = 0; i < format->num_planes; i++) {
		if (format->plane_fmt[i].sizeimage > vb2_plane_size(vb, i))
			return -EINVAL;

		vb2_set_plane_payload(vb, i, format->plane_fmt[i].sizeimage);
	}

	vbuf->field = V4L2_FIELD_NONE;

	return 0;
}

static void video_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);

	video->ops->queue_buffer(video, buffer);
}

static int video_check_format(struct camss_video *video)
{
	struct v4l2_pix_format_mplane *pix = &video->active_fmt.fmt.pix_mp;
	struct v4l2_format format;
	struct v4l2_pix_format_mplane *sd_pix = &format.fmt.pix_mp;
	int ret;

	sd_pix->pixelformat = pix->pixelformat;
	ret = video_get_subdev_format(video, &format);
	if (ret < 0)
		return ret;

	if (pix->pixelformat != sd_pix->pixelformat ||
	    pix->height != sd_pix->height ||
	    pix->width != sd_pix->width ||
	    pix->num_planes != sd_pix->num_planes ||
	    pix->field != format.fmt.pix_mp.field)
		return -EPIPE;

	return 0;
}

static int video_prepare_streaming(struct vb2_queue *q)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = &video->vdev;
	int ret;

	ret = v4l2_pipeline_pm_get(&vdev->entity);
	if (ret < 0)
		dev_err(video->camss->dev, "Failed to power up pipeline: %d\n",
			ret);
	else
		video->pipeline_powered = true;

	return ret;
}

static struct v4l2_subdev *video_pipeline_upstream(struct media_entity *entity)
{
	struct media_pad *pad = &entity->pads[0];

	if (!(pad->flags & MEDIA_PAD_FL_SINK))
		return NULL;

	pad = media_pad_remote_pad_first(pad);
	if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
		return NULL;

	return media_entity_to_v4l2_subdev(pad->entity);
}

static void video_pipeline_stop_subdevs(struct camss_video *video,
				       struct v4l2_subdev *stop_before)
{
	struct v4l2_subdev *subdev;
	int ret;

	lockdep_assert_held(&video->camss->pipeline_lock);

	/* Stop the capture engine first, while the source can still run. */
	for (subdev = video_pipeline_upstream(&video->vdev.entity);
	     subdev && subdev != stop_before;
	     subdev = video_pipeline_upstream(&subdev->entity)) {
		if (!subdev->s_stream_enabled)
			continue;

		ret = v4l2_subdev_call(subdev, video, s_stream, 0);
		if (ret)
			dev_err(video->camss->dev, "Failed to stop %s: %d\n",
				subdev->name, ret);
	}
}

static int video_pipeline_start_subdevs(struct camss_video *video)
{
	struct v4l2_subdev *subdev;
	int ret;

	lockdep_assert_held(&video->camss->pipeline_lock);

	for (subdev = video_pipeline_upstream(&video->vdev.entity);
	     subdev; subdev = video_pipeline_upstream(&subdev->entity)) {
		/* Legacy s_stream has one enable state for each subdevice. */
		if (subdev->s_stream_enabled) {
			ret = -EBUSY;
			goto unwind;
		}

		ret = v4l2_subdev_call(subdev, video, s_stream, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			goto unwind;
	}

	return 0;

unwind:
	video_pipeline_stop_subdevs(video, subdev);
	return ret;
}

static void video_pipeline_power_put(struct camss_video *video)
{
	if (video->pipeline_powered) {
		v4l2_pipeline_pm_put(&video->vdev.entity);
		video->pipeline_powered = false;
	}
}

static int video_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = &video->vdev;
	int ret;

	mutex_lock(&video->camss->pipeline_lock);

	ret = video_device_pipeline_alloc_start(vdev);
	if (ret < 0) {
		dev_err(video->camss->dev, "Failed to start media pipeline: %d\n", ret);
		goto flush_buffers;
	}

	ret = video_check_format(video);
	if (ret < 0)
		goto stop_pipeline;

	ret = video_pipeline_start_subdevs(video);
	if (ret)
		goto stop_pipeline;

	goto unlock;

stop_pipeline:
	video_device_pipeline_stop(vdev);
flush_buffers:
	video->ops->flush_buffers(video, VB2_BUF_STATE_QUEUED);
unlock:
	mutex_unlock(&video->camss->pipeline_lock);

	return ret;
}

static void video_stop_streaming(struct vb2_queue *q)
{
	struct camss_video *video = vb2_get_drv_priv(q);

	mutex_lock(&video->camss->pipeline_lock);
	video_pipeline_stop_subdevs(video, NULL);
	video_device_pipeline_stop(&video->vdev);
	video->ops->flush_buffers(video, VB2_BUF_STATE_ERROR);
	mutex_unlock(&video->camss->pipeline_lock);
}

static void video_unprepare_streaming(struct vb2_queue *q)
{
	video_pipeline_power_put(vb2_get_drv_priv(q));
}

static void video_suspend_hold_buffers(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct camss *camss = vfe->camss;
	struct vfe_output *output = &line->output;
	unsigned long flags;
	unsigned int active;

	/* The bus is halted and no IRQ handler can still return a buffer. */
	spin_lock_irqsave(&vfe->output_lock, flags);
	INIT_LIST_HEAD(&camss->sleep_buffers);
	list_splice_init(&output->pending_bufs, &camss->sleep_buffers);
	active = output->gen1.active_buf;
	if (output->buf[!active])
		list_add(&output->buf[!active]->queue, &camss->sleep_buffers);
	if (output->buf[active])
		list_add(&output->buf[active]->queue, &camss->sleep_buffers);
	output->buf[0] = NULL;
	output->buf[1] = NULL;
	camss->sleep_sequence = output->sequence;
	if (output->last_buffer) {
		list_add(&output->last_buffer->queue, &camss->sleep_buffers);
		camss->sleep_sequence = output->last_buffer->vb.sequence;
		output->last_buffer = NULL;
	}
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

/*
 * Count the VFE power references a single route holds: one for the VFE
 * line and one for each CSID in the route that powers this VFE as its parent.
 */
static unsigned int video_route_vfe_power_refs(struct camss_video *video,
					       struct vfe_device *vfe)
{
	struct camss *camss = video->camss;
	struct v4l2_subdev *subdev;
	unsigned int i, refs = 1;

	for (subdev = video_pipeline_upstream(&video->vdev.entity);
	     subdev; subdev = video_pipeline_upstream(&subdev->entity))
		for (i = 0; i < camss->res->csid_num; i++)
			if (subdev == &camss->csid[i].subdev &&
			    camss->csid[i].id == vfe->id)
				refs++;

	return refs;
}

int msm_video_suspend(struct vfe_line *line)
{
	struct camss_video *video = &line->video_out;
	struct vfe_device *vfe = to_vfe(line);
	struct camss *camss = video->camss;
	struct media_device *mdev = &camss->media_dev;
	bool exclusive_power, sole_stream;
	unsigned int refs;
	int ret;

	lockdep_assert_held(&video->q_lock);
	lockdep_assert_held(&camss->pipeline_lock);

	mutex_lock(&mdev->graph_mutex);
	exclusive_power = line->subdev.entity.use_count == 1;
	mutex_unlock(&mdev->graph_mutex);
	refs = video_route_vfe_power_refs(video, vfe);
	mutex_lock(&vfe->power_lock);
	exclusive_power = exclusive_power && vfe->power_count == refs;
	mutex_unlock(&vfe->power_lock);
	mutex_lock(&vfe->stream_lock);
	sole_stream = vfe->stream_count == 1;
	mutex_unlock(&vfe->stream_lock);
	if (!video->pipeline_powered || !video->vb2_q.start_streaming_called ||
	    !exclusive_power || !sole_stream)
		return -EBUSY;

	video_pipeline_stop_subdevs(video, NULL);
	ret = vfe_gen1_halt(vfe);
	if (ret) {
		/* Drop the pipeline's power reference before releasing buffers. */
		video_pipeline_power_put(video);
		synchronize_irq(vfe->irq);
		video->ops->flush_buffers(video, VB2_BUF_STATE_ERROR);
		vb2_queue_error(&video->vb2_q);
		return ret;
	}
	synchronize_irq(vfe->irq);
	video_suspend_hold_buffers(line);
	camss->sleep_line = line;

	/* The bus already acknowledged halt, so power down without another. */
	mutex_lock(&vfe->power_lock);
	vfe->was_streaming = false;
	mutex_unlock(&vfe->power_lock);
	video_pipeline_power_put(video);
	return 0;
}

void msm_video_resume(struct vfe_line *line)
{
	struct camss_video *video = &line->video_out;
	struct vfe_device *vfe = to_vfe(line);
	struct camss *camss = video->camss;
	unsigned long flags;
	int ret;

	lockdep_assert_held(&video->q_lock);
	lockdep_assert_held(&camss->pipeline_lock);

	ret = v4l2_pipeline_pm_get(&video->vdev.entity);
	video->pipeline_powered = !ret;

	spin_lock_irqsave(&vfe->output_lock, flags);
	list_splice_init(&camss->sleep_buffers, &line->output.pending_bufs);
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	if (!ret)
		ret = video_pipeline_start_subdevs(video);
	camss->sleep_line = NULL;
	if (ret) {
		dev_err(camss->dev, "Failed to restart capture after sleep: %d\n", ret);
		video_pipeline_power_put(video);
		synchronize_irq(vfe->irq);
		video->ops->flush_buffers(video, VB2_BUF_STATE_ERROR);
		vb2_queue_error(&video->vb2_q);
	}
}

static const struct vb2_ops msm_video_vb2_q_ops = {
	.queue_setup     = video_queue_setup,
	.buf_init        = video_buf_init,
	.buf_prepare     = video_buf_prepare,
	.buf_queue       = video_buf_queue,
	.prepare_streaming = video_prepare_streaming,
	.start_streaming = video_start_streaming,
	.stop_streaming  = video_stop_streaming,
	.unprepare_streaming = video_unprepare_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int video_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "qcom-camss", sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm Camera Subsystem", sizeof(cap->card));

	return 0;
}

static int video_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct camss_video *video = video_drvdata(file);
	int i, j, k;
	u32 mcode = f->mbus_code;

	if (f->type != video->type)
		return -EINVAL;

	if (f->index >= video->nformats)
		return -EINVAL;

	/*
	 * Find index "i" of "k"th unique pixelformat in formats array.
	 *
	 * If f->mbus_code passed to video_enum_fmt() is not zero, a device
	 * with V4L2_CAP_IO_MC capability restricts enumeration to only the
	 * pixel formats that can be produced from that media bus code.
	 * This is implemented by skipping video->formats[] entries with
	 * code != f->mbus_code (if f->mbus_code is not zero).
	 * If the f->mbus_code passed to video_enum_fmt() is not supported,
	 * -EINVAL is returned.
	 * If f->mbus_code is zero, all the pixel formats are enumerated.
	 */
	k = -1;
	for (i = 0; i < video->nformats; i++) {
		if (mcode != 0 && video->formats[i].code != mcode)
			continue;

		for (j = 0; j < i; j++) {
			if (mcode != 0 && video->formats[j].code != mcode)
				continue;
			if (video->formats[i].pixelformat ==
					video->formats[j].pixelformat)
				break;
		}

		if (j == i)
			k++;

		if (k == f->index)
			break;
	}

	if (k == -1 || k < f->index)
		/*
		 * All the unique pixel formats matching the arguments
		 * have been enumerated (k >= 0 and f->index > 0), or
		 * no pixel formats match the non-zero f->mbus_code (k == -1).
		 */
		return -EINVAL;

	f->pixelformat = video->formats[i].pixelformat;

	return 0;
}

static int video_enum_framesizes(struct file *file, void *fh,
				 struct v4l2_frmsizeenum *fsize)
{
	struct camss_video *video = video_drvdata(file);
	int i;

	if (fsize->index)
		return -EINVAL;

	/* Only accept pixel format present in the formats[] table */
	for (i = 0; i < video->nformats; i++) {
		if (video->formats[i].pixelformat == fsize->pixel_format)
			break;
	}

	if (i == video->nformats)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = CAMSS_FRAME_MIN_WIDTH;
	fsize->stepwise.max_width = CAMSS_FRAME_MAX_WIDTH;
	fsize->stepwise.min_height = CAMSS_FRAME_MIN_HEIGHT;
	fsize->stepwise.max_height = video->max_height ?: (video->line_based ?
		CAMSS_FRAME_MAX_HEIGHT_PIX : CAMSS_FRAME_MAX_HEIGHT_RDI);
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int video_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);

	*f = video->active_fmt;

	return 0;
}

static int __video_try_fmt(struct camss_video *video, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp;
	const struct camss_format_info *fi;
	struct v4l2_plane_pix_format *p;
	u32 bytesperline[3] = { 0 };
	u32 sizeimage[3] = { 0 };
	u32 max_height = video->max_height ?: CAMSS_FRAME_MAX_HEIGHT_RDI;
	u32 width, height;
	u32 bpl, lines;
	int i, j;

	pix_mp = &f->fmt.pix_mp;

	if (video->line_based)
		for (i = 0; i < pix_mp->num_planes && i < 3; i++) {
			p = &pix_mp->plane_fmt[i];
			bytesperline[i] = clamp_t(u32, p->bytesperline,
						  1, 65528);
			sizeimage[i] = clamp_t(u32, p->sizeimage,
					       bytesperline[i],
					       bytesperline[i] * CAMSS_FRAME_MAX_HEIGHT_PIX);
		}

	for (j = 0; j < video->nformats; j++)
		if (pix_mp->pixelformat == video->formats[j].pixelformat)
			break;

	if (j == video->nformats)
		j = 0; /* default format */

	fi = &video->formats[j];
	width = pix_mp->width;
	height = pix_mp->height;

	memset(pix_mp, 0, sizeof(*pix_mp));

	pix_mp->pixelformat = fi->pixelformat;
	pix_mp->width = clamp_t(u32, width, 1, CAMSS_FRAME_MAX_WIDTH);
	pix_mp->height = clamp_t(u32, height, 1, max_height);
	pix_mp->num_planes = fi->planes;
	for (i = 0; i < pix_mp->num_planes; i++) {
		bpl = pix_mp->width / fi->hsub[i].numerator *
			fi->hsub[i].denominator * fi->bpp[i] / 8;
		bpl = ALIGN(bpl, video->bpl_alignment);
		pix_mp->plane_fmt[i].bytesperline = bpl;
		pix_mp->plane_fmt[i].sizeimage = pix_mp->height /
			fi->vsub[i].numerator * fi->vsub[i].denominator * bpl;
	}

	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->colorspace = V4L2_COLORSPACE_SRGB;
	pix_mp->flags = 0;
	pix_mp->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix_mp->colorspace);
	pix_mp->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
					pix_mp->colorspace, pix_mp->ycbcr_enc);
	pix_mp->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix_mp->colorspace);

	if (video->line_based)
		for (i = 0; i < pix_mp->num_planes; i++) {
			p = &pix_mp->plane_fmt[i];
			p->bytesperline = clamp_t(u32, p->bytesperline,
						  1, 65528);
			p->sizeimage = clamp_t(u32, p->sizeimage,
					       p->bytesperline,
					       p->bytesperline * CAMSS_FRAME_MAX_HEIGHT_PIX);
			lines = p->sizeimage / p->bytesperline;

			if (p->bytesperline < bytesperline[i])
				p->bytesperline = ALIGN(bytesperline[i], 8);

			if (p->sizeimage < p->bytesperline * lines)
				p->sizeimage = p->bytesperline * lines;

			if (p->sizeimage < sizeimage[i])
				p->sizeimage = sizeimage[i];
		}

	return 0;
}

static int video_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);

	return __video_try_fmt(video, f);
}

static int video_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&video->vb2_q))
		return -EBUSY;

	ret = __video_try_fmt(video, f);
	if (ret < 0)
		return ret;

	video->active_fmt = *f;

	return 0;
}

static int video_enum_input(struct file *file, void *fh,
			    struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strscpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int video_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

static int video_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

static const struct v4l2_ioctl_ops msm_vid_ioctl_ops = {
	.vidioc_querycap		= video_querycap,
	.vidioc_enum_fmt_vid_cap	= video_enum_fmt,
	.vidioc_enum_framesizes		= video_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane	= video_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= video_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= video_try_fmt,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_enum_input		= video_enum_input,
	.vidioc_g_input			= video_g_input,
	.vidioc_s_input			= video_s_input,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static const struct v4l2_file_operations msm_vid_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.poll           = vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.read		= vb2_fop_read,
};

/* -----------------------------------------------------------------------------
 * CAMSS video core
 */

static void msm_video_release(struct video_device *vdev)
{
	struct camss_video *video = video_get_drvdata(vdev);

	media_entity_cleanup(&vdev->entity);

	mutex_destroy(&video->q_lock);
	mutex_destroy(&video->lock);

	if (atomic_dec_and_test(&video->camss->ref_count))
		camss_delete(video->camss);
}

/*
 * msm_video_init_format - Helper function to initialize format
 * @video: struct camss_video
 *
 * Initialize pad format with default value.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int msm_video_init_format(struct camss_video *video)
{
	int ret;
	struct v4l2_format format = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.fmt.pix_mp = {
			.width = 1920,
			.height = 1080,
			.pixelformat = video->formats[0].pixelformat,
		},
	};

	ret = __video_try_fmt(video, &format);
	if (ret < 0)
		return ret;

	video->active_fmt = format;

	return 0;
}

/*
 * msm_video_register - Register a video device node
 * @video: struct camss_video
 * @v4l2_dev: V4L2 device
 * @name: name to be used for the video device node
 *
 * Initialize and register a video device node to a V4L2 device. Also
 * initialize the vb2 queue.
 *
 * Return 0 on success or a negative error code otherwise
 */

int msm_video_register(struct camss_video *video, struct v4l2_device *v4l2_dev,
		       const char *name)
{
	struct media_pad *pad = &video->pad;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret;

	vdev = &video->vdev;

	mutex_init(&video->q_lock);

	q = &video->vb2_q;
	q->drv_priv = video;
	q->mem_ops = &vb2_dma_sg_memops;
	q->ops = &msm_video_vb2_q_ops;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_DMABUF | VB2_MMAP | VB2_READ;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->buf_struct_size = sizeof(struct camss_buffer);
	q->dev = video->camss->dev;
	q->lock = &video->q_lock;
	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init vb2 queue: %d\n", ret);
		goto error_vb2_init;
	}

	pad->flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, pad);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init video entity: %d\n",
			ret);
		goto error_vb2_init;
	}

	mutex_init(&video->lock);

	ret = msm_video_init_format(video);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init format: %d\n", ret);
		goto error_video_register;
	}

	vdev->fops = &msm_vid_fops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING
			  | V4L2_CAP_READWRITE | V4L2_CAP_IO_MC;
	vdev->ioctl_ops = &msm_vid_ioctl_ops;
	vdev->release = msm_video_release;
	vdev->v4l2_dev = v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->queue = &video->vb2_q;
	vdev->lock = &video->lock;
	strscpy(vdev->name, name, sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to register video device: %d\n",
			ret);
		goto error_video_register;
	}

	video_set_drvdata(vdev, video);
	atomic_inc(&video->camss->ref_count);

	return 0;

error_video_register:
	media_entity_cleanup(&vdev->entity);
	mutex_destroy(&video->lock);
error_vb2_init:
	mutex_destroy(&video->q_lock);

	return ret;
}

void msm_video_unregister(struct camss_video *video)
{
	atomic_inc(&video->camss->ref_count);
	vb2_video_unregister_device(&video->vdev);
	atomic_dec(&video->camss->ref_count);
}
