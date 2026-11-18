// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for QCOM secure IOMMUs.  Somewhat based on arm-smmu.c
 *
 * Copyright (C) 2013 ARM Limited
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Copyright (c) 2026 Timothy Schmidt
 * Copyright (C) 2017 Red Hat
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/io-pgtable.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kconfig.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "arm-smmu.h"

#define SMMU_INTR_SEL_NS     0x2000

/* GR1 sits one 4K page above GR0 on the msm8974 QSMMU */
#define QCOM_IOMMU_GR1			0x1000

/* The implementation-defined space sits two 4K pages above GR0 */
#define QCOM_IOMMU_IMPL_DEF		0x2000

/* Micro-MMU control, at the start of the implementation-defined space */
#define QCOM_IOMMU_MICRO_MMU_CTRL	(QCOM_IOMMU_IMPL_DEF + 0x000)
#define MICRO_MMU_CTRL_HALT_REQ		BIT(2)
#define MICRO_MMU_CTRL_IDLE		BIT(3)
#define QCOM_IOMMU_WRAPPER_CLK_SEL	BIT(0)

/* Redirect all cacheable requests to the L2 slave port */
#define QCOM_IOMMU_ACTLR_BPRC		(BIT(28) | BIT(29) | BIT(30))

enum qcom_iommu_clk {
	CLK_IFACE,
	CLK_BUS,
	CLK_TBU,
	CLK_NUM,
};

struct qcom_iommu_ctx;

struct qcom_iommu_bfb_reg {
	u32			 offset;
	u32			 value;
};

struct qcom_iommu_sid {
	u8			 cbndx;
	u8			 sid;
};

/*
 * Per-instance configuration for instances whose global register space
 * is at least partially OS-managed (reg points at the global space and
 * SMMU_INTR_SEL_NS must not be written). The stream ID map is only used
 * on non-secured instances.
 */
struct qcom_iommu_cfg {
	enum io_pgtable_fmt		 fmt;
	/* the walker faults on the AF bit despite it being set */
	bool				 no_afe;
	/* context banks lose their state over GDSC power collapse */
	bool				 ctx_restore;
	/* the micro-MMU must be halted while its registers are programmed */
	bool				 halt;
	/* faulting transactions must terminate rather than stall */
	bool				 no_stall;
	bool				 has_wrapper_clk_sel;
	const struct qcom_iommu_bfb_reg	*bfb;
	unsigned int			 num_bfb;
	const struct qcom_iommu_sid	*sids;	/* one SMR slot per entry */
	unsigned int			 num_sids;
};

struct qcom_iommu_dev {
	/* IOMMU core code handle */
	struct iommu_device	 iommu;
	struct device		*dev;
	const struct qcom_iommu_cfg *cfg;
	struct clk_bulk_data clks[CLK_NUM];
	void __iomem		*local_base;
	void __iomem		*global_base;
	void __iomem		*wrapper_clk_sel;
	/* Serializes Venus context ownership, TLB access and clock gating. */
	spinlock_t		 ctx_lock;
	bool			 ctx_online;
	u32			 sec_id;
	bool			 non_secure;
	u8			 max_asid;
	struct qcom_iommu_ctx	*ctxs[];   /* indexed by asid */
};

static enum io_pgtable_fmt qcom_iommu_pgtbl_fmt(struct qcom_iommu_dev *qcom_iommu)
{
	return qcom_iommu->cfg ? qcom_iommu->cfg->fmt : ARM_32_LPAE_S1;
}

struct qcom_iommu_ctx_state {
	u64			 ttbr0;
	u32			 tcr[2];
	u32			 mair[2];
	u32			 contextidr;
	u32			 sctlr;
};

struct qcom_iommu_ctx {
	struct device		*dev;
	void __iomem		*base;
	bool			 secure_init;
	bool			 secured_ctx;
	u8			 asid;      /* asid and ctx bank # are 1:1 */
	int			 irq;
	struct iommu_domain	*domain;
	/* CB state replayed after power collapse on OS-managed instances */
	struct qcom_iommu_ctx_state state;
};

struct qcom_iommu_domain {
	struct io_pgtable_ops	*pgtbl_ops;
	spinlock_t		 pgtbl_lock;
	struct mutex		 init_mutex; /* Protects iommu pointer */
	struct iommu_domain	 domain;
	struct qcom_iommu_dev	*iommu;
	u32			*asids;
	unsigned int		 num_asids;
};

static struct qcom_iommu_domain *to_qcom_iommu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct qcom_iommu_domain, domain);
}

static const struct iommu_ops qcom_iommu_ops;

static struct qcom_iommu_ctx * to_ctx(struct qcom_iommu_domain *d, unsigned asid)
{
	struct qcom_iommu_dev *qcom_iommu = d->iommu;
	if (!qcom_iommu)
		return NULL;
	return qcom_iommu->ctxs[asid];
}

static inline void
iommu_writel(struct qcom_iommu_ctx *ctx, unsigned reg, u32 val)
{
	writel_relaxed(val, ctx->base + reg);
}

static inline void
iommu_writeq(struct qcom_iommu_ctx *ctx, unsigned reg, u64 val)
{
	writeq_relaxed(val, ctx->base + reg);
}

static inline u32
iommu_readl(struct qcom_iommu_ctx *ctx, unsigned reg)
{
	return readl_relaxed(ctx->base + reg);
}

static inline u64
iommu_readq(struct qcom_iommu_ctx *ctx, unsigned reg)
{
	return readq_relaxed(ctx->base + reg);
}

static inline void
qcom_iommu_gr0_write(struct qcom_iommu_dev *qcom_iommu, unsigned int reg,
		      u32 val)
{
	writel_relaxed(val, qcom_iommu->global_base + reg);
}

static inline u32
qcom_iommu_gr0_read(struct qcom_iommu_dev *qcom_iommu, unsigned int reg)
{
	return readl_relaxed(qcom_iommu->global_base + reg);
}

static inline void
qcom_iommu_gr1_write(struct qcom_iommu_dev *qcom_iommu, unsigned int reg,
		      u32 val)
{
	writel_relaxed(val, qcom_iommu->global_base + QCOM_IOMMU_GR1 + reg);
}

static int qcom_iommu_ctx_tlb_sync(struct qcom_iommu_ctx *ctx)
{
	unsigned int val;
	int ret;

	iommu_writel(ctx, ARM_SMMU_CB_TLBSYNC, 0);
	ret = readl_poll_timeout(ctx->base + ARM_SMMU_CB_TLBSTATUS, val,
				 (val & 0x1) == 0, 0, 5000000);
	if (ret)
		dev_err(ctx->dev, "timeout waiting for TLB SYNC\n");

	return ret;
}

static bool qcom_iommu_tlb_lock(struct qcom_iommu_dev *iommu, unsigned long *flags)
{
	if (!iommu->wrapper_clk_sel)
		return true;

	spin_lock_irqsave(&iommu->ctx_lock, *flags);
	if (iommu->ctx_online)
		return true;

	/*
	 * The linked consumer is inactive. Its page tables can still change
	 * and the bank will be invalidated before the next successful resume.
	 */
	spin_unlock_irqrestore(&iommu->ctx_lock, *flags);
	return false;
}

static void qcom_iommu_tlb_unlock(struct qcom_iommu_dev *iommu, unsigned long flags)
{
	if (iommu->wrapper_clk_sel)
		spin_unlock_irqrestore(&iommu->ctx_lock, flags);
}

static void qcom_iommu_tlb_sync(struct qcom_iommu_domain *qcom_domain)
{
	unsigned int i;

	for (i = 0; i < qcom_domain->num_asids; i++)
		qcom_iommu_ctx_tlb_sync(to_ctx(qcom_domain, qcom_domain->asids[i]));
}

static void qcom_iommu_tlb_inv_context(void *cookie)
{
	struct qcom_iommu_domain *qcom_domain = cookie;
	struct qcom_iommu_dev *iommu = qcom_domain->iommu;
	unsigned long flags = 0;
	unsigned int i;

	if (!qcom_iommu_tlb_lock(iommu, &flags))
		return;

	for (i = 0; i < qcom_domain->num_asids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, qcom_domain->asids[i]);

		iommu_writel(ctx, ARM_SMMU_CB_S1_TLBIASID, ctx->asid);
	}

	qcom_iommu_tlb_sync(qcom_domain);
	qcom_iommu_tlb_unlock(iommu, flags);
}

static void qcom_iommu_tlb_inv_range_nosync(unsigned long iova, size_t size,
					    size_t granule, bool leaf, void *cookie)
{
	struct qcom_iommu_domain *qcom_domain = cookie;
	struct qcom_iommu_dev *iommu = qcom_domain->iommu;
	unsigned long flags = 0;
	unsigned i, reg;

	if (!qcom_iommu_tlb_lock(iommu, &flags))
		return;

	reg = leaf ? ARM_SMMU_CB_S1_TLBIVAL : ARM_SMMU_CB_S1_TLBIVA;
	iova = (iova >> 12) << 12;

	for (i = 0; i < qcom_domain->num_asids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, qcom_domain->asids[i]);
		unsigned long addr = iova | ctx->asid;
		size_t s = size;

		do {
			iommu_writel(ctx, reg, addr);
			addr += granule;
		} while (s -= granule);
	}

	qcom_iommu_tlb_unlock(iommu, flags);
}

static void qcom_iommu_tlb_sync_online(struct qcom_iommu_domain *qcom_domain)
{
	struct qcom_iommu_dev *iommu = qcom_domain->iommu;
	unsigned long flags = 0;

	if (!qcom_iommu_tlb_lock(iommu, &flags))
		return;

	qcom_iommu_tlb_sync(qcom_domain);
	qcom_iommu_tlb_unlock(iommu, flags);
}

static void qcom_iommu_tlb_flush_walk(unsigned long iova, size_t size,
				      size_t granule, void *cookie)
{
	qcom_iommu_tlb_inv_range_nosync(iova, size, granule, false, cookie);
	qcom_iommu_tlb_sync_online(cookie);
}

static void qcom_iommu_tlb_add_page(struct iommu_iotlb_gather *gather,
				    unsigned long iova, size_t granule,
				    void *cookie)
{
	qcom_iommu_tlb_inv_range_nosync(iova, granule, granule, true, cookie);
}

static const struct iommu_flush_ops qcom_flush_ops = {
	.tlb_flush_all	= qcom_iommu_tlb_inv_context,
	.tlb_flush_walk = qcom_iommu_tlb_flush_walk,
	.tlb_add_page	= qcom_iommu_tlb_add_page,
};

static irqreturn_t qcom_iommu_fault(int irq, void *dev)
{
	struct qcom_iommu_ctx *ctx = dev;
	struct device *parent = ctx->dev->parent;
	struct qcom_iommu_dev *iommu = dev_get_drvdata(parent);
	struct iommu_domain *domain;
	irqreturn_t handled = IRQ_NONE;
	u32 fsr, fsynr;
	u64 iova;

	if (iommu->wrapper_clk_sel && pm_runtime_get_if_active(parent) <= 0)
		return IRQ_NONE;

	fsr = iommu_readl(ctx, ARM_SMMU_CB_FSR);

	if (!(fsr & ARM_SMMU_CB_FSR_FAULT))
		goto out;

	fsynr = iommu_readl(ctx, ARM_SMMU_CB_FSYNR0);
	iova = iommu_readq(ctx, ARM_SMMU_CB_FAR);

	domain = READ_ONCE(ctx->domain);
	if (!domain || report_iommu_fault(domain, ctx->dev, iova, 0)) {
		dev_err_ratelimited(ctx->dev,
				    "Unhandled context fault: fsr=0x%x, "
				    "iova=0x%016llx, fsynr=0x%x, cb=%d\n",
				    fsr, iova, fsynr, ctx->asid);
	}

	iommu_writel(ctx, ARM_SMMU_CB_FSR, fsr);
	iommu_writel(ctx, ARM_SMMU_CB_RESUME, ARM_SMMU_RESUME_TERMINATE);
	handled = IRQ_HANDLED;
out:
	if (iommu->wrapper_clk_sel)
		pm_runtime_put(parent);

	return handled;
}

static void qcom_iommu_bfb_setup(struct qcom_iommu_dev *qcom_iommu)
{
	const struct qcom_iommu_cfg *cfg = qcom_iommu->cfg;
	unsigned int i;

	if (!cfg)
		return;

	/* The BFB registers sit in the implementation-defined space */
	for (i = 0; i < cfg->num_bfb; i++) {
		if (WARN_ON_ONCE(cfg->bfb[i].offset >= SZ_4K))
			continue;

		writel_relaxed(cfg->bfb[i].value,
			       qcom_iommu->global_base + QCOM_IOMMU_IMPL_DEF +
			       cfg->bfb[i].offset);
	}
}

/*
 * Reset and configure the global register space of an instance the
 * secure world does not manage: global fault state, TLB, stream
 * mapping (SMR/S2CR/CBAR) and the global configuration register.
 */
static int qcom_iommu_reset_ns(struct qcom_iommu_dev *qcom_iommu)
{
	const struct qcom_iommu_cfg *cfg = qcom_iommu->cfg;
	unsigned int i, num_smr;
	u32 reg;
	int ret;

	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sACR, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sCR2, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sGFAR, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sGFAR + 4, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sGFSRRESTORE, 0);

	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_TLBIALLNSNH, 0);
	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sTLBGSYNC, 0);
	ret = read_poll_timeout(qcom_iommu_gr0_read, reg,
				!(reg & ARM_SMMU_sTLBGSTATUS_GSACTIVE),
				0, 5000000, false,
				qcom_iommu, ARM_SMMU_GR0_sTLBGSTATUS);
	if (ret) {
		dev_err(qcom_iommu->dev,
			"timeout waiting for global TLB SYNC\n");
		return ret;
	}

	num_smr = FIELD_GET(ARM_SMMU_ID0_NUMSMRG,
			    qcom_iommu_gr0_read(qcom_iommu, ARM_SMMU_GR0_ID0));
	for (i = 0; i < num_smr; i++)
		qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_SMR(i), 0);

	for (i = 0; i < cfg->num_sids; i++) {
		const struct qcom_iommu_sid *sid = &cfg->sids[i];

		qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_SMR(i),
				      ARM_SMMU_SMR_VALID |
				      FIELD_PREP(ARM_SMMU_SMR_ID, sid->sid));
		qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_S2CR(i),
				      FIELD_PREP(ARM_SMMU_S2CR_TYPE, S2CR_TYPE_TRANS) |
				      FIELD_PREP(ARM_SMMU_S2CR_CBNDX, sid->cbndx) |
				      FIELD_PREP(ARM_SMMU_S2CR_MEMATTR, 0xa) |
				      FIELD_PREP(ARM_SMMU_S2CR_NSCFG, 3));
		qcom_iommu_gr1_write(qcom_iommu,
				      ARM_SMMU_GR1_CBAR(sid->cbndx),
				      FIELD_PREP(ARM_SMMU_CBAR_TYPE,
						 CBAR_TYPE_S1_TRANS_S2_BYPASS) |
				      FIELD_PREP(ARM_SMMU_CBAR_IRPTNDX, 1) |
				      FIELD_PREP(ARM_SMMU_CBAR_VMID, 3) |
				      FIELD_PREP(ARM_SMMU_CBAR_S1_BPSHCFG, 2) |
				      FIELD_PREP(ARM_SMMU_CBAR_S1_MEMATTR, 0xa));
	}

	qcom_iommu_gr0_write(qcom_iommu, ARM_SMMU_GR0_sCR0,
			      ARM_SMMU_sCR0_SMCFCFG | ARM_SMMU_sCR0_USFCFG |
			      ARM_SMMU_sCR0_STALLD | ARM_SMMU_sCR0_GCFGFIE |
			      ARM_SMMU_sCR0_GCFGFRE | ARM_SMMU_sCR0_GFIE |
			      ARM_SMMU_sCR0_GFRE);

	return 0;
}

/*
 * Halting the micro-MMU quiesces the translation front-end: it stops new
 * client transactions being accepted and waits for the outstanding ones to
 * retire, so that the context bank registers can be reprogrammed without
 * in-flight traffic racing the change.
 */
static int qcom_iommu_halt(struct qcom_iommu_dev *qcom_iommu)
{
	void __iomem *reg;
	u32 val;
	int ret;

	if (!qcom_iommu->cfg || !qcom_iommu->cfg->halt)
		return 0;

	reg = qcom_iommu->global_base + QCOM_IOMMU_MICRO_MMU_CTRL;

	writel_relaxed(readl_relaxed(reg) | MICRO_MMU_CTRL_HALT_REQ, reg);

	ret = readl_poll_timeout(reg, val, val & MICRO_MMU_CTRL_IDLE, 0, 100000);
	if (ret)
		dev_err(qcom_iommu->dev, "timeout waiting for micro-MMU halt\n");

	return ret;
}

static void qcom_iommu_unhalt(struct qcom_iommu_dev *qcom_iommu)
{
	void __iomem *reg;

	if (!qcom_iommu->cfg || !qcom_iommu->cfg->halt)
		return;

	reg = qcom_iommu->global_base + QCOM_IOMMU_MICRO_MMU_CTRL;

	writel_relaxed(readl_relaxed(reg) & ~MICRO_MMU_CTRL_HALT_REQ, reg);
}

static void qcom_iommu_write_ctx(struct qcom_iommu_dev *qcom_iommu,
				struct qcom_iommu_ctx *ctx,
				const struct qcom_iommu_ctx_state *state)
{
	/* Disable context bank before programming */
	iommu_writel(ctx, ARM_SMMU_CB_SCTLR, 0);

	/* Clear context bank fault address fault status registers */
	iommu_writel(ctx, ARM_SMMU_CB_FAR, 0);
	iommu_writel(ctx, ARM_SMMU_CB_FSR, ARM_SMMU_CB_FSR_FAULT);

	if (qcom_iommu->cfg)
		iommu_writel(ctx, ARM_SMMU_CB_ACTLR, QCOM_IOMMU_ACTLR_BPRC);

	/* TTBRs */
	iommu_writeq(ctx, ARM_SMMU_CB_TTBR0, state->ttbr0);
	iommu_writeq(ctx, ARM_SMMU_CB_TTBR1, 0);

	/* TCR; the v7s ASID lives in CONTEXTIDR instead of TTBR0 */
	if (qcom_iommu_pgtbl_fmt(qcom_iommu) == ARM_V7S)
		iommu_writel(ctx, ARM_SMMU_CB_CONTEXTIDR, state->contextidr);
	else
		iommu_writel(ctx, ARM_SMMU_CB_TCR2, state->tcr[1]);
	iommu_writel(ctx, ARM_SMMU_CB_TCR, state->tcr[0]);

	/* MAIRs (stage-1 only) */
	iommu_writel(ctx, ARM_SMMU_CB_S1_MAIR0, state->mair[0]);
	iommu_writel(ctx, ARM_SMMU_CB_S1_MAIR1, state->mair[1]);

	/* SCTLR */
	iommu_writel(ctx, ARM_SMMU_CB_SCTLR, state->sctlr);
}

static void qcom_iommu_program_ctx(struct qcom_iommu_dev *qcom_iommu,
				  struct qcom_iommu_ctx *ctx)
{
	qcom_iommu_halt(qcom_iommu);
	qcom_iommu_write_ctx(qcom_iommu, ctx, &ctx->state);

	qcom_iommu_unhalt(qcom_iommu);
}

/*
 * Venus uses one Linux-owned context bank. Flush it even if the old
 * domain was freed while clocks were off. The device link keeps the
 * client inactive until both this operation and runtime resume succeed.
 */
static int qcom_iommu_rearm_venus_ctx(struct qcom_iommu_dev *iommu,
				     struct qcom_iommu_ctx *ctx,
				     const struct qcom_iommu_ctx_state *state)
{
	int ret;

	ret = qcom_iommu_halt(iommu);
	if (ret)
		goto unhalt;

	iommu_writel(ctx, ARM_SMMU_CB_SCTLR, 0);
	iommu_writel(ctx, ARM_SMMU_CB_S1_TLBIASID, ctx->asid);
	ret = qcom_iommu_ctx_tlb_sync(ctx);
	if (!ret && state)
		qcom_iommu_write_ctx(iommu, ctx, state);

unhalt:
	qcom_iommu_unhalt(iommu);
	return ret;
}

static int qcom_iommu_init_domain(struct iommu_domain *domain,
				  struct qcom_iommu_dev *qcom_iommu,
				  struct device *dev)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct io_pgtable_ops *pgtbl_ops;
	struct io_pgtable_cfg pgtbl_cfg;
	enum io_pgtable_fmt fmt;
	unsigned long flags = 0;
	int i, ret = 0;
	u32 reg;

	mutex_lock(&qcom_domain->init_mutex);
	if (qcom_domain->iommu)
		goto out_unlock;

	fmt = qcom_iommu_pgtbl_fmt(qcom_iommu);

	pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= domain->pgsize_bitmap,
		.ias		= 32,
		.oas		= fmt == ARM_V7S ? 32 : 40,
		.tlb		= &qcom_flush_ops,
		.iommu_dev	= qcom_iommu->dev,
	};

	qcom_domain->asids = kmemdup_array(fwspec->ids, fwspec->num_ids,
					 sizeof(*fwspec->ids), GFP_KERNEL);
	if (!qcom_domain->asids) {
		ret = -ENOMEM;
		goto out_unlock;
	}
	qcom_domain->num_asids = fwspec->num_ids;
	qcom_domain->iommu = qcom_iommu;

	pgtbl_ops = alloc_io_pgtable_ops(fmt, &pgtbl_cfg, qcom_domain);
	if (!pgtbl_ops) {
		dev_err(qcom_iommu->dev, "failed to allocate pagetable ops\n");
		ret = -ENOMEM;
		goto out_clear_iommu;
	}

	domain->geometry.aperture_end = (1ULL << pgtbl_cfg.ias) - 1;
	domain->geometry.force_aperture = true;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, fwspec->ids[i]);

		if (!qcom_iommu->non_secure && !ctx->secure_init) {
			ret = qcom_scm_restore_sec_cfg(qcom_iommu->sec_id, ctx->asid);
			if (ret) {
				dev_err(qcom_iommu->dev, "secure init failed: %d\n", ret);
				goto out_clear_iommu;
			}
			ctx->secure_init = true;
		}
	}

	/* Finish all fallible secure setup before programming any context bank. */
	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, fwspec->ids[i]);
		struct qcom_iommu_ctx_state state = {};

		/* Secured QSMMU-500/QSMMU-v2 contexts cannot be programmed */
		if (ctx->secured_ctx) {
			ctx->domain = domain;
			continue;
		}

		if (fmt == ARM_V7S) {
			state.ttbr0 = pgtbl_cfg.arm_v7s_cfg.ttbr;
			state.tcr[0] = pgtbl_cfg.arm_v7s_cfg.tcr;
			/* PRRR/NMRR share the MAIR0/MAIR1 offsets */
			state.mair[0] = pgtbl_cfg.arm_v7s_cfg.prrr;
			state.mair[1] = pgtbl_cfg.arm_v7s_cfg.nmrr;
			state.contextidr = ctx->asid;
		} else {
			state.ttbr0 = pgtbl_cfg.arm_lpae_s1_cfg.ttbr |
				      FIELD_PREP(ARM_SMMU_TTBRn_ASID, ctx->asid);
			state.tcr[0] = arm_smmu_lpae_tcr(&pgtbl_cfg) | ARM_SMMU_TCR_EAE;
			state.tcr[1] = arm_smmu_lpae_tcr2(&pgtbl_cfg);
			state.mair[0] = pgtbl_cfg.arm_lpae_s1_cfg.mair;
			state.mair[1] = pgtbl_cfg.arm_lpae_s1_cfg.mair >> 32;
		}

		reg = ARM_SMMU_SCTLR_CFIE | ARM_SMMU_SCTLR_CFRE |
		      ARM_SMMU_SCTLR_AFE | ARM_SMMU_SCTLR_TRE |
		      ARM_SMMU_SCTLR_M | ARM_SMMU_SCTLR_S1_ASIDPNE |
		      ARM_SMMU_SCTLR_CFCFG;

		if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
			reg |= ARM_SMMU_SCTLR_E;

		if (qcom_iommu->cfg && qcom_iommu->cfg->no_afe)
			reg &= ~ARM_SMMU_SCTLR_AFE;

		if (qcom_iommu->cfg && qcom_iommu->cfg->no_stall)
			reg &= ~ARM_SMMU_SCTLR_CFCFG;

		state.sctlr = reg;

		if (qcom_iommu->wrapper_clk_sel) {
			if (!qcom_iommu_tlb_lock(qcom_iommu, &flags)) {
				ret = -EIO;
				goto out_clear_iommu;
			}
			ret = qcom_iommu_rearm_venus_ctx(qcom_iommu, ctx, &state);
			if (ret) {
				qcom_iommu_tlb_unlock(qcom_iommu, flags);
				goto out_clear_iommu;
			}
			ctx->state = state;
		} else {
			ctx->state = state;
			qcom_iommu_program_ctx(qcom_iommu, ctx);
		}
		WRITE_ONCE(ctx->domain, domain);
		qcom_iommu_tlb_unlock(qcom_iommu, flags);
	}

	/* Publish page table ops for map/unmap */
	qcom_domain->pgtbl_ops = pgtbl_ops;

	mutex_unlock(&qcom_domain->init_mutex);

	return 0;
out_clear_iommu:
	free_io_pgtable_ops(pgtbl_ops);
	qcom_domain->iommu = NULL;
	kfree(qcom_domain->asids);
	qcom_domain->asids = NULL;
	qcom_domain->num_asids = 0;
out_unlock:
	mutex_unlock(&qcom_domain->init_mutex);
	return ret;
}

static struct iommu_domain *qcom_iommu_domain_alloc_paging(struct device *dev)
{
	struct qcom_iommu_domain *qcom_domain;

	/*
	 * Allocate the domain and initialise some of its data structures.
	 * We can't really do anything meaningful until we've added a
	 * master.
	 */
	qcom_domain = kzalloc(sizeof(*qcom_domain), GFP_KERNEL);
	if (!qcom_domain)
		return NULL;

	mutex_init(&qcom_domain->init_mutex);
	spin_lock_init(&qcom_domain->pgtbl_lock);
	qcom_domain->domain.pgsize_bitmap = SZ_4K;

	return &qcom_domain->domain;
}

static void qcom_iommu_venus_forget_domain(struct qcom_iommu_domain *domain)
{
	struct qcom_iommu_dev *iommu = domain->iommu;
	struct qcom_iommu_ctx *ctx = iommu->ctxs[0];
	unsigned long flags;

	spin_lock_irqsave(&iommu->ctx_lock, flags);
	if (ctx->domain == &domain->domain) {
		if (iommu->ctx_online) {
			iommu_writel(ctx, ARM_SMMU_CB_SCTLR, 0);
			/* Complete the disable before releasing the page table. */
			iommu_readl(ctx, ARM_SMMU_CB_SCTLR);
		}
		WRITE_ONCE(ctx->domain, NULL);
	}
	spin_unlock_irqrestore(&iommu->ctx_lock, flags);

	/* A handler that read the old owner may still be reporting a fault. */
	synchronize_irq(ctx->irq);
}

static void qcom_iommu_domain_free(struct iommu_domain *domain)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct qcom_iommu_dev *iommu = qcom_domain->iommu;

	if (iommu && iommu->wrapper_clk_sel) {
		qcom_iommu_venus_forget_domain(qcom_domain);
		free_io_pgtable_ops(qcom_domain->pgtbl_ops);
	} else if (iommu) {
		/*
		 * NOTE: unmap can be called after client device is powered
		 * off, for example, with GPUs or anything involving dma-buf.
		 * So we cannot rely on the device_link.  Make sure the IOMMU
		 * is on to avoid unclocked accesses in the TLB inv path:
		 */
		pm_runtime_get_sync(iommu->dev);
		free_io_pgtable_ops(qcom_domain->pgtbl_ops);
		pm_runtime_put_sync(iommu->dev);
	}

	kfree(qcom_domain->asids);
	kfree(qcom_domain);
}

static int qcom_iommu_attach_dev(struct iommu_domain *domain,
				 struct device *dev, struct iommu_domain *old)
{
	struct qcom_iommu_dev *qcom_iommu = dev_iommu_priv_get(dev);
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	int ret;

	if (!qcom_iommu) {
		dev_err(dev, "cannot attach to IOMMU, is it on the same bus?\n");
		return -ENXIO;
	}

	/* Ensure that the domain is finalized */
	ret = pm_runtime_resume_and_get(qcom_iommu->dev);
	if (ret < 0)
		return ret;

	ret = qcom_iommu_init_domain(domain, qcom_iommu, dev);
	pm_runtime_put_sync(qcom_iommu->dev);
	if (ret < 0)
		return ret;

	/*
	 * Sanity check the domain. We don't support domains across
	 * different IOMMUs.
	 */
	if (qcom_domain->iommu != qcom_iommu)
		return -EINVAL;

	return 0;
}

static int qcom_iommu_identity_attach(struct iommu_domain *identity_domain,
				      struct device *dev,
				      struct iommu_domain *old)
{
	struct qcom_iommu_domain *qcom_domain;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct qcom_iommu_dev *qcom_iommu = dev_iommu_priv_get(dev);
	unsigned long flags = 0;
	unsigned int i;
	int ret;

	if (old == identity_domain || !old)
		return 0;

	qcom_domain = to_qcom_iommu_domain(old);
	if (WARN_ON(!qcom_domain->iommu))
		return -EINVAL;

	ret = pm_runtime_resume_and_get(qcom_iommu->dev);
	if (ret < 0)
		return ret;

	if (!qcom_iommu_tlb_lock(qcom_iommu, &flags)) {
		ret = -EIO;
		goto out_put;
	}

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain, fwspec->ids[i]);

		/* Disable the context bank: */
		iommu_writel(ctx, ARM_SMMU_CB_SCTLR, 0);

		WRITE_ONCE(ctx->domain, NULL);
	}
	qcom_iommu_tlb_unlock(qcom_iommu, flags);
out_put:
	pm_runtime_put_sync(qcom_iommu->dev);
	return ret;
}

static struct iommu_domain_ops qcom_iommu_identity_ops = {
	.attach_dev = qcom_iommu_identity_attach,
};

static struct iommu_domain qcom_iommu_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &qcom_iommu_identity_ops,
};

static int qcom_iommu_map(struct iommu_domain *domain, unsigned long iova,
			  phys_addr_t paddr, size_t pgsize, size_t pgcount,
			  int prot, gfp_t gfp, size_t *mapped)
{
	int ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;

	if (!ops)
		return -ENODEV;

	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->map_pages(ops, iova, paddr, pgsize, pgcount, prot, GFP_ATOMIC, mapped);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);
	return ret;
}

static size_t qcom_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
			       size_t pgsize, size_t pgcount,
			       struct iommu_iotlb_gather *gather)
{
	size_t ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;
	struct qcom_iommu_dev *iommu = qcom_domain->iommu;

	if (!ops)
		return 0;

	/*
	 * Unmap can run after the consumer powers down. Venus removes the
	 * PTEs even when inactive and flushes the whole bank on resume.
	 * The other instances still need to power up for invalidation.
	 */
	if (!iommu->wrapper_clk_sel)
		pm_runtime_get_sync(iommu->dev);

	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->unmap_pages(ops, iova, pgsize, pgcount, gather);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);

	if (!iommu->wrapper_clk_sel)
		pm_runtime_put_sync(iommu->dev);

	return ret;
}

static void qcom_iommu_sync_domain(struct iommu_domain *domain, bool invalidate)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct qcom_iommu_dev *iommu = qcom_domain->iommu;

	if (!qcom_domain->pgtbl_ops)
		return;

	if (!iommu->wrapper_clk_sel)
		pm_runtime_get_sync(iommu->dev);

	if (invalidate)
		qcom_iommu_tlb_inv_context(qcom_domain);
	else
		qcom_iommu_tlb_sync_online(qcom_domain);

	if (!iommu->wrapper_clk_sel)
		pm_runtime_put_sync(iommu->dev);
}

static void qcom_iommu_flush_iotlb_all(struct iommu_domain *domain)
{
	qcom_iommu_sync_domain(domain, true);
}

static void qcom_iommu_iotlb_sync(struct iommu_domain *domain,
				  struct iommu_iotlb_gather *gather)
{
	qcom_iommu_sync_domain(domain, false);
}

static phys_addr_t qcom_iommu_iova_to_phys(struct iommu_domain *domain,
					   dma_addr_t iova)
{
	phys_addr_t ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;

	if (!ops)
		return 0;

	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->iova_to_phys(ops, iova);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);

	return ret;
}

static bool qcom_iommu_capable(struct device *dev, enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		/*
		 * Return true here as the SMMU can always send out coherent
		 * requests.
		 */
		return true;
	case IOMMU_CAP_NOEXEC:
		return true;
	default:
		return false;
	}
}

static struct iommu_device *qcom_iommu_probe_device(struct device *dev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_iommu_priv_get(dev);
	struct device_link *link;

	if (!qcom_iommu)
		return ERR_PTR(-ENODEV);

	/*
	 * Establish the link between iommu and master, so that the
	 * iommu gets runtime enabled/disabled as per the master's
	 * needs.
	 */
	link = device_link_add(dev, qcom_iommu->dev, DL_FLAG_PM_RUNTIME);
	if (!link) {
		dev_err(qcom_iommu->dev, "Unable to create device link between %s and %s\n",
			dev_name(qcom_iommu->dev), dev_name(dev));
		return ERR_PTR(-ENODEV);
	}

	return &qcom_iommu->iommu;
}

static int qcom_iommu_of_xlate(struct device *dev,
			       const struct of_phandle_args *args)
{
	struct qcom_iommu_dev *qcom_iommu;
	struct platform_device *iommu_pdev;
	unsigned asid = args->args[0];

	if (args->args_count != 1) {
		dev_err(dev, "incorrect number of iommu params found for %s "
			"(found %d, expected 1)\n",
			args->np->full_name, args->args_count);
		return -EINVAL;
	}

	iommu_pdev = of_find_device_by_node(args->np);
	if (WARN_ON(!iommu_pdev))
		return -EINVAL;

	qcom_iommu = platform_get_drvdata(iommu_pdev);

	put_device(&iommu_pdev->dev);

	/* make sure the asid specified in dt is valid, so we don't have
	 * to sanity check this elsewhere:
	 */
	if (WARN_ON(asid > qcom_iommu->max_asid) ||
	    WARN_ON(qcom_iommu->ctxs[asid] == NULL))
		return -EINVAL;

	if (!dev_iommu_priv_get(dev)) {
		dev_iommu_priv_set(dev, qcom_iommu);
	} else {
		/* make sure devices iommus dt node isn't referring to
		 * multiple different iommu devices.  Multiple context
		 * banks are ok, but multiple devices are not:
		 */
		if (WARN_ON(qcom_iommu != dev_iommu_priv_get(dev)))
			return -EINVAL;
	}

	return iommu_fwspec_add_ids(dev, &asid, 1);
}

static const struct iommu_ops qcom_iommu_ops = {
	.identity_domain = &qcom_iommu_identity_domain,
	.capable	= qcom_iommu_capable,
	.domain_alloc_paging = qcom_iommu_domain_alloc_paging,
	.probe_device	= qcom_iommu_probe_device,
	.device_group	= generic_device_group,
	.of_xlate	= qcom_iommu_of_xlate,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev	= qcom_iommu_attach_dev,
		.map_pages	= qcom_iommu_map,
		.unmap_pages	= qcom_iommu_unmap,
		.flush_iotlb_all = qcom_iommu_flush_iotlb_all,
		.iotlb_sync	= qcom_iommu_iotlb_sync,
		.iova_to_phys	= qcom_iommu_iova_to_phys,
		.free		= qcom_iommu_domain_free,
	}
};

static int qcom_iommu_sec_ptbl_init(struct device *dev)
{
	size_t psize = 0;
	unsigned int spare = 0;
	void *cpu_addr;
	dma_addr_t paddr;
	unsigned long attrs;
	static bool allocated = false;
	int ret;

	if (allocated)
		return 0;

	ret = qcom_scm_iommu_secure_ptbl_size(spare, &psize);
	if (ret) {
		dev_err(dev, "failed to get iommu secure pgtable size (%d)\n",
			ret);
		return ret;
	}

	dev_info(dev, "iommu sec: pgtable size: %zu\n", psize);

	attrs = DMA_ATTR_NO_KERNEL_MAPPING;

	cpu_addr = dma_alloc_attrs(dev, psize, &paddr, GFP_KERNEL, attrs);
	if (!cpu_addr) {
		dev_err(dev, "failed to allocate %zu bytes for pgtable\n",
			psize);
		return -ENOMEM;
	}

	ret = qcom_scm_iommu_secure_ptbl_init(paddr, psize, spare);
	if (ret) {
		dev_err(dev, "failed to init iommu pgtable (%d)\n", ret);
		goto free_mem;
	}

	allocated = true;
	return 0;

free_mem:
	dma_free_attrs(dev, psize, cpu_addr, paddr, attrs);
	return ret;
}

static int get_asid(const struct device_node *np)
{
	u32 reg, val;
	int asid;

	/* read the "reg" property directly to get the relative address
	 * of the context bank, and calculate the asid from that:
	 */
	if (of_property_read_u32_index(np, "reg", 0, &reg))
		return -ENODEV;

	/*
	 * Context banks are 0x1000 apart but, in some cases, the ASID
	 * number doesn't match to this logic and needs to be passed
	 * from the DT configuration explicitly.
	 */
	if (!of_property_read_u32(np, "qcom,ctx-asid", &val))
		asid = val;
	else
		asid = reg / 0x1000;

	return asid;
}

static int qcom_iommu_ctx_probe(struct platform_device *pdev)
{
	struct qcom_iommu_ctx *ctx;
	struct device *dev = &pdev->dev;
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev->parent);
	int ret, irq;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	platform_set_drvdata(pdev, ctx);

	ctx->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctx->base))
		return PTR_ERR(ctx->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ctx->irq = irq;

	if (of_device_is_compatible(dev->of_node, "qcom,msm-iommu-v2-sec"))
		ctx->secured_ctx = true;

	/* clear IRQs before registering fault handler, just in case the
	 * boot-loader left us a surprise.  Instances with a power domain
	 * may not be accessible yet; they are reset at first resume.
	 */
	if (!ctx->secured_ctx && !qcom_iommu->cfg) {
		ret = pm_runtime_resume_and_get(dev->parent);
		if (ret)
			return ret;
		iommu_writel(ctx, ARM_SMMU_CB_FSR, iommu_readl(ctx, ARM_SMMU_CB_FSR));
		pm_runtime_put_sync(dev->parent);
	}

	ret = devm_request_irq(dev, irq,
			       qcom_iommu_fault,
			       IRQF_SHARED,
			       "qcom-iommu-fault",
			       ctx);
	if (ret) {
		dev_err(dev, "failed to request IRQ %u\n", irq);
		return ret;
	}

	ret = get_asid(dev->of_node);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	ctx->asid = ret;

	dev_dbg(dev, "found asid %u\n", ctx->asid);

	qcom_iommu->ctxs[ctx->asid] = ctx;

	return 0;
}

static void qcom_iommu_ctx_remove(struct platform_device *pdev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(pdev->dev.parent);
	struct qcom_iommu_ctx *ctx = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	qcom_iommu->ctxs[ctx->asid] = NULL;
}

static const struct of_device_id ctx_of_match[] = {
	{ .compatible = "qcom,msm-iommu-v1-ns" },
	{ .compatible = "qcom,msm-iommu-v1-sec" },
	{ .compatible = "qcom,msm-iommu-v2-ns" },
	{ .compatible = "qcom,msm-iommu-v2-sec" },
	{ /* sentinel */ }
};

static struct platform_driver qcom_iommu_ctx_driver = {
	.driver	= {
		.name		= "qcom-iommu-ctx",
		.of_match_table	= ctx_of_match,
	},
	.probe	= qcom_iommu_ctx_probe,
	.remove = qcom_iommu_ctx_remove,
};

static bool qcom_iommu_has_secure_context(struct qcom_iommu_dev *qcom_iommu)
{
	struct device_node *child;

	for_each_child_of_node(qcom_iommu->dev->of_node, child) {
		if (of_device_is_compatible(child, "qcom,msm-iommu-v1-sec") ||
		    of_device_is_compatible(child, "qcom,msm-iommu-v2-sec")) {
			of_node_put(child);
			return true;
		}
	}

	return false;
}

static int qcom_iommu_device_probe(struct platform_device *pdev)
{
	struct device_node *child;
	struct qcom_iommu_dev *qcom_iommu;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct clk *clk;
	int ret, max_asid = 0;

	/* find the max asid (which is 1:1 to ctx bank idx), so we know how
	 * many child ctx devices we have:
	 */
	for_each_child_of_node(dev->of_node, child)
		max_asid = max(max_asid, get_asid(child));

	qcom_iommu = devm_kzalloc(dev, struct_size(qcom_iommu, ctxs, max_asid + 1),
				  GFP_KERNEL);
	if (!qcom_iommu)
		return -ENOMEM;
	qcom_iommu->max_asid = max_asid;
	qcom_iommu->dev = dev;
	qcom_iommu->cfg = of_device_get_match_data(dev);
	spin_lock_init(&qcom_iommu->ctx_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		if (qcom_iommu->cfg) {
			qcom_iommu->global_base = devm_ioremap_resource(dev, res);
			if (IS_ERR(qcom_iommu->global_base))
				return PTR_ERR(qcom_iommu->global_base);
		} else {
			qcom_iommu->local_base = devm_ioremap_resource(dev, res);
			if (IS_ERR(qcom_iommu->local_base))
				return PTR_ERR(qcom_iommu->local_base);
		}
	} else if (qcom_iommu->cfg) {
		return dev_err_probe(dev, -EINVAL,
				     "missing global register space\n");
	}

	if (qcom_iommu->cfg && qcom_iommu->cfg->has_wrapper_clk_sel) {
		if (max_asid || of_get_available_child_count(dev->of_node) != 1)
			return dev_err_probe(dev, -EINVAL, "Venus requires context bank 0\n");

		for_each_available_child_of_node_scoped(dev->of_node, child) {
			if (!of_device_is_compatible(child, "qcom,msm-iommu-v1-ns") ||
			    get_asid(child))
				return dev_err_probe(dev, -EINVAL, "invalid Venus context bank\n");
		}

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "clock-select");
		if (!res || resource_size(res) != sizeof(u32))
			return dev_err_probe(dev, -EINVAL, "invalid wrapper clock register\n");
		qcom_iommu->wrapper_clk_sel = devm_ioremap_resource(dev, res);
		if (IS_ERR(qcom_iommu->wrapper_clk_sel))
			return PTR_ERR(qcom_iommu->wrapper_clk_sel);
	}

	clk = devm_clk_get(dev, "iface");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get iface clock\n");
		return PTR_ERR(clk);
	}
	qcom_iommu->clks[CLK_IFACE].clk = clk;

	clk = devm_clk_get(dev, "bus");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get bus clock\n");
		return PTR_ERR(clk);
	}
	qcom_iommu->clks[CLK_BUS].clk = clk;

	clk = devm_clk_get_optional(dev, "tbu");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get tbu clock\n");
		return PTR_ERR(clk);
	}
	if (!clk && qcom_iommu->wrapper_clk_sel)
		return dev_err_probe(dev, -EINVAL, "missing Venus TBU clock\n");
	qcom_iommu->clks[CLK_TBU].clk = clk;

	if (of_property_read_u32(dev->of_node, "qcom,iommu-secure-id",
				 &qcom_iommu->sec_id)) {
		if (!qcom_iommu->cfg || qcom_iommu->cfg->has_wrapper_clk_sel) {
			dev_err(dev, "missing qcom,iommu-secure-id property\n");
			return -ENODEV;
		}
		/* The secure world does not manage this instance at all */
		qcom_iommu->non_secure = true;
	}
	if (qcom_iommu->wrapper_clk_sel && qcom_iommu->sec_id)
		return dev_err_probe(dev, -EINVAL, "invalid secure ID for Venus IOMMU\n");

	if (qcom_iommu_has_secure_context(qcom_iommu)) {
		ret = qcom_iommu_sec_ptbl_init(dev);
		if (ret) {
			dev_err(dev, "cannot init secure pg table(%d)\n", ret);
			return ret;
		}
	}

	platform_set_drvdata(pdev, qcom_iommu);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/* register context bank devices, which are child nodes: */
	ret = devm_of_platform_populate(dev);
	if (ret) {
		dev_err(dev, "Failed to populate iommu contexts\n");
		return ret;
	}

	ret = iommu_device_sysfs_add(&qcom_iommu->iommu, dev, NULL,
				     dev_name(dev));
	if (ret) {
		dev_err(dev, "Failed to register iommu in sysfs\n");
		return ret;
	}

	ret = iommu_device_register(&qcom_iommu->iommu, &qcom_iommu_ops, dev);
	if (ret) {
		dev_err(dev, "Failed to register iommu\n");
		goto err_sysfs_remove;
	}

	if (qcom_iommu->local_base) {
		ret = pm_runtime_resume_and_get(dev);
		if (ret)
			goto err_iommu_unregister;
		writel_relaxed(0xffffffff, qcom_iommu->local_base + SMMU_INTR_SEL_NS);
		pm_runtime_put_sync(dev);
	}

	return 0;

err_iommu_unregister:
	iommu_device_unregister(&qcom_iommu->iommu);
err_sysfs_remove:
	iommu_device_sysfs_remove(&qcom_iommu->iommu);
	return ret;
}

static void qcom_iommu_device_remove(struct platform_device *pdev)
{
	struct qcom_iommu_dev *qcom_iommu = platform_get_drvdata(pdev);

	/* Retire clients while their context banks and parent PM are available. */
	iommu_device_unregister(&qcom_iommu->iommu);
	iommu_device_sysfs_remove(&qcom_iommu->iommu);
	devm_of_platform_depopulate(&pdev->dev);

	pm_runtime_force_suspend(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
}

static int qcom_iommu_select_wrapper_clk(struct qcom_iommu_dev *qcom_iommu)
{
	void __iomem *select = qcom_iommu->wrapper_clk_sel;
	u32 value;

	if (!select)
		return 0;

	value = readl(select);
	writel(value & ~QCOM_IOMMU_WRAPPER_CLK_SEL, select);
	if (readl(select) & QCOM_IOMMU_WRAPPER_CLK_SEL)
		return -EIO;

	return 0;
}

static int qcom_iommu_resume_venus(struct qcom_iommu_dev *iommu)
{
	struct qcom_iommu_ctx *ctx;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&iommu->ctx_lock, flags);
	ctx = iommu->ctxs[0];
	if (ctx)
		ret = qcom_iommu_rearm_venus_ctx(iommu, ctx, ctx->domain ? &ctx->state : NULL);
	if (!ret)
		iommu->ctx_online = true;
	spin_unlock_irqrestore(&iommu->ctx_lock, flags);

	return ret;
}

static int __maybe_unused qcom_iommu_resume(struct device *dev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev);
	unsigned int i;
	int ret;

	ret = clk_bulk_prepare_enable(CLK_NUM, qcom_iommu->clks);
	if (ret < 0)
		goto out_error;

	ret = qcom_iommu_select_wrapper_clk(qcom_iommu);
	if (ret)
		goto err_disable_clks;

	if (qcom_iommu->non_secure) {
		ret = qcom_iommu_reset_ns(qcom_iommu);
		if (ret)
			goto err_disable_clks;
	} else if (dev->pm_domain) {
		ret = qcom_scm_restore_sec_cfg(qcom_iommu->sec_id, 0);
		if (ret)
			goto err_disable_clks;
	}

	qcom_iommu_bfb_setup(qcom_iommu);

	if (qcom_iommu->wrapper_clk_sel) {
		ret = qcom_iommu_resume_venus(qcom_iommu);
		if (ret)
			goto err_disable_clks;
	} else if (qcom_iommu->cfg && qcom_iommu->cfg->ctx_restore) {
		/* Restore context banks lost over power collapse */
		for (i = 0; i <= qcom_iommu->max_asid; i++) {
			struct qcom_iommu_ctx *ctx = qcom_iommu->ctxs[i];

			if (ctx && ctx->domain && !ctx->secured_ctx)
				qcom_iommu_program_ctx(qcom_iommu, ctx);
		}
	}

	return 0;

err_disable_clks:
	clk_bulk_disable_unprepare(CLK_NUM, qcom_iommu->clks);
out_error:
	/* Device links reserve -EACCES for a supplier with PM disabled. */
	if (qcom_iommu->wrapper_clk_sel && ret == -EACCES)
		ret = -EIO;
	return ret;
}

static int __maybe_unused qcom_iommu_suspend(struct device *dev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev);
	struct qcom_iommu_ctx *ctx;
	unsigned long flags;

	if (qcom_iommu->wrapper_clk_sel) {
		spin_lock_irqsave(&qcom_iommu->ctx_lock, flags);
		ctx = qcom_iommu->ctxs[0];
		if (ctx && qcom_iommu->ctx_online) {
			/*
			 * The device link orders this after the Venus engine
			 * quiesces. Disable the context before offline page
			 * table cleanup. Resume invalidates the ASID before
			 * restoring the context and releasing the consumer.
			 */
			iommu_writel(ctx, ARM_SMMU_CB_SCTLR, 0);
			iommu_readl(ctx, ARM_SMMU_CB_SCTLR);
		}
		qcom_iommu->ctx_online = false;
		spin_unlock_irqrestore(&qcom_iommu->ctx_lock, flags);
	}

	clk_bulk_disable_unprepare(CLK_NUM, qcom_iommu->clks);

	return 0;
}

static const struct dev_pm_ops qcom_iommu_pm_ops = {
	SET_RUNTIME_PM_OPS(qcom_iommu_suspend, qcom_iommu_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct qcom_iommu_bfb_reg msm8974_gpu_bfb[] = {
	{ 0x04c, 0x00000003 },
	{ 0x050, 0x00000000 },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000010 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00000000 },
	{ 0x15c, 0x00000000 },
	{ 0x20c, 0x00000020 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000001 },
	{ 0x414, 0x00000081 },
	{ 0x008, 0x00000000 },
};

static const struct qcom_iommu_sid msm8974_gpu_sids[] = {
	{ .cbndx = 0, .sid = 0 },	/* GFX3D_USER */
	{ .cbndx = 1, .sid = 1 },	/* GFX3D_PRIV */
};

static const struct qcom_iommu_cfg msm8974_gpu_cfg = {
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8974_gpu_bfb,
	.num_bfb = ARRAY_SIZE(msm8974_gpu_bfb),
	.sids = msm8974_gpu_sids,
	.num_sids = ARRAY_SIZE(msm8974_gpu_sids),
};

/* Values from the MSM8226 vendor kgsl_iommu description. */
static const struct qcom_iommu_bfb_reg msm8226_gpu_bfb[] = {
	{ 0x04c, 0x00000003 },
	{ 0x050, 0x00000000 },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000010 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00000000 },
	{ 0x15c, 0x00000001 },
	{ 0x20c, 0x00000011 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000001 },
	{ 0x414, 0x00000041 },
	{ 0x008, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8226_gpu_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8226_gpu_bfb,
	.num_bfb = ARRAY_SIZE(msm8226_gpu_bfb),
	.sids = msm8974_gpu_sids,
	.num_sids = ARRAY_SIZE(msm8974_gpu_sids),
};

/* Values from the MSM8226 vendor vfe_iommu description. */
static const struct qcom_iommu_bfb_reg msm8226_vfe_bfb[] = {
	{ 0x04c, 0xffffffff },
	{ 0x050, 0x00000000 },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000008 },
	{ 0x56c, 0x00000000 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x0000001b },
	{ 0x414, 0x0000005b },
	{ 0x0ac, 0x00000000 },
	{ 0x15c, 0x0000001b },
	{ 0x20c, 0x0000002b },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
	{ 0x018, 0x00000000 },
	{ 0x01c, 0x00000000 },
	{ 0x020, 0x00000000 },
};

/* Stream IDs from the vendor vfe_iommu context descriptions. */
static const struct qcom_iommu_sid msm8226_vfe_sids[] = {
	{ .cbndx = 0, .sid = 0 },	/* VFE0 */
	{ .cbndx = 1, .sid = 1 },	/* VFE1 */
	{ .cbndx = 2, .sid = 2 },	/* CPP */
};

static const struct qcom_iommu_cfg msm8226_vfe_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8226_vfe_bfb,
	.num_bfb = ARRAY_SIZE(msm8226_vfe_bfb),
	.sids = msm8226_vfe_sids,
	.num_sids = ARRAY_SIZE(msm8226_vfe_sids),
};

static const struct qcom_iommu_bfb_reg msm8226_mdp_bfb[] = {
	{ 0x04c, 0xffffffff },
	{ 0x050, 0x00000000 },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000010 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00000000 },
	{ 0x15c, 0x00000013 },
	{ 0x20c, 0x00000017 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000013 },
	{ 0x414, 0x00000023 },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
	{ 0x018, 0x00000000 },
	{ 0x01c, 0x00000000 },
	{ 0x020, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8226_mdp_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8226_mdp_bfb,
	.num_bfb = ARRAY_SIZE(msm8226_mdp_bfb),
};

static const struct qcom_iommu_bfb_reg msm8974_mdp_bfb[] = {
	{ 0x04c, 0xffffffff },
	{ 0x050, 0x00000000 },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000010 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00006800 },
	{ 0x15c, 0x00006221 },
	{ 0x20c, 0x00016231 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000034 },
	{ 0x414, 0x00000074 },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
	{ 0x018, 0x00000000 },
	{ 0x01c, 0x00000000 },
	{ 0x020, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8974_mdp_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8974_mdp_bfb,
	.num_bfb = ARRAY_SIZE(msm8974_mdp_bfb),
};

static const struct qcom_iommu_bfb_reg msm8974_venus_bfb[] = {
	{ 0x04c, 0xffffffff },
	{ 0x050, 0xffffffff },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000008 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00013205 },
	{ 0x15c, 0x00004000 },
	{ 0x20c, 0x00014020 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000094 },
	{ 0x414, 0x00000114 },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
	{ 0x018, 0x00000000 },
	{ 0x01c, 0x00000000 },
	{ 0x020, 0x00000000 },
	{ 0x024, 0x00000000 },
	{ 0x028, 0x00000000 },
	{ 0x02c, 0x00000000 },
	{ 0x030, 0x00000000 },
	{ 0x034, 0x00000000 },
	{ 0x038, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8974_venus_cfg = {
	.halt = true,
	.no_stall = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8974_venus_bfb,
	.num_bfb = ARRAY_SIZE(msm8974_venus_bfb),
};

/* Qualcomm MSM8226 downstream Venus bandwidth-fetch programming. */
static const struct qcom_iommu_bfb_reg msm8226_venus_bfb[] = {
	{ 0x04c, 0xffffffff },
	{ 0x050, 0xffffffff },
	{ 0x514, 0x00000004 },
	{ 0x540, 0x00000008 },
	{ 0x56c, 0x00000000 },
	{ 0x0ac, 0x00000000 },
	{ 0x15c, 0x00000094 },
	{ 0x20c, 0x000000b4 },
	{ 0x314, 0x00000000 },
	{ 0x394, 0x00000094 },
	{ 0x414, 0x00000114 },
	{ 0x008, 0x00000000 },
	{ 0x00c, 0x00000000 },
	{ 0x010, 0x00000000 },
	{ 0x014, 0x00000000 },
	{ 0x018, 0x00000000 },
	{ 0x01c, 0x00000000 },
	{ 0x020, 0x00000000 },
	{ 0x024, 0x00000000 },
	{ 0x028, 0x00000000 },
	{ 0x02c, 0x00000000 },
	{ 0x030, 0x00000000 },
	{ 0x034, 0x00000000 },
	{ 0x038, 0x00000000 },
};

static const struct qcom_iommu_cfg msm8226_venus_cfg = {
	.halt = true,
	.no_stall = true,
	.has_wrapper_clk_sel = true,
	.fmt = ARM_V7S,
	.no_afe = true,
	.ctx_restore = true,
	.bfb = msm8226_venus_bfb,
	.num_bfb = ARRAY_SIZE(msm8226_venus_bfb),
};

static const struct of_device_id qcom_iommu_of_match[] = {
	{ .compatible = "qcom,msm-iommu-v1" },
	{ .compatible = "qcom,msm-iommu-v2" },
	{ .compatible = "qcom,msm8974-gpu-iommu", .data = &msm8974_gpu_cfg },
	{ .compatible = "qcom,msm8226-gpu-iommu", .data = &msm8226_gpu_cfg },
	{ .compatible = "qcom,msm8226-mdp-iommu", .data = &msm8226_mdp_cfg },
	{ .compatible = "qcom,msm8226-venus-iommu", .data = &msm8226_venus_cfg },
	{ .compatible = "qcom,msm8226-vfe-iommu", .data = &msm8226_vfe_cfg },
	{ .compatible = "qcom,msm8974-mdp-iommu", .data = &msm8974_mdp_cfg },
	{ .compatible = "qcom,msm8974-venus-iommu", .data = &msm8974_venus_cfg },
	{ /* sentinel */ }
};

static struct platform_driver qcom_iommu_driver = {
	.driver	= {
		.name		= "qcom-iommu",
		.of_match_table	= qcom_iommu_of_match,
		.pm		= &qcom_iommu_pm_ops,
	},
	.probe	= qcom_iommu_device_probe,
	.remove = qcom_iommu_device_remove,
};

static int __init qcom_iommu_init(void)
{
	int ret;

	ret = platform_driver_register(&qcom_iommu_ctx_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&qcom_iommu_driver);
	if (ret)
		platform_driver_unregister(&qcom_iommu_ctx_driver);

	return ret;
}
device_initcall(qcom_iommu_init);
