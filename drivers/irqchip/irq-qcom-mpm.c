// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Linaro Limited
 * Copyright (c) 2010-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/irqdesc.h>
#include <linux/rcupdate.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/soc/qcom/mpm.h>
#include <linux/soc/qcom/irq.h>
#include <linux/spinlock.h>

/*
 * This is the driver for Qualcomm MPM (MSM Power Manager) interrupt controller,
 * which is commonly found on Qualcomm SoCs built on the RPM architecture.
 * Sitting in always-on domain, MPM monitors the wakeup interrupts when SoC is
 * asleep, and wakes up the AP when one of those interrupts occurs.  This driver
 * doesn't directly access physical MPM registers though.  Instead, the access
 * is bridged via a piece of internal memory (SRAM) that is accessible to both
 * AP and RPM.  This piece of memory is called 'vMPM' in the driver.
 *
 * When SoC is awake, the vMPM is owned by AP and the register setup by this
 * driver all happens on vMPM.  When AP is about to get power collapsed, the
 * driver sends a mailbox notification to RPM, which will take over the vMPM
 * ownership and dump vMPM into physical MPM registers.  On wakeup, AP is woken
 * up by a MPM pin/interrupt, and RPM will copy STATUS registers into vMPM.
 * Then AP start owning vMPM again.
 *
 * vMPM register map:
 *
 *    31                              0
 *    +--------------------------------+
 *    |            TIMER0              | 0x00
 *    +--------------------------------+
 *    |            TIMER1              | 0x04
 *    +--------------------------------+
 *    |            ENABLE0             | 0x08
 *    +--------------------------------+
 *    |              ...               | ...
 *    +--------------------------------+
 *    |            ENABLEn             |
 *    +--------------------------------+
 *    |          FALLING_EDGE0         |
 *    +--------------------------------+
 *    |              ...               |
 *    +--------------------------------+
 *    |            STATUSn             |
 *    +--------------------------------+
 *
 *    n = DIV_ROUND_UP(pin_cnt, 32)
 *
 */

#define MPM_REG_ENABLE		0
#define MPM_REG_FALLING_EDGE	1
#define MPM_REG_RISING_EDGE	2
#define MPM_REG_POLARITY	3
#define MPM_REG_STATUS		4

/* MPM pin map to GIC hwirq */
struct mpm_gic_map {
	int pin;
	irq_hw_number_t hwirq;
};

struct qcom_mpm_wake_domain {
	struct device_node *node;
	unsigned int pin;
};

struct qcom_mpm_priv {
	void __iomem *base;
	raw_spinlock_t lock;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
	struct mpm_gic_map *maps;
	unsigned int map_cnt;
	unsigned int reg_stride;
	struct irq_domain *domain;
	struct generic_pm_domain genpd;
	struct device *sleep_owner;
	u32 *sleep_image;
	unsigned int pin_count;
	bool sleep_prepared;
	int irq;
	struct qcom_mpm_wake_domain *wake_domains;
	unsigned int wake_domain_count;
};

static u32 qcom_mpm_read(struct qcom_mpm_priv *priv, unsigned int reg,
			 unsigned int index)
{
	unsigned int offset = (reg * priv->reg_stride + index + 2) * 4;

	return readl_relaxed(priv->base + offset);
}

static void qcom_mpm_write(struct qcom_mpm_priv *priv, unsigned int reg,
			   unsigned int index, u32 val)
{
	unsigned int offset = (reg * priv->reg_stride + index + 2) * 4;

	writel_relaxed(val, priv->base + offset);

	/* Ensure the write is completed */
	wmb();
}

static void qcom_mpm_enable_irq(struct irq_data *d, bool en)
{
	struct qcom_mpm_priv *priv = d->chip_data;
	int pin = d->hwirq;
	unsigned int index = pin / 32;
	unsigned int shift = pin % 32;
	unsigned long flags, val;

	raw_spin_lock_irqsave(&priv->lock, flags);

	val = qcom_mpm_read(priv, MPM_REG_ENABLE, index);
	__assign_bit(shift, &val, en);
	qcom_mpm_write(priv, MPM_REG_ENABLE, index, val);

	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static void qcom_mpm_mask(struct irq_data *d)
{
	qcom_mpm_enable_irq(d, false);

	if (d->parent_data)
		irq_chip_mask_parent(d);
}

static void qcom_mpm_unmask(struct irq_data *d)
{
	qcom_mpm_enable_irq(d, true);

	if (d->parent_data)
		irq_chip_unmask_parent(d);
}

static void mpm_set_type(struct qcom_mpm_priv *priv, bool set, unsigned int reg,
			 unsigned int index, unsigned int shift)
{
	unsigned long flags, val;

	raw_spin_lock_irqsave(&priv->lock, flags);

	val = qcom_mpm_read(priv, reg, index);
	__assign_bit(shift, &val, set);
	qcom_mpm_write(priv, reg, index, val);

	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static int qcom_mpm_set_type(struct irq_data *d, unsigned int type)
{
	struct qcom_mpm_priv *priv = d->chip_data;
	int pin = d->hwirq;
	unsigned int index = pin / 32;
	unsigned int shift = pin % 32;

	if (type & IRQ_TYPE_EDGE_RISING)
		mpm_set_type(priv, true, MPM_REG_RISING_EDGE, index, shift);
	else
		mpm_set_type(priv, false, MPM_REG_RISING_EDGE, index, shift);

	if (type & IRQ_TYPE_EDGE_FALLING)
		mpm_set_type(priv, true, MPM_REG_FALLING_EDGE, index, shift);
	else
		mpm_set_type(priv, false, MPM_REG_FALLING_EDGE, index, shift);

	if (type & IRQ_TYPE_LEVEL_HIGH)
		mpm_set_type(priv, true, MPM_REG_POLARITY, index, shift);
	else
		mpm_set_type(priv, false, MPM_REG_POLARITY, index, shift);

	if (!d->parent_data)
		return 0;

	if (type & IRQ_TYPE_EDGE_BOTH)
		type = IRQ_TYPE_EDGE_RISING;

	if (type & IRQ_TYPE_LEVEL_MASK)
		type = IRQ_TYPE_LEVEL_HIGH;

	return irq_chip_set_type_parent(d, type);
}

static struct irq_chip qcom_mpm_chip = {
	.name			= "mpm",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= qcom_mpm_mask,
	.irq_unmask		= qcom_mpm_unmask,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= qcom_mpm_set_type,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_MASK_ON_SUSPEND |
				  IRQCHIP_SKIP_SET_WAKE,
};

static struct mpm_gic_map *get_mpm_gic_map(struct qcom_mpm_priv *priv, int pin)
{
	struct mpm_gic_map *maps = priv->maps;
	int i;

	for (i = 0; i < priv->map_cnt; i++) {
		if (maps[i].pin == pin)
			return &maps[i];
	}

	return NULL;
}

static int qcom_mpm_alloc(struct irq_domain *domain, unsigned int virq,
			  unsigned int nr_irqs, void *data)
{
	struct qcom_mpm_priv *priv = domain->host_data;
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	struct mpm_gic_map *map;
	irq_hw_number_t pin;
	unsigned int type;
	int  ret;

	ret = irq_domain_translate_twocell(domain, fwspec, &pin, &type);
	if (ret)
		return ret;

	if (pin == GPIO_NO_WAKE_IRQ)
		return irq_domain_disconnect_hierarchy(domain, virq);

	ret = irq_domain_set_hwirq_and_chip(domain, virq, pin,
					    &qcom_mpm_chip, priv);
	if (ret)
		return ret;

	map = get_mpm_gic_map(priv, pin);
	if (map == NULL)
		return irq_domain_disconnect_hierarchy(domain->parent, virq);

	if (type & IRQ_TYPE_EDGE_BOTH)
		type = IRQ_TYPE_EDGE_RISING;

	if (type & IRQ_TYPE_LEVEL_MASK)
		type = IRQ_TYPE_LEVEL_HIGH;

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 3;
	parent_fwspec.param[0] = 0;
	parent_fwspec.param[1] = map->hwirq;
	parent_fwspec.param[2] = type;

	return irq_domain_alloc_irqs_parent(domain, virq, nr_irqs,
					    &parent_fwspec);
}

static const struct irq_domain_ops qcom_mpm_ops = {
	.alloc		= qcom_mpm_alloc,
	.free		= irq_domain_free_irqs_common,
	.translate	= irq_domain_translate_twocell,
};

/* Triggered by RPM when system resumes from deep sleep */
static irqreturn_t qcom_mpm_handler(int irq, void *dev_id)
{
	struct qcom_mpm_priv *priv = dev_id;
	unsigned long enable, pending;
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;
	int i, j;

	for (i = 0; i < priv->reg_stride; i++) {
		raw_spin_lock_irqsave(&priv->lock, flags);
		enable = qcom_mpm_read(priv, MPM_REG_ENABLE, i);
		pending = qcom_mpm_read(priv, MPM_REG_STATUS, i);
		pending &= enable;
		if (priv->sleep_owner)
			qcom_mpm_write(priv, MPM_REG_STATUS, i, 0);
		raw_spin_unlock_irqrestore(&priv->lock, flags);

		for_each_set_bit(j, &pending, 32) {
			unsigned int pin = 32 * i + j;
			struct irq_desc *desc = irq_resolve_mapping(priv->domain, pin);
			struct irq_data *d;

			ret = IRQ_HANDLED;
			if (!desc)
				continue;
			d = &desc->irq_data;

			if (!irqd_is_level_type(d))
				irq_set_irqchip_state(d->irq,
						IRQCHIP_STATE_PENDING, true);
			ret = IRQ_HANDLED;
		}
	}

	return ret;
}

/* The coordinator retains a supplier link and serializes with system sleep. */
int qcom_mpm_register_sleep(struct device *dev, struct device *owner)
{
	struct qcom_mpm_priv *priv = dev_get_drvdata(dev);

	if (!priv || !owner)
		return -EPROBE_DEFER;
	if (!priv->sleep_image)
		return -EOPNOTSUPP;
	if (cmpxchg(&priv->sleep_owner, NULL, owner))
		return -EBUSY;
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_mpm_register_sleep);

int qcom_mpm_unregister_sleep(struct device *dev, struct device *owner)
{
	struct qcom_mpm_priv *priv = dev_get_drvdata(dev);

	if (priv->sleep_prepared)
		return -EBUSY;
	return cmpxchg(&priv->sleep_owner, owner, NULL) == owner ? 0 : -ESTALE;
}
EXPORT_SYMBOL_GPL(qcom_mpm_unregister_sleep);

int qcom_mpm_prepare_sleep(struct device *dev, struct device *owner)
{
	struct qcom_mpm_priv *priv = dev_get_drvdata(dev);
	unsigned long flags, enabled;
	bool any = false;
	int i, j, reg, ret = 0;

	if (priv->sleep_owner != owner || priv->sleep_prepared)
		return -EINVAL;
	if (irqs_disabled())
		return -EPERM;
	raw_spin_lock_irqsave(&priv->lock, flags);
	for (reg = 0; reg < MPM_REG_STATUS; reg++)
		for (i = 0; i < priv->reg_stride; i++)
			priv->sleep_image[reg * priv->reg_stride + i] =
				qcom_mpm_read(priv, reg, i);
	for (i = 0; i < priv->reg_stride; i++) {
		enabled = priv->sleep_image[i];
		for_each_set_bit(j, &enabled, 32) {
			unsigned int pin = i * 32 + j;

			if (pin >= priv->pin_count || !irq_resolve_mapping(priv->domain, pin)) {
				ret = -EINVAL;
				goto unlock;
			}
			any = true;
		}
	}
	if (!any) {
		ret = -EINVAL;
		goto unlock;
	}
	/* System RAM sleep uses device/RTC wake, not a running Linux tick. */
	writel_relaxed(U32_MAX, priv->base);
	writel_relaxed(U32_MAX, priv->base + 4);
	for (i = 0; i < priv->reg_stride; i++)
		qcom_mpm_write(priv, MPM_REG_STATUS, i, 0);
	/* Publish timer and wake-mask state before notifying RPM. */
	wmb();
unlock:
	raw_spin_unlock_irqrestore(&priv->lock, flags);
	if (ret)
		return ret;

	/* Mailbox and regmap may use ordinary locks on RT: local IRQs stay on. */
	ret = mbox_send_message(priv->mbox_chan, NULL);
	if (ret < 0)
		return ret;
	mbox_client_txdone(priv->mbox_chan, 0);
	WRITE_ONCE(priv->sleep_prepared, true);
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_mpm_prepare_sleep);

static void qcom_mpm_put_node(void *node)
{
	of_node_put(node);
}

/* Non-hierarchical interrupt controllers need their physical parent recorded. */
static int qcom_mpm_init_wake_domains(struct device *dev, struct qcom_mpm_priv *priv)
{
	struct of_phandle_args parent;
	struct device_node *node;
	unsigned int count = 0;
	int ret;

	for_each_node_with_property(node, "interrupt-controller") {
		if (!of_device_is_available(node) || of_irq_count(node) != 1 ||
		    of_irq_parse_one(node, 0, &parent))
			continue;
		if (parent.np == dev->of_node && parent.args_count == 2 &&
		    parent.args[0] < priv->pin_count &&
		    get_mpm_gic_map(priv, parent.args[0]))
			count++;
		of_node_put(parent.np);
	}
	if (!count)
		return 0;
	priv->wake_domains = devm_kcalloc(dev, count, sizeof(*priv->wake_domains), GFP_KERNEL);
	if (!priv->wake_domains)
		return -ENOMEM;
	for_each_node_with_property(node, "interrupt-controller") {
		if (!of_device_is_available(node) || of_irq_count(node) != 1 ||
		    of_irq_parse_one(node, 0, &parent))
			continue;
		if (parent.np == dev->of_node && parent.args_count == 2 &&
		    parent.args[0] < priv->pin_count &&
		    get_mpm_gic_map(priv, parent.args[0])) {
			/* The target has a fixed DT; tolerate concurrent additions safely. */
			if (priv->wake_domain_count == count) {
				of_node_put(parent.np);
				of_node_put(node);
				return -EAGAIN;
			}
			of_node_get(node);
			ret = devm_add_action_or_reset(dev, qcom_mpm_put_node, node);
			if (ret) {
				of_node_put(parent.np);
				of_node_put(node);
				return ret;
			}
			priv->wake_domains[priv->wake_domain_count].node = node;
			priv->wake_domains[priv->wake_domain_count++].pin = parent.args[0];
		}
		of_node_put(parent.np);
	}
	return 0;
}

static bool qcom_mpm_pin_enabled(struct qcom_mpm_priv *priv, irq_hw_number_t pin)
{
	return pin < priv->pin_count && priv->sleep_image[pin / 32] & BIT(pin % 32);
}

static bool qcom_mpm_irq_covered(struct qcom_mpm_priv *priv, struct irq_data *data)
{
	struct irq_data *parent;
	unsigned int i;

	for (parent = data; parent; parent = parent->parent_data)
		if (parent->domain == priv->domain)
			return qcom_mpm_pin_enabled(priv, parent->hwirq) &&
			       (parent != data || get_mpm_gic_map(priv, parent->hwirq));
	if (!data->domain)
		return false;
	for (i = 0; i < priv->wake_domain_count; i++)
		if (data->domain->fwnode == of_fwnode_handle(priv->wake_domains[i].node))
			return qcom_mpm_pin_enabled(priv, priv->wake_domains[i].pin);
	return false;
}

static bool qcom_mpm_wake_covered(struct qcom_mpm_priv *priv)
{
	struct irq_desc *desc;
	struct irq_data *data;
	struct irqaction *action;
	unsigned int irq;
	bool covered = true, required;

	/* No other CPU or interrupt can change action/domain ownership at entry. */
	rcu_read_lock();
	for_each_irq_desc(irq, desc) {
		data = irq_desc_get_irq_data(desc);
		if (irq == priv->irq)
			continue;
		required = irqd_is_wakeup_armed(data);
		if (!required && !irqd_irq_disabled(data)) {
			/* CPU-local timers/IPIs are saved and stopped by CPU PM/syscore. */
			if (data->domain == priv->domain->parent && data->hwirq < 32)
				continue;
			for (action = desc->action; action; action = action->next)
				/*
				 * Clock event timers (IRQF_TIMER implies NO_SUSPEND) are shut
				 * down by tick/timekeeping suspend; system RAM sleep wakes on
				 * device and RTC interrupts, not on a Linux tick.
				 */
				if ((action->flags & IRQF_NO_SUSPEND) &&
				    !(action->flags & __IRQF_TIMER))
					required = true;
		}
		if (required && !qcom_mpm_irq_covered(priv, data)) {
			covered = false;
			/* FX0 DIAGNOSTIC: name every interrupt that blocks system sleep. */
			pr_info("FX0_MPM_UNCOVERED irq=%u hwirq=%lu domain=%s armed=%d disabled=%d flags=0x%x name=%s\n",
				irq, data->hwirq, data->domain ? data->domain->name : "-",
				irqd_is_wakeup_armed(data), irqd_irq_disabled(data),
				desc->action ? desc->action->flags : 0,
				desc->action && desc->action->name ? desc->action->name : "-");
		}
	}
	rcu_read_unlock();
	return covered;
}

bool qcom_mpm_sleep_ready(struct device *dev, struct device *owner)
{
	struct qcom_mpm_priv *priv = dev_get_drvdata(dev);
	unsigned long flags;
	bool ready = true;
	int reg, i;

	if (!irqs_disabled() || raw_smp_processor_id() || num_online_cpus() != 1 ||
	    READ_ONCE(priv->sleep_owner) != owner || !READ_ONCE(priv->sleep_prepared) ||
	    !qcom_mpm_wake_covered(priv))
		return false;
	raw_spin_lock_irqsave(&priv->lock, flags);
	for (reg = 0; reg < MPM_REG_STATUS; reg++)
		for (i = 0; i < priv->reg_stride; i++)
			if (qcom_mpm_read(priv, reg, i) !=
			    priv->sleep_image[reg * priv->reg_stride + i])
				ready = false;
	if (readl_relaxed(priv->base) != U32_MAX || readl_relaxed(priv->base + 4) != U32_MAX)
		ready = false;
	raw_spin_unlock_irqrestore(&priv->lock, flags);
	return ready;
}
EXPORT_SYMBOL_GPL(qcom_mpm_sleep_ready);

int qcom_mpm_finish_sleep(struct device *dev, struct device *owner)
{
	struct qcom_mpm_priv *priv = dev_get_drvdata(dev);

	if (priv->sleep_owner != owner)
		return -ESTALE;
	if (!priv->sleep_prepared)
		return 0;
	/* GIC/CPU context is restored; device IRQs have not resumed yet. */
	qcom_mpm_handler(0, priv);
	WRITE_ONCE(priv->sleep_prepared, false);
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_mpm_finish_sleep);

static int mpm_pd_power_off(struct generic_pm_domain *genpd)
{
	struct qcom_mpm_priv *priv = container_of(genpd, struct qcom_mpm_priv,
						  genpd);
	int i, ret;

	if (READ_ONCE(priv->sleep_owner))
		return -EBUSY;
	for (i = 0; i < priv->reg_stride; i++)
		qcom_mpm_write(priv, MPM_REG_STATUS, i, 0);

	/* Notify RPM to write vMPM into HW */
	ret = mbox_send_message(priv->mbox_chan, NULL);
	if (ret < 0)
		return ret;

	mbox_client_txdone(priv->mbox_chan, 0);

	return 0;
}

static bool gic_hwirq_is_mapped(struct mpm_gic_map *maps, int cnt, u32 hwirq)
{
	int i;

	for (i = 0; i < cnt; i++)
		if (maps[i].hwirq == hwirq)
			return true;

	return false;
}

static int qcom_mpm_probe(struct platform_device *pdev, struct device_node *parent)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct irq_domain *parent_domain;
	struct generic_pm_domain *genpd;
	struct device_node *msgram_np;
	struct qcom_mpm_priv *priv;
	unsigned int pin_cnt;
	struct resource res;
	int i, irq;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = of_property_read_u32(np, "qcom,mpm-pin-count", &pin_cnt);
	if (ret) {
		dev_err(dev, "failed to read qcom,mpm-pin-count: %d\n", ret);
		return ret;
	}

	if (!pin_cnt || pin_cnt > 1024)
		return -EINVAL;
	priv->pin_count = pin_cnt;
	priv->reg_stride = DIV_ROUND_UP(pin_cnt, 32);
	if (IS_ENABLED(CONFIG_QCOM_MSM8226_PM) && dev->parent &&
	    of_device_is_compatible(dev->parent->of_node, "qcom,msm8226-rpm-proc")) {
		priv->sleep_image = devm_kcalloc(dev, MPM_REG_STATUS * priv->reg_stride,
						 sizeof(*priv->sleep_image), GFP_KERNEL);
		if (!priv->sleep_image)
			return -ENOMEM;
	}

	ret = of_property_count_u32_elems(np, "qcom,mpm-pin-map");
	if (ret < 0) {
		dev_err(dev, "failed to read qcom,mpm-pin-map: %d\n", ret);
		return ret;
	}

	if (ret % 2) {
		dev_err(dev, "invalid qcom,mpm-pin-map\n");
		return -EINVAL;
	}

	priv->map_cnt = ret / 2;
	priv->maps = devm_kcalloc(dev, priv->map_cnt, sizeof(*priv->maps),
				  GFP_KERNEL);
	if (!priv->maps)
		return -ENOMEM;

	for (i = 0; i < priv->map_cnt; i++) {
		u32 pin, hwirq;

		of_property_read_u32_index(np, "qcom,mpm-pin-map", i * 2, &pin);
		of_property_read_u32_index(np, "qcom,mpm-pin-map", i * 2 + 1, &hwirq);

		if (gic_hwirq_is_mapped(priv->maps, i, hwirq)) {
			dev_warn(dev, "failed to map pin %d as GIC hwirq %d is already mapped\n",
				 pin, hwirq);
			continue;
		}

		priv->maps[i].pin = pin;
		priv->maps[i].hwirq = hwirq;
	}

	raw_spin_lock_init(&priv->lock);
	if (priv->sleep_image) {
		ret = qcom_mpm_init_wake_domains(dev, priv);
		if (ret)
			return ret;
	}

	/* If we have a handle to an RPM message ram partition, use it. */
	msgram_np = of_parse_phandle(np, "qcom,rpm-msg-ram", 0);
	if (msgram_np) {
		ret = of_address_to_resource(msgram_np, 0, &res);
		if (ret) {
			of_node_put(msgram_np);
			return ret;
		}

		/* Don't use devm_ioremap_resource, as we're accessing a shared region. */
		priv->base = devm_ioremap(dev, res.start, resource_size(&res));
		of_node_put(msgram_np);
		if (!priv->base)
			return -ENOMEM;
	} else {
		/* Otherwise, fall back to simple MMIO. */
		priv->base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(priv->base))
			return PTR_ERR(priv->base);
	}

	for (i = 0; i < priv->reg_stride; i++) {
		qcom_mpm_write(priv, MPM_REG_ENABLE, i, 0);
		qcom_mpm_write(priv, MPM_REG_FALLING_EDGE, i, 0);
		qcom_mpm_write(priv, MPM_REG_RISING_EDGE, i, 0);
		qcom_mpm_write(priv, MPM_REG_POLARITY, i, 0);
		qcom_mpm_write(priv, MPM_REG_STATUS, i, 0);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	priv->irq = irq;

	genpd = &priv->genpd;
	genpd->flags = GENPD_FLAG_IRQ_SAFE;
	genpd->power_off = mpm_pd_power_off;

	genpd->name = devm_kasprintf(dev, GFP_KERNEL, "%s", dev_name(dev));
	if (!genpd->name)
		return -ENOMEM;

	ret = pm_genpd_init(genpd, NULL, false);
	if (ret) {
		dev_err(dev, "failed to init genpd: %d\n", ret);
		return ret;
	}

	ret = of_genpd_add_provider_simple(np, genpd);
	if (ret) {
		dev_err(dev, "failed to add genpd provider: %d\n", ret);
		goto remove_genpd;
	}

	priv->mbox_client.dev = dev;
	priv->mbox_client.knows_txdone = true;
	priv->mbox_chan = mbox_request_channel(&priv->mbox_client, 0);
	if (IS_ERR(priv->mbox_chan)) {
		ret = PTR_ERR(priv->mbox_chan);
		dev_err(dev, "failed to acquire IPC channel: %d\n", ret);
		goto remove_provider;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		dev_err(dev, "failed to find MPM parent domain\n");
		ret = -ENXIO;
		goto free_mbox;
	}

	priv->domain = irq_domain_create_hierarchy(parent_domain,
				IRQ_DOMAIN_FLAG_QCOM_MPM_WAKEUP, pin_cnt,
				of_fwnode_handle(np), &qcom_mpm_ops, priv);
	if (!priv->domain) {
		dev_err(dev, "failed to create MPM domain\n");
		ret = -ENOMEM;
		goto free_mbox;
	}

	irq_domain_update_bus_token(priv->domain, DOMAIN_BUS_WAKEUP);

	ret = devm_request_irq(dev, irq, qcom_mpm_handler, IRQF_NO_SUSPEND,
			       "qcom_mpm", priv);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		goto remove_domain;
	}

	dev_set_drvdata(dev, priv);
	return 0;

remove_domain:
	irq_domain_remove(priv->domain);
free_mbox:
	mbox_free_channel(priv->mbox_chan);
remove_provider:
	of_genpd_del_provider(np);
remove_genpd:
	pm_genpd_remove(genpd);
	return ret;
}

IRQCHIP_PLATFORM_DRIVER_BEGIN(qcom_mpm)
IRQCHIP_MATCH("qcom,mpm", qcom_mpm_probe)
IRQCHIP_PLATFORM_DRIVER_END(qcom_mpm)
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. MSM Power Manager");
MODULE_LICENSE("GPL v2");
