// SPDX-License-Identifier: GPL-2.0-only
/* MSM8226 system sleep: device preparation, transport handoff and CPU/L2 entry. */
#include <linux/cache.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/smp.h>
#include <linux/slab.h>
#include <linux/soc/qcom/cpr.h>
#include <linux/soc/qcom/mpm.h>
#include <linux/soc/qcom/smd-rpm.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <soc/qcom/msm8226-pm.h>
#include <soc/qcom/spm.h>

#include <linux/cacheflush.h>
#include <asm/outercache.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#include "msm8226-hotplug-probe.h"

#define MSM8226_CPUS 4
#define MSM8226_SLEEP_STATUS 0x8
#define MSM8226_CPU_COLLAPSED BIT(19)

struct msm8226_pm {
	struct device *dev;
	struct platform_device *cpr;
	struct platform_device *mpm;
	struct platform_device *l2_device;
	struct spm_driver_data *l2;
	struct qcom_smd_rpm *rpm;
	struct device_link *cpr_link;
	struct device_link *mpm_link;
	struct device_link *rpm_link;
	struct work_struct install_work;
	struct spm_driver_data *cpu_spm[MSM8226_CPUS];
	void __iomem *acc[MSM8226_CPUS];
	bool voltage_prepared;
	bool transport_prepared;
	bool mpm_prepared;
};

static struct msm8226_pm *msm8226_pm;
/* SPM is built in, has no unregister path and suppresses manual unbinding. */
static struct spm_driver_data *msm8226_cpu_spm[MSM8226_CPUS];

static bool hotplug_probe;
module_param(hotplug_probe, bool, 0400);
MODULE_PARM_DESC(hotplug_probe, "Park CPU3 at the firmware warm vector until reboot");

/* Firmware enters with the MMU off and writes the probe word directly. */
u32 msm8226_warm_probe_state __cacheline_aligned;

static int msm8226_get_probe_state(char *buf, const struct kernel_param *param)
{
	sync_cache_r(&msm8226_warm_probe_state);
	return sprintf(buf, "%u\n", READ_ONCE(msm8226_warm_probe_state));
}

static const struct kernel_param_ops msm8226_probe_state_ops = {
	.get = msm8226_get_probe_state,
};
module_param_cb(hotplug_probe_state, &msm8226_probe_state_ops, NULL, 0400);

int qcom_msm8226_set_warm_boot_addr(void *resume)
{
	struct device_node *node;

	/* Both the idle driver and the coordinator can register this vector. */
	if (hotplug_probe) {
		node = of_find_compatible_node(NULL, NULL, "qcom,msm8226-system-pm");
		if (of_device_is_available(node))
			resume = msm8226_warm_probe_entry;
		of_node_put(node);
	}

	return qcom_scm_set_warm_boot_addr(resume);
}

bool qcom_msm8226_pm_active(void)
{
	return !!READ_ONCE(msm8226_pm);
}

bool qcom_msm8226_hotplug_probe_active(void)
{
	return hotplug_probe && qcom_msm8226_pm_active();
}

static void msm8226_publish_probe_state(u32 state)
{
	WRITE_ONCE(msm8226_warm_probe_state, state);
	sync_cache_w(&msm8226_warm_probe_state);
	dsb(sy);
}

void __noreturn qcom_msm8226_hotplug_probe(bool quiesced)
{
	struct spm_driver_data *spm =
		READ_ONCE(msm8226_cpu_spm[QCOM_MSM8226_HOTPLUG_PROBE_CPU]);
	u32 failure = MSM8226_PROBE_GIC_ERROR;

	if (quiesced) {
		failure = MSM8226_PROBE_SPM_ERROR;
		if (!spm_msm8226_cpu_ready(spm) &&
		    !spm_set_low_power_mode(spm, PM_SLEEP_MODE_SPC)) {
			msm8226_publish_probe_state(MSM8226_PROBE_ARMED);
			flush_cache_louis();
			qcom_scm_cpu_hotplug_power_down();
			spm_set_low_power_mode(spm, PM_SLEEP_MODE_STBY);
			failure = MSM8226_PROBE_SCM_RETURNED;
		}
	}
	msm8226_publish_probe_state(failure);
	for (;;)
		asm volatile("wfi" ::: "memory");
}

static int msm8226_core_finisher(unsigned long arg)
{
	qcom_scm_cpu_power_down(QCOM_SCM_CPU_PWR_DOWN_L2_ON);
	return -EAGAIN;
}

int qcom_msm8226_cpu_collapse(unsigned int cpu)
{
	struct spm_driver_data *spm;
	int ret, restore;

	if (cpu >= MSM8226_CPUS || !cpu || cpu != raw_smp_processor_id() || !irqs_disabled())
		return -EINVAL;
	spm = READ_ONCE(msm8226_cpu_spm[cpu]);
	ret = spm_msm8226_cpu_ready(spm);
	if (ret)
		return ret;
	ret = cpu_pm_enter();
	if (ret)
		return ret;
	ret = spm_set_low_power_mode(spm, PM_SLEEP_MODE_SPC);
	if (!ret)
		ret = cpu_suspend(0, msm8226_core_finisher);
	restore = spm_set_low_power_mode(spm, PM_SLEEP_MODE_STBY);
	cpu_pm_exit();
	return ret ?: restore;
}

int qcom_msm8226_cpu_kill(unsigned int cpu)
{
	struct msm8226_pm *pm = READ_ONCE(msm8226_pm);
	u32 status;

	/* The CPU-hotplug writer excludes coordinator detachment here. */
	if (!pm)
		return 1;
	if (cpu >= MSM8226_CPUS || !cpu)
		return 0;
	return !readl_poll_timeout_atomic(pm->acc[cpu] + MSM8226_SLEEP_STATUS, status,
					 status & MSM8226_CPU_COLLAPSED, 1, 2000);
}

static bool msm8226_secondaries_collapsed(struct msm8226_pm *pm)
{
	int cpu;

	for (cpu = 1; cpu < MSM8226_CPUS; cpu++)
		if (!(readl_relaxed(pm->acc[cpu] + MSM8226_SLEEP_STATUS) &
		      MSM8226_CPU_COLLAPSED))
			return false;
	return true;
}

static int msm8226_system_finisher(unsigned long arg)
{
	struct msm8226_pm *pm = (void *)arg;

	if (!msm8226_secondaries_collapsed(pm))
		return -EBUSY;
	/* ARM cpu_suspend only cleans LoUIS; SCM L2_OFF needs all cache levels. */
	flush_cache_all();
	outer_flush_all();
	qcom_scm_cpu_power_down(QCOM_SCM_CPU_PWR_DOWN_L2_OFF);
	/* A pending interrupt can make SCM return without collapsing. */
	return -EAGAIN;
}

static int msm8226_suspend_enter(suspend_state_t state)
{
	struct msm8226_pm *pm = READ_ONCE(msm8226_pm);
	int ret, cpu_restore, l2_restore;

	if (!pm || state != PM_SUSPEND_MEM || raw_smp_processor_id() ||
	    !irqs_disabled() || num_online_cpus() != 1)
		return -EINVAL;
	if (!pm->voltage_prepared || !pm->transport_prepared || !pm->mpm_prepared ||
	    !qcom_cpr_msm8926_suspend_ready(&pm->cpr->dev, pm->dev) ||
	    !qcom_rpm_smd_sleep_ready(pm->rpm, pm->dev) ||
	    !qcom_mpm_sleep_ready(&pm->mpm->dev, pm->dev) ||
	    !msm8226_secondaries_collapsed(pm))
		return -EBUSY;
	ret = spm_msm8226_cpu_ready(msm8226_cpu_spm[0]);
	if (ret)
		return ret;
	ret = cpu_pm_enter();
	if (ret)
		return ret;
	ret = cpu_cluster_pm_enter();
	if (ret)
		goto cpu_exit;
	if (!msm8226_secondaries_collapsed(pm)) {
		ret = -EBUSY;
		goto cluster_exit;
	}
	ret = spm_set_l2_low_power_mode(pm->l2, SPM_L2_MODE_COLLAPSE_RPM);
	if (ret)
		goto cluster_exit;
	ret = spm_set_low_power_mode(msm8226_cpu_spm[0], PM_SLEEP_MODE_PC);
	if (!ret)
		ret = cpu_suspend((unsigned long)pm, msm8226_system_finisher);

	/* Both wake and failed entry must disarm before generic WFI/notifiers. */
	cpu_restore = spm_set_low_power_mode(msm8226_cpu_spm[0], PM_SLEEP_MODE_STBY);
	l2_restore = spm_set_l2_low_power_mode(pm->l2, SPM_L2_MODE_DISABLED);
	ret = ret ?: cpu_restore ?: l2_restore;
cluster_exit:
	cpu_cluster_pm_exit();
cpu_exit:
	cpu_pm_exit();
	return ret;
}

static int msm8226_platform_prepare_late(void)
{
	struct msm8226_pm *pm = READ_ONCE(msm8226_pm);
	int ret, restore;

	if (!pm || !pm->voltage_prepared)
		return -EINVAL;
	/* SMD's explicit NO_SUSPEND receive IRQ survived device noirq callbacks. */
	ret = qcom_rpm_smd_prepare_sleep(pm->rpm, pm->dev);
	if (ret)
		return ret;
	pm->transport_prepared = true;
	ret = qcom_mpm_prepare_sleep(&pm->mpm->dev, pm->dev);
	if (!ret) {
		pm->mpm_prepared = true;
		return 0;
	}
	/* A failed preparation owns its rollback, even if wake is also called. */
	restore = qcom_rpm_smd_finish_sleep(pm->rpm, pm->dev);
	if (restore)
		panic("MSM8226 RPM preparation rollback failed: %d", restore);
	pm->transport_prepared = false;
	return ret;
}

static void msm8226_platform_wake(void)
{
	struct msm8226_pm *pm = READ_ONCE(msm8226_pm);
	int ret;

	if (!pm)
		return;
	if (pm->mpm_prepared) {
		ret = qcom_mpm_finish_sleep(&pm->mpm->dev, pm->dev);
		if (ret)
			panic("MSM8226 MPM wake restore failed: %d", ret);
		pm->mpm_prepared = false;
	}
	if (pm->transport_prepared) {
		/* Reopen ACK delivery before genpd's device noirq power-on requests. */
		ret = qcom_rpm_smd_finish_sleep(pm->rpm, pm->dev);
		if (ret)
			panic("MSM8226 RPM wake restore failed: %d", ret);
		pm->transport_prepared = false;
	}
}

static const struct platform_suspend_ops msm8226_suspend_ops = {
	.valid = suspend_valid_only_mem,
	.prepare_late = msm8226_platform_prepare_late,
	.enter = msm8226_suspend_enter,
	.wake = msm8226_platform_wake,
};

static int msm8226_pm_suspend_late(struct device *dev)
{
	struct msm8226_pm *pm = dev_get_drvdata(dev);
	int ret;

	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;
	ret = qcom_cpr_msm8926_suspend_prepare(&pm->cpr->dev, dev);
	if (!ret)
		pm->voltage_prepared = true;
	return ret;
}

static int msm8226_pm_resume_early(struct device *dev)
{
	struct msm8226_pm *pm = dev_get_drvdata(dev);
	int ret;

	if (!pm->voltage_prepared)
		return 0;
	ret = qcom_cpr_msm8926_suspend_restore(&pm->cpr->dev, dev);
	if (!ret)
		pm->voltage_prepared = false;
	return ret;
}

static const struct dev_pm_ops msm8226_pm_ops = {
	.suspend_late = msm8226_pm_suspend_late,
	.resume_early = msm8226_pm_resume_early,
};

static struct platform_device *msm8226_pm_supplier(struct device *dev, const char *name,
						   const char *compatible)
{
	struct device_node *node;
	struct platform_device *pdev;

	node = of_parse_phandle(dev->of_node, name, 0);
	if (!node)
		return ERR_PTR(-EINVAL);
	if (!of_device_is_available(node) || !of_device_is_compatible(node, compatible)) {
		of_node_put(node);
		return ERR_PTR(-EINVAL);
	}
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	if (!dev_get_drvdata(&pdev->dev)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return pdev;
}

static int msm8226_pm_get_cpus(struct msm8226_pm *pm)
{
	struct platform_device *pdev;
	struct device_node *cpu_node, *node;
	struct resource resource;
	const char *method;
	int cpu, ret;

	if (num_present_cpus() != MSM8226_CPUS)
		return -EINVAL;
	for (cpu = 0; cpu < MSM8226_CPUS; cpu++) {
		if (!cpu_present(cpu) || cpu_logical_map(cpu) != cpu)
			return -EINVAL;
		cpu_node = of_cpu_device_node_get(cpu);
		if (!cpu_node)
			return -EINVAL;
		if (of_property_read_string(cpu_node, "enable-method", &method) ||
		    strcmp(method, "qcom,msm8226-smp") ||
		    !of_device_is_compatible(cpu_node, "arm,cortex-a7")) {
			of_node_put(cpu_node);
			return -EINVAL;
		}
		node = of_parse_phandle(cpu_node, "qcom,saw", 0);
		pdev = node && of_device_is_available(node) &&
			of_device_is_compatible(node, "qcom,msm8226-saw2-v2.1-cpu") ?
			of_find_device_by_node(node) : NULL;
		of_node_put(node);
		if (!pdev || !platform_get_drvdata(pdev)) {
			of_node_put(cpu_node);
			if (pdev)
				put_device(&pdev->dev);
			return -EPROBE_DEFER;
		}
		/* Provider lifetime is permanent; a dying CPU may retain this pointer. */
		pm->cpu_spm[cpu] = platform_get_drvdata(pdev);
		put_device(&pdev->dev);
		node = of_parse_phandle(cpu_node, "qcom,acc", 0);
		of_node_put(cpu_node);
		if (!node)
			return -EINVAL;
		ret = of_address_to_resource(node, 0, &resource);
		of_node_put(node);
		if (ret)
			return ret;
		if (resource.start != 0xf9088000 + cpu * 0x10000 ||
		    resource_size(&resource) < MSM8226_SLEEP_STATUS + sizeof(u32))
			return -EINVAL;
		/* Shared with the built-in SMP bring-up path, which maps it briefly. */
		pm->acc[cpu] = devm_ioremap(pm->dev, resource.start, resource_size(&resource));
		if (!pm->acc[cpu])
			return -ENOMEM;
	}
	return 0;
}

/* Run outside driver probe so system PM can finish waiting for probes. */
static void msm8226_pm_install(struct work_struct *work)
{
	suspend_set_ops(&msm8226_suspend_ops);
}

static int msm8226_pm_probe(struct platform_device *pdev)
{
	struct msm8226_pm *pm;
	struct device *dev = &pdev->dev;
	int ret, cpu;

	if (READ_ONCE(msm8226_pm))
		return -EBUSY;
	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;
	pm = devm_kzalloc(dev, sizeof(*pm), GFP_KERNEL);
	if (!pm)
		return -ENOMEM;
	pm->dev = dev;
	if (!dev->parent ||
	    !of_device_is_compatible(dev->parent->of_node, "qcom,rpm-msm8226"))
		return -EINVAL;
	if (!device_is_bound(dev->parent))
		return -EPROBE_DEFER;
	pm->rpm = dev_get_drvdata(dev->parent);
	if (!pm->rpm)
		return -EPROBE_DEFER;
	INIT_WORK(&pm->install_work, msm8226_pm_install);
	pm->cpr = msm8226_pm_supplier(dev, "qcom,cpr", "qcom,msm8926-cpr");
	if (IS_ERR(pm->cpr))
		return PTR_ERR(pm->cpr);
	pm->mpm = msm8226_pm_supplier(dev, "qcom,mpm", "qcom,mpm");
	if (IS_ERR(pm->mpm)) {
		ret = PTR_ERR(pm->mpm);
		goto put_cpr;
	}
	pm->l2_device = msm8226_pm_supplier(dev, "qcom,l2-spm",
					    "qcom,msm8226-saw2-v2.1-l2");
	if (IS_ERR(pm->l2_device)) {
		ret = PTR_ERR(pm->l2_device);
		goto put_mpm;
	}
	pm->l2 = platform_get_drvdata(pm->l2_device);
	ret = msm8226_pm_get_cpus(pm);
	if (ret)
		goto put_l2;
	/* CPR is the consumer: it must quiesce before, and resume after, us. */
	pm->cpr_link = device_link_add(&pm->cpr->dev, dev, DL_FLAG_STATELESS);
	if (!pm->cpr_link) {
		ret = -EINVAL;
		goto put_l2;
	}
	pm->mpm_link = device_link_add(dev, &pm->mpm->dev, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!pm->mpm_link) {
		ret = -EINVAL;
		goto unlink_cpr;
	}
	pm->rpm_link = device_link_add(dev, dev->parent, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!pm->rpm_link) {
		ret = -EINVAL;
		goto unlink_cpr;
	}
	ret = qcom_mpm_register_sleep(&pm->mpm->dev, dev);
	if (ret)
		goto unlink_cpr;
	ret = qcom_rpm_smd_register_sleep(pm->rpm, dev);
	if (ret)
		goto unregister_mpm;
	ret = qcom_cpr_msm8926_suspend_attach(&pm->cpr->dev, dev);
	if (ret)
		goto unregister_rpm;
	ret = qcom_msm8226_set_warm_boot_addr(cpu_resume_arm);
	if (ret)
		goto detach_cpr;
	cpus_write_lock();
	if (!cpumask_equal(cpu_online_mask, cpu_present_mask) || READ_ONCE(msm8226_pm)) {
		cpus_write_unlock();
		ret = -EBUSY;
		goto detach_cpr;
	}
	for (cpu = 0; cpu < MSM8226_CPUS; cpu++)
		WRITE_ONCE(msm8226_cpu_spm[cpu], pm->cpu_spm[cpu]);
	platform_set_drvdata(pdev, pm);
	WRITE_ONCE(msm8226_pm, pm);
	cpus_write_unlock();
	if (!hotplug_probe)
		schedule_work(&pm->install_work);
	return 0;

detach_cpr:
	qcom_cpr_msm8926_suspend_detach(&pm->cpr->dev, dev);
unregister_rpm:
	qcom_rpm_smd_unregister_sleep(pm->rpm, dev);
unregister_mpm:
	qcom_mpm_unregister_sleep(&pm->mpm->dev, dev);
unlink_cpr:
	device_link_del(pm->cpr_link);
put_l2:
	put_device(&pm->l2_device->dev);
put_mpm:
	put_device(&pm->mpm->dev);
put_cpr:
	put_device(&pm->cpr->dev);
	return ret;
}

static void msm8226_pm_remove(struct platform_device *pdev)
{
	struct msm8226_pm *pm = platform_get_drvdata(pdev);
	int ret;

	/* RPM lifecycle admission excludes removal during a PM transition. */
	cancel_work_sync(&pm->install_work);
	suspend_set_ops(NULL);
	cpus_write_lock();
	WRITE_ONCE(msm8226_pm, NULL);
	cpus_write_unlock();
	ret = qcom_cpr_msm8926_suspend_detach(&pm->cpr->dev, pm->dev);
	ret = ret ?: qcom_rpm_smd_unregister_sleep(pm->rpm, pm->dev);
	ret = ret ?: qcom_mpm_unregister_sleep(&pm->mpm->dev, pm->dev);
	if (ret)
		panic("MSM8226 coordinator teardown lost ownership: %d", ret);
	device_link_del(pm->cpr_link);
	put_device(&pm->l2_device->dev);
	put_device(&pm->mpm->dev);
	put_device(&pm->cpr->dev);
}

static const struct of_device_id msm8226_pm_match[] = {
	{ .compatible = "qcom,msm8226-system-pm" },
	{ }
};

static struct platform_driver msm8226_pm_driver = {
	.probe = msm8226_pm_probe,
	.remove = msm8226_pm_remove,
	.driver = {
		.name = "qcom-msm8226-system-pm",
		.of_match_table = msm8226_pm_match,
		.pm = &msm8226_pm_ops,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(msm8226_pm_driver);
MODULE_DESCRIPTION("MSM8226 system suspend coordinator");
MODULE_LICENSE("GPL");
