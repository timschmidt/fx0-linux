// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014,2015, Linaro Ltd.
 *
 * SAW power controller driver
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <soc/qcom/msm8226-pm.h>
#include <soc/qcom/spm.h>

#include <asm/proc-fns.h>
#include <asm/suspend.h>

#include "dt_idle_states.h"

struct cpuidle_qcom_spm_data {
	struct cpuidle_driver cpuidle_driver;
	struct spm_driver_data *spm;
};

static int qcom_pm_collapse(unsigned long int unused)
{
	qcom_scm_cpu_power_down(QCOM_SCM_CPU_PWR_DOWN_L2_ON);

	/*
	 * Returns here only if there was a pending interrupt and we did not
	 * power down as a result.
	 */
	return -1;
}

static int qcom_cpu_spc(struct spm_driver_data *drv)
{
	int ret, restore_ret;

	ret = spm_set_low_power_mode(drv, PM_SLEEP_MODE_SPC);
	if (ret)
		return ret;
	ret = cpu_suspend(0, qcom_pm_collapse);
	/*
	 * ARM common code executes WFI without calling into our driver and
	 * if the SPM mode is not reset, then we may accidentally power down the
	 * cpu when we intended only to gate the cpu clock.
	 * Restore standby, or verify EN is clear if the restore cannot be written.
	 */
	restore_ret = spm_set_low_power_mode(drv, PM_SLEEP_MODE_STBY);

	return ret ?: restore_ret;
}

static __cpuidle int spm_enter_idle_state(struct cpuidle_device *dev,
					  struct cpuidle_driver *drv, int idx)
{
	struct cpuidle_qcom_spm_data *data = container_of(drv, struct cpuidle_qcom_spm_data,
							  cpuidle_driver);

	return CPU_PM_CPU_IDLE_ENTER_PARAM(qcom_cpu_spc, idx, data->spm);
}

static struct cpuidle_driver qcom_spm_idle_driver = {
	.name = "qcom_spm",
	.owner = THIS_MODULE,
	.states[0] = {
		.enter			= spm_enter_idle_state,
		.exit_latency		= 1,
		.target_residency	= 1,
		.power_usage		= UINT_MAX,
		.name			= "WFI",
		.desc			= "ARM WFI",
	}
};

static const struct of_device_id qcom_idle_state_match[] = {
	{ .compatible = "qcom,idle-state-spc", .data = spm_enter_idle_state },
	{ },
};

static struct spm_driver_data *spm_cpuidle_get_spm(int cpu)
{
	struct platform_device *pdev;
	struct device_node *cpu_node, *saw_node;
	struct spm_driver_data *spm;

	cpu_node = of_cpu_device_node_get(cpu);
	if (!cpu_node)
		return ERR_PTR(-ENODEV);

	saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
	of_node_put(cpu_node);
	if (!saw_node)
		return ERR_PTR(-ENODEV);
	if (!of_device_is_available(saw_node)) {
		of_node_put(saw_node);
		return ERR_PTR(-ENODEV);
	}

	pdev = of_find_device_by_node(saw_node);
	of_node_put(saw_node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	spm = dev_get_drvdata(&pdev->dev);
	put_device(&pdev->dev);
	return spm ?: ERR_PTR(-EPROBE_DEFER);
}

static int spm_cpuidle_register(struct device *cpuidle_dev, int cpu)
{
	struct cpuidle_qcom_spm_data *data;
	struct spm_driver_data *spm;
	int ret;

	spm = spm_cpuidle_get_spm(cpu);
	if (IS_ERR(spm))
		return PTR_ERR(spm);

	data = devm_kzalloc(cpuidle_dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->spm = spm;

	data->cpuidle_driver = qcom_spm_idle_driver;
	data->cpuidle_driver.cpumask = (struct cpumask *)cpumask_of(cpu);

	ret = dt_init_idle_driver(&data->cpuidle_driver,
				  qcom_idle_state_match, 1);
	if (ret <= 0)
		return ret ? : -ENODEV;

	return cpuidle_register(&data->cpuidle_driver, NULL);
}

static int spm_cpuidle_drv_probe(struct platform_device *pdev)
{
	struct spm_driver_data *spm;
	int cpu, ret;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	/* Defer before publishing any CPU-idle driver, not partway through. */
	for_each_present_cpu(cpu) {
		spm = spm_cpuidle_get_spm(cpu);
		if (IS_ERR(spm) && PTR_ERR(spm) != -ENODEV)
			return PTR_ERR(spm);
	}

	ret = qcom_msm8226_set_warm_boot_addr(cpu_resume_arm);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "set warm boot addr failed");

	for_each_present_cpu(cpu) {
		ret = spm_cpuidle_register(&pdev->dev, cpu);
		if (ret && ret != -ENODEV) {
			dev_err(&pdev->dev,
				"Cannot register for CPU%d: %d\n", cpu, ret);
		}
	}

	return 0;
}

static struct platform_driver spm_cpuidle_driver = {
	.probe = spm_cpuidle_drv_probe,
	.driver = {
		.name = "qcom-spm-cpuidle",
		.suppress_bind_attrs = true,
	},
};

static bool __init qcom_spm_find_any_cpu(void)
{
	struct device_node *cpu_node, *saw_node;

	for_each_of_cpu_node(cpu_node) {
		saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
		if (of_device_is_available(saw_node)) {
			of_node_put(saw_node);
			of_node_put(cpu_node);
			return true;
		}
		of_node_put(saw_node);
	}
	return false;
}

static int __init qcom_spm_cpuidle_init(void)
{
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&spm_cpuidle_driver);
	if (ret)
		return ret;

	/* Make sure there is actually any CPU managed by the SPM */
	if (!qcom_spm_find_any_cpu())
		return 0;

	pdev = platform_device_register_simple("qcom-spm-cpuidle",
					       -1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&spm_cpuidle_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(qcom_spm_cpuidle_init);
