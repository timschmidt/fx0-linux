/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MSM8226_HOTPLUG_PROBE_H__
#define __MSM8226_HOTPLUG_PROBE_H__

#define MSM8226_PROBE_ARMED		1
#define MSM8226_PROBE_WARM_ENTRY	2
#define MSM8226_PROBE_SCM_RETURNED	3
#define MSM8226_PROBE_SPM_ERROR		4
#define MSM8226_PROBE_GIC_ERROR		5

#ifndef __ASSEMBLY__
void msm8226_warm_probe_entry(void);
#endif

#endif
