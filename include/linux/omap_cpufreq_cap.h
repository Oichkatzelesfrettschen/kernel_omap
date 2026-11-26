/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_OMAP_CPUFREQ_CAP_H
#define _LINUX_OMAP_CPUFREQ_CAP_H

#ifdef CONFIG_ARM_OMAP2PLUS_CPUFREQ
void omap_thermal_set_cap(unsigned int khz);
#else
static inline void omap_thermal_set_cap(unsigned int khz)
{
}
#endif

#endif
