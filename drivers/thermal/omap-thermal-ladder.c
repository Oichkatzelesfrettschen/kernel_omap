/* SPDX-License-Identifier: GPL-2.0 */
/*
 * OMAP4460 die-temperature ladder on the bandgap hooks.
 *
 * The hardware TSHUT comparator resets the SoC at the TSHUT hot threshold
 * programmed by omap-bandgap. The ladder below caps the MPU in steps ahead
 * of that reset.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mfd/omap_control.h>
#include <linux/mutex.h>
#include <linux/omap_cpufreq_cap.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "omap-bandgap.h"

/*
 * Thermal ladder, in sensor millidegrees. Reaching level n's trip caps the
 * MPU at its cap; the hot alert window always sits at the next level's
 * trip, so a fast climb escalates through several levels in one
 * evaluation. A level steps down one at a time, once the sensor reads
 * below its clear point after LADDER_DWELL_MS in that level.
 */
struct omap_thermal_level {
	int trip;
	int clear;
	unsigned int cap_khz;
};

static const struct omap_thermal_level ladder[] = {
	{ 0, 0, 0 },
	{ 70000, 62000, 920000 },
	{ 78000, 70000, 700000 },
	{ 99000, 90000, 350000 },
};

#define LADDER_TOP		((int)ARRAY_SIZE(ladder) - 1)
#define LADDER_DWELL_MS		2000
#define LADDER_FIRST_EVAL_MS	1000

struct omap_ladder {
	struct omap_bandgap *bg_ptr;
	int id;
	struct work_struct alert_work;
	struct delayed_work dwell_work;
	struct mutex lock;	/* level, enabled and the alert window */
	int level;
	unsigned long level_jiffies;
	bool enabled;
	bool reprogram_next;
	struct notifier_block pm_notifier;
};

static int ladder_code_to_mc(struct omap_bandgap *bg_ptr, int id, u32 code,
			     int *mc)
{
	struct temp_sensor_data *ts_data = bg_ptr->conf->sensors[id].ts_data;

	if (code < ts_data->adc_start_val || code > ts_data->adc_end_val)
		return -ERANGE;
	*mc = bg_ptr->conv_table[code - ts_data->adc_start_val];

	return 0;
}

/*
 * Programs the alert window for the current level: T_HOT at the next
 * level's trip and T_COLD at this level's clear point. The write order
 * keeps T_COLD below T_HOT at every step, since omap-bandgap moves the
 * other threshold by one hysteresis otherwise.
 */
static int ladder_program(struct omap_ladder *l)
{
	struct omap_bandgap *bg_ptr = l->bg_ptr;
	int hot = ladder[l->level < LADDER_TOP ? l->level + 1 : l->level].trip;
	int cold = ladder[l->level ? l->level : 1].clear;
	int old_cold, ret;

	ret = omap_bandgap_read_tcold(bg_ptr, l->id, &old_cold);
	if (ret)
		return ret;

	if (hot <= old_cold) {
		ret = omap_bandgap_write_tcold(bg_ptr, l->id, cold);
		if (!ret)
			ret = omap_bandgap_write_thot(bg_ptr, l->id, hot);
	} else {
		ret = omap_bandgap_write_thot(bg_ptr, l->id, hot);
		if (!ret)
			ret = omap_bandgap_write_tcold(bg_ptr, l->id, cold);
	}

	return ret;
}

static void ladder_log(struct omap_ladder *l, int temp)
{
	struct omap_bandgap *bg_ptr = l->bg_ptr;
	struct temp_sensor_registers *tsr = bg_ptr->conf->sensors[l->id].registers;
	u32 thresh = 0, tshut = 0;

	omap_control_readl(bg_ptr->dev->parent, tsr->bgap_threshold, &thresh);
	omap_control_readl(bg_ptr->dev->parent, tsr->tshut_threshold, &tshut);
	dev_info(bg_ptr->dev,
		 "ladder level %d at %d mC, cap %u kHz, threshold 0x%08x, tshut 0x%08x\n",
		 l->level, temp, ladder[l->level].cap_khz, thresh, tshut);
}

/*
 * Moves the ladder to the level the sensor reading calls for, applies its
 * MPU cap and, for an alert or a forced move, reprograms the alert window.
 * An unreadable sensor holds the level; the hardware TSHUT comparator stays
 * armed either way. While throttled the evaluation repeats every
 * LADDER_DWELL_MS, which is what steps the ladder down.
 */
static void ladder_evaluate(struct omap_ladder *l, int force, bool reprogram)
{
	int temp = 0, level;
	bool valid;

	mutex_lock(&l->lock);
	if (!l->enabled)
		goto out;
	if (l->reprogram_next) {
		reprogram = true;
		l->reprogram_next = false;
	}

	level = l->level;
	valid = !omap_bandgap_read_temperature(l->bg_ptr, l->id, &temp);
	if (force > 0 && level < LADDER_TOP) {
		level++;
	} else if (force < 0) {
		level = 0;
	} else if (valid) {
		while (level < LADDER_TOP && temp >= ladder[level + 1].trip)
			level++;
		if (level == l->level && level && temp < ladder[level].clear &&
		    time_after_eq(jiffies, l->level_jiffies +
				  msecs_to_jiffies(LADDER_DWELL_MS)))
			level--;
	}

	if (level != l->level) {
		l->level = level;
		l->level_jiffies = jiffies;
		reprogram = true;
	}
	if (reprogram) {
		if (ladder_program(l))
			dev_err(l->bg_ptr->dev, "alert window write failed\n");
		ladder_log(l, valid ? temp : 0);
	}
	omap_thermal_set_cap(ladder[l->level].cap_khz);

	if (l->level)
		schedule_delayed_work(&l->dwell_work,
				      msecs_to_jiffies(LADDER_DWELL_MS));
out:
	mutex_unlock(&l->lock);
}

static void ladder_alert_fn(struct work_struct *work)
{
	struct omap_ladder *l = container_of(work, struct omap_ladder,
					     alert_work);

	ladder_evaluate(l, 0, true);
}

static void ladder_dwell_fn(struct work_struct *work)
{
	struct omap_ladder *l = container_of(work, struct omap_ladder,
					     dwell_work.work);

	ladder_evaluate(l, 0, false);
}

static struct omap_ladder *ladder_of(struct device *dev)
{
	struct omap_bandgap *bg_ptr = dev_get_drvdata(dev);
	void *data;

	if (!bg_ptr)
		return NULL;
	data = omap_bandgap_get_sensor_data(bg_ptr, 0);

	return IS_ERR_OR_NULL(data) ? NULL : data;
}

static ssize_t temperature_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct omap_ladder *l = ladder_of(dev);
	int temp, ret;

	if (!l)
		return -ENODEV;
	ret = omap_bandgap_read_temperature(l->bg_ptr, l->id, &temp);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", temp);
}

static ssize_t level_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct omap_ladder *l = ladder_of(dev);

	if (!l)
		return -ENODEV;

	return sprintf(buf, "%d\n", l->level);
}

/*
 * Writing 1 raises the ladder one level and 0 drops it to level 0; the
 * periodic evaluation then moves it to the level the sensor calls for.
 */
static ssize_t throttle_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct omap_ladder *l = ladder_of(dev);

	if (!l)
		return -ENODEV;
	ladder_evaluate(l, count && buf[0] == '1' ? 1 : -1, true);

	return count;
}

/* Alert window as programmed in hardware: "t_hot t_cold" in mC. */
static ssize_t alert_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct omap_ladder *l = ladder_of(dev);
	int hot, cold, ret;

	if (!l)
		return -ENODEV;
	ret = omap_bandgap_read_thot(l->bg_ptr, l->id, &hot);
	if (!ret)
		ret = omap_bandgap_read_tcold(l->bg_ptr, l->id, &cold);
	if (ret)
		return ret;

	return sprintf(buf, "%d %d\n", hot, cold);
}

/* TSHUT window as programmed in hardware: "hot cold gpio" in mC and number. */
static ssize_t tshut_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct omap_ladder *l = ladder_of(dev);
	struct omap_bandgap *bg_ptr;
	struct temp_sensor_registers *tsr;
	u32 reg;
	int hot, cold, ret;

	if (!l)
		return -ENODEV;
	bg_ptr = l->bg_ptr;
	tsr = bg_ptr->conf->sensors[l->id].registers;
	ret = omap_control_readl(bg_ptr->dev->parent, tsr->tshut_threshold,
				 &reg);
	if (ret)
		return ret;
	ret = ladder_code_to_mc(bg_ptr, l->id,
				(reg & tsr->tshut_hot_mask) >>
				__ffs(tsr->tshut_hot_mask), &hot);
	if (!ret)
		ret = ladder_code_to_mc(bg_ptr, l->id,
					(reg & tsr->tshut_cold_mask) >>
					__ffs(tsr->tshut_cold_mask), &cold);
	if (ret)
		return ret;

	return sprintf(buf, "%d %d %d\n", hot, cold, bg_ptr->tshut_gpio);
}

static DEVICE_ATTR(temperature, S_IRUGO, temperature_show, NULL);
static DEVICE_ATTR(level, S_IRUGO, level_show, NULL);
static DEVICE_ATTR(throttle, S_IWUSR, NULL, throttle_store);
static DEVICE_ATTR(alert, S_IRUGO, alert_show, NULL);
static DEVICE_ATTR(tshut, S_IRUGO, tshut_show, NULL);

static struct attribute *ladder_attrs[] = {
	&dev_attr_temperature.attr,
	&dev_attr_level.attr,
	&dev_attr_throttle.attr,
	&dev_attr_alert.attr,
	&dev_attr_tshut.attr,
	NULL
};

static const struct attribute_group ladder_group = {
	.attrs = ladder_attrs,
};

/*
 * cpufreq is locked across suspend and the sensor clock stops, so the
 * ladder holds its level and resumes evaluating once the system is back.
 */
static int ladder_pm_cb(struct notifier_block *nb, unsigned long event,
			void *unused)
{
	struct omap_ladder *l = container_of(nb, struct omap_ladder,
					     pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&l->lock);
		l->enabled = false;
		mutex_unlock(&l->lock);
		cancel_work_sync(&l->alert_work);
		cancel_delayed_work_sync(&l->dwell_work);
		break;
	case PM_POST_SUSPEND:
		mutex_lock(&l->lock);
		l->enabled = true;
		mutex_unlock(&l->lock);
		schedule_work(&l->alert_work);
		break;
	}

	return NOTIFY_DONE;
}

int omap_thermal_report_temperature(struct omap_bandgap *bg_ptr, int id)
{
	struct omap_ladder *l = omap_bandgap_get_sensor_data(bg_ptr, id);

	if (IS_ERR_OR_NULL(l))
		return -EINVAL;
	if (l->enabled)
		schedule_work(&l->alert_work);

	return 0;
}

int omap_thermal_expose_sensor(struct omap_bandgap *bg_ptr, int id,
			       char *domain)
{
	struct omap_ladder *l;
	struct kobject *omap_kobj;
	int ret;

	l = devm_kzalloc(bg_ptr->dev, sizeof(*l), GFP_KERNEL);
	if (!l)
		return -ENOMEM;

	l->bg_ptr = bg_ptr;
	l->id = id;
	l->enabled = true;
	l->reprogram_next = true;
	mutex_init(&l->lock);
	INIT_WORK(&l->alert_work, ladder_alert_fn);
	INIT_DELAYED_WORK(&l->dwell_work, ladder_dwell_fn);
	l->pm_notifier.notifier_call = ladder_pm_cb;

	ret = omap_bandgap_set_sensor_data(bg_ptr, id, l);
	if (ret)
		return ret;

	ret = sysfs_create_group(&bg_ptr->dev->kobj, &ladder_group);
	if (ret) {
		dev_err(bg_ptr->dev, "ladder sysfs group failed (%d)\n", ret);
		return ret;
	}

	/* Path of the 3.0 omap_temp_sensor driver. */
	if (bg_ptr->dev->parent && bg_ptr->dev->parent->parent) {
		omap_kobj = &bg_ptr->dev->parent->parent->kobj;
		if (sysfs_create_link(omap_kobj, &bg_ptr->dev->kobj,
				      "omap_temp_sensor.0"))
			dev_warn(bg_ptr->dev, "omap_temp_sensor.0 link failed\n");
	}

	if (register_pm_notifier(&l->pm_notifier))
		dev_err(bg_ptr->dev, "ladder PM notifier failed\n");

	/*
	 * The first read waits one second so the continuous-mode counter
	 * has produced a conversion; it also writes the level 0 alert window.
	 */
	schedule_delayed_work(&l->dwell_work,
			      msecs_to_jiffies(LADDER_FIRST_EVAL_MS));
	dev_info(bg_ptr->dev, "ladder %s sensor: trips %d/%d/%d mC\n", domain,
		 ladder[1].trip, ladder[2].trip, ladder[3].trip);

	return 0;
}

int omap_thermal_remove_sensor(struct omap_bandgap *bg_ptr, int id)
{
	struct omap_ladder *l = omap_bandgap_get_sensor_data(bg_ptr, id);

	if (IS_ERR_OR_NULL(l))
		return -EINVAL;

	mutex_lock(&l->lock);
	l->enabled = false;
	mutex_unlock(&l->lock);
	cancel_work_sync(&l->alert_work);
	cancel_delayed_work_sync(&l->dwell_work);
	unregister_pm_notifier(&l->pm_notifier);
	if (bg_ptr->dev->parent && bg_ptr->dev->parent->parent)
		sysfs_remove_link(&bg_ptr->dev->parent->parent->kobj,
				  "omap_temp_sensor.0");
	sysfs_remove_group(&bg_ptr->dev->kobj, &ladder_group);
	omap_thermal_set_cap(0);

	return 0;
}
