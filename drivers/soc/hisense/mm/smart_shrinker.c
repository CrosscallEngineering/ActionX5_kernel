/*
 * Copyright (c) 2019, Hisense, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "SSHRINK: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/sort.h>
#include <linux/sched.h>
#include <linux/kernel_stat.h>
#include <linux/kthread.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/notifier.h>
#include <linux/vmpressure.h>
#include <linux/version.h>
#include <linux/his_debug_base.h>
#include "smart_shrinker.h"

static int debug_sshrink = 0;

#define SSPRINT(args)	\
		if (1 == debug_sshrink) pr_info args;

static struct smart_shrinker ss_info;

enum {
	SS_FREE_MEM_LOW        = 1 << 0,
	SS_ANON_PAGE_HIGH      = 1 << 1,
	SS_NEED_COMPACT_SWAP   = 1 << 2,
	SS_CPU_LOAD_HIGH       = 1 << 3,
};

#define DEFAULT_CPUIDLE_THRESHOLD    30
#define DEFAULT_SWAPPAGE_PERCENT     30
#define WAKE_THREAD_VMPR_PERCENT     80
#define DEFAULT_MEM_PRESSURE_MIN     90
#define DEFAULT_MEM_PRESSURE_MAX     98
#define DEFAULT_SWAPFULL_PERCENT     85
#define MAX_MEM_PER_SWAP             100

#define SWAPMEM_INTERVAL             (10 * 60 * HZ)
#define COMPACTZRAM_INTERVAL         (30 * 60 * HZ)
#define SHRINKMEM_INTERVAL           (60 * 60 * HZ)

#define DEFAULT_NR_FREE_PAGES        ((300 * SZ_1M) >> (PAGE_SHIFT))
#define DEFAULT_NR_ANON_PAGES        ((300 * SZ_1M) >> (PAGE_SHIFT))

#define SSHRINKER_DELAY_TIME         5
#define SHRINKER_NR_FREE_PAGES       ((150 * SZ_1M) >> (PAGE_SHIFT))
/* swap page size once */
#define SS_NR_SWAP_SIZE              (SZ_1M >> (PAGE_SHIFT))
#define CHECK_MEMSTAT_TIMEOUT        (30 * 1000)

static bool check_twice_interval(unsigned long last, int interval)
{
	unsigned long next_time;

	next_time = last + interval;

	if (time_after(jiffies, next_time))
		return true;

	return false;
}

static inline unsigned long elapsed_jiffies(unsigned long start)
{
	unsigned long end = jiffies;

	if (end >= start)
		return (unsigned long)(end - start);

	return (unsigned long)(end + (MAX_JIFFY_OFFSET - start) + 1);
}

static int smart_shrink_swap_pages(int nr_pages)
{
	int unit_pages, total, real = 0;

	for (total = 0; total < nr_pages; total += SWAP_CLUSTER_MAX) {
		unit_pages = ((total + SWAP_CLUSTER_MAX) > nr_pages)
			? (nr_pages - total) : SWAP_CLUSTER_MAX;
		real += try_to_free_pages_ex(unit_pages, SS_MODE_ANON);
		cond_resched();
	}

	/* return real reclaimed page count */
	return real;
}

static bool free_mem_is_low(int free_pages_min)
{
	unsigned long nr_free_pages = 0;

	nr_free_pages = global_zone_page_state(NR_FREE_PAGES);
	SSPRINT(("curr free pages: %ld\n", nr_free_pages));
	if (nr_free_pages > free_pages_min)
		return false;

	return true;
}

static bool anon_pages_is_high(int anon_pages_min)
{
	unsigned long nr_pages = 0;

	nr_pages = global_node_page_state(NR_INACTIVE_ANON);
	nr_pages += global_node_page_state(NR_ACTIVE_ANON);
	SSPRINT(("curr anon pages: %ld\n", nr_pages));
	if (nr_pages > anon_pages_min)
		return true;

	return false;
}

static inline bool display_is_off(void)
{
	return ss_info.backlight_off;
}

static inline bool is_cpu_idle(struct smart_shrinker *ss)
{
	int threshold = ss->idle_threshold;

	/* when display off, we try do more work. */
	if(ss->backlight_off)
		threshold += 5;

	if ( ss->cpu_load[0] <= threshold
		 && ss->cpu_load[1] <= threshold
		 && ss->cpu_load[2] <= threshold ) {
		return true;
	}

	return false;
}

static bool swap_need_compact(void)
{
	struct sysinfo si;
	unsigned long max_used;

	si_swapinfo(&si);
	if (si.totalswap == 0)
		return false;

	max_used = si.totalswap * ss_info.compact_threshold;
	/* used/total > compact_percent */
	if (((si.totalswap - si.freeswap) * 100) >= max_used)
		return true;

	ss_info.compact_finished = 0;
	return false;
}

/**
 * __get_allcpu_load() - get load for all cpu since last updated
 *
 * Return: The average load of all cpu in percentage since this
 * function was last called.
 */
static u32 __get_allcpu_load(struct smart_shrinker *ss)
{
	int i = 0, j = 0;
	u64 delta_time = 0;
	u64 delta_idle = 0;
	u32 load = 0;
	u64 now = 0;
	u64 now_idle = 0;

	for_each_possible_cpu(i) {
		now_idle += get_cpu_idle_time(i, &now, 0);
		j ++;
	}

	SSPRINT(("now idle =%llu, j=%d, now=%llu\n", now_idle, j, now));
	now_idle = now_idle / j;
	delta_idle = now_idle - ss->last_idle_time;
	delta_time = now - ss->last_timestamp;
	ss->last_idle_time = now_idle;
	ss->last_timestamp = now;

	if (delta_time <= delta_idle)
		load = 0;
	else
		load = div64_u64(100 * (delta_time - delta_idle), delta_time);

	return load;
}

static int get_allcpu_load(struct smart_shrinker *ss)
{
	int cpu_load = 0;

	cpu_load = __get_allcpu_load(ss);
	SSPRINT(("the cpu load is %d\n", cpu_load));
	if (cpu_load >= 0) {
		ss->cpu_load[2] = ss->cpu_load[1];
		ss->cpu_load[1] = ss->cpu_load[0];
		ss->cpu_load[0] = cpu_load;
	}

	return cpu_load;
}

static int get_memory_stat(struct smart_shrinker *ss)
{
	int ret = 0, value;

	/* update cpu load stat. */
	get_allcpu_load(ss);
	if (!is_cpu_idle(ss))
		ret |= SS_CPU_LOAD_HIGH;

	value = ss->free_pages_min;
	if (free_mem_is_low(value))
		ret |= SS_FREE_MEM_LOW;

	value = ss->anon_pages_min;
	if (anon_pages_is_high(value))
		ret |= SS_ANON_PAGE_HIGH;

	if (swap_need_compact())
		ret |= SS_NEED_COMPACT_SWAP;

	return ret;
}

static void smart_shrinker_work_fn(struct work_struct *work)
{
	if (!check_twice_interval(ss_info.last_shrink_jiffies, SHRINKMEM_INTERVAL)) {
		pr_err("shrinker run is not allowed, interval is small\n");
		return;
	}

	if (!free_mem_is_low(SHRINKER_NR_FREE_PAGES)) {
		pr_err("shrinker run is not allowed, free memory is high\n");
		return;
	}

	ss_info.shrink_count ++;
	/* if shrinker effective is worse, reduce frenqucyly call it */
	shrink_mem_by_lmk_rpt(30);
	ss_info.last_shrink_jiffies = jiffies;
}
DECLARE_DELAYED_WORK(smart_shrinker_work, smart_shrinker_work_fn);

static int smart_swap_thread(void *unused)
{
	int ret, nr_pages = 0;
	int swap_nr = 0;
	int check_timeout = 0;
	int factor = 1;
	unsigned long time_jiffies;
	struct task_struct *tsk = current;
	struct smart_shrinker *ss = &ss_info;

	/* need swap out, PF_FREEZER_SKIP is protection from hung_task. */
	tsk->flags |= PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD | PF_FREEZER_SKIP;

	while (true) {
		check_timeout = factor * CHECK_MEMSTAT_TIMEOUT;
		ret = down_timeout(&ss->wait, msecs_to_jiffies(CHECK_MEMSTAT_TIMEOUT));
		if (kthread_should_stop())
			goto out;

		ret = get_memory_stat(ss);
		if ((ret & SS_NEED_COMPACT_SWAP)
			&& !(ret & SS_CPU_LOAD_HIGH)) {
			factor = 1;
			if (check_twice_interval(ss->last_compact_jiffies, COMPACTZRAM_INTERVAL)) {
				if (ss->compact_finished == 0) {
					pr_err("run zram compact\n");
					run_compact_zram();
					ss->last_compact_jiffies = jiffies;
					ss->compact_count ++;
					ss->compact_finished = 1;
				}
			} else {
				pr_err("compact time inerval is small\n");
			}
		} else if ((ret & SS_FREE_MEM_LOW) && (ret & SS_ANON_PAGE_HIGH)
			&& !(ret & SS_CPU_LOAD_HIGH)) {
			if (!check_twice_interval(ss->last_swap_jiffies, SWAPMEM_INTERVAL)) {
				pr_err("swap memory interval is small\n");
				continue;
			}

			pr_err("start swap pages\n");
			/* swap out pages. */
			nr_pages = 0;
			swap_nr = 0;
			time_jiffies = jiffies;
			do {
				nr_pages += smart_shrink_swap_pages(SS_NR_SWAP_SIZE);
				swap_nr ++;
				msleep(20);
				if (kthread_should_stop())
					goto out;
				ret = get_memory_stat(ss);
			} while(!(ret & SS_NEED_COMPACT_SWAP)
					&& !(ret & SS_CPU_LOAD_HIGH)
					&& (ret & SS_FREE_MEM_LOW)
					&& (ret & SS_ANON_PAGE_HIGH)
					&& (swap_nr < MAX_MEM_PER_SWAP));
			time_jiffies = elapsed_jiffies(time_jiffies);

			if ((swap_nr > MAX_MEM_PER_SWAP)
				|| (ret & SS_CPU_LOAD_HIGH))
				factor ++;
			else
				factor = 1;

			ss->nr_swap_pages += nr_pages;
			ss->total_swap_time += time_jiffies;
			SSPRINT(("SWAP reclaimed pages=%d, time=%d ms\n",
					nr_pages, jiffies_to_msecs(time_jiffies)));
			ss->last_swap_jiffies = jiffies;
			ss->swap_count++;
		} else {
			factor ++;
		}
	}

out:
	tsk->flags &=
	    ~(PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD | PF_FREEZER_SKIP);
	return 0;
}

static int ss_wakeup_thread(struct smart_shrinker *ss)
{
	if (ss->ktask)
		up(&ss->wait);

	return 0;
}

static int ss_start_thread(struct smart_shrinker *ss)
{
	if (ss->ktask)
		return -EINVAL;

	ss->ktask = kthread_run(smart_swap_thread, NULL, "smart_shrink");
	if (IS_ERR(ss->ktask)) {
		pr_err("Failed to start thread\n");
		return -ENOMEM;
	}

	SSPRINT(("The smart_swap_thread start\n"));
	return 0;
}

static void ss_stop_thread(struct smart_shrinker *ss)
{
	if (!ss->ktask)
		return;

	ss_wakeup_thread(ss);
	kthread_stop(ss->ktask);
	ss->ktask = NULL;
	SSPRINT(("smart_swap_thread stopped\n"));
}

static ssize_t debug_sshrink_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d%%\n", debug_sshrink);
}

static ssize_t debug_sshrink_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	uint value;

	ret = kstrtouint(buf, 10, &value);
	if (ret)
		return ret;

	debug_sshrink = value;

	return len;
}

static ssize_t enable_swap_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", ss_info.enable_swap);
}

static ssize_t enable_swap_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	u16 enable;

	ret = kstrtou16(buf, 10, &enable);
	if (ret)
		return ret;

	if (enable) {
		ss_start_thread(&ss_info);
		ss_info.enable_swap = 1;
	} else {
		ss_info.enable_swap = 0;
		ss_stop_thread(&ss_info);
	}

	return len;
}

static ssize_t enable_shrinker_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", ss_info.enable_shrinker);
}

static ssize_t enable_shrinker_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	u16 enable;

	ret = kstrtou16(buf, 10, &enable);
	if (ret)
		return ret;

	if (enable)
		ss_info.enable_shrinker = 1;
	else
		ss_info.enable_shrinker = 0;

	return len;
}

static ssize_t idle_threshold_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d%%\n", ss_info.idle_threshold);
}

static ssize_t idle_threshold_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	u16 value;

	ret = kstrtou16(buf, 10, &value);
	if (ret)
		return ret;

	if ((value < 0) || (value > 100))
		return -EINVAL;

	ss_info.idle_threshold = value;

	return len;
}

static ssize_t compact_threshold_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d%%\n", ss_info.compact_threshold);
}

static ssize_t compact_threshold_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	long value;

	ret = kstrtol(buf, 10, &value);
	if (ret)
		return ret;

	if ((value < 0) || (value > 100))
		return -EINVAL;

	ss_info.compact_threshold = value;

	return len;
}

static ssize_t free_pages_min_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d pages\n", ss_info.free_pages_min);
}

static ssize_t free_pages_min_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	long value;

	ret = kstrtol(buf, 10, &value);
	if (ret)
		return ret;

	ss_info.free_pages_min = value / PAGE_SIZE;

	return len;
}

static ssize_t anon_pages_max_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d pages\n", ss_info.anon_pages_min);
}

static ssize_t anon_pages_max_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	u16 value;

	ret = kstrtou16(buf, 10, &value);
	if (ret)
		return ret;

	ss_info.anon_pages_min = value / PAGE_SIZE;

	return len;
}

static ssize_t shrink_min_vmpr_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d%%\n", ss_info.shrink_min_vmpr);
}

static ssize_t shrink_min_vmpr_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	u16 value;

	ret = kstrtou16(buf, 10, &value);
	if (ret)
		return ret;

	if ((value < 0) || (value > 100))
		return -EINVAL;

	ss_info.shrink_min_vmpr = value;

	return len;
}

static ssize_t shrink_max_vmpr_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d%%\n", ss_info.shrink_max_vmpr);
}

static ssize_t shrink_max_vmpr_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	u16 value;

	ret = kstrtou16(buf, 10, &value);
	if (ret)
		return ret;

	if ((value < 0) || (value > 100))
		return -EINVAL;

	ss_info.shrink_max_vmpr = value;

	return len;
}

static ssize_t sshrinker_stats_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	ssize_t size = 0;
	u32 kill_cnt = 0, shrink_cnt = 0;

	size += scnprintf(buf, PAGE_SIZE, "swap count: %u\n",
				  ss_info.swap_count);
	size += scnprintf(buf + size, PAGE_SIZE, "total swap time: %u ms\n",
				  ss_info.total_swap_time);
	size += scnprintf(buf + size, PAGE_SIZE, "total swap pages: %u\n",
				  ss_info.nr_swap_pages);
	size += scnprintf(buf + size, PAGE_SIZE, "compact count: %u\n",
				  ss_info.compact_count);
	size += scnprintf(buf + size, PAGE_SIZE, "shrink count: %u\n",
				  ss_info.shrink_count);
	lmk_get_shrink_count(&kill_cnt, &shrink_cnt);
	size += scnprintf(buf + size, PAGE_SIZE,
				  "lmk kill cnt: %u, shrink cnt %u\n",
				  kill_cnt, shrink_cnt);

	return size;
}

struct kobj_attribute attr_debug_sshrink = __ATTR_RW(debug_sshrink);
struct kobj_attribute attr_enable_swap = __ATTR_RW(enable_swap);
struct kobj_attribute attr_enable_shrinker = __ATTR_RW(enable_shrinker);
struct kobj_attribute attr_idle_threshold = __ATTR_RW(idle_threshold);
struct kobj_attribute attr_free_pages_min = __ATTR_RW(free_pages_min);
struct kobj_attribute attr_anon_pages_max = __ATTR_RW(anon_pages_max);
struct kobj_attribute attr_compact_threshold = __ATTR_RW(compact_threshold);
struct kobj_attribute attr_shrink_min_vmpr = __ATTR_RW(shrink_min_vmpr);
struct kobj_attribute attr_shrink_max_vmpr = __ATTR_RW(shrink_max_vmpr);
struct kobj_attribute attr_sshrinker_stats = __ATTR_RO(sshrinker_stats);

static struct attribute *ss_attrs[] = {
	&attr_debug_sshrink.attr,
	&attr_enable_swap.attr,
	&attr_enable_shrinker.attr,
	&attr_free_pages_min.attr,
	&attr_anon_pages_max.attr,
	&attr_idle_threshold.attr,
	&attr_compact_threshold.attr,
	&attr_shrink_min_vmpr.attr,
	&attr_shrink_max_vmpr.attr,
	&attr_sshrinker_stats.attr,
	NULL,
};

static struct attribute_group smart_shrinker_attr_group = {
	.attrs = ss_attrs,
};

static void create_ss_sysfs_attrs(struct smart_shrinker *ss)
{
	int err;
	struct kobject *kobj = NULL;

	kobj = kobject_create_and_add("smart_shrink", kernel_kobj);
	if (!kobj) {
		pr_err("Failed to create ss sysfs root node\n");
		return;
	}

	err = sysfs_create_group(kobj, &smart_shrinker_attr_group);
	if (err) {
		pr_err("Failed to create ss sysfs attrs.\n");
		kobject_put(kobj);
		return;
	}

	ss->kobj = kobj;
}

static int ss_vmpressure_notifier(struct notifier_block *nb,
			unsigned long action, void *data)
{
	static int high_vmpr = 0;
	static int mid_vmpr = 0;
	unsigned long pressure = action;

	if (!ss_info.enable_shrinker)
		return 0;

	if (!current_is_kswapd())
		return 0;

	SSPRINT(("the pressure is %ld\n", pressure));
	if (pressure >= ss_info.shrink_min_vmpr) {
		high_vmpr ++;
		mid_vmpr = 0;
		if (high_vmpr == 5) {
			queue_delayed_work(system_unbound_wq,
					&smart_shrinker_work, SSHRINKER_DELAY_TIME*HZ);
			high_vmpr = 0;
		}
	} else if (pressure > WAKE_THREAD_VMPR_PERCENT) {
		high_vmpr = 0;
		mid_vmpr ++;
		if (mid_vmpr == 5) {
			ss_wakeup_thread(&ss_info);
			mid_vmpr = 0;
		}
	} else {
		high_vmpr = 0;
		mid_vmpr = 0;
	}

	return 0;
}

static struct notifier_block vmpressure_nb = {
	.notifier_call = ss_vmpressure_notifier,
};

static void init_smart_shrinker_var(void)
{
	int total_mem = 0;

	sema_init(&ss_info.wait, 0);

	ss_info.idle_threshold = DEFAULT_CPUIDLE_THRESHOLD;
	ss_info.free_pages_min = DEFAULT_NR_FREE_PAGES;
	total_mem = get_hs_total_ram() / SZ_1G;
	if (total_mem >= 6)
		ss_info.anon_pages_min = DEFAULT_NR_ANON_PAGES * 3;
	else if (total_mem >= 4)
		ss_info.anon_pages_min = DEFAULT_NR_ANON_PAGES * 2;
	else
		ss_info.anon_pages_min = DEFAULT_NR_ANON_PAGES;

	ss_info.shrink_min_vmpr = DEFAULT_MEM_PRESSURE_MIN;
	ss_info.shrink_max_vmpr = DEFAULT_MEM_PRESSURE_MAX;
	ss_info.compact_threshold = DEFAULT_SWAPFULL_PERCENT;

	create_ss_sysfs_attrs(&ss_info);
}

static int __init smart_shrinker_init(void)
{
	init_smart_shrinker_var();

	vmpressure_notifier_register(&vmpressure_nb);
	return 0;
}

static void __exit smart_shrinker_exit(void)
{
	if (!ss_info.kobj)
		return;
	kobject_put(ss_info.kobj);

	vmpressure_notifier_unregister(&vmpressure_nb);
}

module_init(smart_shrinker_init);
module_exit(smart_shrinker_exit);
