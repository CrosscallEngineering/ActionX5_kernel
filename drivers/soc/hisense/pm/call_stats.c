#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/his_debug_base.h>


static bool in_call_status;

bool hs_get_phone_call_status(void)
{  
  return  in_call_status;
}
EXPORT_SYMBOL(hs_get_phone_call_status);

static ssize_t phone_call_stats_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	pr_debug("in_call_status %d\n", in_call_status);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", in_call_status);

	return ret;
}

static ssize_t phone_call_stats_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int ret = 0;
	long val = 0;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		pr_err("%s: kstrtol is wrong!\n", __func__);
		return -EINVAL;
	}

	pr_err("Set debug call stats %ld\n", val);
	if (val)
		in_call_status = true;
	else
		in_call_status = false;

	return len;
}
static struct kobj_attribute call_stats_attrs = __ATTR_RW(phone_call_stats);

static int __init init_phone_call_stats(void)
{
	pr_err("%s(%d) --**\n", __func__, __LINE__);
	in_call_status = false;
	his_register_sysfs_attr(&call_stats_attrs.attr);

	return 0;
}
late_initcall(init_phone_call_stats);

