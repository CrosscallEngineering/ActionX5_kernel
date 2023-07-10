
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/his_debug_base.h>

extern void exec_restart_modem_ctrl(void);

static ssize_t modem_restart_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int value;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("debug_store: sscanf is wrong!\n");
		return -EINVAL;
	}

	pr_info("modem will be restart by force .\n");
	exec_restart_modem_ctrl();

	return len;
}

static ssize_t modem_restart_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static DEVICE_ATTR(modem_restart, S_IWUSR | S_IWGRP | S_IRUGO,
		modem_restart_show, modem_restart_store);

static int __init modem_restart_control_init(void)
{
	int ret;

	ret = his_register_sysfs_attr(&dev_attr_modem_restart.attr);
	if (ret < 0) {
		pr_err("Error creating modem_restart sysfs node\n");
		return -1;
	}

	pr_info("%s: OK.\n", __func__);
	return 0;
}
late_initcall(modem_restart_control_init);
