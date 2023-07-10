/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#include <linux/of_gpio.h>

int gpio_num;
static int hall_inter_num;
char hall_state;
DECLARE_WAIT_QUEUE_HEAD(hall_wait_queue);
static int hall_debug;
#define HALL_DEBUG(x...) ({if (hall_debug) printk(x); })
static atomic_t state_cnt = ATOMIC_INIT(0);
//static struct regulator *pldo;
//struct wake_lock hall_wakelock;
struct wakeup_source hall_wake_src;
static wait_queue_head_t hall_wait_q;
struct pinctrl *pinctrl_hall;
struct pinctrl_state *pin_active;

static irqreturn_t hall_interrupt_handler(int irq, void *dev_id)
{
	char state = gpio_get_value(gpio_num)?0:1;
	if (hall_state == state)
		return IRQ_HANDLED;

	hall_state = state;
	HALL_DEBUG("%s  hall state %d, state_cnt %d\n", __func__, hall_state, atomic_read(&state_cnt));
	wake_up(&hall_wait_q);
	//wake_lock_timeout(&hall_wakelock, HZ);
	if (!(hall_wake_src.active) ) {
		__pm_wakeup_event(&hall_wake_src, jiffies_to_msecs(HZ));
	}

	return IRQ_HANDLED;
}



static int hall_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	HALL_DEBUG("%s\n", __func__);

	return ret;
}

static int hall_release(struct inode *inode, struct file *filp)
{
	return 0;
}

unsigned int hall_poll(struct file *filep, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	static char state;
	if (hall_state != state) {
		mask = POLLIN | POLLRDNORM;
		state = hall_state;
	}
	HALL_DEBUG("%s\n", __func__);
	poll_wait(filep, &hall_wait_q, wait);
	HALL_DEBUG("%s wait complete\n", __func__);
	return mask;
}

ssize_t hall_read(struct file *file, char __user *buf,
			  size_t count, loff_t *pos)
{

	const char __user *start = buf;
	int res = 0;

	if (copy_to_user(buf, &hall_state, sizeof(char))) {
		printk(KERN_ERR "return hall state error!\n");
		res = -EFAULT;
	} else {
		printk("%s, read state %d\n", __func__, hall_state);
	}

	res = buf - start;
	return res;
}

static int hall_pinctrl_init(struct platform_device *pdev)
{
	int err = 0;
	pinctrl_hall = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pinctrl_hall)) {
		HALL_DEBUG("Failed to get pinctrl\n");
		return 0;//PTR_ERR(pinctrl_hall);
	}
	pin_active =
	pinctrl_lookup_state(pinctrl_hall, "hall_int_active");
	if (IS_ERR_OR_NULL(pin_active)) {
		HALL_DEBUG("Failed to look up active state\n");
		return 0;//PTR_ERR(pin_active);
	} else {
		err = pinctrl_select_state(pinctrl_hall, pin_active);
		if (err) {
			HALL_DEBUG("Can't select pinctrl active state\n");
			return 0;//err;
		}
	}
	return 0;
}
static const struct file_operations hall_fops = {
	.owner = THIS_MODULE,
	.poll = hall_poll,
	.open = hall_open,
	.read = hall_read,
	.release = hall_release,
};

static struct miscdevice hall_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hall",
	.fops = &hall_fops
};

static ssize_t hall_int_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char state = gpio_get_value(gpio_num)?0:1;
	return snprintf(buf, 20,
		"int_state = %d\n", state);
}
static DEVICE_ATTR(hall_int, 0600, hall_int_show, NULL);
static struct attribute *attributes_hall[] = {
	&dev_attr_hall_int.attr,
	NULL
};
static struct attribute_group attribute_group_hall = {
	.attrs = attributes_hall
};

static int hall_probe(struct platform_device *pdev)
{
	struct device_node *node;

	int rc = 0;

	node = pdev->dev.of_node;
	if (node == NULL)
		return -ENODEV;

	gpio_num = of_get_named_gpio_flags(node, "hall-gpio", 0, NULL);

	if (gpio_num < 0) {
		return -EPERM;
	}
	HALL_DEBUG("read gpio num %d\n", gpio_num);

	/*pldo = regulator_get(&pdev->dev,
					"vdd");

	if (IS_ERR(pldo)) {
		printk("error: No regulator specified for hall\n");
		goto fail;
	}*/

	rc = hall_pinctrl_init(pdev);
	if (rc)
		goto fail;
	gpio_request(gpio_num, "hall");
	gpio_direction_input(gpio_num);

	hall_inter_num = gpio_to_irq(gpio_num);
	HALL_DEBUG("hall interrupt num is %d\n", hall_inter_num);
	init_waitqueue_head(&hall_wait_q);
	//wake_lock_init(&hall_wakelock,
	//	       WAKE_LOCK_SUSPEND, "hall_wakelock");
	memset(&hall_wake_src, 0, sizeof(hall_wake_src));
	hall_wake_src.name = "hall_wakelock";	
	wakeup_source_add(&hall_wake_src);

	//rc = regulator_enable(pldo);

	if (hall_inter_num) {
		rc = request_irq(hall_inter_num, hall_interrupt_handler,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "hall", NULL);
		if (rc) {
			printk(KERN_ERR "request hall interrupt error!\n");
			goto fail;
		}
		enable_irq_wake(hall_inter_num);
	}

	rc = misc_register(&hall_dev);
	if (rc < 0)
		goto fail;
	hall_state = gpio_get_value(gpio_num)?0:1;

	rc = sysfs_create_group(&((hall_dev.this_device)->kobj),
								&attribute_group_hall);
	if(rc)
			goto fail;
	printk("hall driver probed\n");
	return 0;

fail:
	return rc;
}

static int hall_remove(struct platform_device *plat)
{
	free_irq(hall_inter_num, NULL);
	HALL_DEBUG("%s release irq %d\n", __func__, hall_inter_num);

	//if (pldo)
	//	regulator_disable(pldo);
	disable_irq_wake(hall_inter_num);
	gpio_free(gpio_num);
	misc_deregister(&hall_dev);
	printk("Removing hall driver\n");
	return 0;
}

static struct of_device_id hall_match_table[] = {
	{
		.compatible = "hisense,hall-device",
	}
};

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.remove = hall_remove,
	.driver = {
		.name = "hall_driver",
		.owner = THIS_MODULE,
		.of_match_table = hall_match_table,
	},
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Driver to report hall state.");
MODULE_VERSION("1.00");
MODULE_AUTHOR("lizengbo@hisensecom.com");
module_param(hall_debug, int, S_IRUGO | S_IWUSR | S_IWGRP);

module_init(hall_init);
module_exit(hall_exit);
