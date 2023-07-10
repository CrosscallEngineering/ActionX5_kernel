/* Copyright (c) 2015 The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "--PM-SGM2540: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <../power/supply/qcom/smb-lib.h>
#include <linux/his_debug_base.h>

struct sgm2540_chip {
	struct platform_device *pdev;
	struct delayed_work chg_redet_work;
	struct power_supply		*batt_psy;
	//struct power_supply		*usb_psy;
	struct notifier_block	nb;
	struct delayed_work	usb_init_work;
	u32             sgm2540_af_gpio;
	u32             sgm2540_bf_gpio;
	int             sgm2540_af_irq;
	int             sgm2540_bf_irq;
	u32             b_channel_gpio;
	u32             usb_switch_gpio;
	int				a_otg_present;
	int				b_otg_present;
	int battery_fcc_ma;
	unsigned long inputs;
};

enum sgm2540_mode
{
    sgm2540_default_mode = 0,
    magcon_otg_mode,
    usb_otg_mode,
    magcon_chg_mode,
    usb_chg_mode,
    magcon_usb_chg_mode,
};

#define AF_BUS_VLD	0
#define BF_BUS_VLD	1
#define A_OTG_VLD	2
#define B_OTG_VLD	3
#define SGM_INIT	7

extern struct smb_charger *global_smb_charger;
extern int smb5_configure_micro_usb(struct smb_charger *chg);

static struct sgm2540_chip *global_chip = NULL;
static char *doubleinput[2] = { "AF_BF_EVENT=AF_BF_EVENT", NULL };

bool sgm2540_af_charger_ok(void)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!test_bit(AF_BUS_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_af_charger_ok);

bool sgm2540_af_gpio_status(void)
{
	bool af_status = false;

	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}
	af_status = !!gpio_get_value(global_chip->sgm2540_af_gpio);

	return af_status;
}
EXPORT_SYMBOL_GPL(sgm2540_af_gpio_status);

bool sgm2540_bf_charger_ok(void)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!test_bit(BF_BUS_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_bf_charger_ok);

bool sgm2540_af_otg_ok(void)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!test_bit(A_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_af_otg_ok);

bool sgm2540_bf_otg_ok(void)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!test_bit(B_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_bf_otg_ok);

bool sgm2540_double_charger_ok(void)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!(test_bit(AF_BUS_VLD, &global_chip->inputs)
		&& test_bit(BF_BUS_VLD, &global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_double_charger_ok);

bool sgm2540_af_online(void)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!(test_bit(A_OTG_VLD, &(global_chip->inputs))
		|| test_bit(AF_BUS_VLD, &(global_chip->inputs)));
}
EXPORT_SYMBOL_GPL(sgm2540_af_online);

bool sgm2540_bf_online(void)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!(test_bit(B_OTG_VLD, &(global_chip->inputs))
		|| test_bit(BF_BUS_VLD, &(global_chip->inputs)));
}
EXPORT_SYMBOL_GPL(sgm2540_bf_online);

void sgm2540_usb_switch(bool enable)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
	}

	gpio_direction_output(global_chip->usb_switch_gpio, enable);
	pr_buf_info("usb_switch_gpio %s\n", enable?"enable":"disable");
}
EXPORT_SYMBOL_GPL(sgm2540_usb_switch);

static irqreturn_t sgm2540_af_irq(int irq, void *data)
{
	struct sgm2540_chip *chip = data;

	if (!global_chip)
		return -EINVAL;

	if(test_bit(A_OTG_VLD, &(global_chip->inputs))
		|| test_bit(B_OTG_VLD, &(global_chip->inputs)))
		return IRQ_HANDLED;

	//change delay time to 70ms, to solve system shows blue and power-off caused by USB_SWITCH.
	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(70));

	return IRQ_HANDLED;
}

static irqreturn_t sgm2540_bf_irq(int irq, void *data)
{
	struct sgm2540_chip *chip = data;

	if (!global_chip)
		return -EINVAL;

	if(test_bit(A_OTG_VLD, &(global_chip->inputs))
		|| test_bit(B_OTG_VLD, &(global_chip->inputs)))
		return IRQ_HANDLED;

	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(30));

	return IRQ_HANDLED;
}

int sgm2540_a_otg_vbus_enable(bool enable)
{
	int ret = 1;

	if (!global_chip) {
		pr_buf_err("sgm2540:%s err,no chip\n", __func__);
		return -EINVAL;
	}

	if (enable)
		global_chip->a_otg_present = true;
	else
		global_chip->a_otg_present = false;

	if (test_bit(BF_BUS_VLD, &(global_chip->inputs))) {
		pr_buf_info("B charger work, a_otg_present = %d\n",
			global_chip->a_otg_present);
		return ret;
	}

	if (test_bit(B_OTG_VLD, &(global_chip->inputs))) {
		pr_buf_info("B OTG work, a_otg_present = %d\n",
			global_chip->a_otg_present);
		return ret;
	}

	if (enable) {
		free_irq(global_chip->sgm2540_af_irq, global_chip);
		disable_irq(global_chip->sgm2540_bf_irq);
		ret = gpio_direction_output(global_chip->sgm2540_af_gpio, 1);
		clear_bit(AF_BUS_VLD, &global_chip->inputs);
		clear_bit(BF_BUS_VLD, &global_chip->inputs);
	} else {
		ret = gpio_direction_input(global_chip->sgm2540_af_gpio);
		ret |= request_irq(global_chip->sgm2540_af_irq,
					  sgm2540_af_irq,
					  IRQF_TRIGGER_RISING |
					  IRQF_TRIGGER_FALLING,
					  "sgm2540_af_irq", global_chip);
		enable_irq_wake(global_chip->sgm2540_af_irq);
		enable_irq(global_chip->sgm2540_bf_irq);
		//sgm2540_af_irq(0, global_chip);
	}

	ret = gpio_direction_output(global_chip->usb_switch_gpio, enable);
	enable ? set_bit(A_OTG_VLD, &global_chip->inputs) : clear_bit(A_OTG_VLD, &global_chip->inputs);
	//kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);

	pr_buf_info("%s:magconn otg vbus channel %s\n",
		enable?"enable":"disable", !ret?"success":"fail");

	return ret;
}
EXPORT_SYMBOL_GPL(sgm2540_a_otg_vbus_enable);

int sgm2540_b_otg_vbus_enable(bool enable)
{
	int ret = 1;

	if (!global_chip) {
		pr_buf_info("no chip\n");
		return -EINVAL;
	}

	if (enable)
		global_chip->b_otg_present = true;
	else
		global_chip->b_otg_present = false;

	if (test_bit(AF_BUS_VLD, &(global_chip->inputs))) {
		pr_buf_info("A charger work, b_otg_present =%d\n",
			global_chip->b_otg_present);
		return ret;
	}

	if (test_bit(A_OTG_VLD, &(global_chip->inputs))) {
		pr_buf_info("A OTG work, b_otg_present = %d\n",
			global_chip->b_otg_present);
		return ret;
	}

	if (enable)
		ret = gpio_direction_output(global_chip->b_channel_gpio, 1);
	else
		ret = gpio_direction_output(global_chip->b_channel_gpio, 0);

	enable ? set_bit(B_OTG_VLD, &global_chip->inputs) : clear_bit(B_OTG_VLD, &global_chip->inputs);
	kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);

	pr_buf_info("%s:typec otg vbus channel %s\n", enable?"enable":"disable", !ret?"success":"fail");

	return ret;
}
EXPORT_SYMBOL_GPL(sgm2540_b_otg_vbus_enable);

static void sgm2540_chg_redet_work(struct work_struct *w)
{
	struct sgm2540_chip *chip = container_of(w, struct sgm2540_chip,
		chg_redet_work.work);
	bool af_status = !!gpio_get_value(chip->sgm2540_af_gpio);
	bool bf_status = !!gpio_get_value(chip->sgm2540_bf_gpio);
	bool need_redet = false;
	union power_supply_propval pval = {2000000, };	//default usb_in_icl is 2A
	union power_supply_propval val = {1, };

	pr_buf_info("input 0x%lx, af=%d, bf=%d\n", chip->inputs, af_status, bf_status);

	if(((af_status << AF_BUS_VLD) | (bf_status << BF_BUS_VLD))
		== (chip->inputs & (BIT(AF_BUS_VLD) | BIT(BF_BUS_VLD)))) {
		pr_buf_info("the same as last, do nothing\n");
		return;
	}

	if (af_status && !bf_status) {
		if (!global_smb_charger) {
			pr_buf_info("Could not get global_smb_charger\n");
			schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(1000));
			return;
		}
		smb5_configure_micro_usb(global_smb_charger);
	}

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");

		if (!chip->batt_psy) {
			dev_err(&chip->pdev->dev, "Could not get battery power_supply\n");
			return;
		}
	}

	gpio_direction_output(chip->b_channel_gpio, bf_status && (!af_status));

	if(test_bit(AF_BUS_VLD, &chip->inputs) && test_bit(BF_BUS_VLD, &chip->inputs)
		&& ((!af_status && bf_status) || (af_status && !bf_status))) {
		/*double charger to single charger*/
		need_redet = true;
		pr_buf_info("sgm2540:need to redet charger type\n");
		goto set_afbf_flag;
	} else if(af_status && bf_status &&
		((test_bit(AF_BUS_VLD, &chip->inputs) && !test_bit(BF_BUS_VLD, &chip->inputs)) ||
		(!test_bit(AF_BUS_VLD, &chip->inputs) && test_bit(BF_BUS_VLD, &chip->inputs)))) {
		/*single charger to double charger*/
		pval.intval = 500 * 1000;
		pr_buf_info("double input, data not switch to new bus\n");
		goto set_afbf_flag;
	} else {
		gpio_direction_output(chip->usb_switch_gpio, af_status && (!bf_status));
set_afbf_flag:
		af_status ? set_bit(AF_BUS_VLD, &chip->inputs) : clear_bit(AF_BUS_VLD, &chip->inputs);
		bf_status ? set_bit(BF_BUS_VLD, &chip->inputs) : clear_bit(BF_BUS_VLD, &chip->inputs);

		if (need_redet) {
			val.intval = 1;
			gpio_direction_output(chip->usb_switch_gpio, af_status && (!bf_status));
			chip->batt_psy->desc->set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CHARGER_REDET, &val);

			pr_buf_info("charger need redet\n");
		}

		chip->batt_psy->desc->set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);

		kobject_uevent_env(&chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
		pr_buf_info("set fcc %d ma\n", pval.intval / 1000);
	}
}

static void sgm2540_hw_init(struct device *dev, struct sgm2540_chip *chip)
{
	bool af_status = !!gpio_get_value(chip->sgm2540_af_gpio);
	bool bf_status = !!gpio_get_value(chip->sgm2540_bf_gpio);

	pr_buf_info("af_status=%d, bf_status=%d\n", af_status, bf_status);
	if (af_status || bf_status)
		schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(6000));
}

static int sgm2540_parse_dt(struct device *dev,
		struct sgm2540_chip *chip)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	/* sgm2540_af_gpio */
	ret = of_get_named_gpio(np, "sgmicro,sgm2540-af-gpio", 0);
	chip->sgm2540_af_gpio = ret;
	if(!gpio_is_valid(chip->sgm2540_af_gpio)) {
		dev_err(dev, "invalid sgm2540_af gpio. ret=%d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->sgm2540_af_gpio, "sgm2540-af-gpio");
	if (ret < 0) {
		dev_err(dev,
			"gpio %d request failed. ret=%d\n", chip->sgm2540_af_gpio, ret);
		return ret;
	}

	/* sgm2540_bf_gpio */
	ret = of_get_named_gpio(np, "sgmicro,sgm2540-bf-gpio", 0);
	chip->sgm2540_bf_gpio = ret;
	if (!gpio_is_valid(chip->sgm2540_bf_gpio)) {
		dev_err(dev, "invalid sgm2540-bf gpio: ret=%d\n", ret);
	}

	ret = gpio_request(chip->sgm2540_bf_gpio, "sgm2540-bf-gpio");
	if (ret < 0) {
		dev_err(dev,
			"gpio %d request failed. ret=%d\n", chip->sgm2540_bf_gpio, ret);
		return ret;
	}

	/*b_channel_gpio*/
	ret = of_get_named_gpio(np, "sgmicro,b-channel-gpio", 0);
	chip->b_channel_gpio = ret;
	if(!gpio_is_valid(chip->b_channel_gpio)) {
		dev_err(dev, "invalid b_channel gpio: %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->b_channel_gpio, "sgm2540-b-channel-gpio");
	if (ret < 0) {
		dev_err(dev,
			"gpio %d request failed. ret=%d\n",
			chip->b_channel_gpio, ret);
		return ret;
	}

	/*usb_switch_gpio*/
	ret = of_get_named_gpio(np, "sgmicro,usb-switch-gpio", 0);
	chip->usb_switch_gpio = ret;
	if(!gpio_is_valid(chip->usb_switch_gpio)) {
		dev_err(dev, "invalid usb_switch gpio: %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->usb_switch_gpio, "sgm2540-usb-switch-gpio");
	if (ret < 0) {
		dev_err(dev,
			"gpio %d request failed. ret=%d\n",
			chip->usb_switch_gpio, ret);
		return ret;
	}

	return 0;
}

static ssize_t sgm2540_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	 int mode = 0;

	if (test_bit(A_OTG_VLD, &global_chip->inputs))
		mode = magcon_otg_mode;
	else if (test_bit(B_OTG_VLD, &global_chip->inputs))
		mode = usb_otg_mode;
	else if ((test_bit(AF_BUS_VLD, &global_chip->inputs))
		&& (test_bit(BF_BUS_VLD, &global_chip->inputs)))
		mode = magcon_usb_chg_mode;
	else if (test_bit(AF_BUS_VLD, &global_chip->inputs))
		mode = magcon_chg_mode;
	else if (test_bit(BF_BUS_VLD, &global_chip->inputs))
		mode = usb_chg_mode;
	else
		mode = sgm2540_default_mode;

	pr_buf_info("curent_mode=0x%x\n", mode);

	return sprintf(buf, "%d\n", mode);
}

static ssize_t typec_attached_state_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int attached_state = 0;
	bool bf_status = !!gpio_get_value(global_chip->sgm2540_bf_gpio);

	if (global_chip->b_otg_present || bf_status)
		attached_state = 1;
	else
		attached_state = 0;

	pr_buf_info("attached_state=0x%x\n", attached_state);

	return sprintf(buf, "%d\n", attached_state);
}

static ssize_t double_charger_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int double_charger_state = sgm2540_double_charger_ok();

	pr_buf_info("attached_state=0x%x\n", double_charger_state);

	return sprintf(buf, "%d\n", double_charger_state);
}

static DEVICE_ATTR(sgm2540_mode, S_IRUGO, sgm2540_mode_show, NULL);
static DEVICE_ATTR(typec_attached_state, S_IRUGO, typec_attached_state_show, NULL);
static DEVICE_ATTR(double_charger, S_IRUGO, double_charger_show, NULL);

static struct attribute *sgm_sys_node_attrs[] = {
	&dev_attr_sgm2540_mode.attr,
	&dev_attr_typec_attached_state.attr,
	&dev_attr_double_charger.attr,
	NULL,
};

static struct attribute_group sgm_sys_node_attr_group = {
	.attrs = sgm_sys_node_attrs,
};

static int sgm2540_debugfs_init(struct sgm2540_chip *chip)
{
	int ret;

	ret = his_register_sysfs_attr_group(&sgm_sys_node_attr_group);
	if (ret < 0)
		pr_buf_err("%s: Error create sgm_sys_node_attr_group %d\n", __func__, ret);

	return 0;
}

static int sgm2540_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sgm2540_chip *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->pdev = pdev;
	dev_set_drvdata(&pdev->dev, chip);
	if (pdev->dev.of_node) {
		ret = sgm2540_parse_dt(&pdev->dev, chip);
		if (ret) {
			dev_err(&pdev->dev,"sgm2540_parse_dt() err\n");
			goto err_parse_dt;
		}
	} else {
		dev_err(&pdev->dev, "No dts data\n");
		goto err_parse_dt;
	}

	global_chip = chip;
	INIT_DELAYED_WORK(&chip->chg_redet_work, sgm2540_chg_redet_work);

	chip->sgm2540_af_irq = gpio_to_irq(chip->sgm2540_af_gpio);
	chip->sgm2540_bf_irq = gpio_to_irq(chip->sgm2540_bf_gpio);

	ret = request_irq(chip->sgm2540_af_irq,
				  sgm2540_af_irq,
				  IRQF_TRIGGER_RISING |
				  IRQF_TRIGGER_FALLING,
				  "sgm2540_af_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for af\n");
		goto err_parse_dt;
	}

	ret |= request_irq(chip->sgm2540_bf_irq,
				  sgm2540_bf_irq,
				  IRQF_TRIGGER_RISING |
				  IRQF_TRIGGER_FALLING,
				  "sgm2540_bf_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for bf\n");
		goto err_parse_dt;
	}

	enable_irq_wake(chip->sgm2540_af_irq);
	enable_irq_wake(chip->sgm2540_bf_irq);

	set_bit(SGM_INIT, &chip->inputs);
	sgm2540_hw_init(&pdev->dev, chip);
	sgm2540_debugfs_init(chip);

	pr_buf_info("sgm2540_probe success\n");

	return 0;

err_parse_dt:
	devm_kfree(&pdev->dev,chip);

	return ret;
}

static void sgm2540_shutdown(struct platform_device *pdev)
{
	struct sgm2540_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->chg_redet_work);
	}
	global_chip = NULL;
}

static int sgm2540_remove(struct platform_device *pdev)
{
	struct sgm2540_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->chg_redet_work);
	}
	global_chip = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sgm2540_suspend(struct device *dev)
{
	struct sgm2540_chip *chip = dev_get_drvdata(dev);

	pr_buf_info("b_channel_gpio is %d\n",
		gpio_get_value(chip->b_channel_gpio));

	return 0;
}

static int sgm2540_resume(struct device *dev)
{
	struct sgm2540_chip *chip = dev_get_drvdata(dev);

	pr_buf_info("b_channel_gpio is %d\n",
		gpio_get_value(chip->b_channel_gpio));

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sgm2540_pm_ops, sgm2540_suspend,
			  sgm2540_resume);

static struct of_device_id sgm2540_match_table[] = {
	{	.compatible = "sgmicro,sgm2540",
	},
	{}
};

static struct platform_driver sgm2540_driver = {
	.driver		= {
		.name		= "sgm2540",
		.owner		= THIS_MODULE,
		.of_match_table	= sgm2540_match_table,
		.pm	= &sgm2540_pm_ops,
	},
	.probe		= sgm2540_probe,
	.remove		= sgm2540_remove,
	.shutdown 	= sgm2540_shutdown,
};

module_platform_driver(sgm2540_driver);
MODULE_DESCRIPTION("sgm2540");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sgm2540");
