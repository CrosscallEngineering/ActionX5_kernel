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
#define pr_fmt(fmt) "--PM-SGM: %s: " fmt, __func__

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
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include "smb5-lib.h"

struct sgm_chip {
	struct platform_device *pdev;
	struct delayed_work	chg_redet_work;
	struct delayed_work	ext_otg_detect_work;
	struct power_supply	*batt_psy;
	struct power_supply	*sgm_chg_psy;
	u32		usb_switch_gpio;
	u32		mag_chg_gpio;
	int		mag_chg_irq;
	u32		ext_otg_gpio;
	int		ext_otg_irq;
	u32		typec_chg_gpio;
	int		typec_chg_irq;
	int		ext_otg_present;
	bool		typec_otg_present;
	unsigned long inputs;
};

#define MAG_BUS_VLD		0
#define TYPEC_BUS_VLD	1
#define EXT_OTG_VLD		2
#define TYPEC_OTG_VLD	3
#define MAG_USB_VLD		4
#define SGM_INIT			7

enum sgm_mode
{
    sgm_default_mode = 0,
    ext_otg_mode,
    usb_otg_mode,
    magcon_chg_mode,
    usb_chg_mode,
    magcon_usb_chg_mode,
};

static struct sgm_chip *global_chip = NULL;
static char *doubleinput[2] = { "AF_BF_EVENT=AF_BF_EVENT", NULL };
static bool ext_otg_status = false;

extern struct smb_charger *global_smb_charger;

bool sgm_mag_charger_ok(void)
{
	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(MAG_BUS_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_mag_charger_ok);

bool sgm_typec_charger_ok(void)
{
	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(TYPEC_BUS_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_typec_charger_ok);

bool sgm_mag_usb_ok(void)
{
	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(MAG_USB_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_mag_usb_ok);

bool sgm_ext_otg_ok(void)
{
	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(EXT_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_ext_otg_ok);

bool sgm_typec_otg_ok(void)
{
	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(TYPEC_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_typec_otg_ok);

bool sgm_typec_port_online(void)
{
	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!(test_bit(TYPEC_OTG_VLD, &(global_chip->inputs))
		|| test_bit(TYPEC_BUS_VLD, &(global_chip->inputs)));
}
EXPORT_SYMBOL_GPL(sgm_typec_port_online);

static bool sgm_double_charger_ok(void)
{
	if (!global_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!(test_bit(MAG_BUS_VLD, &global_chip->inputs)
		&& test_bit(TYPEC_BUS_VLD, &global_chip->inputs));
}

static irqreturn_t sgm_mag_chg_irq(int irq, void *data)
{
	struct sgm_chip *chip = data;

	pr_buf_info("sgm: trigger.\n");

	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

static irqreturn_t sgm_typec_chg_irq(int irq, void *data)
{
	struct sgm_chip *chip = data;

	pr_buf_info("sgm: trigger.\n");

	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

static irqreturn_t sgm_ext_otg_irq(int irq, void *data)
{
	struct sgm_chip *chip = data;

	pr_buf_info("sgm: triggered\n");

	schedule_delayed_work(&chip->ext_otg_detect_work, msecs_to_jiffies(50));

	return IRQ_HANDLED;
}

int sgm_usb_gpio_switch(bool enable)
{
	int ret = 1;

	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}

	if (enable) {
		if (test_bit(EXT_OTG_VLD, &(global_chip->inputs))) {
			pr_buf_info("EXT OTG working, usb_switch can't set %s\n",
				enable ? "enable" : "disable");
			return ret;
		}

		if (test_bit(TYPEC_OTG_VLD, &(global_chip->inputs))) {
			pr_buf_info("TYPEC OTG working, usb_switch can't set %s\n",
				enable ? "enable" : "disable");
			return ret;
		}

		if (test_bit(TYPEC_BUS_VLD, &(global_chip->inputs))) {
			pr_buf_info("TYPEC BUS working, usb_switch can't set %s\n",
				enable ? "enable" : "disable");
			return ret;
		}
	}

	ret = gpio_direction_output(global_chip->usb_switch_gpio, enable);

	enable ? set_bit(MAG_USB_VLD, &global_chip->inputs) : clear_bit(MAG_USB_VLD, &global_chip->inputs);

	pr_buf_info("sgm: %s MAG usb_switch_gpio %s\n",
		enable ? "enable" : "disable", !ret ? "success" : "fail");

	return ret;
}
EXPORT_SYMBOL_GPL(sgm_usb_gpio_switch);

int sgm_typec_otg_usb_switch(bool enable)
{
	int ret = 0;

	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}

	if (enable)
		global_chip->typec_otg_present = true;
	else
		global_chip->typec_otg_present = false;

	if (test_bit(EXT_OTG_VLD, &(global_chip->inputs))) {
		pr_buf_info("EXT OTG working, usb_switch can't set %s\n",
			enable ? "enable" : "disable");
		kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
		return ret;
	}

	if (enable) {
		disable_irq(global_chip->typec_chg_irq);
		ret = gpio_direction_output(global_chip->usb_switch_gpio, false);
		clear_bit(TYPEC_BUS_VLD, &global_chip->inputs);
	} else {
		enable_irq(global_chip->typec_chg_irq);
		//enable_irq_wake(global_chip->typec_chg_irq);
	}

	enable ? set_bit(TYPEC_OTG_VLD, &global_chip->inputs) : clear_bit(TYPEC_OTG_VLD, &global_chip->inputs);
	kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);

	pr_buf_info("sgm: %s usb_switch_gpio %s\n",
		enable ? "enable" : "disable", !ret ? "success" : "fail");

	return ret;
}
EXPORT_SYMBOL_GPL(sgm_typec_otg_usb_switch);

static int sgm_ext_otg_usb_switch(bool enable)
{
	int ret = 1;

	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}

	if (test_bit(TYPEC_OTG_VLD, &(global_chip->inputs))) {
		pr_buf_info("TYPEC OTG working, usb_switch can't set %s\n",
			enable ? "enable" : "disable");
		return ret;
	}

	if (enable) {
		disable_irq(global_chip->mag_chg_irq);
		clear_bit(MAG_BUS_VLD, &global_chip->inputs);
	} else {
		enable_irq(global_chip->mag_chg_irq);
		//enable_irq_wake(global_chip->mag_chg_irq);
	}

	ret = gpio_direction_output(global_chip->usb_switch_gpio, enable);
	enable ? set_bit(EXT_OTG_VLD, &global_chip->inputs) : clear_bit(EXT_OTG_VLD, &global_chip->inputs);

	pr_buf_info("sgm: %s usb_switch_gpio %s\n",
		enable ? "enable" : "disable", !ret ? "success" : "fail");

	return ret;
}

static int sgm_check_power_supply_psy(struct sgm_chip *chip)
{
	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy) {
			dev_err(&chip->pdev->dev, "Could not get battery power_supply\n");
			return -ENODEV;
		}
	}

	if (!chip->sgm_chg_psy) {
		chip->sgm_chg_psy= power_supply_get_by_name("sgm_charger");
		if (!chip->sgm_chg_psy) {
			dev_err(&chip->pdev->dev, "Could not get sgm_charger power_supply\n");
			return -ENODEV;
		}
	}

	if (!global_smb_charger)
		return  -ENODEV;

	return 0;
}

static void sgm_ext_otg_detect_work(struct work_struct *work)
{
	struct sgm_chip *chip = container_of(work, struct sgm_chip,
		ext_otg_detect_work.work);
	union power_supply_propval val = {0, };
	bool otg_id_status = !!gpio_get_value(chip->ext_otg_gpio);

	pr_buf_info("sgm: !otg_id=%d otg_mode=%d\n", !otg_id_status, ext_otg_status);

	if (!otg_id_status != ext_otg_status)
	{
		if (!sgm_typec_otg_ok() && !global_smb_charger->otg_present) {
			pr_buf_info("sgm: switch to ext otg mode = %d\n", !otg_id_status);
			val.intval = !otg_id_status;

			//switch usb D+ D- to ext otg side first.
			if (!otg_id_status)
				sgm_ext_otg_usb_switch(true);

			//ask SGM41512 for a 5V vbus power supply.
			if (!chip->sgm_chg_psy) {
				if (sgm_check_power_supply_psy(chip))
					return;
			}

			power_supply_set_property(chip->sgm_chg_psy, POWER_SUPPLY_PROP_USB_OTG, &val);

			//switch D+ D- to default and enable mag irq after disable otg mode.
			if (otg_id_status)
				sgm_ext_otg_usb_switch(false);

			if (global_smb_charger->usb_psy)
				power_supply_changed(global_smb_charger->usb_psy);
		}
		ext_otg_status = !otg_id_status;
	}
}

static void sgm_chg_redet_work(struct work_struct *w)
{
	struct sgm_chip *chip = container_of(w, struct sgm_chip,
		chg_redet_work.work);
	bool ext_vbus_status = !gpio_get_value(chip->mag_chg_gpio);
	bool typec_vbus_status = !gpio_get_value(chip->typec_chg_gpio);
	bool need_redet = false;
	union power_supply_propval pval = {2000000, };	//default usb_in_icl is 2A
	union power_supply_propval val = {0, };

	pr_buf_info("sgm: input 0x%lx, ext_status=%d typec_status=%d\n", chip->inputs, ext_vbus_status, typec_vbus_status);

	if(((ext_vbus_status << MAG_BUS_VLD) | (typec_vbus_status << TYPEC_BUS_VLD))
		== (chip->inputs & (BIT(MAG_BUS_VLD) | BIT(TYPEC_BUS_VLD)))) {
		pr_buf_info("sgm: the same as last, do nothing\n");
		return;
	}

	if (sgm_check_power_supply_psy(chip))
		return;

	/*If ext otg is working, ignore ext vbus status.*/
	if (sgm_ext_otg_ok() && ext_vbus_status)
		ext_vbus_status = 0;

	if (sgm_typec_otg_ok() && typec_vbus_status)
		typec_vbus_status = 0;

	/*IF ext otg is working, we need reload the ext otg device*/
	if (sgm_ext_otg_ok() && typec_vbus_status) {
		power_supply_set_property(chip->sgm_chg_psy, POWER_SUPPLY_PROP_USB_OTG, &val);
		ext_otg_status = false;
		enable_irq(global_chip->mag_chg_irq);
		pr_buf_err(" %s enable mag_chg_irq\n",__func__);
		cancel_delayed_work_sync(&chip->ext_otg_detect_work);
		//schedule work, after typec status is readly.
		schedule_delayed_work(&chip->ext_otg_detect_work, msecs_to_jiffies(500));
	}

	if (test_bit(MAG_BUS_VLD, &chip->inputs) && test_bit(TYPEC_BUS_VLD, &chip->inputs)
		&& ((!ext_vbus_status && typec_vbus_status) || (ext_vbus_status && !typec_vbus_status))) {
		/*double charger to single charger*/
		need_redet = true;
		pr_buf_info("sgm: need to redet charger type\n");
		goto set_chg_flag;
	} else if(ext_vbus_status && typec_vbus_status &&
		((test_bit(MAG_BUS_VLD, &chip->inputs) && !test_bit(TYPEC_BUS_VLD, &chip->inputs)) ||
		(!test_bit(MAG_BUS_VLD, &chip->inputs) && test_bit(TYPEC_BUS_VLD, &chip->inputs)))) {
		/*single charger to double charger*/
		pval.intval = 500 * 1000;
		if (test_bit(MAG_BUS_VLD, &chip->inputs) && !test_bit(TYPEC_BUS_VLD, &chip->inputs))
			need_redet = true;
		pr_buf_info("sgm: double input, data not switch to new bus, need_redet %d\n", need_redet);
		goto set_chg_flag;
	} else {
set_chg_flag:
		ext_vbus_status ? set_bit(MAG_BUS_VLD, &chip->inputs) : clear_bit(MAG_BUS_VLD, &chip->inputs);
		typec_vbus_status ? set_bit(TYPEC_BUS_VLD, &chip->inputs) : clear_bit(TYPEC_BUS_VLD, &chip->inputs);

		if (need_redet) {
			if (ext_vbus_status) {
				power_supply_set_property(chip->sgm_chg_psy,
					POWER_SUPPLY_PROP_FORCE_RECHARGE, &pval);
			}

			if (!sgm_ext_otg_ok()  && (!ext_vbus_status)) {
				gpio_direction_output(chip->usb_switch_gpio, false);
				smblib_rerun_apsd_if_required(global_smb_charger);
				pr_buf_info("sgm: charger need redet, usb_switch_gpio to low & rerun apsd.\n");
			}
		}

		power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
		power_supply_set_property(chip->sgm_chg_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &pval);

		kobject_uevent_env(&chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
		pr_buf_info("sgm: set fcc %d ma\n", pval.intval / 1000);
	}

	return;
}

static void sgm_hw_init(struct device *dev, struct sgm_chip *chip)
{
	schedule_delayed_work(&chip->ext_otg_detect_work, msecs_to_jiffies(5500));
	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(5000));
}

static int sgm_parse_dt(struct device *dev,
		struct sgm_chip *chip)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	/* mag-chg-flag-gpio */
	ret = of_get_named_gpio(np, "sgm,mag-chg-flag-gpio", 0);
	chip->mag_chg_gpio = ret;
	if(!gpio_is_valid(chip->mag_chg_gpio)) {
		dev_err(dev, "invalid mag-chg-flag-gpio. ret = %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->mag_chg_gpio, "sgm-,mag-chg-flag-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->mag_chg_gpio, ret);
		return ret;
	}


	/*ext-otg-id-gpio*/
	ret = of_get_named_gpio(np, "sgm,ext-otg-id-gpio", 0);
	chip->ext_otg_gpio = ret;
	if(!gpio_is_valid(chip->ext_otg_gpio)) {
		dev_err(dev, "invalid ext_otg_gpio gpio: %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->ext_otg_gpio, "sgm-ext-otg-id-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->ext_otg_gpio, ret);
		return ret;
	}

	/*typec-chg-flag-gpio*/
	ret = of_get_named_gpio(np, "sgm,typec-chg-flag-gpio", 0);
	chip->typec_chg_gpio = ret;
	if(!gpio_is_valid(chip->typec_chg_gpio))
		dev_err(dev, "invalid typec_chg_gpio rc = %d\n", ret);

	ret = gpio_request(chip->typec_chg_gpio, "sgm-typec-chg-flag-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->typec_chg_gpio, ret);

	/*usb-switch-flag-gpio*/
	ret = of_get_named_gpio(np, "sgm,usb-switch-flag-gpio", 0);
	chip->usb_switch_gpio= ret;
	if(!gpio_is_valid(chip->usb_switch_gpio))
		dev_err(dev, "invalid usb-switch-flag-gpio rc = %d\n", ret);

	ret = gpio_request(chip->usb_switch_gpio, "sgm-usb-switch-flag-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->usb_switch_gpio, ret);

	return 0;
}

static ssize_t sgm_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	 int mode = 0;

	if (test_bit(EXT_OTG_VLD, &global_chip->inputs))
		mode = ext_otg_mode;
	else if (test_bit(TYPEC_OTG_VLD, &global_chip->inputs))
		mode = usb_otg_mode;
	else if ((test_bit(MAG_BUS_VLD, &global_chip->inputs))
		&& (test_bit(TYPEC_BUS_VLD, &global_chip->inputs)))
		mode = magcon_usb_chg_mode;
	else if (test_bit(MAG_BUS_VLD, &global_chip->inputs))
		mode = magcon_chg_mode;
	else if (test_bit(TYPEC_BUS_VLD, &global_chip->inputs))
		mode = usb_chg_mode;
	else
		mode = sgm_default_mode;

	pr_buf_info("sgm: global_chip->inputs=0x%x\n", global_chip->inputs);

	return sprintf(buf, "%d\n", mode);
}

static ssize_t typec_attached_state_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int attached_state = 0;
	bool typec_vbus_status = !gpio_get_value(global_chip->typec_chg_gpio);

	if (global_chip->typec_otg_present || typec_vbus_status)
		attached_state = 1;
	else
		attached_state = 0;

	pr_buf_info("attached_state=0x%x\n", attached_state);

	return sprintf(buf, "%d\n", attached_state);
}

static ssize_t double_charger_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int double_charger_state = sgm_double_charger_ok();

	pr_buf_info("attached_state=0x%x\n", double_charger_state);

	return sprintf(buf, "%d\n", double_charger_state);
}

static DEVICE_ATTR(sgm2540_mode, S_IRUGO, sgm_mode_show, NULL);
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

static int sgm_debugfs_init(struct sgm_chip *chip)
{
	int ret;

	ret = his_register_sysfs_attr_group(&sgm_sys_node_attr_group);
	if (ret < 0)
		pr_buf_err("Error create sgm_sys_node_attr_group %d\n", ret);

	return 0;
}

static int sgm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sgm_chip *chip;

	//pr_buf_info("sgm:sgm_probe start\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->pdev = pdev;
	dev_set_drvdata(&pdev->dev, chip);
	if (pdev->dev.of_node) {
		ret = sgm_parse_dt(&pdev->dev, chip);
		if (ret) {
			dev_err(&pdev->dev,"%s: sgm_parse_dt() err\n", __func__);
			goto err_parse_dt;
		}
	} else {
		dev_err(&pdev->dev, "No dts data\n");
		goto err_parse_dt;
	}

	global_chip = chip;
	INIT_DELAYED_WORK(&chip->chg_redet_work, sgm_chg_redet_work);
	INIT_DELAYED_WORK(&chip->ext_otg_detect_work, sgm_ext_otg_detect_work);

	chip->typec_chg_irq = gpio_to_irq(chip->typec_chg_gpio);
	chip->mag_chg_irq = gpio_to_irq(chip->mag_chg_gpio);
	chip->ext_otg_irq = gpio_to_irq(chip->ext_otg_gpio);

	ret = request_threaded_irq(chip->typec_chg_irq,
			NULL,
			sgm_typec_chg_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"sgm_typec_chg_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for typec chg det\n");
		goto err_parse_dt;
	}

	ret = request_threaded_irq(chip->mag_chg_irq,
			NULL,
			sgm_mag_chg_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"sgm_mag_chg_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for sgm_mag_chg\n");
		goto err_parse_dt;
	}

	ret = request_threaded_irq(chip->ext_otg_irq,
			NULL,
			sgm_ext_otg_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"sgm_ext_otg_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for ext otg fault\n");
		goto err_parse_dt;
	}

	enable_irq_wake(chip->typec_chg_irq);
	enable_irq_wake(chip->mag_chg_irq);
	enable_irq_wake(chip->ext_otg_irq);

	set_bit(SGM_INIT, &chip->inputs);
	sgm_hw_init(&pdev->dev, chip);

	sgm_debugfs_init(chip);

	pr_buf_info("sgm:sgm_probe success\n");
	return 0;

err_parse_dt:
	devm_kfree(&pdev->dev,chip);
	return ret;
}

static void sgm_shutdown(struct platform_device *pdev)
{
	struct sgm_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->ext_otg_detect_work);
		cancel_delayed_work_sync(&chip->chg_redet_work);
	}

	global_chip = NULL;
}

static int sgm_remove(struct platform_device *pdev)
{
	struct sgm_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->ext_otg_detect_work);
		cancel_delayed_work_sync(&chip->chg_redet_work);
	}

	global_chip = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sgm_suspend(struct device *dev)
{
	//struct sgm_chip *chip  = dev_get_drvdata(dev);

	return 0;
}

static int sgm_resume(struct device *dev)
{
	//struct sgm_chip *chip  = dev_get_drvdata(dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sgm_pm_ops, sgm_suspend,
			  sgm_resume);

static struct of_device_id sgm_match_table[] = {
	{	.compatible = "sgmicro,double-chg",
	},
	{}
};

static struct platform_driver sgm_driver = {
	.driver		= {
		.name		= "sgm",
		.owner		= THIS_MODULE,
		.of_match_table	= sgm_match_table,
		.pm	= &sgm_pm_ops,
	},
	.probe		= sgm_probe,
	.remove		= sgm_remove,
	.shutdown 	= sgm_shutdown,
};

module_platform_driver(sgm_driver);

MODULE_DESCRIPTION("SGM Double Charger Driver");
MODULE_AUTHOR("lishaoxiang@hisense.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sgm");
