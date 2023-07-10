/*
 * drivers/input/touchscreen/sitronix_i2c_touch.c
 *
 * Touchscreen driver for Sitronix (I2C bus)
 *
 * Copyright (C) 2011 Sitronix Technology Co., Ltd.
 *	Rudy Huang <rudy_huang@sitronix.com.tw>
 */
/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h> // to be compatible with linux kernel 3.2.15
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/path.h>
#include <linux/namei.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_panel.h>

#include "sitronix_ts_custom_func.h"
#include "sitronix_ts.h"
#include "../ts_func_test.h"
#include <linux/productinfo.h>

//#define SITRONIX_SWAP_XY

#define DRIVER_AUTHOR "Sitronix, Inc."
#define DRIVER_NAME "sitronix"
#define DRIVER_DESC "Sitronix I2C touch"
#define DRIVER_DATE "20181115"
#define DRIVER_MAJOR 2
#define DRIVER_MINOR 11
#define DRIVER_PATCHLEVEL 1

#define MAX_BUTTONS 4
MODULE_AUTHOR("Petitk Kao<petitk_kao@sitronix.com.tw>");
MODULE_DESCRIPTION("Sitronix I2C multitouch panels");
MODULE_LICENSE("GPL");

#define SITRONIX_TOUCH_DRIVER_VERSION 0x03
#define SITRONIX_I2C_TOUCH_DRV_NAME "sitronix"
#define SITRONIX_I2C_TOUCH_MT_INPUT_DEV_NAME "SITRONIX"
#define SITRONIX_I2C_TOUCH_KEY_INPUT_DEV_NAME "sitronix-i2c-touch-key"

char sitronix_sensor_key_status = 0;
struct sitronix_sensor_key_t sitronix_sensor_key_array[] = {
	{KEY_MENU},		// bit 2
	{KEY_HOMEPAGE}, // bit 1
	{KEY_BACK},		// bit 0
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sitronix_ts_early_suspend(struct early_suspend *h);
static void sitronix_ts_late_resume(struct early_suspend *h);
#endif
static int fb_notifier_callback(struct notifier_block *self,
								unsigned long event, void *data);
static void drm_checkdt_delay_work(struct work_struct *work);
static int sitronix_ts_suspend(struct i2c_client *client);
static int sitronix_ts_resume(struct i2c_client *client);

struct sitronix_ts_data stx_gpts = {0};

static inline void sitronix_ts_pen_down(struct input_dev *input_dev, int id, u16 x, u16 y)
{
#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#else
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id + 1);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 255);
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 255);
	input_report_abs(input_dev, ABS_MT_PRESSURE, 255);

	input_mt_sync(input_dev);
#endif
	STX_DEBUG("sitronix: [%d](%d, %d)+", id, x, y);
}

static inline void sitronix_ts_pen_up(struct input_dev *input_dev, int id)
{
#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
#else
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
#endif
	STX_DEBUG("[%d]-", id);
}

static inline void sitronix_ts_pen_allup(struct sitronix_ts_data *ts_data)
{
	int i;
	for (i = 0; i < ts_data->ts_dev_info.max_touches; i++)
		sitronix_ts_pen_up(ts_data->input_dev, i);

	input_report_key(ts_data->input_dev, BTN_TOUCH, 0);
	input_sync(ts_data->input_dev);
}

static inline void sitronix_ts_handle_sensor_key(struct input_dev *input_dev, struct sitronix_sensor_key_t *key_array, char *pre_key_status, char cur_key_status, int key_count)
{
	int i = 0;
	for (i = 0; i < key_count; i++)
	{
		if (cur_key_status & (1 << i))
		{
			STX_DEBUG("sitronix down now key %d i=%d", cur_key_status, i);
			input_report_key(input_dev, key_array[i].code, 1);
			input_sync(input_dev);
		}
		else
		{
			if (*pre_key_status & (1 << i))
			{
				STX_DEBUG("sitronix sensor key i=[%d] up", i);
				input_report_key(input_dev, key_array[i].code, 0);
				input_sync(input_dev);
			}
		}
	}
	*pre_key_status = cur_key_status;
}

void stx_irq_enable(void)
{
	unsigned long flags;

	if(stx_gpts.irq_registered == 0)
	{
		STX_ERROR("INT have not been registered yet");
		return;
	}

	spin_lock_irqsave(&stx_gpts.irq_lock, flags);

	if (stx_gpts.irq_is_enable == 0)
	{
		stx_gpts.irq_is_enable = 1;
		spin_unlock_irqrestore(&stx_gpts.irq_lock, flags);
		enable_irq(stx_gpts.client->irq);
		STX_DEBUG("stx_irq_enable, stx_gpts.irq_is_enable=%d", stx_gpts.irq_is_enable);
	}
	else if (stx_gpts.irq_is_enable == 1)
	{
		spin_unlock_irqrestore(&stx_gpts.irq_lock, flags);
		STX_INFO("Touch Eint already enabled!");
	}
	else
	{
		spin_unlock_irqrestore(&stx_gpts.irq_lock, flags);
		STX_ERROR("Invalid stx_gpts.irq_is_enable %d!", stx_gpts.irq_is_enable);
	}
}

void stx_irq_disable(void)
{
	unsigned long flags;

	if(stx_gpts.irq_registered == 0)
	{
		STX_ERROR("irq registered failed ");
		return;
	}

	spin_lock_irqsave(&stx_gpts.irq_lock, flags);

	if (stx_gpts.irq_is_enable == 1)
	{
		stx_gpts.irq_is_enable = 0;
		spin_unlock_irqrestore(&stx_gpts.irq_lock, flags);
		disable_irq_nosync(stx_gpts.client->irq);
		STX_DEBUG("stx_irq_disable, stx_gpts.irq_is_enable=%d", stx_gpts.irq_is_enable);
	}
	else if (stx_gpts.irq_is_enable == 0)
	{
		spin_unlock_irqrestore(&stx_gpts.irq_lock, flags);
		STX_INFO("Touch Eint already disabled!");
	}
	else
	{
		spin_unlock_irqrestore(&stx_gpts.irq_lock, flags);
		STX_ERROR("Invalid stx_gpts.irq_is_enable %d!", stx_gpts.irq_is_enable);
	}
}

static void sitronix_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	struct sitronix_ts_data *ts = container_of(work, struct sitronix_ts_data, work);
	u16 x, y;
	uint8_t buf[1 + ST_MAX_TOUCHES * PIXEL_DATA_LENGTH_A] = {0};
	uint8_t PixelCount = 0;

	STX_DEBUG("%s", __FUNCTION__);

#ifdef SITRONIX_GESTURE
	if (!ts->suspend_state)
	{
		ret = stx_i2c_read_bytes(FINGERS, buf, 1);
		STX_DEBUG("SITRONIX_GESTURE ret:%d ,value:0x%X", ret, buf[0]);
		buf[0] &= 0xF;
		if ((ret == 1 && buf[0] == G_PALM))
		{
			sitronix_gesture_func(ts->input_dev, buf[0]);
			goto exit_data_handled;
		}
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE
	if (stx_gpts.fsmart_wakeup == 1)
	{
		if (ts->suspend_state)
		{
			if (sitronix_swk_func(ts->input_dev) == 0)
				goto exit_data_handled;
		}
	}
#endif
	ret = stx_i2c_read_bytes(0x11, buf, ts->ts_dev_info.max_touches * 4);
	if (ret < 0)
	{
		STX_ERROR("read finger error (%d)", ret);
		goto exit_invalid_data;
	}
	for (i = 0; i < ts->ts_dev_info.max_touches; i++)
	{
		if (buf[1 + 4 * i] & 0x80)
		{
			x = (int)(buf[1 + i * 4] & 0x70) << 4 | buf[1 + i * 4 + 1];
			y = (int)(buf[1 + i * 4] & 0x0F) << 8 | buf[1 + i * 4 + 2];
			if (sitronix_cases_mode_check(x, y))
			{
				PixelCount++;
				STX_DEBUG("SITRONIX touch point: %d (%d,%d)", i, x, y);
				sitronix_ts_pen_down(ts->input_dev, i, x, y);
			}
		}
		else
		{
#ifdef SITRONIX_SUPPORT_MT_SLOT
			sitronix_ts_pen_up(ts->input_dev, i);
#endif
		}
	}
#ifndef SITRONIX_SUPPORT_MT_SLOT
	if (PixelCount == 0)
		sitronix_ts_pen_up(ts->input_dev, 0);
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, PixelCount > 0);
	input_sync(ts->input_dev);

	sitronix_ts_handle_sensor_key(ts->input_dev, sitronix_sensor_key_array, &sitronix_sensor_key_status, buf[0], (sizeof(sitronix_sensor_key_array) / sizeof(struct sitronix_sensor_key_t)));

exit_invalid_data:
#if defined(SITRONIX_GESTURE) || defined(CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE)
exit_data_handled:
#endif
	stx_irq_enable();
}

static irqreturn_t sitronix_ts_irq_handler(int irq, void *dev_id)
{
	unsigned long flags;
	struct sitronix_ts_data *ts = dev_id;

	spin_lock_irqsave(&stx_gpts.irq_lock, flags);

	if (stx_gpts.irq_is_enable == 0)
	{
		spin_unlock_irqrestore(&stx_gpts.irq_lock, flags);
		STX_ERROR("%s irq_is_enable is %d", __FUNCTION__, stx_gpts.irq_is_enable);
		return IRQ_HANDLED;
	}
	stx_gpts.irq_is_enable = 0;

	spin_unlock_irqrestore(&stx_gpts.irq_lock, flags);
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	disable_irq_nosync(ts->client->irq);
#ifdef ST_MONITOR_THREAD
	sitronix_monitor_delay();
#endif
	schedule_work(&ts->work);
	return IRQ_HANDLED;
}

void st_reset_ic(void)
{
	STX_DEBUG("%s", __func__);
	gpio_direction_output(stx_gpts.host_if->reset_gpio, 1);
	msleep(10);
	gpio_direction_output(stx_gpts.host_if->reset_gpio, 0);
	msleep(10);
	gpio_direction_output(stx_gpts.host_if->reset_gpio, 1);
	msleep(150);
}

static int sitronix_parse_dt(struct device *dev,
							 struct sitronix_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;

	STX_DEBUG("%s,%d", __FUNCTION__, __LINE__);

	pdata->name = "sitronix";

	/*vdd-supply 2.8v info*/
	pdata->vdd_gpio = of_get_named_gpio_flags(np, "sitronix,vdd-gpio",0,&pdata->vdd_gpio_flags);	
	STX_INFO("%s,pdata->vdd_gpio=%d\n",__FUNCTION__,pdata->vdd_gpio);
	
	if (pdata->vdd_gpio < 0)
		return pdata->vdd_gpio;
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "sitronix,reset-gpio", 0, &pdata->reset_gpio_flags);
	STX_INFO("%s,pdata->reset_gpio=%d", __FUNCTION__, pdata->reset_gpio);

	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "sitronix,irq-gpio",0, &pdata->irq_gpio_flags);	
	STX_INFO("%s,pdata->irq_gpio=%d\n",__FUNCTION__,pdata->irq_gpio);
	
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
	prop = of_find_property(np, "sitronix,fw-rawdata-range", NULL);
	if (prop) {
		rc = of_property_read_u32_array(np,
			"sitronix,fw-rawdata-range", pdata->rawdata_range, ARRAY_SIZE(pdata->rawdata_range));
	}
	rc = of_property_read_string(np, "sitronix,factory-info", &pdata->factory_info);
	if (rc < 0) {
		STX_ERROR("factory info read failed!\n");
		pdata->factory_info = "NULL";
	}
	prop = of_find_property(np, "sitronix,button-map", NULL);
	if (prop)
	{
		pdata->num_button = prop->length / sizeof(u32);
		if (pdata->num_button > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
										"sitronix,button-map", pdata->button_map, pdata->num_button);
		if (rc)
		{
			STX_ERROR("Unable to read key codes");
			return rc;
		}
	}
	return 0;
}
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_POWER_SUPPLY_EN
static int sitronix_power_on(struct sitronix_ts_data *ts)
{
	int ret = 0;
#if 0
	if (ts->vdd_ana)
	{
		ret = regulator_enable(ts->vdd_ana);
		if (ret)
		{
			STX_ERROR("Regulator vdd enable failed ret =%d", ret);
		}
	}
#endif
	printk(KERN_ERR "DTT: vdd_gpio = %d\n", ts->host_if->vdd_gpio);
	ret = gpio_request(ts->host_if->vdd_gpio, "vdd_gpio");
	if (ret < 0)
	{
		STX_ERROR("sitonix failed to request vdd gpio\n");
	}else
		gpio_direction_output(ts->host_if->vdd_gpio, 1);
	if (ts->vcc_i2c)
	{
		ret = regulator_enable(ts->vcc_i2c);
		if (ret)
		{
			STX_ERROR("Regulator vcc_i2c enable failed ret =%d", ret);
		}
	}
	STX_FUNC_EXIT();
	return ret;
}

static int sitronix_power_off(struct sitronix_ts_data *ts)
{
	int ret = 0;
	if (ts->vcc_i2c)
	{
		ret = regulator_disable(ts->vcc_i2c);
		if (ret)
		{
			STX_ERROR("Regulator vcc_i2c disable failed ret =%d", ret);
		}
	}
	ret= gpio_request(ts->host_if->vdd_gpio, "vdd_gpio");
	if (ret< 0)
	{
		STX_ERROR("sitonix failed to request gpio vdd_gpio\n");
	}else{
		gpio_direction_output(ts->host_if->vdd_gpio, 0);
	}
#if 0
	if (ts->vdd_ana)
	{
		ret = regulator_disable(ts->vdd_ana);
		if (ret)
		{
			STX_ERROR("Regulator vdd_ana disable failed ret =%d", ret);
		}
#endif
	return ret;
}

static int sitronix_power_init(struct sitronix_ts_data *ts)
{
	int ret;
	ts->vdd_ana = regulator_get(&ts->client->dev, "vdd_ana");
	if (IS_ERR(ts->vdd_ana))
	{
		ts->vdd_ana = NULL;
		ret = PTR_ERR(ts->vdd_ana);
		STX_ERROR("get vdd_ana regulator failed,ret=%d", ret);
		return ret;
	}
	ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
	if (IS_ERR(ts->vcc_i2c))
	{
		ts->vcc_i2c = NULL;
		ret = PTR_ERR(ts->vcc_i2c);
		STX_ERROR("get vcc_i2c regulator failed,ret=%d", ret);
		goto err_get_vcc;
	}
	STX_FUNC_EXIT();
	return 0;

err_get_vcc:
	regulator_put(ts->vdd_ana);
	STX_FUNC_EXIT();
	return ret;
}

static int sitronix_power_deinit(struct sitronix_ts_data *ts)
{
	if (ts->vdd_ana)
		regulator_put(ts->vdd_ana);
	if (ts->vcc_i2c)
		regulator_put(ts->vcc_i2c);

	return 0;
}
#endif

static void sitronix_ts_input_set_params(struct sitronix_ts_data *ts_data)
{

#ifdef SITRONIX_SUPPORT_MT_SLOT
	input_mt_init_slots(ts_data->input_dev, ts_data->ts_dev_info.max_touches, INPUT_MT_DIRECT);
#else
	set_bit(ABS_MT_TOUCH_MAJOR, ts_data->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MINOR, ts_data->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts_data->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts_data->input_dev->absbit);
	set_bit(ABS_MT_BLOB_ID, ts_data->input_dev->absbit);
	set_bit(ABS_MT_TRACKING_ID, ts_data->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts_data->input_dev->propbit);
	input_set_abs_params(ts_data->input_dev, ABS_MT_TRACKING_ID, 0, ts_data->ts_dev_info.max_touches, 0, 0);
	input_set_abs_params(ts_data->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts_data->input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(ts_data->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif

#ifndef SITRONIX_SWAP_XY
	input_set_abs_params(ts_data->input_dev, ABS_MT_POSITION_X, 0, (ts_data->ts_dev_info.x_res - 1), 0, 0);
	input_set_abs_params(ts_data->input_dev, ABS_MT_POSITION_Y, 0, (ts_data->ts_dev_info.y_res - 1), 0, 0);
#else
	input_set_abs_params(ts_data->input_dev, ABS_MT_POSITION_X, 0, (ts_data->ts_dev_info.y_res - 1), 0, 0);
	input_set_abs_params(ts_data->input_dev, ABS_MT_POSITION_Y, 0, (ts_data->ts_dev_info.x_res - 1), 0, 0);
#endif

	return;
}

static int sitronix_ts_input_dev_init(struct sitronix_ts_data *ts_data)
{
	int ret = 0;
	int i;

	ts_data->input_dev = input_allocate_device();
	if (ts_data->input_dev == NULL)
	{
		STX_ERROR("%s: Can not allocate input device!", __func__);
		return -ENOMEM;
	}

	ts_data->input_dev->name = SITRONIX_I2C_TOUCH_MT_INPUT_DEV_NAME; //ts_data->name;
	ts_data->input_dev->id.bustype = BUS_I2C;
	ts_data->input_dev->id.product = 0;
	ts_data->input_dev->id.version = 0;
	ts_data->input_dev->dev.parent = &ts_data->client->dev;
	input_set_drvdata(ts_data->input_dev, ts_data);

	set_bit(EV_KEY, ts_data->input_dev->evbit);
	set_bit(EV_ABS, ts_data->input_dev->evbit);
	set_bit(BTN_TOUCH, ts_data->input_dev->keybit);

#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE
	st_gesture_init();
#endif
	for (i = 0; i < 3; i++)
	{
		set_bit(sitronix_sensor_key_array[i].code, ts_data->input_dev->keybit);
	}

	sitronix_ts_input_set_params(ts_data);

	ret = input_register_device(ts_data->input_dev);
	if (ret)
	{
		STX_ERROR("%s: Failed to register input device", __func__);
		goto err_register_input;
	}

	return 0;

err_register_input:
	input_free_device(ts_data->input_dev);
	ts_data->input_dev = NULL;

	return ret;
}

/*****************************************************************************
*  Name: stx_gpio_configure
*  Brief: IRQ & RESET GPIO INIT
*  Input:
*  Output:
*  Return: return 0 if succuss
*****************************************************************************/
static int stx_gpio_configure(struct sitronix_ts_data *data)
{
	int ret = 0;

	STX_FUNC_ENTER();
	/* request irq gpio */
	if (gpio_is_valid(data->host_if->irq_gpio))
	{
		ret = gpio_request(data->host_if->irq_gpio, "stx_irq_gpio");
		if (ret)
		{
			STX_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		ret = gpio_direction_input(data->host_if->irq_gpio);
		if (ret)
		{
			STX_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	/* request reset gpio */
	if (gpio_is_valid(data->host_if->reset_gpio))
	{
		ret = gpio_request(data->host_if->reset_gpio, "stx_reset_gpio");
		if (ret)
		{
			STX_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		ret = gpio_direction_output(data->host_if->reset_gpio, 1);
		if (ret)
		{
			STX_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}
		msleep(10);
		gpio_direction_output(data->host_if->reset_gpio, 0);
		msleep(10);
		gpio_direction_output(data->host_if->reset_gpio, 1);
		msleep(150);
	}

	STX_FUNC_EXIT();
	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(data->host_if->reset_gpio))
		gpio_free(data->host_if->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->host_if->irq_gpio))
		gpio_free(data->host_if->irq_gpio);
err_irq_gpio_req:
	STX_FUNC_EXIT();
	return ret;
}
static int factory_get_calibration_ret(struct device *dev)
{
	return 1;
}
static int factory_get_ic_fw_version(struct device *dev, char *buf)
{
	struct sitronix_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	char buffer[8];

	ret = stx_i2c_read(ts->client, buffer, 1, FIRMWARE_VERSION);
	if (ret != 1)
	{
		STX_ERROR(" i2c communcate error in getting FW version : 0x%x", ret);
		return snprintf(buf, PAGE_SIZE, "0x00\n");
	}
	STX_INFO("Touch Panel Firmware version %x", buffer[0]);

	ts->ts_dev_info.fw_version[0] = buffer[0];
	return snprintf(buf, PAGE_SIZE, "0x%02X\n", ts->ts_dev_info.fw_version[0]);
}
static int factory_get_module_id(struct device *dev, char *buf)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);
	data->pannel_id = 0x00;

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", data->pannel_id);
}
#if 0
static int factory_get_rawdata(struct device *dev, char *buf)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);

	if (ft5x06_check_rawdata_need(data))
		return ft5x06_get_rawdata(data,  buf);
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "NOT SUPPORTED");
}
#endif
static int factory_get_rawdata_info(struct device *dev, char *buf)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);
	int ret;
	u8 tx = 0, rx = 0;

	pr_buf_info(" %s enter \n",  __func__);
	//ret = sitronix_get_xy_chs(data);
	ret = sitronix_get_rawdata_rowcol(data, &tx, &rx);
	if (ret == 0)
		 return snprintf(buf, PAGE_SIZE, "RX:%u TX:%u HIGH:%u LOW:%u\n",
					   rx, tx, data->host_if->rawdata_range[0], data->host_if->rawdata_range[1]);
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "NOT SUPPORTED");
}

static int factory_proc_hibernate_test(struct device *dev)
{
	int err = 0;
	int i;
	char buffer[8];
	struct sitronix_ts_data *data = dev_get_drvdata(dev);

	pr_buf_info(" %s enter \n",  __func__);

	gpio_direction_output(stx_gpts.host_if->reset_gpio, 1);
	mdelay(1);
	gpio_direction_output(stx_gpts.host_if->reset_gpio, 0);
	mdelay(5);
	pr_buf_info("%s: reset is low\n", __func__);

	err = stx_i2c_read(data->client, buffer, 1, FIRMWARE_VERSION);
	if (err == 1)
	{
		pr_buf_err("%s: read i2c ok when reset is low,with result=%d\n", __func__, err);
		goto out;
	}

	gpio_direction_output(stx_gpts.host_if->reset_gpio, 1);
	msleep(200);

	err = stx_i2c_read(data->client, buffer, 1, FIRMWARE_VERSION);
	if (err != 1) {
		pr_buf_err("%s:read i2c failed when reset is high, reset test Fail!\n", __func__);
		goto out;
	}
		pr_buf_info("%s:read i2c ok when reset is high, reset test success!\n", __func__);

	/* release all touches */
	for (i = 0; i < data->ts_dev_info.max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 1;
out:
	/* release all touches */
	for (i = 0; i < data->ts_dev_info.max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 0;
}
static int factory_get_chip_type(struct device *dev, char *buf)
{	
	return snprintf(buf, PAGE_SIZE, "sitronix\n");

}
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GLOVE
static bool get_glove_switch(struct device *dev)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);
	
	pr_buf_info("%s: glove_enable = %d\n", __func__, data->glove_mode);
	return data->glove_mode;
}

static int set_glove_switch(struct device *dev, bool enable)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);

	data->glove_mode = enable;
	pr_buf_info("%s: glove_enable = %d\n", __func__, data->glove_mode);
	if(data->glove_mode == 1)
		st_enter_glove_mode(data);
	if(data->glove_mode == 0)
		st_leave_glove_mode(data);
	return 0;
}
#endif
static int factory_get_factory_info(struct device *dev, char *buf)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);

	pr_buf_info("%s: %s\n", __func__, data->host_if->factory_info);
	return snprintf(buf, PAGE_SIZE, "%s\n", data->host_if->factory_info);
}
static bool get_tp_enable_switch(struct device *dev)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);

	return !data->suspend_state;
}
static int set_tp_enable_switch(struct device *dev, bool enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	static bool is_the_first_set = 1;
#if defined(CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE)
	struct sitronix_ts_data *data = dev_get_drvdata(dev);
	static bool gesture_switch;
#endif

	if (is_the_first_set) {
#if defined(CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE)
		gesture_switch = data->fsmart_wakeup;
#endif
		is_the_first_set = 0;
	}
	STX_INFO("%s: %d.\n", __func__, enable);

	if (enable) {
		//fts_reset_proc(200);
		sitronix_ts_resume(client);
#if defined(CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE)
		data->fsmart_wakeup = gesture_switch;
#endif
		return 0;
	} else {
#if defined(CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE)
		gesture_switch = data->fsmart_wakeup;
		data->fsmart_wakeup = 0;
#endif
		sitronix_ts_suspend(client);
		return 0;
	}
}

static	int factory_get_fw_update_progress(struct device *dev)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);

	return data->loading_fw ? FW_IS_UPDATETING : FW_UPGRADE_SUCCESS;
}
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE
static unsigned int asic_to_hex(unsigned char val)
{
	if ((val >= '0') && (val <= '9'))
		val -= '0';
	else if ((val >= 'a') && (val <= 'z'))
		val = val - 'a' + 10;
	else if ((val >= 'A') && (val <= 'Z'))
		val = val - 'A' + 10;

	return (unsigned int)val;
}

bool get_gesture_switch(struct device *dev)
{
	struct sitronix_ts_data *ts_data = dev_get_drvdata(dev);

	if(ts_data->fsmart_wakeup == 0x01)
		return true;
	else
		return false;
}

static int set_gesture_switch(struct device *dev, const char *buf)
{
	struct sitronix_ts_data *ts_data = dev_get_drvdata(dev);
	unsigned char gesture[10], len;
	strlcpy(gesture, buf, sizeof(gesture));
	len = strlen(gesture);
	if (len > 0) {
		if ((gesture[len-1] == '\n') || (gesture[len-1] == '\0'))
			len--;
	}
	STX_INFO("Chipone_tddi set gesture state len: %d gesture_state: %d,%d,%d",
		len, gesture[0], gesture[1], gesture[2]);
	if (len == 1) {
		if (gesture[0] == '1')
			ts_data->gesture_state = 0xffff;
		else if (gesture[0] == '0')
			ts_data->gesture_state = 0x0;
	} else if (len == 4) {
		ts_data->gesture_state = asic_to_hex(gesture[0])*0x1000
		   + asic_to_hex(gesture[1]) * 0x100
		   + asic_to_hex(gesture[2]) * 0x10
		   + asic_to_hex(gesture[3]);
	} else {
		STX_ERROR("Set gesture switch write wrong cmd");
		return 0;
	}
	/*fsmart_wakeup is only for double tap */
	if (ts_data->gesture_state & 0x0002)
		ts_data->fsmart_wakeup = 1;
	else
		ts_data->fsmart_wakeup = 0;
	STX_INFO("%s is %x.\n", __func__, ts_data->gesture_state);
	return 0;
}
#endif
static int factory_ts_func_test_register(struct sitronix_ts_data *data)
{
	ts_gen_func_test_init();
	data->ts_test_dev.dev = &data->client->dev;
	data->ts_test_dev.check_fw_update_need = sitronix_check_fw_update_need;
	data->ts_test_dev.get_fw_update_progress = factory_get_fw_update_progress;
	data->ts_test_dev.proc_fw_update = sitronix_proc_fw_update;
	data->ts_test_dev.get_calibration_ret = factory_get_calibration_ret;
	data->ts_test_dev.get_ic_fw_version = factory_get_ic_fw_version;
	data->ts_test_dev.get_fs_fw_version = sitronix_get_fs_fw_version;
	data->ts_test_dev.get_module_id = factory_get_module_id;
	data->ts_test_dev.proc_hibernate_test = factory_proc_hibernate_test;
	data->ts_test_dev.get_rawdata_info = factory_get_rawdata_info;
	data->ts_test_dev.get_rawdata = sitronix_ts_get_rawdata;
	data->ts_test_dev.get_short_test = sitronix_ts_get_short_test;
	//data->ts_test_dev.get_open_test = sitronix_ts_get_open_test;
	data->ts_test_dev.get_chip_type = factory_get_chip_type;
	data->ts_test_dev.get_factory_info = factory_get_factory_info;
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GLOVE
	data->ts_test_dev.get_glove_switch = get_glove_switch;
	data->ts_test_dev.set_glove_switch = set_glove_switch;
#endif
	data->ts_test_dev.get_tp_enable_switch = get_tp_enable_switch;
	data->ts_test_dev.set_tp_enable_switch = set_tp_enable_switch;
#if defined(CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE)
	data->ts_test_dev.get_gesture_switch = get_gesture_switch;
	data->ts_test_dev.set_gesture_switch = set_gesture_switch;
#endif
	register_ts_func_test_device(&data->ts_test_dev);
	return 0;
}
static void sitronix_ts_register_productinfo(struct sitronix_ts_data *ts_data)
{
	/* format as flow: version:0x01 Module id:0x57 */
	char deviceinfo[64];
	int ret;
	char buffer[8];

	ret = stx_i2c_read(ts_data->client, buffer, 1, FIRMWARE_VERSION);
	snprintf(deviceinfo, ARRAY_SIZE(deviceinfo), "FW version:0x%2x Module id:0x%2x",buffer[0], ts_data->pannel_id);
	productinfo_register(PRODUCTINFO_CTP_ID, NULL, deviceinfo);
}
static int sitronix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	uint8_t err_code = 0;
	uint8_t dev_status = 0;
	struct sitronix_ts_platform_data *pdata;
	STX_FUNC_ENTER();
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		STX_ERROR("sitronix Failed check I2C functionality");
		return -ENODEV;
	}

	if (client->dev.of_node)
	{
		pdata = devm_kzalloc(&client->dev,sizeof(struct sitronix_ts_platform_data), GFP_KERNEL);
		if (!pdata)
		{
			STX_ERROR("Failed to allocate memory for platform data");
			return -ENOMEM;
		}
		ret = sitronix_parse_dt(&client->dev, pdata);
		if (ret)
			STX_ERROR("[DTS]DT parsing failed");
	}
	else
	{
		pdata = client->dev.platform_data;
	}

	if (!pdata)
	{
		STX_ERROR("no ts platform data found");
		return -EINVAL;
	}

	stx_gpts.host_if = pdata;
	stx_gpts.client = client;
	i2c_set_clientdata(client, &stx_gpts);

	stx_gpts.is_upgrading = false;
	stx_gpts.is_testing = false;
	stx_gpts.suspend_state = 0;
	stx_gpts.irq_is_enable = 1;
	stx_gpts.fsmart_wakeup = 0;
	stx_gpts.rawTestResult = 1;
	stx_gpts.irq_registered = 0;
    stx_gpts.fapp_in = 0;
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_POWER_SUPPLY_EN
	ret = sitronix_power_init(&stx_gpts);
	if (ret)
	{
		STX_ERROR("sitronix failed get regulator");
		ret = -EINVAL;
		return ret;
	}

	ret = sitronix_power_on(&stx_gpts);
	if (ret)
	{
		STX_ERROR("regulator failed power on device");
		ret = -EINVAL;
		goto exit_deinit_power;
	}
#endif

	stx_gpio_configure(&stx_gpts);

#ifdef ST_UPGRADE_FIRMWARE
	//kthread_run(st_upgrade_fw_handler, "Sitronix", "sitronix_update");
	st_upgrade_fw();
#endif

	ret = st_get_dev_status(client, &err_code, &dev_status);
	if ((ret < 0) || (dev_status == 0x6) || ((err_code == 0x8) && (dev_status == 0x0)))
	{
		ret = -EPERM;
		goto err_device_info_error;
	}

	ret = st_get_touch_info(&stx_gpts);
	if (ret < 0)
		goto err_device_info_error;

	mutex_init(&stx_gpts.dev_mutex);
	spin_lock_init(&stx_gpts.irq_lock);

	INIT_WORK(&(stx_gpts.work), sitronix_ts_work_func);

#ifdef ST_MONITOR_THREAD
	sitronix_monitor_delay();
	sitronix_monitor_start();
#endif

	if ((ret = sitronix_ts_input_dev_init(&stx_gpts)))
	{
		STX_ERROR("%s: Failed to set up input device", __func__);
		ret = -EINVAL;
		goto err_input_register_device_failed;
	}

	stx_gpts.client->irq = gpio_to_irq(stx_gpts.host_if->irq_gpio);
	STX_INFO("irq num:%d", stx_gpts.client->irq);
	if (stx_gpts.client->irq)
	{
		STX_INFO("irq = %d", stx_gpts.client->irq);
		stx_gpts.irq_flags = IRQF_TRIGGER_FALLING; // | IRQF_ONESHOT;// IRQF_NO_SUSPEND;
		//ret = request_threaded_irq(stx_gpts.client->irq, NULL, sitronix_ts_irq_handler, stx_gpts.irq_flags, client->name, &stx_gpts);
		ret = request_irq(stx_gpts.client->irq, sitronix_ts_irq_handler, stx_gpts.irq_flags, client->name, &stx_gpts);
		if (ret < 0)
		{
			STX_ERROR("request_irq failed");
			goto err_request_irq_failed;
		}
		stx_gpts.irq_registered = 1;
	}

	ret = st_create_sysfs(client);
	if (ret < 0)
	{
		STX_ERROR("create sysfs node fail");
	}

#ifdef ST_DEVICE_NODE
	ret = st_dev_node_init();
	if (ret < 0)
		STX_ERROR("create sitronix node fail");
#endif

	STX_DEBUG("%s,line=%d", __FUNCTION__, __LINE__);

#if defined(CONFIG_FB)
	stx_gpts.fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&stx_gpts.fb_notif);

	if (ret)
		STX_ERROR("Unable to register fb_notifier: %d", ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	stx_gpts.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	stx_gpts.early_suspend.suspend = sitronix_ts_early_suspend;
	stx_gpts.early_suspend.resume = sitronix_ts_late_resume;
	register_early_suspend(&stx_gpts.early_suspend);
#endif
/*Modified by Hisesne to add DRM function*/
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
		stx_gpts.checkdrm_workqueue = create_singlethread_workqueue("fts_drmcheck_wq");
		INIT_DELAYED_WORK(&stx_gpts.checkdrm_delay_work, drm_checkdt_delay_work);
		queue_delayed_work(stx_gpts.checkdrm_workqueue, &stx_gpts.checkdrm_delay_work,
							usecs_to_jiffies(5000000));
#endif
#endif
/*END OF Hisense add*/
	factory_ts_func_test_register(&stx_gpts);
	sitronix_ts_register_productinfo(&stx_gpts);
	STX_FUNC_EXIT();
	return 0;

err_request_irq_failed:
	if (stx_gpts.input_dev)
	{
		input_free_device(stx_gpts.input_dev);
		stx_gpts.input_dev = NULL;
	}
err_input_register_device_failed:
#ifdef ST_MONITOR_THREAD
	sitronix_monitor_stop();
#endif
err_device_info_error:
	if (gpio_is_valid(stx_gpts.host_if->vdd_gpio))
		gpio_free(stx_gpts.host_if->vdd_gpio);
	if (gpio_is_valid(stx_gpts.host_if->reset_gpio))
		gpio_free(stx_gpts.host_if->reset_gpio);
	if (gpio_is_valid(stx_gpts.host_if->irq_gpio))
		gpio_free(stx_gpts.host_if->irq_gpio);
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_POWER_SUPPLY_EN
	sitronix_power_off(&stx_gpts);
exit_deinit_power:
	sitronix_power_deinit(&stx_gpts);
#endif
	return ret;
}

static int sitronix_ts_remove(struct i2c_client *client)
{
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		STX_ERROR("Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef ST_DEVICE_NODE
	st_dev_node_exit();
#endif

#ifdef ST_MONITOR_THREAD
	sitronix_monitor_stop();
#endif
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_POWER_SUPPLY_EN
	sitronix_power_off(&stx_gpts);
	sitronix_power_deinit(&stx_gpts);
#endif
	i2c_set_clientdata(client, NULL);
	free_irq(client->irq, ts);
	if (ts->input_dev)
		input_unregister_device(ts->input_dev);

	mutex_destroy(&stx_gpts.dev_mutex);
	return 0;
}

static int sitronix_ts_suspend(struct i2c_client *client)
{
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);
	STX_FUNC_ENTER();

    if(ts->fapp_in)
    {
        STX_INFO("%s fapp_in = %d",__func__,ts->fapp_in);
        return 0;
    }

	if (ts->suspend_state)
	{
		STX_ERROR("Already in suspend state");
		return 0;
	}

#ifdef ST_MONITOR_THREAD //hfst1
	sitronix_monitor_stop();
#endif

#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE
	if (stx_gpts.fsmart_wakeup == 1)
	{
		sitronix_swk_enable(); //hfst001
		st_power_down(ts);
		enable_irq_wake(stx_gpts.client->irq);//hfst
		sitronix_ts_pen_allup(ts);
		ts->suspend_state = 1;
		return 0;
	}
#endif
	stx_irq_disable();
	st_power_down(ts);
	gpio_direction_output(ts->host_if->reset_gpio, 0);
	sitronix_ts_pen_allup(ts);
	ts->suspend_state = 1;

	STX_FUNC_EXIT();
	return 0;
}

static int sitronix_ts_resume(struct i2c_client *client)
{
	struct sitronix_ts_data *ts = i2c_get_clientdata(client);
	STX_FUNC_ENTER();

    if(ts->fapp_in)
    {
        STX_INFO("%s fapp_in = %d",__func__,ts->fapp_in);
        return 0;
    }
	
	if (!ts->suspend_state)
	{
		gpio_direction_output(stx_gpts.host_if->reset_gpio, 0);
		msleep(10);
		gpio_direction_output(stx_gpts.host_if->reset_gpio, 1);
		msleep(150);
		STX_ERROR("Already in resume state");
		return 0;
	}
	gpio_direction_output(ts->host_if->reset_gpio, 0);
	msleep(10);
	gpio_direction_output(ts->host_if->reset_gpio, 1);
	msleep(150);

	st_power_up(ts);
#ifdef ST_MONITOR_THREAD
	sitronix_monitor_start();
#endif
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GLOVE
	if(ts->glove_mode == 1)
		st_enter_glove_mode(ts);
	if(ts->glove_mode == 0)
		st_leave_glove_mode(ts);
#endif
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE
	if (stx_gpts.fsmart_wakeup == 1)
	{
		sitronix_swk_disable();
		disable_irq_wake(stx_gpts.client->irq);//hfst
		ts->suspend_state = 0;
		return 0;
	}
#endif

	ts->suspend_state = 0;
	stx_irq_enable();

	STX_FUNC_EXIT();
	return 0;
}

#if defined(CONFIG_FB)

/*******************************************************************************
*  Name: fb_notifier_callback
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fb_notifier_callback(struct notifier_block *self,
								unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct sitronix_ts_data *sitronix_data = container_of(self, struct sitronix_ts_data, fb_notif);
	STX_FUNC_ENTER();
	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
		sitronix_data && sitronix_data->client)
	{
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			sitronix_ts_resume(sitronix_data->client);
		else if (*blank == FB_BLANK_POWERDOWN)
			sitronix_ts_suspend(sitronix_data->client);
	}
	STX_FUNC_EXIT();
	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void sitronix_ts_early_suspend(struct early_suspend *h)
{
	struct sitronix_ts_data *ts;
	STX_DEBUG("%s", __FUNCTION__);
	ts = container_of(h, struct sitronix_ts_data, early_suspend);
	sitronix_ts_suspend(ts->client);
}

static void sitronix_ts_late_resume(struct early_suspend *h)
{
	struct sitronix_ts_data *ts;
	STX_DEBUG("%s", __FUNCTION__);
	ts = container_of(h, struct sitronix_ts_data, early_suspend);
	sitronix_ts_resume(ts->client);
}
#endif
/*Modified by Hisense to add DRM function*/
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)

static struct drm_panel *active_panel;

static int drm_check_dt(struct sitronix_ts_data *data)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(data->client->dev.of_node, "panel", NULL);
    if (count <= 0) {
        STX_ERROR("find drm_panel count(%d) fail\n", count);
        return -ENODEV;
    }

    for (i = 0; i < count; i++) {
        node = of_parse_phandle(data->client->dev.of_node, "panel", i);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) {
            STX_ERROR("find drm_panel successfully\n");
            active_panel = panel;
            return 0;
        }
    }

    STX_ERROR("no find drm_panel\n");
    return -ENODEV;
}

static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct drm_panel_notifier *evdata = data;
    int *blank = NULL;
    struct sitronix_ts_data *ts_data = container_of(self, struct sitronix_ts_data,
                                  fb_notif);
	STX_INFO("drm_notifier_callback enter\n");

    if (!evdata) {
        STX_ERROR("evdata is null\n");
        return 0;
    }

    if (!((event == DRM_PANEL_EARLY_EVENT_BLANK )
          || (event == DRM_PANEL_EVENT_BLANK))) {
        STX_ERROR("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    STX_INFO("DRM event:%lu,blank:%d\n", event, *blank);
    switch (*blank) {
    case DRM_PANEL_BLANK_UNBLANK:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
            STX_ERROR("resume: event = %lu, not care\n", event);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
			sitronix_ts_resume(ts_data->client);
        }
        break;
    case DRM_PANEL_BLANK_POWERDOWN:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
			sitronix_ts_suspend(ts_data->client);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            STX_ERROR("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        STX_ERROR("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}

static void drm_checkdt_delay_work(struct work_struct *work)
{
	int ret = 0;
	STX_FUNC_ENTER();
	ret = drm_check_dt(&stx_gpts);
	if(ret){
		STX_ERROR("[Hisense] drm_checkdt_delay_work: faile to drm check error=%d\n", ret);
		//kfree(stx_gpts);
		return ;
	}

   stx_gpts.fb_notif.notifier_call = drm_notifier_callback;
   if (active_panel) {
        ret = drm_panel_notifier_register(active_panel, &stx_gpts.fb_notif);
        if (ret)
          pr_buf_err("drm_panel_notifier_register fail: %d\n", ret);
   }
	
}

#endif /*END OF CONFIG_DRM_PANEL*/
#endif /*CONFIG_DRM*/
/*END OF Hisense add*/
static const struct i2c_device_id sitronix_ts_id[] = {
	{SITRONIX_I2C_TOUCH_DRV_NAME, 0},
	{}};

static struct of_device_id sitronix_match_table[] = {
	{
		.compatible = "sitronix,st1633i",
	},
	{
		.compatible = "sitronix,cf1216",
	},
	{},
};

#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static int sitronix_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	sitronix_ts_suspend(client);
	return 0;
}
static int sitronix_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	sitronix_ts_resume(client);
	return 0;
}

static const struct dev_pm_ops sitronix_ts_dev_pm_ops = {
	.suspend = sitronix_pm_suspend,
	.resume = sitronix_pm_resume,
};
#endif
static struct i2c_driver sitronix_ts_driver = {
	.probe = sitronix_ts_probe,
	.remove = sitronix_ts_remove,
	.id_table = sitronix_ts_id,
	.driver = {
		.name = SITRONIX_I2C_TOUCH_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sitronix_match_table,
#if CONFIG_PM
		.suspend = NULL,
		.resume = NULL,
#endif
	},
};

static int __init sitronix_ts_init(void)
{
	s32 ret_iic = -2;

	STX_DEBUG("Sitronix touch driver %d.%d.%d", DRIVER_MAJOR, DRIVER_MINOR, DRIVER_PATCHLEVEL);

	ret_iic = i2c_add_driver(&sitronix_ts_driver);
	STX_INFO("IIC ADD DRIVER %s,ret_iic=%d", __FUNCTION__, ret_iic);

	return ret_iic;
}

static void __exit sitronix_ts_exit(void)
{
	i2c_del_driver(&sitronix_ts_driver);
}

module_init(sitronix_ts_init);
module_exit(sitronix_ts_exit);

MODULE_DESCRIPTION("Sitronix Multi-Touch Driver");
MODULE_LICENSE("GPL");