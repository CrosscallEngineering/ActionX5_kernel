/*
*****************************************************************************
* Copyright by hisense                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH HISENSE PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-HISENSE-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*! \file
* \brief Device driver for monitoring ambient light intensity in (lux) and
* long proximity detection (prox) within the Capella CM36781 device.
*/

#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/freezer.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <linux/productinfo.h>

#include "cm36781.h"

static char cm36781_ps_name_base[32] = {0};
static char cm36781_als_name_base[32] = {0};
struct cm36781_chip *G_cm36781_chip = NULL;
bool bReboot = false;
/*als resolution for calculate lux, 1/als_resolution[index], see Spec for details*/
static const int als_resolution[] = {
	12, 24, 48, 96
};

static const int ps_duty_array[] = {
	CM36781_PS_DR_1_160,
	CM36781_PS_DR_1_320,
	CM36781_PS_DR_1_640,
	CM36781_PS_DR_1_1280
};

static const int ps_pers_array[] = {
	CM36781_PS_PERS_1,
	CM36781_PS_PERS_2,
	CM36781_PS_PERS_3,
	CM36781_PS_PERS_4
};

static const int ps_it_array[] = {
	CM36781_PS_IT_1T,
	CM36781_PS_IT_1_5T,
	CM36781_PS_IT_2T,
	CM36781_PS_IT_4T,
	CM36781_PS_IT_8T,
	CM36781_PS_IT_9T
};

static const int ps_mp_array[] = {
	CM36781_PS_MP_1,
	CM36781_PS_MP_2,
	CM36781_PS_MP_4,
	CM36781_PS_MP_8
};

static const int ps_led_drv_array[] = {
	CM36781_LED_I_50,
	CM36781_LED_I_75,
	CM36781_LED_I_100,
	CM36781_LED_I_120,
	CM36781_LED_I_140,
	CM36781_LED_I_160,
	CM36781_LED_I_180,
	CM36781_LED_I_200
};


struct sensors_classdev cm36781_ps_cdev = {
	.vendor = "capella",
	.version = 1,
	.handle = SENSOR_HANDLE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 1,
	.flags = SENSOR_FLAG_ON_CHANGE_MODE | SENSOR_FLAG_WAKE_UP,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
	.sensors_calibrate = NULL,
};

struct sensors_classdev cm36781_als_cdev = {
	.vendor = "capella",
	.version = 1,
	.handle = SENSOR_HANDLE_LIGHT,
	.max_range = "10240.0",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 2,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
	.sensors_calibrate = NULL,
};

static uint8_t confuse_i = 0;

static int cm36781_power_switch(struct cm36781_chip *chip, bool on);

/*read cm36781 als adc value*/
static int cm36781_read_als(struct cm36781_chip *chip)
{
	int ret = 0;
	/* Read ALS data: */
	if (bReboot) {
		pr_err(
			"%s:[ALS] not read, because reboot\n", __func__);
		return 0;
	}

	ret = i2c_smbus_read_word_data(chip->client, ALS_DATA);
	if (ret < 0) {
		pr_err(
			"%s:[ALS] read als data fail\n", __func__);
		return -EIO;
	}else {
		chip->als_inf.raw = (u16)ret;
		//pr_info("%s:[ALS] als data is %d", __func__, ret);
	}
	return ret;
}

/*calculate the delay time(us) acording to als i-time*/
static void cm36781_als_calc_udelay(struct cm36781_chip *chip)
{
	chip->als_u_delay = (chip->atime + 1) * ALS_U_DELAY_PER_ATIME * 12 / 10;
}

//static void cm36781_als_calc_max_raw(struct cm36781_chip *chip);

/*init cm36781 als conf reg: i-time setting, disable int func, shut down als func*/
static int cm36781_als_init_paras(struct cm36781_chip *chip)
{
	int ret = 0;
	/*choose als i-time by setting*/
	switch (chip->atime){
		case 0:
			chip->ls_cmd |= CM36781_ALS_IT_50MS;
			break;
		case 1:
			chip->ls_cmd |= CM36781_ALS_IT_100MS;
			break;
		case 2:
			chip->ls_cmd |= CM36781_ALS_IT_200MS;
			break;
		case 3:
			chip->ls_cmd |= CM36781_ALS_IT_400MS;
			break;
		default:
			chip->ls_cmd |= CM36781_ALS_IT_100MS;
			break;
	}
	if (0 <= chip->atime && 3 >= chip->atime)
		chip->als_resolution = als_resolution[chip->atime];
	else
		chip->als_resolution = als_resolution[0];
	cm36781_als_calc_udelay(chip);
	pr_err("%s:[ALS] set als_dwork delay time is %d us!\n", __func__, chip->als_u_delay);
	/*must disable als interrupt befrore IST create*//*disable ALS func*/
	chip->ls_cmd &= CM36781_ALS_INT_MASK;
	chip->ls_cmd |= CM36781_ALS_SD;
	chip->ls_cmd &= CM36781_ALS_SD_MASK;
	pr_err("%s:[ALS] set als conf is %d !\n", __func__, chip->ls_cmd);
	ret = i2c_smbus_write_word_data(chip->client, ALS_CONF, chip->ls_cmd);
	if (ret < 0) {
		pr_err(
			"%s:[ALS] set als conf fail\n", __func__);
		return -EIO;
	}
	return ret;
}

/*turn on cm36781 als func*/
/*static int cm36781_als_function_on(struct cm36781_chip *chip)
{
	int ret = 0;
	if(chip->als_functioned){
			pr_err("%s:[ALS] als has already functioned on!\n", __func__);            
	}else {
		pr_err("%s:[ALS] als is alwaws on\n", __func__);

		chip->ls_cmd &= CM36781_ALS_SD_MASK;
		ret = i2c_smbus_write_word_data(chip->client, ALS_CONF, chip->ls_cmd);
		if (ret < 0) {
			pr_err("%s:[ALS] set als conf fail\n", __func__);
			return -EIO;
		}

		chip->als_functioned = true;
	}
	//printk("delay time %d\n",(chip->atime + 1) * ALS_U_DELAY_PER_ATIME  /1000);
	//msleep((chip->atime + 1) * ALS_U_DELAY_PER_ATIME  /1000);
	return ret;
}*/

/*shut down cm36781 als func*/
static int cm36781_als_function_off(struct cm36781_chip *chip)
{
	int ret  = 0;
	if(!chip->als_functioned){
			pr_err("%s:[ALS] als has already functioned off!\n", __func__);            
	}else {
		pr_err("%s:[ALS] als is alwaws on, no need off\n", __func__);    

		chip->ls_cmd |= CM36781_ALS_SD;
		ret = i2c_smbus_write_word_data(chip->client, ALS_CONF, chip->ls_cmd);
		if (ret < 0) {
			pr_err("%s:[ALS] set als conf fail\n", __func__);
			return -EIO;
		}

		chip->als_functioned = false;
	}
	return ret;
}

static int cm36781_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct cm36781_chip *chip =
		container_of(sensors_cdev, struct cm36781_chip, als_cdev);
	int ret = 0;

	sensors_marker("cm36781_als_set_enable()",
		"receive", "als_en", enable ? "1" : "0");

	if (enable != chip->als_enabled) {
		if (enable) {
			//ret = cm36781_als_init_paras(chip);
			//ret = cm36781_als_function_on(chip);
			if (ret < 0){
				pr_err("%s:[ALS] turn on cm36781 error, cease the als func!\n", __func__);
			}else {
				chip->first_als_value = true;
				queue_delayed_work(chip->workqueue, &chip->als_dwork,
					usecs_to_jiffies(chip->als_u_delay));
			}
		} else {
			cancel_delayed_work(&chip->als_dwork);
			//ret = cm36781_als_function_off(chip);
			if (ret < 0){
				pr_err("%s:[ALS] turn off cm36781 error, do not re-schedule the als_work!\n", __func__);
			}
		}
		chip->als_enabled = enable;
	}
	return ret;
}

static int cm36781_get_lux(struct cm36781_chip *chip)
{
	chip->als_inf.lux = chip->als_inf.raw;
	if (chip->als_inf.lux < 0)
		chip->als_inf.lux = 0;
	else if (chip->als_inf.lux > 65535)
		chip->als_inf.lux = 65535;
	return chip->als_inf.lux;
}

static void cm36781_als_dwork_handler(struct work_struct *work)
{
	struct cm36781_chip *chip =
		container_of((struct delayed_work *)work,
			struct cm36781_chip, als_dwork);
	struct i2c_client *client = chip->client;

	if (0 == chip->als_enabled) {
		dev_err(&client->dev,
			"%s(): als not enabled, do not re-schedule als_dwork!\n", __func__);
	} else {
		int ret = cm36781_read_als(chip);
		if (0 > ret) {
			dev_err(&client->dev,
				"%s(): cm36781_read_als() failed! Do not re-schedule als_dwork!\n",
				__func__);
		} else {
			cm36781_get_lux(chip);
			if(chip->first_als_value){
				dev_err(&client->dev,
				"first als value %d\n",chip->als_inf.lux);
				chip->first_als_value = false;
			}	
			sensors_report_2(chip->a_idev,
				chip->als_inf.lux, chip->als_inf.raw,
				chip->als_resolution, &confuse_i);

			queue_delayed_work(chip->workqueue, &chip->als_dwork,
				usecs_to_jiffies(chip->als_u_delay));
		}
	}
	return;
}

/*****************/
/* ABI Functions */
/*****************/
static ssize_t cm36781_als_atime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(als_cdev, struct cm36781_chip, als_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->atime);
}

static ssize_t cm36781_als_atime_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(als_cdev, struct cm36781_chip, als_cdev);
	unsigned long atime;
	int rc = kstrtoul(buf, 10, &atime);
	if (rc)
		return -EINVAL;

	chip->atime = atime;
	return size;
}
static DEVICE_ATTR(als_atime, 0640,
	cm36781_als_atime_show, cm36781_als_atime_store);

static ssize_t cm36781_als_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(als_cdev, struct cm36781_chip, als_cdev);

	u16 conf = 0;
	u16 thd_h = 0;
	u16 thd_l = 0;

	conf = i2c_smbus_read_word_data(chip->client, ALS_CONF);
	thd_h = i2c_smbus_read_word_data(chip->client, ALS_THDH);
	thd_l = i2c_smbus_read_word_data(chip->client, ALS_THDL);

	return snprintf(buf, PAGE_SIZE,
		"conf =   %4x\thd_h  =   %4x\thd_l  =   %4x\n",
			conf, thd_h, thd_l);
}
static DEVICE_ATTR(als_regs, 0440, cm36781_als_regs_show, NULL);

static ssize_t cm36781_als_adc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(als_cdev, struct cm36781_chip, als_cdev);
	
	cm36781_read_als(chip);
	cm36781_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "LUX: %d RAW: %d\n",
		chip->als_inf.lux,chip->als_inf.raw);
}
static DEVICE_ATTR(als_adc, 0440, cm36781_als_adc_show, NULL);

static ssize_t als_fittness_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip = container_of(sensors_cdev,
			struct cm36781_chip, als_cdev);
	return snprintf(buf, 20,
		"%d\n", chip->als_fittness);
}
static ssize_t als_fittness_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip = container_of(sensors_cdev,
			struct cm36781_chip, als_cdev);
	int value = 0;
	sscanf(buf, "%d", &value);
	chip->als_fittness = value;
	return count;
}
static DEVICE_ATTR(als_fittness, 0660, als_fittness_show, als_fittness_store);


struct attribute *cm36781_als_attributes[] = {
	&dev_attr_als_atime.attr,
	&dev_attr_als_regs.attr,
	&dev_attr_als_adc.attr,
	&dev_attr_als_fittness.attr,
    NULL
};

const struct attribute_group cm36781_als_attr_group = {
	.attrs = cm36781_als_attributes,
};

static int cm36781_parse_als_dt(struct device *dev,
		struct cm36781_chip *chip)
{
	struct device_node *np = dev->of_node;
	u32 tmp;

	int rc = of_property_read_u32(np, "als,extend_num", &tmp);
	if (rc) {
		dev_err(dev,
			"%s(): Unable to read extend_num! Treat as 1 ALS sensor!\n",
			__func__);
		tmp = 0;
	}

	if (tmp > 0) {
		// Find a extend als sensor now, try to parse the description.
		const char *position;
		rc = of_property_read_string_index(np, "als,position", 0, &position);
		if (0 > rc) {
			dev_err(dev, "%s(): position can not be parsed!\n", __func__);
			goto quit_parse;
		}
		sensors_construct_extend_name(cm36781_als_name_base,
			LIGHT_NAME, position, tmp);
		cm36781_als_cdev.type =
			sensors_construct_extend_type_num(SENSOR_TYPE_LIGHT,
			position, tmp);
		dev_err(dev, "%s(): get ALS dev type: %d\n",
			__func__, cm36781_als_cdev.type);
	} else {
		strncpy(cm36781_als_name_base, LIGHT_NAME, strlen(LIGHT_NAME));
		cm36781_als_cdev.type = SENSOR_TYPE_LIGHT;
	}
	cm36781_als_cdev.name = cm36781_als_name_base;
	dev_err(dev, "%s(): get ALS dev name: %s\n",
		__func__, cm36781_als_cdev.name);

	/* ALS tuning value */
	rc = of_property_read_u32(np, "capella,als_time", &tmp);
	if (rc) {
		dev_err(dev, "%s(): Unable to read als_time, use default!\n", __func__);
		chip->atime = 0;
	} else {
		chip->atime = tmp & 0x3;
	}
	/* ALS fittness value, using to calculate lux just for capella devices*/
	rc = of_property_read_u32(np, "capella,als_fittness", &tmp);
	if (rc) {
		dev_err(dev, "%s(): Unable to read als_fittness, use default!\n", __func__);
		chip->als_fittness = 10;
	} else {
		chip->als_fittness = tmp;
	}
	return 0;
quit_parse:

	return rc;
}

static int cm36781_als_constructor(struct cm36781_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret = cm36781_parse_als_dt(&client->dev, chip);
	if (0 > ret) {
		dev_err(&client->dev,
			"%s(): failed to parse als dts!\n", __func__);
		return ret;
	}
	chip->a_idev =
		sensors_construct_input_dev(&client->dev,
			cm36781_als_name_base, 2, chip);
	if (chip->a_idev == NULL) {
		dev_err(&client->dev,
			"%s(): failed to construct als_input_dev!\n", __func__);
		goto quit_0;
	}

	/*Register  Als class */
	chip->als_cdev = cm36781_als_cdev;
	chip->als_cdev.sensors_enable = &cm36781_als_set_enable;
	chip->als_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&chip->a_idev->dev, &chip->als_cdev);
	if (ret) {
		dev_err(&client->dev,
			"Unable to register to sensors class light.\n");
		goto quit_1;
    }

	ret = sysfs_create_group(&((chip->als_cdev.dev)->kobj), &cm36781_als_attr_group);
	if (ret) {
		dev_err(&client->dev,
			"Unable to creat sysfs als.\n");
		goto quit_2;
	}
	ret = cm36781_als_init_paras(chip);
	INIT_DELAYED_WORK(&chip->als_dwork, cm36781_als_dwork_handler);
	chip->als_enabled = 0;
	return 0;
quit_2:
	sensors_classdev_unregister(&chip->als_cdev);
quit_1:
	input_unregister_device(chip->a_idev);
	//input_free_device(chip->a_idev);
quit_0:
	return ret;
}

static void cm36781_als_clean_up(struct cm36781_chip *chip)
{
	sysfs_remove_group(&((chip->als_cdev.dev)->kobj), &cm36781_als_attr_group);
	sensors_classdev_unregister(&chip->als_cdev);
	input_unregister_device(chip->a_idev);
	//input_free_device(chip->a_idev);
}

/**************************/
/* General Prox Functions */
/**************************/

static void psensor_set_duty(struct cm36781_chip *chip)
{
	chip->ps_conf1_2_val &= CM36781_PS_DR_1_MASK;
	chip->ps_conf1_2_val |= ps_duty_array[chip->ps_duty];
}

static void psensor_set_persistence(struct cm36781_chip *chip)
{
	chip->ps_conf1_2_val &= CM36781_PS_PERS_MASK;
	chip->ps_conf1_2_val |= ps_pers_array[chip->ps_pers];
}

static void psensor_set_it(struct cm36781_chip *chip)
{
	chip->ps_conf1_2_val &= CM36781_PS_IT_MASK;
	chip->ps_conf1_2_val |= ps_it_array[chip->ptime];
}

static void psensor_set_multi_pulse(struct cm36781_chip *chip)
{
	chip->ps_conf3_val &= CM36781_PS_MP_MASK;
	chip->ps_conf3_val |= ps_mp_array[chip->ps_mp];
}

static void psensor_set_led_drv(struct cm36781_chip *chip)
{
	chip->ps_conf3_val &= CM36781_LED_I_MASK;
	chip->ps_conf3_val |= ps_led_drv_array[chip->ps_led_drv];
}

static void cm36781_ps_get_state(struct cm36781_chip *chip)
{
	chip->ps_inf.state_changed = false;

	if (chip->ps_inf.raw > chip->ps_thd_close) {
		chip->ps_inf.state = NEAR_CODE;
	} else if (chip->ps_inf.raw < chip->ps_thd_away) {
		chip->ps_inf.state = FAR_CODE;
	}
	if (chip->ps_inf.state != chip->ps_inf.last_state) {
		chip->ps_inf.last_state = chip->ps_inf.state;
		chip->ps_inf.state_changed = true;
	}
}


static int cm36781_read_ps(struct cm36781_chip *chip)
{
	int ret;
	if (bReboot) {
		pr_err(
			"%s:[PS] not read, because reboot\n", __func__);
		return 0;
	}

	ret = i2c_smbus_read_word_data(chip->client, PS_DATA);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to read cm36781_REG_PDATA!\n", __func__);
	} else {
		chip->ps_inf.raw = ret & 0xFFFF;
		dev_err(&chip->client->dev,
			"%s(): get PS raw %d!\n", __func__, chip->ps_inf.raw);
	}
	return ret;
}

static int cm36781_ps_update_thresholds(struct cm36781_chip *chip,
		u16 high_thd, u16 low_thd)
{
	int ret = 0;
	ret = i2c_smbus_write_word_data(chip->client, PS_THDL, low_thd);
	ret = i2c_smbus_write_word_data(chip->client, PS_THDH, high_thd);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to write ps thresholds!\n", __func__);
	}
	return ret;
}
/*
static int cm36781_ps_update_offset(struct cm36781_chip *chip, u16 offset)
{
	int ret =
		i2c_smbus_write_word_data(chip->client, PS_CANC, offset);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to write ps offset!\n", __func__);
	}
	return ret;
}
*/
static int cm36781_ps_init_paras(struct cm36781_chip *chip)
{
	int ret = 0;
	/*choose ps config paras*/
	psensor_set_duty(chip);
	psensor_set_persistence(chip);
	psensor_set_it(chip);
	psensor_set_multi_pulse(chip);
	psensor_set_led_drv(chip);
	/*choos ps 16bits output data*/
	chip->ps_conf1_2_val |= CM36781_PS_HD_16;
	
	/*must disable p-sensor interrupt befrore IST create*//*disable PS func*/		
	chip->ps_conf1_2_val |= CM36781_PS_SD;
	chip->ps_conf1_2_val &= CM36781_PS_INT_MASK;  
	i2c_smbus_write_word_data(chip->client, PS_CONF1_2, chip->ps_conf1_2_val);   
	i2c_smbus_write_word_data(chip->client, PS_CONF3, chip->ps_conf3_val);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to write ps paras!\n", __func__);
	}
	return ret;
}

/** read and clear the irq state flags**/
static int cm36781_read_irq_status(struct cm36781_chip *chip)
{
	// Simplify read to clear all state in cm36781 INT_FLAG.
	return i2c_smbus_read_word_data(chip->client, INT_FLAG);
}

/** turn on ps of the chip to get the ps_canc before enable the ps func**/
static int cm36781_offset_calibration(struct cm36781_chip *chip)
{
	int ret = 0;

	chip->ps_thd_away = 0;
	chip->ps_thd_close = 0;
	ret = cm36781_ps_update_thresholds(chip, 0xFFFF, 0);
	ret = cm36781_read_irq_status(chip);
	ret = cm36781_ps_init_paras(chip);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to init paras!\n", __func__);
		return ret;
	}

	//enable ps to read ps raw_data
	chip->ps_conf1_2_val &= CM36781_PS_SD_MASK;
	chip->ps_conf1_2_val |= CM36781_PS_INT_IN_AND_OUT;
	ret = i2c_smbus_write_word_data(chip->client, PS_CONF1_2, chip->ps_conf1_2_val);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to start ps only!\n", __func__);
		return ret;
	}
	//Delay somewhile to make sure the ps can intergrate raw data directly
	msleep(50);
	ret = cm36781_read_ps(chip);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to get a ps data!\n", __func__);
		return ret;
	}

	// Disable Proximity and Proximity Interrupt
	chip->ps_conf1_2_val |= CM36781_PS_SD;
	chip->ps_conf1_2_val &= CM36781_PS_INT_MASK;
	ret = i2c_smbus_write_word_data(chip->client, PS_CONF1_2, chip->ps_conf1_2_val);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to shut down ps func!\n", __func__);
	}

	if (chip->ps_inf.raw < chip->ps_crosstalk_max) {
		if(chip->ps_inf.raw == 0){
			dev_err(&chip->client->dev,
			"%s(): pw_rawdata = %d, chip->poffset = %d, chip->ps_crosstalk_max = %d!\n", __func__, chip->ps_inf.raw, chip->poffset, chip->ps_crosstalk_max);
		}
		else{	
			chip->poffset = chip->ps_inf.raw;
			dev_err(&chip->client->dev,
				"%s(): ps calib OK, ps_canc = %d!\n", __func__, chip->poffset);
		}
	} else {
		dev_err(&chip->client->dev,
			"%s(): corsstalk-(%d) is larger than MAX-(%d)!\n", __func__,
			chip->ps_inf.raw, chip->ps_crosstalk_max);
	}

	if (0xFFFF != chip->poffset) {
		chip->ps_thd_close = chip->ps_th_max + chip->poffset;
		chip->ps_thd_away = chip->ps_th_min + chip->poffset;
		chip->ps_thd_more_close = chip->ps_thd_more_close_base + chip->poffset;
		if (chip->ps_thd_close >= 65535) {
			chip->ps_thd_close = 32768 + (chip->ps_thd_away / 2);
			dev_err(&chip->client->dev,
				"%s(): fix ps_thd_close = %d!\n",
				__func__, chip->ps_thd_close);
		}
		if (chip->ps_thd_more_close_base >= 65535){
			chip->ps_thd_more_close_base = 65535;
			chip->ps_thd_covered = 65535;
			dev_err(&chip->client->dev,
				"%s(): ps canc is too large set more_close&coverd thd to 0xffff!\n",
				__func__);
		}
		//chip->ps_thd_more_close = 32768 + chip->ps_thd_close / 2;
		dev_err(&chip->client->dev,
			"%s() end: get ps_thd_away = %d, ps_thd_close = %d, ps_thd_more_close = %d!\n",
			__func__, chip->ps_thd_away, chip->ps_thd_close, chip->ps_thd_more_close);
	} else {
		/* First time use prox after power on and crosstalk too large */
		dev_err(&chip->client->dev,
			"%s(): ps raw data is too large, use last ps_canc!",__func__);
	}

	return 0;
}

void cm36781_ps_thread(struct work_struct *work)
{
	struct cm36781_chip *chip
		= container_of(work, struct cm36781_chip, ps_work);
	int ret = 0;

	/*power on the V_led(5V) of the chip before turn on the ps func*/
	ret = cm36781_power_switch(chip, true);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s(): cm36781 power switch on error! cease the ps function!\n", __func__);
		return;
	}

	ret = cm36781_offset_calibration(chip);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): cm36781_offset_calibration IO failed when bootup\n", __func__);
	} else if (0xFFFF == chip->poffset) {
		dev_err(&chip->client->dev,
			"%s(): Something cover the lens caused ps calib failed when bootup\n",
			__func__);
	}

	/*power off the V_led(5V) of the chip after turn off the ps func*/
	ret = cm36781_power_switch(chip, false);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s(): cm36781 power switch off error! be attention!\n", __func__);
	}
}

static int cm36781_ps_function_on(struct cm36781_chip *chip)
{
	int ret = 0;

	/*power on the V_led(5V) of the chip before turn on the ps func*/
	ret = cm36781_power_switch(chip, true);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s(): cm36781 power switch on error! cease the ps function!\n", __func__);
		goto quit_0;
	}
	if (atomic_read(&chip->engtest_mode) == 1) {
		if (chip->fac_canc != 0xFFFF) {
			chip->poffset = chip->fac_canc;
			chip->ps_thd_close = chip->ps_th_max + chip->poffset;
			chip->ps_thd_away = chip->ps_th_min + chip->poffset;
			dev_err(&chip->client->dev,
				"%s(): EngMode, get ps_thd_away = %d, ps_thd_close = %d!\n",
				__func__, chip->ps_thd_away, chip->ps_thd_close);
		}
	} else {
		ret = cm36781_offset_calibration(chip);
		if (0 > ret) {
			dev_err(&chip->client->dev,
				"%s(): failed to execute offset calibration!\n", __func__);
			goto quit_0;
		}
	}

	if (0xFFFF != chip->poffset) {
		ret = cm36781_ps_update_thresholds(chip, chip->ps_thd_close, chip->ps_thd_away);
		chip->ps_functioned = true;
		chip->ps_conf1_2_val &= CM36781_PS_SD_MASK;
		chip->ps_conf1_2_val |= CM36781_PS_INT_IN_AND_OUT;
		ret = i2c_smbus_write_word_data(chip->client, PS_CONF1_2, chip->ps_conf1_2_val);
		if (0 > ret) {
		    dev_err(&chip->client->dev,
			    "%s(): failed to write ps_enable register!\n", __func__);
			chip->ps_functioned = false;
		    goto quit_0;
	    }

		if (chip->ps_inf.raw < chip->ps_thd_close) {
			dev_err(&chip->client->dev,
				"%s(): raw = %d, thd_close = %d!\n",
				__func__, chip->ps_inf.raw, chip->ps_thd_close);
			sensors_report_1(chip->p_idev, FAR_CODE);
			chip->ps_inf.last_state = FAR_CODE;
		}
	} else {
		dev_err(&chip->client->dev,
			"%s(): Since can't got a valid ps calib, cease the ps function!\n",
			__func__);
		chip->ps_conf1_2_val |= CM36781_PS_SD;
		chip->ps_conf1_2_val &= CM36781_PS_INT_MASK;  
		ret = i2c_smbus_write_word_data(chip->client, PS_CONF1_2, chip->ps_conf1_2_val);
		if (0 > ret) {
			dev_err(&chip->client->dev,
				"%s(): failed to shut down ps func!\n", __func__);
		}
		sensors_report_1(chip->p_idev, FAR_CODE);
		chip->ps_inf.last_state = FAR_CODE;
	}

quit_0:
	return ret;
}

static int cm36781_ps_function_off(struct cm36781_chip *chip)
{
	int ret = 0;

	// Disable Proximity and Proximity Interrupt
	chip->ps_conf1_2_val |= CM36781_PS_SD;
	chip->ps_conf1_2_val &= CM36781_PS_INT_MASK;
	ret = i2c_smbus_write_word_data(chip->client, PS_CONF1_2, chip->ps_conf1_2_val);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to shut down ps func!\n", __func__);
	}
	ret = cm36781_read_irq_status(chip);
	/*power off the V_led(5V) of the chip after turn off the ps func*/
	ret = cm36781_power_switch(chip, false);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s(): cm36781 power switch off error! be attention!\n", __func__);
	}

	if (atomic_read(&chip->ps_poll_work_on) == 1) {
		cancel_delayed_work(&chip->ps_poll_dwork);
		atomic_set(&chip->ps_poll_work_on, 0);
	}
	sensors_report_1(chip->p_idev, FAR_CODE);
	chip->ps_inf.last_state = FAR_CODE;
	chip->ps_functioned = false;

	return ret;
}

static int cm36781_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct cm36781_chip *chip =
		container_of(sensors_cdev, struct cm36781_chip, ps_cdev);
	int ret = 0;

	sensors_marker("cm36781_ps_set_enable()",
		"receive", "ps_en", enable ? "1" : "0");

	if (enable != chip->ps_enabled) {
		chip->ps_inf.state = -1;
		chip->ps_inf.last_state = -1;
		chip->ps_inf.near_last_count = 0;
		ret = enable ?
			cm36781_ps_function_on(chip) :
			cm36781_ps_function_off(chip);
		chip->ps_enabled = enable;
	}

	return ret;
}

/*add flush func to resolve CTS issues*/
static int cm36781_ps_flush(struct sensors_classdev *sensors_cdev)
{
	struct cm36781_chip *chip =
		container_of(sensors_cdev, struct cm36781_chip, ps_cdev);
	int ret = 0;
	sensors_marker("cm36781_ps_flush()",
		"conducted", "#", "#");

	if (chip->ps_functioned) {
		ret = cm36781_read_ps(chip);
		cm36781_ps_get_state(chip);
		dev_err(&chip->client->dev,
			"%s(): %s reported while PS enabled!\n", __func__,
			(chip->ps_inf.state) ? "FAR_TO_NEAR" : "NEAR_TO_FAR");
	} else {
		dev_err(&chip->client->dev,
			"%s(): %s reported while PS disabled!\n", __func__,
			(chip->ps_inf.state) ? "FAR_TO_NEAR" : "NEAR_TO_FAR");
	}
	sensors_report_1(chip->p_idev, chip->ps_inf.state);
	return 0;
}


/*****************/
/* ABI Functions */
/*****************/
static ssize_t cm36781_device_ps_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	int ret = -EPERM;
	int val = 0;
	if (chip->ps_enabled) {
		ret = cm36781_read_ps(chip);
		val = ((chip->ps_inf.raw - chip->poffset) > 0) ?
			(chip->ps_inf.raw - chip->poffset) : 0;
		if (ret > 0)
			ret = 0;
	}

	return ret ? ret : snprintf(buf, PAGE_SIZE, "%d\n", val);
}
static DEVICE_ATTR(ps_data, 0440, cm36781_device_ps_raw, NULL);

static ssize_t cm36781_device_ps_detected(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	return (chip->ps_enabled) ?
		snprintf(buf, PAGE_SIZE, "%s\n", chip->ps_inf.state ? "Near" : "Far") : -EPERM;
}
static DEVICE_ATTR(ps_state, 0440, cm36781_device_ps_detected, NULL);

static ssize_t cm36781_ps_crosstalk_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_crosstalk_max);
}

static ssize_t cm36781_ps_crosstalk_max_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->ps_crosstalk_max = (u16)val;
	return size;
}
static DEVICE_ATTR(ps_crosstalk_maxthd, 0640,
	cm36781_ps_crosstalk_max_show, cm36781_ps_crosstalk_max_store);

static ssize_t cm36781_ps_pdrv_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_led_drv);
}

static ssize_t cm36781_ps_pdrv_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->ps_led_drv = (u16)(val & 0x7);
	return size;
}
static DEVICE_ATTR(ps_pdrv, 0640,
	cm36781_ps_pdrv_show, cm36781_ps_pdrv_store);

static ssize_t cm36781_ps_ptime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ptime);
}

static ssize_t cm36781_ps_ptime_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->ptime = (u16)(val & 0x7);
	return size;
}
static DEVICE_ATTR(ps_ptime, 0640,
	cm36781_ps_ptime_show, cm36781_ps_ptime_store);

static ssize_t cm36781_ps_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->poffset);
}
static DEVICE_ATTR(ps_canc, 0440, cm36781_ps_offset_show, NULL);

static ssize_t cm36781_ps_threshold_high_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_th_max);
}

static ssize_t cm36781_ps_threshold_high_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	unsigned long temp;
	int rc = kstrtoul(buf, 10, &temp);
	if (rc)
		return -EINVAL;
	if (temp <= 0) {
		return -EINVAL;
	}
	chip->ps_th_max = (temp > 65535) ? 65535 : temp;
	return size;
}
static DEVICE_ATTR(ps_thd_close, 0640,
	cm36781_ps_threshold_high_show, cm36781_ps_threshold_high_store);

static ssize_t cm36781_ps_threshold_low_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_th_min);
}

static ssize_t cm36781_ps_threshold_low_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	unsigned long temp;
	int rc = kstrtoul(buf, 10, &temp);
	if (rc)
		return -EINVAL;
	if (temp <= 0) {
		return -EINVAL;
	}
	chip->ps_th_min = (temp > 65536) ? 65535 : temp;

	return size;
}
static DEVICE_ATTR(ps_thd_away, 0640,
	cm36781_ps_threshold_low_show, cm36781_ps_threshold_low_store);

static ssize_t cm36781_ps_regs_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);

	u16 conf1_2 = 0;
	u16 conf3 = 0;

	conf1_2 = i2c_smbus_read_word_data(chip->client, PS_CONF1_2);
	conf3 = i2c_smbus_read_word_data(chip->client, PS_CONF3);

	return snprintf(buf, PAGE_SIZE,
		"PS_CONF1_2 =   %2x\nPS_CONF3 =   %2x\n",
			conf1_2, conf3);
}
static DEVICE_ATTR(ps_regs, 0440, cm36781_ps_regs_show, NULL);

static ssize_t cm36781_reg_read_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);

	u8 cmd_addr = 0;
	u16 reg_val = 0;
	if (sscanf(buf, "%x", &cmd_addr) != 1) {
		pr_err("%s: The number of data are wrong\n", __func__);
		return -EINVAL;
	}
	reg_val = i2c_smbus_read_word_data(chip->client, cmd_addr);
	if (reg_val < 0)
		pr_err("%s: read 0x%x reg failed!", __func__, cmd_addr);
	else
		pr_err("%s: read the value of 0x%x reg is 0x%x!", __func__, cmd_addr, reg_val);

	return size;
}
static DEVICE_ATTR(regs_read, 0660, NULL, cm36781_reg_read_store);

static ssize_t cm36781_reg_write_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);

	u8 cmd_addr = 0;
	u16 reg_val = 0;
	if (sscanf(buf, "%x,%x", &cmd_addr, &reg_val) != 2) {
		pr_err("%s: The number of data are wrong\n",__func__);
		return -EINVAL;
	}
	reg_val = i2c_smbus_write_word_data(chip->client, cmd_addr, reg_val);
	if (reg_val < 0)
		pr_err("%s: write 0x%x reg failed!", __func__, cmd_addr);
	else
		pr_err("%s: write the value to 0x%x reg is 0x%x!", __func__, cmd_addr, reg_val);

	return size;
}
static DEVICE_ATTR(regs_write, 0660, NULL, cm36781_reg_write_store);

static ssize_t cm36781_ps_engmode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
		atomic_read(&chip->engtest_mode));
}

static ssize_t cm36781_ps_engmode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	unsigned long temp;
	int rc = kstrtoul(buf, 10, &temp);
	if (rc)
		return -EINVAL;
	atomic_set(&chip->engtest_mode, temp);

	return size;
}
static DEVICE_ATTR(ps_engmode, 0640,
	cm36781_ps_engmode_show, cm36781_ps_engmode_store);

static ssize_t cm36781_ps_factory_canc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->fac_canc);
}

static ssize_t cm36781_ps_factory_canc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct cm36781_chip *chip =
		container_of(ps_cdev, struct cm36781_chip, ps_cdev);
	unsigned long temp;
	int rc = kstrtoul(buf, 10, &temp);
	if (rc)
		return -EINVAL;
	chip->fac_canc = temp & 0xFFFF;

	return size;
}
static DEVICE_ATTR(ps_factory_canc, 0640,
	cm36781_ps_factory_canc_show, cm36781_ps_factory_canc_store);


struct attribute *cm36781_ps_attributes[] = {
	&dev_attr_ps_state.attr,
	&dev_attr_ps_regs.attr,
	&dev_attr_ps_thd_close.attr,
	&dev_attr_ps_thd_away.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_ps_canc.attr,
	&dev_attr_ps_crosstalk_maxthd.attr,
	&dev_attr_ps_pdrv.attr,
	&dev_attr_ps_ptime.attr,
	&dev_attr_regs_read.attr,
	&dev_attr_regs_write.attr,
	&dev_attr_ps_engmode.attr,
	&dev_attr_ps_factory_canc.attr,
	NULL
};

const struct attribute_group cm36781_ps_attr_group = {
	.attrs = cm36781_ps_attributes,
};

static int cm36781_irq_process(struct cm36781_chip *chip)
{
	struct i2c_client *client = chip->client;
	bool ps_emergence = false;
	u16 status = 0;

	int ret = cm36781_read_irq_status(chip);
	if (0 > ret) {
		dev_err(&client->dev,
			"%s(): failed to get cm36781 INT_FLAG reg!\n", __func__);
		ps_emergence = (chip->ps_functioned) ? true : false;
		goto err_process_0;
	}
	
	//status = (ret & INT_FLAG_MASK) >> 8;
	status = ret;
	dev_err(&client->dev, "%s(): Got irq state = 0x%02x\n", __func__, status);
	if (status == 0) {
		dev_err(&client->dev, "%s(): Nothing happend, but irq triggered!\n", __func__);
		ps_emergence = (chip->ps_functioned) ? true : false;
		goto err_process_0;
	}
	ret = cm36781_read_ps(chip);
	if (0 > ret)
		goto err_process_0;

	if (chip->ps_functioned) {
		if (status & INT_FLAG_PS_IF_CLOSE){
			if (atomic_read(&chip->engtest_mode) == 0 && atomic_read(&chip->ps_poll_work_on) == 0) {
				//schedule the ps delay work to determine the close/more_close/covered state and report
				queue_delayed_work(chip->workqueue, &chip->ps_poll_dwork,
					msecs_to_jiffies(500));
				atomic_set(&chip->ps_poll_work_on, 1);
			} else {
				sensors_report_1(chip->p_idev, NEAR_CODE);
				chip->ps_inf.last_state = NEAR_CODE;
				dev_err(&client->dev, "%s(): near detected in engmode!\n", __func__);
			}
		}else if (status & INT_FLAG_PS_IF_AWAY){
			if (atomic_read(&chip->engtest_mode) == 0 && atomic_read(&chip->ps_poll_work_on) == 1) {
				cancel_delayed_work(&chip->ps_poll_dwork);
				atomic_set(&chip->ps_poll_work_on, 0);
			}
			if (chip->ps_inf.last_state != FAR_CODE) {
				sensors_report_1(chip->p_idev, FAR_CODE);
				chip->ps_inf.last_state = FAR_CODE;
			}
			dev_err(&client->dev, "%s(): far detected!\n", __func__);
		}else {
			dev_err(&client->dev, "%s(): unexpected thing happend!\n", __func__);
		}
	}
	goto quit;

err_process_0:
	if (ps_emergence)
		sensors_report_1(chip->p_idev, FAR_CODE);
		chip->ps_inf.last_state = FAR_CODE;
quit:
	if (!(chip->wake_src.active)) {
		__pm_wakeup_event(&(chip->wake_src), jiffies_to_msecs(HZ));
	}
	return 0;  // we handled the interrupt
}

irqreturn_t cm36781_irq_handler(int irq, void *handle)
{
	struct cm36781_chip *chip = handle;
	struct i2c_client *client = chip->client;
	int ret = 0;

	sensors_marker("cm36781_irq()", "triggered with",
		chip->ps_functioned ? "ps enabled" : "ps disabled", "#");

	if (chip->ps_functioned) {
		disable_irq_nosync(chip->wake_irq);
		ret = cm36781_irq_process(chip);
		enable_irq(chip->wake_irq);
	} else {
		dev_err(&client->dev,
			"%s(): prox function was not enabled!\n", __func__);
	}

	return IRQ_HANDLED;
}

static int cm36781_ps_work_process(struct cm36781_chip *chip)
{
	int ret = -1;

	ret = cm36781_read_ps(chip);
	if (0 > ret)
		goto far_quit;

	cm36781_ps_get_state(chip);

	if (chip->ps_inf.state_changed) {
		if (chip->ps_inf.state == FAR_CODE){
			dev_err(&chip->client->dev,
				"%s(): FAR state reported, Do not re-schedule ps_dwork!\n",
					__func__);
			goto far_quit;
		}
		if (chip->ps_inf.state == NEAR_CODE)
			chip->ps_inf.near_last_count = 1;
	} else {
		if (chip->ps_inf.state == NEAR_CODE)
			chip->ps_inf.near_last_count++;
		if (chip->ps_inf.near_last_count > 3) {
			if (chip->ps_inf.raw > chip->ps_thd_covered) {
				sensors_report_1(chip->p_idev, COVERED_CODE);
				dev_err(&chip->client->dev,
					"%s(): COVERED reported!\n", __func__);
			} else if (chip->ps_inf.raw > chip->ps_thd_more_close) {
				sensors_report_1(chip->p_idev, TOO_NEAR_CODE);
				dev_err(&chip->client->dev,
					"%s(): VERY_NEAR reported!\n", __func__);
			} else {
				sensors_report_1(chip->p_idev, NEAR_CODE);
				dev_err(&chip->client->dev,
					"%s(): NEAR reported!\n", __func__);
			}
			chip->ps_inf.near_last_count = 0;
		}
	}
	queue_delayed_work(chip->workqueue, &chip->ps_poll_dwork,
		msecs_to_jiffies(500));
	return 0;
far_quit:
	//sensors_report_1(chip->p_idev, FAR_CODE);
	atomic_set(&chip->ps_poll_work_on, 0);
	return ret;
}


static void cm36781_ps_poll_dwork_handler(struct work_struct *work)
{
	struct cm36781_chip *chip =
		container_of((struct delayed_work *)work,
			struct cm36781_chip, ps_poll_dwork);
	struct i2c_client *client = chip->client;

	if (0 == chip->ps_enabled) {
		dev_err(&client->dev,
			"%s(): ps not enabled, do not re-schedule ps_poll_dwork!\n", __func__);
	} else {
		cm36781_ps_work_process(chip);
	}
	return;
}


/**
static int cm36781_pinctrl_init(struct cm36781_chip *chip)
{
	struct i2c_client *client = chip->client;

	chip->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		dev_err(&client->dev,
			"%s(): Failed to get pinctrl\n", __func__);
		return PTR_ERR(chip->pinctrl);
	}

	chip->pin_default =
		pinctrl_lookup_state(chip->pinctrl, "default");
	if (IS_ERR_OR_NULL(chip->pin_default)) {
		dev_err(&client->dev,
			"%s(): Failed to look up default state\n", __func__);
		return PTR_ERR(chip->pin_default);
	}

	if (pinctrl_select_state(chip->pinctrl, chip->pin_default)) {
		dev_err(&client->dev,
			"%s(): Can't select pinctrl default state.\n", __func__);
		return -EAGAIN;
	}

	return 0;
}
**/

static int cm36781_irq_init(struct device *dev, struct cm36781_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct device_node *np = dev->of_node;
	int ret = 0;
	/* irq gpio */
	ret = of_get_named_gpio(np, "capella,interrupt-gpio", 0);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to read irq gpio\n");
		return ret;
	}
	chip->irq_gpio = ret;
	pr_err("%s the gpio num parse from dts is %d\n", __func__, chip->irq_gpio);
	/**
	ret = cm36781_pinctrl_init(chip);
	if (ret) {
		dev_err(&client->dev,
			"%s(): Can't initialize pinctrl.\n", __func__);
		goto pinctrl_failed;
	}
	**/
	ret = gpio_request(chip->irq_gpio, cm36781_ps_name_base);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): gpio %d request failed (%d), and the label is %s\n", __func__,
			chip->irq_gpio, ret, cm36781_ps_name_base);
		goto pinctrl_failed;
	}

	ret = gpio_direction_input(chip->irq_gpio);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): fail to set gpio %d as input (%d)\n", __func__,
			chip->irq_gpio, ret);
		goto fail_free_intr_pin;
	}

	chip->wake_irq = gpio_to_irq(chip->irq_gpio);
	if (chip->wake_irq < 0) {
		dev_err(&client->dev,
			"%s(): Failed to set irq %d\n", __func__, chip->wake_irq);
		goto fail_free_intr_pin;
	}
	client->irq = chip->wake_irq;
	ret = request_threaded_irq(chip->wake_irq,
							NULL,
							cm36781_irq_handler,
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							cm36781_ps_name_base,
							chip);
	if (ret) {
		dev_err(&client->dev,
			"%s(): Failed to request irq %d\n", __func__, chip->wake_irq);
		goto fail_free_intr_pin;
	}

	ret = enable_irq_wake(chip->wake_irq);
	if (0 != ret) {
		dev_err(&client->dev,
			"%s(): enable_irq_wake failed for %s.\n",
			__func__, cm36781_ps_name_base);
		goto fail_set_irq_wake;
	}

	return 0;
fail_set_irq_wake:
	free_irq(chip->wake_irq, chip);
fail_free_intr_pin:
	gpio_free(chip->irq_gpio);
pinctrl_failed:
	return ret;
}

static int cm36781_parse_ps_dt(struct device *dev,
		struct cm36781_chip *chip)
{
	struct device_node *np = dev->of_node;
	u32 tmp;
	int rc = of_property_read_u32(np, "ps,extend_num", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps_extend_num! It is only one PS sensor!\n");
		tmp = 0;
	}

	if (tmp > 0) {
		// Find a extend ps sensor now, try to parse the description.
		const char *position;
		rc = of_property_read_string_index(np, "ps,position", 0, &position);
		if (0 > rc) {
			dev_err(dev, "position can not be parsed!\n");
			goto quit_parse;
		}
		sensors_construct_extend_name(cm36781_ps_name_base,
			PROXIMITY_NAME, position, tmp);
		cm36781_ps_cdev.type =
			sensors_construct_extend_type_num(SENSOR_TYPE_PROXIMITY,
			position, tmp);
	} else {
		strncpy(cm36781_ps_name_base, PROXIMITY_NAME, strlen(PROXIMITY_NAME));
		cm36781_ps_cdev.type = SENSOR_TYPE_PROXIMITY;
	}
	cm36781_ps_cdev.name = cm36781_ps_name_base;
	dev_err(dev, "get dev name: %s\n", cm36781_ps_cdev.name);

	/* ps tuning data*/
	rc = of_property_read_u32(np, "capella,ps_th_max", &tmp);
	if (rc)
		dev_err(dev, "Unable to read capella,ps_th_max\n");
	else
		chip->ps_th_max = tmp & 0xFFFF;

	rc = of_property_read_u32(np, "capella,ps_th_min", &tmp);
	if (rc)
		dev_err(dev, "Unable to read capella,ps_th_min\n");
	else
		chip->ps_th_min = tmp & 0xFFFF;

	rc = of_property_read_u32(np, "capella,ps_duty", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read capella,ps_duty\n");
		chip->ps_duty = 15;
	} else {
		chip->ps_duty = tmp;
	}
	
	rc = of_property_read_u32(np, "capella,ps_persist", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read capella,persist\n");
		chip->ps_pers = 1;
	} else {
		chip->ps_pers = tmp;
	}

	rc = of_property_read_u32(np, "capella,ptime", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read capella,ptime\n");
		chip->ptime = 31;
	} else {
		chip->ptime = tmp;
	}

	rc = of_property_read_u32(np, "capella,ps_led_drv", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read capella,ps_led_drv\n");
		chip->ps_led_drv = 16;
	} else {
		chip->ps_led_drv = tmp;
	}

	rc = of_property_read_u32(np, "capella,ps_multi_pulse", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read capella,ps_multi_pulse\n");
		chip->ps_mp = 1;
	} else {
		chip->ps_mp = tmp;
	}

	rc = of_property_read_u32(np, "capella,ps_crosstalk_max", &tmp);
	if (rc) {
		chip->ps_crosstalk_max = 100;
		dev_err(dev,
			"ps_crosstalk_max do not set and use default value%d\n",
				chip->ps_crosstalk_max);
	} else {
		chip->ps_crosstalk_max = (u16)tmp;
	}
	rc = of_property_read_u32(np, "capella,ps_very_near_thd", &tmp);
	if (rc) {
		chip->ps_thd_more_close_base = 1000;
		dev_err(dev,
			"ps_thd_more_close_base do not set and use default value%d\n",
				chip->ps_thd_more_close_base);
	} else {
		chip->ps_thd_more_close_base = (u16)tmp;
	}
	rc = of_property_read_u32(np, "capella,ps_covered_thd", &tmp);
	if (rc) {
		chip->ps_thd_covered = 10000;
		dev_err(dev,
			"ps_thd_covered do not set and use default value%d\n",
				chip->ps_thd_covered);
	} else {
		chip->ps_thd_covered = (u16)tmp;
	}
	return 0;
quit_parse:
	return rc;
}

static int cm36781_ps_constructor(struct cm36781_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret = cm36781_parse_ps_dt(&client->dev, chip);
	if (0 > ret) {
		dev_err(&client->dev,
			"%s(): failed to parse ps dts!\n", __func__);
		goto quit_0;
	}

	chip->p_idev =
		sensors_construct_input_dev(&client->dev,
			cm36781_ps_name_base, 1, chip);
	if (chip->p_idev == NULL) {
		dev_err(&client->dev,
			"%s(): failed to construct ps_input_dev!\n", __func__);
		goto quit_0;
	}

	/* Register  Prox class */
	chip->ps_cdev = cm36781_ps_cdev;
	chip->ps_cdev.sensors_enable = cm36781_ps_set_enable;
	chip->ps_cdev.sensors_flush = cm36781_ps_flush;
	chip->ps_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&chip->p_idev->dev, &chip->ps_cdev);
	if (ret) {
		dev_err(&client->dev,
			"%s(): Unable to register to sensors class.proximity\n", __func__);
		goto quit_1;
	}

	/* Create Prox sysfs */	
	ret = sysfs_create_group(&((chip->ps_cdev.dev)->kobj), &cm36781_ps_attr_group);
	if (ret) {
		dev_err(&client->dev, "%s(): Unable to creat sysfs ps.\n", __func__);
		goto quit_2;
	}

	chip->poffset = 0xFFFF;  /*for first time use prox when power on*/
	chip->fac_canc = 0xFFFF;
	chip->ps_enabled = 0;
	chip->ps_functioned = false;
	atomic_set(&chip->engtest_mode, 0);
	atomic_set(&chip->ps_poll_work_on, 0);

	INIT_WORK(&chip->ps_work, cm36781_ps_thread);
	INIT_DELAYED_WORK(&chip->ps_poll_dwork, cm36781_ps_poll_dwork_handler);
	queue_work(chip->workqueue, &chip->ps_work);
	return 0;

quit_2:
	sensors_classdev_unregister(&chip->ps_cdev);
quit_1:
	input_unregister_device(chip->p_idev);
	//input_free_device(chip->p_idev);
quit_0:
	return ret;
}

static void cm36781_ps_clean_up(struct cm36781_chip *chip)
{
	//wake_lock_destroy(&(chip->wlock));
	//disable_irq_wake(chip->wake_irq);
	//disable_irq(chip->wake_irq);
	//free_irq(chip->wake_irq, chip);
	//gpio_free(chip->irq_gpio);
	sysfs_remove_group(&((chip->ps_cdev.dev)->kobj), &cm36781_ps_attr_group);
	sensors_classdev_unregister(&chip->ps_cdev);
	input_unregister_device(chip->p_idev);
	//input_free_device(chip->p_idev);
}

/********************************************************************/
/* Validate the appropriate cm36781 device is available for this driver */
/********************************************************************/
static int cm36781_check_chip_id(struct cm36781_chip *chip)
{
	int id = 0;
	int retry_count = 5;
	while(retry_count--){
		id = i2c_smbus_read_word_data(chip->client, ID_REG);
		if (0 > id) {
			dev_err(&chip->client->dev,
				"%s(): failed to read out ChipIDs -- %d!\n", __func__, id);
			if(0 == retry_count)
				return -EIO;
		} else {
			if (id != CM36781_ID){
				dev_err(&chip->client->dev,
					"%s(): Searching for Capella chip failed! id = 0x%x\n",
						__func__, id);
				if(0 == retry_count)
					return -ENODEV;
			}else {
				dev_err(&chip->client->dev,
					"%s(): find Capella CM36781\n", __func__);
				return 0;
			}
		}
		msleep(15);
	}
	return 0;
}

/**func "power_switch" and "power_init" just for the V_led(5V)**/
static int cm36781_power_switch(struct cm36781_chip *chip, bool on)
{
	int rc;

	if (on && chip->unpowered) {
		#if defined(CONFIG_HS_SNS_CM36781_USE_VDD_EN_GPIO)
			rc = gpio_direction_output(chip->vdd_3v_gpio, 1);
			if (rc) {
				dev_err(&chip->client->dev,"set_direction for vdd 3v gpio failed\n");
			}
			rc = gpio_direction_output(chip->vdd_5v_gpio, 1);
			if (rc) {
				dev_err(&chip->client->dev,"set_direction for vdd 5v gpio failed\n");
			}
		#else
			rc = regulator_enable(chip->vdd);
			if (rc) {
				dev_err(&chip->client->dev,
					"Regulator vdd enable failed rc=%d\n", rc);
				goto err_vdd_ena;
			}
			/*
			rc = regulator_enable(chip->vio);
			if (rc) {
				dev_err(&chip->client->dev,
					"Regulator vio enable failed rc=%d\n", rc);
				goto err_vio_ena;
			}
			*/
		#endif

		chip->unpowered = false;
		msleep(80);
		dev_err(&chip->client->dev,
			"%s(): regulator switch ON.\n", __func__);
	} else if (!on && !chip->unpowered) {
		#if defined(CONFIG_HS_SNS_CM36781_USE_VDD_EN_GPIO)
			rc = gpio_direction_output(chip->vdd_5v_gpio, 0);
			if (rc) {
				dev_err(&chip->client->dev,"set_direction for vdd 5v gpio failed\n");
			}
		#else
			rc = regulator_disable(chip->vdd);
			if (rc) {
				dev_err(&chip->client->dev,
					"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
			/*
			rc = regulator_disable(chip->vio);
			if (rc) {
				dev_err(&chip->client->dev,
					"Regulator vio disable failed rc=%d\n", rc);
				return rc;
			}
			*/
		#endif

		chip->unpowered = true;
		dev_err(&chip->client->dev,
			"%s(): regulator switch OFF.\n", __func__);
	}
	return 0;
#ifndef CONFIG_HS_SNS_CM36781_USE_VDD_EN_GPIO
/*
err_vio_ena:
	regulator_disable(chip->vdd);
*/
err_vdd_ena:
	return rc;
#endif
}

static int cm36781_power_init(struct cm36781_chip *chip, bool on)
{
	int rc;

	if (on) {
		#if defined(CONFIG_HS_SNS_CM36781_USE_VDD_EN_GPIO)
			if (gpio_is_valid(chip->vdd_3v_gpio)) {
				rc = gpio_request(chip->vdd_3v_gpio, "cm36781_vdd_3v_gpio");
				if (rc) {
					dev_err(&chip->client->dev,"vdd 3v gpio request failed");
				}
			}
			if (gpio_is_valid(chip->vdd_5v_gpio)) {
				rc = gpio_request(chip->vdd_5v_gpio, "cm36781_vdd_5v_gpio");
				if (rc) {
					dev_err(&chip->client->dev,"vdd 5v gpio request failed");
				}
			}
		#else
			chip->vdd = regulator_get(&chip->client->dev, "vdd");
			if (IS_ERR(chip->vdd)) {
				rc = PTR_ERR(chip->vdd);
				dev_err(&chip->client->dev,
					"Regulator get failed vdd rc=%d\n", rc);
				goto err_vdd_get;
			}
			if (regulator_count_voltages(chip->vdd) > 0) {
				rc = regulator_set_voltage(chip->vdd,
						CM36781_VDD_MIN_UV, CM36781_VDD_MAX_UV);
				if (rc) {
					dev_err(&chip->client->dev,
						"Regulator set failed vdd rc=%d\n", rc);
					goto err_vdd_set_vtg;
				}
			}
			/*
			chip->vio = regulator_get(&chip->client->dev, "vio");
			if (IS_ERR(chip->vio)) {
				rc = PTR_ERR(chip->vio);
				dev_err(&chip->client->dev,
					"Regulator get failed vio rc=%d\n", rc);
				goto err_vio_get;
			}
			if (regulator_count_voltages(chip->vio) > 0) {
				rc = regulator_set_voltage(chip->vio,
					cm36781_VI2C_MIN_UV, cm36781_VI2C_MAX_UV);
				if (rc) {
					dev_err(&chip->client->dev,
					"Regulator set failed vio rc=%d\n", rc);
					goto err_vio_set_vtg;
				}
			}
			*/
		#endif
	} else {
		#if defined(CONFIG_HS_SNS_CM36781_USE_VDD_EN_GPIO)
			// TBD
		#else
			if (regulator_count_voltages(chip->vdd) > 0)
				regulator_set_voltage(chip->vdd, 0, CM36781_VDD_MAX_UV);
			regulator_put(chip->vdd);
			/*
			if (regulator_count_voltages(chip->vio) > 0)
				regulator_set_voltage(chip->vio, 0, cm36781_VI2C_MAX_UV);
			regulator_put(chip->vio);
			*/
		#endif
	}
	return 0;

#ifndef CONFIG_HS_SNS_CM36781_USE_VDD_EN_GPIO
/*
err_vio_set_vtg:
	regulator_put(chip->vio);
err_vio_get:
	if (regulator_count_voltages(chip->vdd) > 0)
		regulator_set_voltage(chip->vdd, 0, CM36781_VDD_MAX_UV);
*/
err_vdd_set_vtg:
	regulator_put(chip->vdd);
err_vdd_get:
	return rc;
#endif
}

static int cm36781_parse_platform_dt(struct device *dev,
		struct cm36781_chip *chip)
{
	struct device_node *np = dev->of_node;
	u32 tmp;
	int rc = 0;

	rc = of_property_read_u32(np, "capella,ps_take_effect", &tmp);
	if (rc) {
		dev_info(dev, "Unable to read capella,ps_take_effect, set default 1\n");
		chip->ps_take_effect = 1;
	} else {
		chip->ps_take_effect = tmp;
	}

	rc = of_property_read_u32(np, "capella,als_take_effect", &tmp);
	if (rc) {
		dev_info(dev, "Unable to read capella,als_take_effect, set default 1\n");
		chip->als_take_effect = 0;
	} else {
		chip->als_take_effect = tmp;
	}

	chip->vdd_3v_gpio = of_get_named_gpio_flags(np, "capella,vdd3v-gpio",
				0, &chip->vdd_3v_gpio_flags);
	chip->vdd_5v_gpio = of_get_named_gpio_flags(np, "capella,vdd5v-gpio",
				0, &chip->vdd_5v_gpio_flags);

	return 0;
}

static int cm36781_probe(struct i2c_client *client,
		const struct i2c_device_id *idp)
{
	struct cm36781_chip *chip = NULL;
	int ret = 0;

	printk(KERN_ERR "\ncm36781: probe()\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
			"%s(): i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	bReboot = false;

	chip = kzalloc(sizeof(struct cm36781_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto init_failed;
	}
#if 0
	if (&client->dev.of_node) {
		ret = cm36781_parse_platform_dt(&client->dev, chip);
		if (ret) {
			dev_err(&client->dev, "cm36781_parse_platform_dt() err.\n");
			goto parse_dt_failed;
		}
	} else {
		goto init_failed;
	}
#endif
	ret = cm36781_parse_platform_dt(&client->dev, chip);
	if (ret) {
		dev_err(&client->dev, "cm36781_parse_platform_dt() err.\n");
		goto parse_dt_failed;
	}
	
	chip->client = client;
	i2c_set_clientdata(client, chip);

	/* power init of the V_led(5V), see details in the chip's Spec*/
	chip->unpowered = true;
	ret = cm36781_power_init(chip, true);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): cm36781 power on error!\n", __func__);
		goto power_on_failed;
	}
	/*check the power switch on and off func*/
	ret = cm36781_power_switch(chip, true);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): cm36781 power switch on error!\n", __func__);
		goto power_switch_failed;
	} else {
		msleep(30);
		ret = cm36781_power_switch(chip, false);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s(): cm36781 power switch off error!\n", __func__);
			goto power_switch_failed;
		}
	}

	/* initialize registers and variables */
	chip->ls_cmd = 0;
	chip->ps_conf1_2_val = 0;
	chip->ps_conf3_val = 0;
	msleep(30);
	ret = cm36781_check_chip_id(chip);
	if (ret < 0) {
		dev_err(&client->dev, "Not a valid chip ID.\n");
		goto verify_chip_failed;
	}

	mutex_init(&chip->mlock);

	chip->workqueue = create_workqueue("cm36781");
	if (NULL == chip->workqueue) {
		dev_err(&client->dev, "%s(): out of memory\n", __func__);
		goto wq_quit;
	}

	if (chip->als_take_effect > 0) {
		ret = cm36781_als_constructor(chip);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s(): Can not construct als function.\n", __func__);
			goto als_construct_quit;
		}
	}

	if (chip->ps_take_effect > 0) {
		ret = cm36781_ps_constructor(chip);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s(): Can not construct prox function.\n", __func__);
			goto ps_construct_quit;
		}
	}

	if ((1 == chip->ps_take_effect) || (2 == chip->als_take_effect)) {
		ret = cm36781_irq_init(&client->dev, chip);
		if (0 > ret) {
			dev_err(&client->dev, "%s(): Init irq failed.\n", __func__);
			goto irq_init_quit;
		}
	}

	productinfo_register(PRODUCTINFO_SENSOR_ALSPS_ID,
		"cm36781", "Capella");
	dev_info(&client->dev, "Probe ok.\n");
	G_cm36781_chip = chip;
	return 0;

als_construct_quit:
	if (chip->als_take_effect > 0) {
		cm36781_als_clean_up(chip);
	}

ps_construct_quit:
irq_init_quit:
	if (chip->ps_take_effect) {
		cm36781_ps_clean_up(chip);
	}
	destroy_workqueue(chip->workqueue);
wq_quit:
verify_chip_failed:
	cm36781_power_switch(chip, false);	
power_switch_failed:
	cm36781_power_init(chip, false);
power_on_failed:
parse_dt_failed:
	dev_err(&client->dev, "%s(): error exit! ret = %d\n", __func__, ret);
init_failed:
	kfree(chip);
	dev_err(&client->dev, "Probe failed.\n");
	return ret;
}

static int cm36781_suspend(struct device *dev)
{
	struct cm36781_chip *chip = dev_get_drvdata(dev);
	dev_err(&chip->client->dev, "%s(): enter !\n", __func__);
	//atomic_set(chip->in_suspend, 1);
	sensors_marker("cm36781_suspend()",
		"do", "suspend", "begin");
	if (chip->als_enabled) {
		cancel_delayed_work_sync(&chip->als_dwork);
		//cm36781_als_function_off(chip);
	}
	if (chip->ps_enabled) {
		cm36781_ps_function_off(chip);
	}
	return 0;
}

static int cm36781_resume(struct device *dev)
{
	struct cm36781_chip *chip = dev_get_drvdata(dev);
	//atomic_set(chip->in_suspend, 0);
	if (chip->als_enabled) {
		//cm36781_als_function_on(chip);
		queue_delayed_work(chip->workqueue, &chip->als_dwork,
			usecs_to_jiffies(chip->als_u_delay));
	}
	if (chip->ps_enabled) {
		cm36781_ps_function_on(chip);
	}
	sensors_marker("cm36781_resume()",
		"do", "resume", "done");
	return 0;
}

static const struct i2c_device_id cm36781_idtable[] = {
	{ "cm36781", 0 },
	{}
};

static struct of_device_id cm36781_match_table[] = {
	{.compatible = "capella,cm36781",},
	{ },
};

static UNIVERSAL_DEV_PM_OPS(cm36781_pm_ops, cm36781_suspend, cm36781_resume, NULL);

static struct i2c_driver cm36781_driver = {
	.id_table = cm36781_idtable,
	.probe = cm36781_probe,
	.driver = {
		.name = "cm36781",
		.owner = THIS_MODULE,
		.pm = &cm36781_pm_ops,
		.of_match_table = cm36781_match_table,
	},
};

static int cm36781_notify_sys(struct notifier_block *this,
					unsigned long code, void *unused)
{
	//if (code != SYS_DOWN || code == SYS_HALT)
	//	pnx833x_wdt_stop(); /* Turn the WDT off */
	pr_err("cm36781_notify_sys enter1\n");
	bReboot = true;
	if (NULL == G_cm36781_chip)
		return NOTIFY_DONE;

	pr_err("cm36781_notify_sys enter2\n");
	cm36781_als_function_off(G_cm36781_chip);
	cm36781_ps_function_off(G_cm36781_chip);

	return NOTIFY_DONE;
}

static struct notifier_block cm36781_notifier = {
	.notifier_call = cm36781_notify_sys,
};

static int __init cm36781_init(void)
{
	int ret;
	ret = register_reboot_notifier(&cm36781_notifier);
	if (ret) {
		pr_err("cm36781_init cannot register reboot notifier (err=%d)\n", ret);
		//return ret;
	}

	return i2c_add_driver(&cm36781_driver);
}

static void __exit cm36781_exit(void)
{
	i2c_del_driver(&cm36781_driver);
}

late_initcall(cm36781_init);
module_exit(cm36781_exit);

MODULE_AUTHOR("wangdong<wangdong24@hisense.com>");
MODULE_DESCRIPTION("Capella cm36781 ALS, Prox sensor driver");
MODULE_LICENSE("GPL");
