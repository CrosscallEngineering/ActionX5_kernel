/*
 *
 * FocalTech ft5x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 #define pr_fmt(fmt) "TP-ft5x06-ts: " fmt
#include"focaltech_ts.h"
#include "../ts_func_test.h"
#include <linux/productinfo.h>

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
#include "ft_gesture_lib.h"

#endif
#include "mcap_test_lib.h"

#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
u8 pb_file_ft8719[] = {
#include "FT8719_Pramboot.i"
};
u8 pb_file_ft8006p[] = {
#include "FT8006P_Pramboot.i"
};

#endif

#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)

#define NEAR_CODE	1
#define FAR_CODE	0

static struct sensors_classdev tp_proximity_cdev = {
	.name = "proximity",
	.vendor = "FocalTech",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static void psensor_report_dist(struct ft5x06_ts_data *data, int dist_code);
static int last_ps_val;
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X06_FH)
extern int usb_flag;
#endif

struct i2c_client *G_Client = NULL;
static int debug_mask;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR);
#undef dev_info
#define dev_info(dev, format, arg...) do { if (debug_mask) { dev_printk(KERN_INFO , dev , format , ## arg); } } while (0)

static int ft5x06_get_fw_upgrade_config(struct ft5x06_ts_data *data,
										struct upgrade_config *config);
static int ft5x06_enter_update(struct ft5x06_ts_data *ts_data);
static int ft5x46_enter_update(struct ft5x06_ts_data *ts_data);
static int ft8719_enter_update(struct ft5x06_ts_data *ts_data);
static int ft5x06_leave_update(struct ft5x06_ts_data *data);
static int ft5x46_leave_update(struct ft5x06_ts_data *data);
static void ft5x06_triger_update(struct device *dev, bool force);
static int ft5x06_ts_suspend(struct device *dev);
static int ft5x06_ts_resume(struct device *dev);
static int ft5x06_ts_resume_force(struct device *dev, bool force);
static int ft5x06_fetch_fw(struct ft5x06_ts_data *data, const struct firmware **ppfw);
static int ft5x06_fw_upgrade(struct device *dev, bool force);
static void ft5x06_ts_register_productinfo(struct ft5x06_ts_data *ts_data);
extern int qpnp_lbc_is_usb_chg_plugged(void);
static int hidi2c_to_stdi2c(struct i2c_client *client);
static int ft5x06_get_fw_filename(struct ft5x06_ts_data *data, u8 pannel_id, char *buf, int size);
#ifdef CONFIG_TOUCHSCREEN_FT5X06_GESTURE
static int ft5x06_init_gesture(struct ft5x06_ts_data *data);
static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on);
#endif
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
static u16 fts_pram_ecc_calc_host(u8 *pbuf, u16 length);
static int fts_pram_ecc_cal_algo(struct i2c_client *client,	u32 start_addr,	u32 ecc_length);
#endif
static int fts_pram_init(struct i2c_client *client);

static unsigned char FT_Reset_delay_5336[RESET_DELAY_ARRAY_LENGTH] = {21, 18, 15, 30, 33, 36, 39, 42, 45, 27, 24};

int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			pr_buf_err("%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_buf_err("%s:i2c read error.\n", __func__);
	}
	return ret;
}

int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		pr_buf_err("%s: i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};
	int i =0, rc=0;

	buf[0] = addr;
	buf[1] = val;

	for(i=0; i < 3; i++){
 		rc = ft5x06_i2c_write(client, buf, sizeof(buf));
		if (rc < 0) {
			pr_buf_err("%s: write reg=0x%02x error, retry= %d\n", __func__, buf[0], i);
			mdelay(5);
		} else {
			break;
		}
	}
	return rc;
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	int i =0, rc=0;

	for(i=0; i < 3; i++){
		rc = ft5x06_i2c_read(client, &addr, 1, val, 1);
		if (rc < 0) {
			pr_buf_err("%s: read reg=0x%02x error, retry= %d\n", __func__, addr, i);
			mdelay(5);
		} else {
			break;
		}
	}
	return rc;
}

int FTS_i2c_read(unsigned char *w_buf, int w_len, unsigned char *r_buf, int r_len)
{
	if (NULL == G_Client)
		return -ENODEV;

	return ft5x06_i2c_read(G_Client, w_buf, w_len, r_buf, r_len);
}

int FTS_i2c_write(unsigned char *w_buf, int w_len)
{
	if (NULL == G_Client)
		return -ENODEV;

	return ft5x06_i2c_write(G_Client, w_buf, w_len);
}

static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		pr_buf_err("fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		pr_buf_err("fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		pr_buf_err("fw sub minor version read failed");

	pr_buf_info("Firmware version = %d.%d.%d\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

static bool ft5x06_check_rawdata_need(struct ft5x06_ts_data *data)
{
	if (FT_FT5336_FAMILY_ID_0x11 == data->family_id
		|| FT_FT5336_FAMILY_ID_0x12 == data->family_id
		|| FT_FT5336_FAMILY_ID_0x13 == data->family_id
		|| FT_FT5336_FAMILY_ID_0x14 == data->family_id) {
		pr_buf_info(" %s return true\n",  __func__);
		return true;
	}
	pr_buf_info(" %s return flase\n",  __func__);
	return false;
}

static int  ft5x06_enter_factory(struct ft5x06_ts_data *data)
{
	int i = 0;
	u8 regval = 0xFF;

	pr_buf_info(" %s enter \n",  __func__);
	while (i++ < 3) {
		/* goto factory mode  */
		ft5x0x_write_reg(data->client, 0x00, 0x40);
		/* make sure already enter factory mode */
		msleep(200);
		if (ft5x0x_read_reg(data->client, 0x00, &regval) >= 0) {
			if ((regval & 0x70) == 0x40)
				break;
		}
	}
	pr_buf_info(" %s leave %d\n",  __func__, i < 3 ? 0 : -EIO);
	return i < 3 ? 0 : -EIO;
}

static int  ft5x06_leave_factory(struct ft5x06_ts_data *data)
{
	int i = 0;
	u8 regval = 0xFF;

	pr_buf_info(" %s enter \n",  __func__);
	while (i++ < 3) {
		/* leve factory mode  */
		ft5x0x_write_reg(data->client, 0x00, 0x00);
		/* make sure already enter factory mode */
		msleep(200);
		if (ft5x0x_read_reg(data->client, 0x00, &regval) >= 0) {
			if ((regval & 0x70) == 0x00)
				break;
		}
	}
	pr_buf_info(" %s leave %d\n",  __func__, i < 3 ? 0 : -EIO);
	return i < 3 ? 0 : -EIO;
}

static int ft5x06_get_rawdata_rowcol(struct ft5x06_ts_data *data, u8 *tx, u8 *rx)
{
	int retval  = 0;
	u8 tx_num = 0, rx_num = 0;

	pr_buf_info(" %s enter \n",  __func__);
	if (FT_FT5446_FAMILY_ID_0x54 == data->family_id) {
		retval = ft5x0x_read_reg(data->client, 0x02, &tx_num);
		if (retval < 0)
			return retval;

		retval = ft5x0x_read_reg(data->client, 0x03, &rx_num);
		if (retval < 0)
			return retval;
	} else {
		retval = ft5x0x_read_reg(data->client, 0x03, &tx_num);
		if (retval < 0)
			return retval;

		retval = ft5x0x_read_reg(data->client, 0x04, &rx_num);
		if (retval < 0)
			return retval;
	}

	rx_num > tx_num ? rx_num-- : tx_num--;

	if (rx_num < FT_RAWDATA_MAX_ROW && tx_num < FT_RAWDATA_MAX_COL) {
		*tx = tx_num;
		*rx =  rx_num;
		return 0;
	} else {
		pr_buf_err(" %s: error(num > 30) \n",  __func__);
		return -ERANGE;
	}
}

static int ft5x06_get_rawdata_info(struct ft5x06_ts_data *data,  char *buf)
{
	int ret = 0;
	u8 tx = 0, rx = 0;

	pr_buf_info(" %s enter \n",  __func__);
	ret = ft5x06_enter_factory(data);
	if (ret < 0)
		return 0;

	ret = ft5x06_get_rawdata_rowcol(data, &tx, &rx);
	if (ret == 0) {
		 ret = snprintf(buf, PAGE_SIZE, "RX:%u TX:%u HIGH:%u LOW:%u\n",
					   rx, tx, data->pdata->rawdata_range[0], data->pdata->rawdata_range[1]);
	}
	ft5x06_leave_factory(data);
	pr_buf_info(" %s leave \n",  __func__);
	return ret;
}

static int ft5x06_get_rawdata(struct ft5x06_ts_data *data,  char *buf)
{
	int ret = 0, size = 0;
	u8 rx_num = 0, tx_num = 0, devmode = 0, i, j;
	u8 *read_buf = NULL, read_size = 0;
	u8 write_buf[2];
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;

	pr_buf_info(" %s enter \n",  __func__);
	ret = ft5x06_enter_factory(data);
	if (ret < 0)
		return 0;
	ret = ft5x06_get_rawdata_rowcol(data, &tx_num, &rx_num);
	if (ret < 0)
		goto LEAVE_RETURN;

	ft5x0x_read_reg(client, 0x00, &devmode);
	devmode |= 0x80;
	ft5x0x_write_reg(client , 0x00, devmode);
	msleep(150);
	ft5x0x_read_reg(client , 0x00, &devmode);

	if ((devmode & 0x80) != 0) {
			pr_buf_err("%s: ERROR: could not scan", __func__);
			goto LEAVE_RETURN;
	}

	read_size = rx_num * 2;
	read_buf = devm_kzalloc(dev, read_size, GFP_KERNEL);
	if (read_buf  == NULL)
		goto LEAVE_RETURN;

	for (i = 0; i < tx_num; i++) {
		if (ft5x0x_write_reg(client, 0x01, i) < 0)
			goto LEAVE_RETURN;

		mdelay(1);
		write_buf[0] = 0x10;
		write_buf[1] = read_size;
		if (ft5x06_i2c_read(client, write_buf, 2, read_buf, read_size) < 0)
			goto LEAVE_RETURN;

		for (j = 0; j < rx_num; j++)
			size += snprintf(buf + size, PAGE_SIZE,
				j < rx_num - 1 ? "%u " : "%u\n",
				(read_buf[2 * j] << 8) + read_buf[2 * j + 1]);
	}

LEAVE_RETURN:
	if (read_buf != NULL)
		devm_kfree(dev, read_buf);
	ft5x06_leave_factory(data);
	pr_buf_info(" %s leave \n",  __func__);

	return size;
}

static int factory_check_fw_update_need(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	struct upgrade_config config = {
		.enter_upgrade_mode = false,
		.need_upgrade = false,
	};
	pr_buf_info(" %s enter \n",  __func__);
	ft5x06_get_fw_upgrade_config(data, &config);

	release_firmware(config.firmware);

	if (config.enter_upgrade_mode == true) {
		if (FT_FT5446_FAMILY_ID_0x54 == data->family_id)
			ft5x46_leave_update(data);
		else
			ft5x06_leave_update(data);
	}
	pr_buf_info("need_upgrade = %d\n", config.need_upgrade);
	return config.need_upgrade;
}

static	int factory_get_fw_update_progress(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return data->loading_fw ? FW_IS_UPDATETING : FW_UPGRADE_SUCCESS;
}

static	int factory_proc_fw_update(struct device *dev, bool force)
{
	ft5x06_triger_update(dev, force);

	return 0;
}

static int factory_get_rawdata(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (ft5x06_check_rawdata_need(data))
		return ft5x06_get_rawdata(data,  buf);
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "NOT SUPPORTED");
}

static int factory_get_rawdata_info(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	pr_buf_info(" %s enter \n",  __func__);
	if (ft5x06_check_rawdata_need(data))
		return ft5x06_get_rawdata_info(data,  buf);
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "NOT SUPPORTED");
}

/************************************************************************
* Name: StartScan(Same function name as FT_MultipleTest)
* Brief:  Scan TP, do it before read Raw Data
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int StartScan(struct i2c_client *client)
{
    u8 RegVal = 0x00;
    u8 times = 0;
    const u8 MaxTimes = 20;  //The longest wait 160ms
    int ReCode = -1;

    ReCode = ft5x0x_write_reg(client, 0x00, 0xC0);
    if (ReCode >= 0)
    {
        while (times++ < MaxTimes)      //Wait for the scan to complete
        {
            msleep(8);      //8ms
            ReCode = ft5x0x_read_reg(client, 0x00, &RegVal);
            if (RegVal == 0x40)
                break;
        }

        if (times > MaxTimes)
            return -1;
    }

    return ReCode;
}

int read_diff(struct i2c_client *client, s16 *data, int len)
{
    u8 reg = 0x6A;
    u8 regdata[1280] = { 0 };
    int remain_bytes;
    int pos = 0;
    int i = 0;

    pr_buf_info("len=%d, is_diff=%d", len, 1);
    ft5x0x_write_reg(client, 0x06, 1);

    if (StartScan(client) < 0)
        return -1;

    ft5x0x_write_reg(client, 0x01, 0xAD);

    if (len <= 256)
        ft5x06_i2c_read(client, &reg, 1, regdata, len);
    else
    {
        ft5x06_i2c_read(client, &reg, 1, regdata, 256);
        remain_bytes = len - 256;
        for (i = 1; remain_bytes > 0; i++)
        {
            if (remain_bytes > 256)
              ft5x06_i2c_read(client, &reg, 0, regdata + i * 256, 256);
            else
                ft5x06_i2c_read(client, &reg, 0, regdata + i * 256, remain_bytes);
            remain_bytes -= 256;
        }
    }

    for (i = 0; i < len;)
    {
        data[pos++] = ((s16)(regdata[i]) << 8) + regdata[i+1];
        i += 2;
    }

    /*
    {
        char debug[4096];
        int i, count=0;
        for (i=0;i<len;i++)
        {
            count += snprintf(debug + count, 4096, "%x ", regdata[i]);
        }
        debug[count+1]='\0';
        FTS_DEBUG("%s", debug);
    }
    */
    return 0;
}

#ifndef CONFIG_TOUCHSCREEN_FT5X06_GFF_PANNEL
int get_diff(struct i2c_client *client, s16 *data, u8 *txlen, u8 *rxlen)
{
    u8 val;
    int i = 0;

    /* 0xEE = 1, not clb */
    ft5x0x_write_reg(client, 0xEE, 1);

    /* Enter Factory Mode */
    ft5x0x_write_reg(client, 0x00, 0x40);
    do {
        ft5x0x_read_reg(client, 0x00, &val);
        if(val == 0x40)
            break;
        msleep(1);
		i++;
    } while(i < 10);

    /* Get Tx/Rx Num */
    ft5x0x_read_reg(client, 0x02, txlen);
    ft5x0x_read_reg(client, 0x03, rxlen);

    /* read_diff */
    read_diff(client, data, (*txlen) * (*rxlen) * 2);

    /* Enter in work mode */
    ft5x0x_write_reg(client, 0x00, 0x00);
	i = 0;
    do {
        ft5x0x_read_reg(client, 0x00, &val);
        if(val == 0x00)
            break;
        msleep(1);
		i++;
    } while(i < 10);

    return 0;
}

/************************************************************************
* Name: fts_rawdata_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static int factory_get_diff(struct device *dev, char *buf)
{
    int count = 0;
    int i = 0, j = 0;
    u8 val;
    s16 data[600] = { 0 };
    u8 txlen = 0;
    u8 rxlen = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    mutex_lock(&dev->mutex);

    get_diff(client, data, &txlen, &rxlen);

    ft5x0x_read_reg(client, 0xEE, &val);
    //count = snprintf(buf, PAGE_SIZE, "0xEE = %d\n", val);
    //count += snprintf(buf + count, PAGE_SIZE, "%s :\n", "DIFF DATA");
    pr_buf_info("0xEE = %d\n", val);
    pr_buf_info("%s :\n", "DIFF DATA");
    for (i = 0; i < txlen; i++)
    {
        for (j= 0; j < rxlen; j++)
        {
            count += snprintf(buf + count, PAGE_SIZE, "%5d ", data[i*rxlen + j]);
        }
        count += snprintf(buf + count, PAGE_SIZE, "\n");
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n\n");

    mutex_unlock(&dev->mutex);

    return count;
}
#endif

static int factory_proc_hibernate_test(struct device *dev)
{
	int err = 0;
	u8 reg_value = 0;
	u8 reg_addr = FT_REG_ID;
	int i;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	pr_buf_info(" %s enter \n",  __func__);
	gpio_direction_output(data->pdata->reset_gpio, 1);
	mdelay(1);
	gpio_direction_output(data->pdata->reset_gpio, 0);
	mdelay(5);
	pr_buf_info("%s: reset is low\n", __func__);
	err = ft5x06_i2c_read(data->client, &reg_addr, 1, &reg_value, 1);
	if (err >= 0) {
		pr_buf_err("%s: read i2c ok when reset is low,with result=%d\n", __func__, err);
		goto out;
	}

	gpio_direction_output(data->pdata->reset_gpio, 1);
	msleep(200);

	err = ft5x06_i2c_read(data->client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		pr_buf_err("%s:read i2c ok when reset is high, reset test Fail!\n", __func__);
		goto out;
	}
		pr_buf_info("%s:read i2c ok when reset is high, reset test success!\n", __func__);

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 1;
out:
	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 0;
}

static int factory_get_ic_fw_version(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", data->fw_ver[0]);
}

static int factory_get_fs_fw_version(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc = 0;
	u8 fs_fw_version = 0;
	
	pr_buf_info(" %s enter \n",  __func__);

	rc = ft5x06_fetch_fw(data, &fw);
	if (rc == 0 && fw != NULL) {
		if (FT_FT6436_FAMILY_ID_0x36 == data->family_id)
			fs_fw_version = FT_6436FW_FILE_MAJ_VER(fw);
        else if (FT_FT5446_FAMILY_ID_0x54 == data->family_id){
			if(0x54 == data->pannel_id || 0x53 == data->pannel_id)
				fs_fw_version = FT_5452FW_FILE_MAJ_VER(fw);
			else if (0x5C == data->pannel_id || 0x5E == data->pannel_id)
				fs_fw_version = FT_5422uFW_FILE_MAJ_VER(fw);
			else
				fs_fw_version = FT_FW_FILE_MAJ_VER(fw);
        }
		else if (FT_FT8006_FAMILY_ID_0x80 == data->family_id)
			fs_fw_version = FT_8006FW_FILE_MAJ_VER(fw);
		else if (FT_FT8006P_FAMILY_ID_0x86 == data->family_id)
			fs_fw_version = FT_8006PFW_FILE_MAJ_VER(fw);
		else if (FT_FT8719_FAMILY_ID_0x87 == data->family_id)
			fs_fw_version = FT_8719FW_FILE_MAJ_VER(fw);
		else
			fs_fw_version = FT_FW_FILE_MAJ_VER(fw);
		release_firmware(fw);
	}
	pr_buf_info(" %s 0x%02X \n",  __func__, fs_fw_version);
	return snprintf(buf, FT_VERSION_SIZE, "0x%02X\n", fs_fw_version);
}

static int factory_get_module_id(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
#if 0
	u8 reg_value, reg_addr = FT_RGE_PANNEL_ID;
	int err = 0;
	err = ft5x06_i2c_read(data->client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		pr_buf_err("Pannel Id version read failed");
	dev_info(dev, "Pannel Id version = 0x%x\n", reg_value);
	data->pannel_id =  reg_value;
#endif
	return snprintf(buf, PAGE_SIZE, "0x%02X\n", data->pannel_id);
}

static int factory_get_calibration_ret(struct device *dev)
{
	return 1;
}

static int factory_set_fw_path(struct device *dev, const char *buf)
{
	size_t len;
	char *pfile;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	strlcpy(data->fw_path, buf, sizeof(data->fw_path));
	len = strlen(data->fw_path);
	if (len > 0) {
		if (data->fw_path[len-1] == '\n')
			data->fw_path[len-1] = '\0';

		pfile = strrchr(data->fw_path, '/');
		if (pfile) {
			strlcpy(data->fw_name, pfile+1, sizeof(data->fw_name));
			pfile[1] = '\0';
		}
	}
	pr_buf_info(" %s leave\n",  __func__);
	return 1;
}

static int factory_get_fw_path(struct device *dev, char *buf, size_t buf_size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	
	pr_buf_info(" %s enter\n",  __func__);
	strlcpy(buf, data->fw_path, buf_size);
	strlcat(buf, data->fw_name, buf_size);

	return 1;
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
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

static int set_gesture_switch(struct device *dev, const char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned char gesture[10], len;
	
	if(0 == data->tp_enable){
	   pr_buf_info("%s: TP gesture can not be set when tp was disabled.\n", __func__);
	   return 0;
	}

	strlcpy(gesture, buf, sizeof(gesture));
	len = strlen(gesture);
	if (len > 0)
		if ((gesture[len-1] == '\n') || (gesture[len-1] == '\0'))
			len--;

	dev_info(&data->client->dev, "%s len: %d gtp_state: %d,%d,%d.\n",
					__func__, len, gesture[0], gesture[1], gesture[2]);
	if (len == 1) {
		if (gesture[0] == '1')
			data->gesture_state = 0xffff;
		else if (gesture[0] == '0')
			data->gesture_state = 0x0;
	} else if (len == 4) {
		data->gesture_state = asic_to_hex(gesture[0])*0x1000
						+ asic_to_hex(gesture[1]) * 0x100
						+ asic_to_hex(gesture[2]) * 0x10
						+ asic_to_hex(gesture[3]);
	} else {
		dev_info(&data->client->dev, "[set_gesture_switch]write wrong cmd.");
		return 0;
	}
	if (!data->gesture_state)
		data->gesture_en = false;
	else
		data->gesture_en = true;
	pr_buf_info("%s is %x.\n", __func__, data->gesture_state);

#ifdef CONFIG_TOUCHSCREEN_FT5X06_SUB
	if (data->suspended) {
		ft5x06_ts_resume(dev);
		ft5x06_ts_suspend(dev);
	}

#endif/*CONFIG_TOUCHSCREEN_FT5X06_SUB*/
	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_SUB) && defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
int ft5x06_ts_get_gesture_fingers_status(struct device *dev,  char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	u8 gesture_finger_num;
	u8 gesture_area_flag;

	if ((data->suspended) && (data->gesture_en)) {
		ft5x0x_read_reg(data->client, 0xb5, &gesture_finger_num);
		ft5x0x_read_reg(data->client, 0xb6, &gesture_area_flag);
		pr_buf_info("gesture_finger_num = %d, gesture_area_flag = %d\n",
				gesture_finger_num, gesture_area_flag);
		return gesture_area_flag || gesture_finger_num;
	}
	return -EINVAL;
}
#endif

static bool get_gesture_switch(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return data->gesture_en;
}

static int get_gesture_pos(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char temp[20] = {0};
	char gesture_pos[512] = {0};
	int i = 0;

	for (i = 0; i < data->gesture_track_pointnum; i++) {
		snprintf(temp, ARRAY_SIZE(temp), "%u,%u;", (unsigned int)data->gesture_track_x[i], (unsigned int)data->gesture_track_y[i]);
		strlcat(gesture_pos, temp, 512);
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", gesture_pos);
}

#endif

static bool get_tp_enable_switch(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return !data->suspended;
}

static int set_tp_enable_switch(struct device *dev, bool enable)
{
#ifdef CONFIG_TOUCHSCREEN_FT5X06_SUB
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	pr_buf_info("%s: %d.\n", __func__, enable);
	if (enable) {
		return ft5x06_ts_resume(dev);
	} else {
		if(data->irq_can_wake == false)
			return ft5x06_ts_suspend(dev);
		else
			return 1;
	}
#else
	static bool is_the_first_set = 1;
	#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	static bool gesture_switch;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	#endif

	if (is_the_first_set) {
	#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
		gesture_switch = data->gesture_en;
	#endif
		is_the_first_set = 0;
	}
	pr_buf_info("%s: %d.\n", __func__, enable);
	data->tp_enable = enable;
	if (enable) {
	#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
		data->gesture_en = gesture_switch;
	#endif
		return ft5x06_ts_resume(dev);
	} else {
	#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
		gesture_switch = data->gesture_en;
		//data->gesture_en = 0;
	#endif
		return ft5x06_ts_suspend(dev);
	}
#endif
}

#ifdef CONFIG_TOUCHSCREEN_FT5X06_GLOVE
static bool get_glove_switch(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	
	pr_buf_info("%s: glove_enable = %d\n", __func__, data->glove_enable);
	return data->glove_enable;
}

static int set_glove_switch(struct device *dev, bool enable)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	u8 reg_val = enable;
	u8 reg_addr = 0xC0;
	int err = 0;

	data->glove_enable = enable;
	pr_buf_info("%s: glove_enable = %d\n", __func__, data->glove_enable);
	err = ft5x0x_write_reg(data->client, reg_addr, reg_val);
	if (err < 0) {
		pr_buf_err("%s: write glove reg failed,with result=%d\n", __func__, err);
		return -EFAULT;
	}

	return 0;
}
#endif
#ifdef CONFIG_TOUCHSCREEN_FT5X06_HALL
static bool get_hall_switch(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	
	pr_buf_info("%s: hall_enable = %d\n", __func__, data->hall_enable);
	return data->hall_enable;
}

static int set_hall_switch(struct device *dev, const char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	u8 reg_addr = 0xC1;
	int err = 0;
	unsigned char hall[5];
	strlcpy(hall, buf, sizeof(hall));
	if (hall[0] == '0')
		data->hall_enable = false;
	else
		data->hall_enable = true;

	err = ft5x0x_write_reg(data->client, reg_addr, data->hall_enable);
	if (err < 0) {
		pr_buf_err("%s: write glove reg failed,with result=%d\n", __func__, err);
		return -EFAULT;
	}
	pr_buf_info("%s: hall_enable = %d\n", __func__, data->hall_enable);
	return 0;
}
#endif

#ifndef CONFIG_TOUCHSCREEN_FT5X06_APK_TEST
static int ft5x06_get_ini_size(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128] = {0};

	pr_buf_info("%s: enter \n", __func__);
	snprintf(filepath, ARRAY_SIZE(filepath), "%s", config_name);
	pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_buf_err("Error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	pr_buf_info("%s: leave \n", __func__);

	return fsize;
}

static int ft5x06_read_ini_data(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128] = {0};
	loff_t pos;
	mm_segment_t old_fs;
	
	pr_buf_info("%s: enter \n", __func__);
	snprintf(filepath, ARRAY_SIZE(filepath), "%s", config_name);
	pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_buf_err("Error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	pr_buf_info("%s: leave \n", __func__);

	return 0;
}
static int ft5x06_get_testparam_from_ini(char *config_name)
{
	char *filedata = NULL;
	int ini_size = ft5x06_get_ini_size(config_name);

	pr_buf_info("%s Ini_size = %d\n", __func__, ini_size);
	if (ini_size <= 0) {
		pr_buf_err("Get firmware size failed.\n");
		return -EIO;
	}
	filedata = vmalloc(ini_size + 1);
	if (filedata == NULL)
		return -ENOMEM;
	if (ft5x06_read_ini_data(config_name, filedata)) {
		pr_buf_err("Request ini file failed.");
		kfree(filedata);
		return -EIO;
	}
	SetParamData(filedata);
	pr_buf_info("%s: leave \n", __func__);

	return 0;
}
static int factory_get_short_test(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int cap_test_flag;

	pr_buf_info("%s: enter \n", __func__);
	mutex_lock(&dev->mutex);
	Init_I2C_Read_Func(FTS_i2c_read);
	Init_I2C_Write_Func(FTS_i2c_write);

	if (ft5x06_get_testparam_from_ini(data->test_config_path) < 0) {
		pr_buf_err("Get testparam from ini failture.\n");
		mutex_unlock(&dev->mutex);
		return 0;
	} else {
		if (true == StartTestTP()) {
			pr_buf_info("Cap test pass.\n");
			cap_test_flag = 1;
		} else {
			pr_buf_err("Cap test failed.\n");
			cap_test_flag = 0;
		}
		/* FreeTestParamData(); */
		mutex_unlock(&dev->mutex);
		return cap_test_flag;
	}
}

static bool factory_get_test_config_need(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	if (FT_FT5446_FAMILY_ID_0x54 == data->family_id) {
		pr_buf_info("%s: leave, return true \n", __func__);
		return true;
	}
	pr_buf_info("%s: leave, return flase \n", __func__);
	return false;
}

static int factory_set_test_config_path(struct device *dev, const char *buf)
{
	size_t len;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	
	pr_buf_info("%s: enter\n", __func__);
	strlcpy(data->test_config_path, buf, sizeof(data->test_config_path));
	len = strlen(data->test_config_path);
	if (len > 0) {
		if (data->test_config_path[len-1] == '\n')
			data->test_config_path[len-1] = '\0';
	}
	pr_buf_info("%s: leave\n", __func__);
	return len;
}
#endif

static int factory_get_chip_type(struct device *dev, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "focaltech\n");
}

static int factory_get_chip_settings(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	struct ft5x06_ts_platform_data *pdata = data->pdata;
	char temp[400]= {0};
	char temp1[FT_VERSION_SIZE] = {0};
	char fw_file_name[FT_FW_NAME_MAX_LEN] = {0};
	#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	int index;
	#endif

	pr_buf_info("%s: enter\n", __func__);
	strlcpy(temp, "\n### focaltech_ts ###\n", sizeof(temp));
	ft5x06_get_fw_filename(data, data->pannel_id, fw_file_name, sizeof(fw_file_name));
	strlcat(temp, fw_file_name, sizeof(temp));
	strlcat(temp, "\n", sizeof(temp));
	snprintf(temp1, ARRAY_SIZE(temp1), "IC_version=0x%02x\nFW_version=", data->fw_ver[0]);
	strlcat(temp, temp1, sizeof(temp));
	factory_get_fs_fw_version(dev,temp1);
	strlcat(temp, temp1, sizeof(temp));
	snprintf(temp1, ARRAY_SIZE(temp1), 
		"family_id=0x%02x\n"\
		"reset_gpio=%d, irq_gpio=%d, irq_flag=0x%08x\n"\
		"x=%d, y=%d\nnum_max_touches=%d\n"\
		"hard_rst_dly=%d, soft_rst_dly=%d\n"\
		"delay_aa=%d, delay_55 =%d\n"\
		"upgrade_id_1=0x%02x, upgrade_id_2=0x%02x\n"\
		"delay_readid=%d, delay_erase_flash=%d\n", 
		data->family_id, 
		pdata->reset_gpio, pdata->irq_gpio, pdata->irq_gpio_flags,
		pdata->x_max, pdata->y_max, pdata->num_max_touches,
		pdata->hard_rst_dly,pdata->soft_rst_dly,
		pdata->info.delay_aa, pdata->info.delay_55,
		pdata->info.upgrade_id_1, pdata->info.upgrade_id_2,
		pdata->info.delay_readid, pdata->info.delay_erase_flash
	);
	strlcat(temp, temp1, sizeof(temp));
	#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	snprintf(temp1, ARRAY_SIZE(temp1), "gesture support:");
	strlcat(temp, temp1, sizeof(temp));
	for (index = 0; index < data->pdata->gesture_num; index++) {
		snprintf(temp1, ARRAY_SIZE(temp1), " %d", data->pdata->gesture_func_map[index]);
		strlcat(temp, temp1, sizeof(temp));
	}
	#endif
	strcpy(buf,temp);
	pr_buf_info("%s: leave\n", __func__);
	return snprintf(buf, PAGE_SIZE, "%s\n\n", buf);
}

static int factory_get_factory_info(struct device *dev, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	pr_buf_info("%s: %s\n", __func__, data->pdata->factory_info);
	return snprintf(buf, PAGE_SIZE, "%s\n", data->pdata->factory_info);
}

#define BYTE_OFF_0(x)           (u8)((x) & 0xFF)
#define BYTE_OFF_8(x)           (u8)((x >> 8) & 0xFF)
#define BYTE_OFF_16(x)          (u8)((x >> 16) & 0xFF)
#define BYTE_OFF_24(x)          (u8)((x >> 24) & 0xFF)

static int ft5x06_set_erase_flash_test(struct device *dev, bool enable)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	struct fw_upgrade_info info = data->pdata->info;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 is_5336_new_bootloader = false;
	int i, ret;
	u8 packet_buf[FT_FW_PKT_LEN + 6];
	u8 auc_i2c_write_buf[10];
	u32 packet_number = 0;
	u32 temp, lenght;
	int pram_ecc = 0;
	u32 remainder = 0;
	u32 fw_packet_length = 0;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
	int j = 0;
#endif
	struct upgrade_config config = {
		.enter_upgrade_mode = false,
		.need_upgrade = false,
		.firmware = NULL,
	};
	u32 ft8006p_erase_lenth = 0;

	printk("%s enter...\n", __func__);
	if (enable) {
		/*Enter update mode*/
		if (FT_FT5446_FAMILY_ID_0x54 == data->family_id)
			ret = ft5x46_enter_update(data); 
		else if (FT_FT8719_FAMILY_ID_0x87 == data->family_id)
			ret = ft8719_enter_update(data);
		else if (FT_FT8006P_FAMILY_ID_0x86 == data->family_id) {
			ret = ft5x06_get_fw_upgrade_config(data, &config);
			if (ret < 0)
			{
				pr_buf_err("FT8006P need request firmware to get erase length, but request firmware fail!\n");
				release_firmware(config.firmware);
				return -EIO;
			}
			ft8006p_erase_lenth = config.firmware->size;
			release_firmware(config.firmware);
			ret = ft8719_enter_update(data);
		}
		else 
			ret = ft5x06_enter_update(data);
		if (ret !=0) { 
			pr_buf_info("enter update mode failed and Reset the CTP.(%s,%d)\n", __func__, __LINE__);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
			mdelay(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
			mdelay(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
			mdelay(data->pdata->soft_rst_dly);
			return -EIO;
		}

		if ( (FT_FT8719_FAMILY_ID_0x87 == data->family_id) ||
			 (FT_FT8006P_FAMILY_ID_0x86 == data->family_id) ) {
			/* write pramboot to pram */
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
			if (FT_FT8719_FAMILY_ID_0x87 == data->family_id) {
				data->pramboot = pb_file_ft8719;
				data->pb_length = sizeof(pb_file_ft8719);
				fw_packet_length = FT_FW_PKT_LEN;
			} else if (FT_FT8006P_FAMILY_ID_0x86 == data->family_id) {
				data->pramboot = pb_file_ft8006p;
				data->pb_length = sizeof(pb_file_ft8006p);
				fw_packet_length = FTS_FLASH_PACKET_LENGTH;
			}
			packet_number = data->pb_length / fw_packet_length;
			remainder = data->pb_length % fw_packet_length;
#endif
		
			if (remainder > 0)
				packet_number++;
			lenght = fw_packet_length;
		
			packet_buf[0] = 0xAE;
			for (i = 0; i < packet_number; i++) {
				temp = i * fw_packet_length;
				packet_buf[1] = (temp >> 16) & 0xFF;
				packet_buf[2] = (temp >> 8) & 0xFF;
				packet_buf[3] = temp & 0xFF;
		
				/* last packet */
				if ((i == (packet_number - 1)) && remainder)
				lenght = remainder;
		
				packet_buf[4] = (lenght >> 8) & 0xFF;
				packet_buf[5] = lenght & 0xFF;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
				for (j = 0; j < lenght; j++) {
					packet_buf[6 + j] = data->pramboot[temp + j];
				}
#endif
				ret = ft5x06_i2c_write(data->client, packet_buf, lenght + 6);
				if (ret < 0) {
					pr_buf_err("pramboot write data(%d) fail!\n", i);
					return ret;
				}
			}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
			pram_ecc = (int)fts_pram_ecc_calc_host(data->pramboot,
				data->pb_length);
#endif
			if (pram_ecc < 0) {
				pr_buf_err( "write pramboot fail!\n");
			}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
			ret = fts_pram_ecc_cal_algo(data->client, 0, data->pb_length);
#endif
			if (ret < 0) {
				pr_buf_err( "read pramboot ecc fail!\n");
			}
			pr_buf_info("pram ecc in tp:%x, host:%x\n", ret, pram_ecc);
			/*	pramboot checksum != fw checksum, upgrade fail */
			if (ret != pram_ecc) {
				pr_buf_err("pramboot ecc check fail!\n");
				//return -EIO;
			}
		
			pr_buf_info("remap to start pramboot!\n");
			auc_i2c_write_buf[0] = 0x08;
			ret = ft5x06_i2c_write(data->client, auc_i2c_write_buf, 1);
			if (ret < 0) {
				pr_buf_err("write start pram cmd fail!\n");
				return ret;
			}
			mdelay(10);
		

			if (FT_FT8719_FAMILY_ID_0x87 == data->family_id) {
				data->pdata->info.upgrade_id_1 = 0x87;
				data->pdata->info.upgrade_id_2 = 0xA9;
			} else if (FT_FT8006P_FAMILY_ID_0x86 == data->family_id) {
				data->pdata->info.upgrade_id_1 = 0x86;
				data->pdata->info.upgrade_id_2 = 0xA2;
			}

			if (ft8719_enter_update(data) != 0) {
				pr_buf_err("enter pramboot mode failed and Reset the CTP.(%s,%d)\n",
					__func__, __LINE__);
				return -EIO;
			}

			if (FT_FT8719_FAMILY_ID_0x87 == data->family_id) {
				data->pdata->info.upgrade_id_1 = 0x87;
				data->pdata->info.upgrade_id_2 = 0x19;
			} else if (FT_FT8006P_FAMILY_ID_0x86 == data->family_id) {
				data->pdata->info.upgrade_id_1 = 0x86;
				data->pdata->info.upgrade_id_2 = 0x22;
			}

			ret = fts_pram_init(data->client);
			if (ret < 0) {
				pr_buf_err("pramboot init fail!\n");
				return ret;
			}

			/*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
			auc_i2c_write_buf[0] = 0x09;
			auc_i2c_write_buf[1] = 0x0B;
			ft5x06_i2c_write(data->client, auc_i2c_write_buf, 2);

			if (FT_FT8719_FAMILY_ID_0x87 == data->family_id) {
				mdelay(800);
			} else if (FT_FT8006P_FAMILY_ID_0x86 == data->family_id) {
				auc_i2c_write_buf[0] = 0x7A;
				auc_i2c_write_buf[1] = BYTE_OFF_16(ft8006p_erase_lenth);
				auc_i2c_write_buf[2] = BYTE_OFF_8(ft8006p_erase_lenth);
				auc_i2c_write_buf[3] = BYTE_OFF_0(ft8006p_erase_lenth);
				ret = ft5x06_i2c_write(data->client, auc_i2c_write_buf, 4);
				if (ret < 0)
					pr_buf_err("data len cmd write fail");
			}
		}

		/*erase app and panel paramenter area*/
		pr_buf_info("[UPGRADE]: erase app and panel paramenter area!!\n");
		w_buf[0] = FT_ERASE_APP_REG;
		ft5x06_i2c_write(data->client, w_buf, 1);
		msleep(info.delay_erase_flash);
		for (i = 0; i < 200; i++) {
			w_buf[0] = 0x6a;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			r_buf[0] = 0x00;
			r_buf[1] = 0x00;
			ft5x06_i2c_read(data->client, w_buf, 4, r_buf, 2);
		
			if ((FT_FT5446_FAMILY_ID_0x54 == data->family_id) ||
				(FT_FT8006_FAMILY_ID_0x80 == data->family_id) ||
				(FT_FT8006P_FAMILY_ID_0x86 == data->family_id) ||
				(FT_FT8719_FAMILY_ID_0x87 == data->family_id) ) {
				if (0xF0 == r_buf[0] && 0xAA == r_buf[1]) {
					pr_buf_info("erase app finished.");
					break;
				}
			} else if (FT_FT6436_FAMILY_ID_0x36 == data->family_id) {
				if (0x0B == r_buf[0] && 0x02 == r_buf[1]) {
					pr_buf_info("erase app finished.");
					break;
				}	
			} else {
				w_buf[0] = 0xcd;
				ft5x06_i2c_read(data->client, w_buf, 1, r_buf, 1);

				if (r_buf[0] <= 4)
					is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
				else if (r_buf[0] == 7)
					is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
				else if (r_buf[0] >= 0x0f &&
					((data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
					(data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
					(data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
					(data->family_id == FT_FT5336_FAMILY_ID_0x14)))
					is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
				else
					is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

				/* erase app and panel paramenter area */
				w_buf[0] = FT_ERASE_APP_REG;
				ft5x06_i2c_write(data->client, w_buf, 1);
				msleep(info.delay_erase_flash);
				pr_buf_info("erase app finished.");
				break;
			}
		} 
	}
	return 0;
}
static int ft5x06_set_tp_irq_awake_switch(struct device *dev, bool enable)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	if(enable == 1)
	{
		//disable_irq(data->client->irq);
		/* make tp can wake the system */
		//enable_irq_wake(data->client->irq);
		data->irq_can_wake = true;
	}
	else
		data->irq_can_wake = false;

	return 0;
}
static bool ft5x06_get_tp_irq_awake_switch(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return data->irq_can_wake;
}

#ifdef CONFIG_TOUCHSCREEN_WORK_MODE
static int get_tp_work_mode(struct device *dev, char *buf)
{
	u8 reg_val;
	u8 reg_addr = 0xA5;
	int err = 0;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if(data->suspended)
	{
		return snprintf(buf, PAGE_SIZE, "%s\n", "IDLE");
	} else {
		err = ft5x0x_read_reg(data->client, reg_addr, &reg_val);
		if (err < 0)
			return snprintf(buf, PAGE_SIZE, "%s\n", "READ MODE ERROR");

		pr_buf_info("%s: read 0xA5 reg value = %#x)\n", __func__, reg_val);

		if (reg_val == 0x00)
			return snprintf(buf, PAGE_SIZE, "%s\n", "OPERATING");
		else if (reg_val == 0x01)
			return snprintf(buf, PAGE_SIZE, "%s\n", "MONITOR");
		else
			return snprintf(buf, PAGE_SIZE, "%s\n", "UNKNOWN ERROR");
	}
}

static int set_tp_work_mode(struct device *dev, const char *mode)
{
	u8 reg_addr = 0xA5;
	u8 reg_val;

	u8 reg_val_readback;
	int err = 0;
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (strncmp(mode, "IDLE", 4)==0) {
		if(!data->suspended)
		set_tp_enable_switch(dev, false);
	} else {
		if (strncmp(mode, "OPERATING", 9)==0)
			reg_val = 0x00;
		else if (strncmp(mode, "MONITOR", 7)==0)
			reg_val = 0x01;
		else
			return -EFAULT;

		if(data->suspended)
			set_tp_enable_switch(dev, true);

		err = ft5x0x_write_reg(data->client, reg_addr, reg_val);
		if (err < 0) {
			pr_buf_err("%s: write reg failed, [err]=%d\n", __func__, err);
			return -EFAULT;
		}

		err = ft5x0x_read_reg(data->client, reg_addr, &reg_val_readback);
		if (err < 0 || reg_val_readback!=reg_val) {
			pr_buf_err("%s: set operating mode failed, write val==%d, read back val==%d, [read err]=%d, []=\n", 
				__func__, reg_val, reg_val_readback, err);
			return -EFAULT;
		}
	}
	pr_buf_info("%s: tp set to %s work mode\n", __func__, mode);

	return 0;
}
#endif/*CONFIG_TOUCHSCREEN_WORK_MODE*/

#ifdef CONFIG_FT_INCELL_CHIP
static int ft8xxx_ts_provide_reset_control(void)
{
	struct ft5x06_ts_data *data;

	if (G_Client == NULL)
		return -ENODEV;

	data = dev_get_drvdata(&G_Client->dev);	
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	return 0;
}

static int ft8xxx_suspend_need_lcd_reset_high(void)
{
	struct ft5x06_ts_data *data;

	if (G_Client == NULL)
		return -ENODEV;

	data = dev_get_drvdata(&G_Client->dev);	
	return data->pdata->keep_lcd_suspend_reset_high;
}

#ifdef CONFIG_TOUCHSCREEN_FT5X06_GESTURE
int ft8xxx_need_lcd_power_reset_keep_flag_get(void)
{
	struct ft5x06_ts_data *data;

	if (G_Client == NULL)
		return -ENODEV;

	data = dev_get_drvdata(&G_Client->dev);	
	return data->gesture_en;
}
#endif
#ifdef CONFIG_PM
static int ft8xxx_ts_suspend_for_lcd_async_use(void)
{
	struct ft5x06_ts_data *data;
#ifdef CONFIG_TOUCHSCREEN_FT5X06_GESTURE
	u8 i = 0;
	u8 gesture_switch;
#endif

	if (G_Client == NULL)
		return -ENODEV;

	data = dev_get_drvdata(&G_Client->dev);	

	if (data->resume_is_running) {
		printk("%s: TP is in work mode, no need do TP async syspend\n", __func__);
		return 0;
	}

#ifdef CONFIG_TOUCHSCREEN_FACE_DETECTION
	if (data->ps_en)
		return 0;
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06_GESTURE
	data->gesture_suspend_en = data->gesture_en;
	if (data->gesture_suspend_en) {
		for (i = 0; i < GESTURE_WRITE_LOOP; i++) {
			/* open ctp's gesture function */
			ft5x0x_write_reg(data->client, GESTURE_REG, GESTURE_ON);
			ft5x0x_read_reg(data->client, GESTURE_REG, &gesture_switch);

			if (GESTURE_ON == gesture_switch) {
				pr_buf_info("open ctp's gesture_fun.i = %d.(%s,%d)\n",
					i, __func__, __LINE__);
				break;
			}

			pr_buf_err("ctp's gesture_switch = %d\n", gesture_switch);
		}
		ft5x06_init_gesture(data);
	} else
#endif
	{
		if (gpio_is_valid(data->pdata->reset_gpio))
			ft5x0x_write_reg(data->client, FT_REG_PMODE,FT_PMODE_HIBERNATE);
	}

	data->suspended = true;
	pr_buf_info("%s suspend success.\n", __func__);

	return 0;
}
#endif
#endif

#if defined(CONFIG_TP_MATCH_HW)
static	int factory_check_gpio_state(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	bool gpio_state;

	gpio_state = gpio_get_value(data->pdata->tp_match_hw_gpio);
	pr_buf_info("gpio_state = %d\n", gpio_state);

	return gpio_state;
}

static	int factory_tp_match_hw(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	pr_buf_info("pannel_id = 0x%02X\n", data->pannel_id);

	if((0xFC == data->pannel_id) && (0 == factory_check_gpio_state(dev)))
		return 0;
	else
		return 1;
}
#endif

static int factory_ts_func_test_register(struct ft5x06_ts_data *data)
{
	ts_gen_func_test_init();
	data->ts_test_dev.dev = &data->client->dev;
	data->ts_test_dev.check_fw_update_need = factory_check_fw_update_need;
	data->ts_test_dev.get_calibration_ret = factory_get_calibration_ret;
	data->ts_test_dev.get_fs_fw_version = factory_get_fs_fw_version;
	data->ts_test_dev.get_fw_update_progress = factory_get_fw_update_progress;
	data->ts_test_dev.get_ic_fw_version = factory_get_ic_fw_version;
	data->ts_test_dev.get_module_id = factory_get_module_id;
	data->ts_test_dev.get_rawdata = factory_get_rawdata;
	data->ts_test_dev.get_rawdata_info = factory_get_rawdata_info;
#ifndef CONFIG_TOUCHSCREEN_FT5X06_GFF_PANNEL
	data->ts_test_dev.get_diff = factory_get_diff;
#endif
	data->ts_test_dev.proc_fw_update = factory_proc_fw_update;
	data->ts_test_dev.proc_hibernate_test = factory_proc_hibernate_test;
	data->ts_test_dev.set_fw_path = factory_set_fw_path;
	data->ts_test_dev.get_fw_path = factory_get_fw_path;
#if defined(CONFIG_TP_MATCH_HW)
	data->ts_test_dev.check_gpio_state = factory_check_gpio_state;
	data->ts_test_dev.tp_match_hw = factory_tp_match_hw;
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	data->ts_test_dev.get_gesture_switch = get_gesture_switch;
	data->ts_test_dev.set_gesture_switch = set_gesture_switch;
	data->ts_test_dev.get_gesture_pos = get_gesture_pos;
	#ifdef CONFIG_TOUCHSCREEN_FT5X06_SUB
	data->ts_test_dev.get_gesture_fingers = ft5x06_ts_get_gesture_fingers_status;
	#endif
#endif
	data->ts_test_dev.get_tp_enable_switch = get_tp_enable_switch;
	data->ts_test_dev.set_tp_enable_switch = set_tp_enable_switch;
#ifdef CONFIG_TOUCHSCREEN_FT5X06_GLOVE
	data->ts_test_dev.get_glove_switch = get_glove_switch;
	data->ts_test_dev.set_glove_switch = set_glove_switch;
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06_HALL
	data->ts_test_dev.get_hall_switch = get_hall_switch;
	data->ts_test_dev.set_hall_switch = set_hall_switch;
#endif

#ifndef CONFIG_TOUCHSCREEN_FT5X06_APK_TEST
	if (FT_FT5446_FAMILY_ID_0x54 == data->family_id) {
		data->ts_test_dev.get_short_test = factory_get_short_test;
		data->ts_test_dev.need_test_config = factory_get_test_config_need;
		data->ts_test_dev.set_test_config_path = factory_set_test_config_path;
	}
#endif
	data->ts_test_dev.get_chip_type = factory_get_chip_type;
	data->ts_test_dev.get_tp_settings_info = factory_get_chip_settings;
	data->ts_test_dev.get_factory_info = factory_get_factory_info;
	data->ts_test_dev.set_erase_flash_test = ft5x06_set_erase_flash_test;
	data->ts_test_dev.get_tp_irq_awake_switch = ft5x06_get_tp_irq_awake_switch;
	data->ts_test_dev.set_tp_irq_awake_switch = ft5x06_set_tp_irq_awake_switch;

#ifdef CONFIG_FT_INCELL_CHIP
	data->ts_test_dev.ts_async_suspend_for_lcd_use = ft8xxx_ts_suspend_for_lcd_async_use;
	data->ts_test_dev.ts_reset_for_lcd_use = ft8xxx_ts_provide_reset_control;
	data->ts_test_dev.ts_suspend_need_lcd_reset_high = ft8xxx_suspend_need_lcd_reset_high;
#endif
#if defined(CONFIG_FT_INCELL_CHIP) && defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	data->ts_test_dev.ts_suspend_need_lcd_power_reset_high = ft8xxx_need_lcd_power_reset_keep_flag_get;
#endif
#ifdef CONFIG_TOUCHSCREEN_WORK_MODE
		data->ts_test_dev.get_tp_work_mode = get_tp_work_mode;
		data->ts_test_dev.set_tp_work_mode = set_tp_work_mode;
#endif

	register_ts_func_test_device(&data->ts_test_dev);
	return 0;
}

static struct device *s_ctp_dev;

static int ctp_register_device(struct device *dev)
{
	s_ctp_dev = dev;
	return 0;
}

static int ctp_unregister_device(struct device *dev)
{
	s_ctp_dev = NULL;
	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_FH)
static int ft5x06_work_with_ac_usb_plugin(struct ft5x06_ts_data *data, int plugin)
{
	u8 val = 0;
	if (plugin != 0)
		val = 1;

	dev_info(&data->client->dev, "%s: %d\n", __func__, val);

	return ft5x0x_write_reg(data->client, 0x8B, val);
}

int ft_ctp_work_with_ac_usb_plugin(int plugin)
{
	struct ft5x06_ts_data *data;

	if (s_ctp_dev == NULL)
		return -ENODEV;

	data = dev_get_drvdata(s_ctp_dev);

	if (!data->suspended) {
	    if (data->pdata->support_usb_check) {
			if (0 == plugin)
				pr_buf_info("USB is plugged Out(%d,%s)\n", __LINE__, __func__);
			else
				pr_buf_info("USB is plugged In(%d,%s)\n", __LINE__, __func__);

			ft5x06_work_with_ac_usb_plugin(data, plugin);
		} else {
			pr_buf_info("ctp no need check usb(%d,%s)\n", __LINE__, __func__);
		}
	} else {
		data->ac_usb_plugin = plugin;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ft_ctp_work_with_ac_usb_plugin);
#endif

static int ft5x06_enter_update(struct ft5x06_ts_data *ts_data)
{
	struct i2c_client *client = ts_data->client;
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	int i, j;

	for (i = 0; i < RESET_DELAY_ARRAY_LENGTH; i++) {
		pr_buf_info("Step 1:Reset the CTP.\n");
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(FT_Reset_delay_5336[i]);
		pr_buf_info(" FT_Reset_delay_5336[%d]=%d\n", i, FT_Reset_delay_5336[i]);

		/* Enter upgrade mode */
		pr_buf_info("Step 2:Enter upgrade mode.\n");
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, w_buf, 1);
		msleep(FT_55_AA_DLY_MS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		pr_buf_info(" Step 3: Check IC ID.\n");
		mdelay(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;
		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
		if ((r_buf[0] != info.upgrade_id_1) || (r_buf[1] != info.upgrade_id_2)) {
			pr_buf_err(" Upgrade ID(%u,%u) mismatch (%u,%u)(%d)\n",
					r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2, i);
		} else {
			pr_buf_info(" IC_ID_1=0x%x,IC_ID_2=0x%x.\n", r_buf[0], r_buf[1]);
			return 0;
	    }
	}

	pr_buf_err("Hard Reset Style Check IC ID ERROR!\n");

	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		for (j = 0; j < FT_UPGRADE_LOOP; j++) {
			if (FT6X06_ID == ts_data->family_id || FT_FT6436_FAMILY_ID_0x36 == ts_data->family_id)
				reset_reg = FT_RST_CMD_REG2;
			else
				reset_reg = FT_RST_CMD_REG1;

			/* Reset the Ctp */
			pr_buf_info(" Step 1:Reset the ctp\n");
			pr_buf_info(" i=%d,info.delay_aa=%d\n", i, (info.delay_aa + 5*i));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
			mdelay(info.delay_aa + 5*i);
			pr_buf_info(" j=%d,info.delay_55=%d\n", j, (info.delay_55 + 5*j));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
			mdelay(info.delay_55 + 5*j);

			/* Enter upgrade mode */
			pr_buf_info(" Step 2:Enter upgrade mode.\n");
			w_buf[0] = FT_UPGRADE_55;
			ft5x06_i2c_write(client, w_buf, 1);
			msleep(FT_55_AA_DLY_MS);
			w_buf[0] = FT_UPGRADE_AA;
			ft5x06_i2c_write(client, w_buf, 1);

			/* check READ_ID */
			pr_buf_info(" Step 3: Check IC ID.\n");
			mdelay(info.delay_readid);
			w_buf[0] = FT_READ_ID_REG;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
			if ((r_buf[0] != info.upgrade_id_1) || (r_buf[1] != info.upgrade_id_2)) {
				pr_buf_err(" Upgrade ID(%u,%u) mismatch (%u,%u)(%d)\n",
						r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2, i);
			} else {
				pr_buf_info("IC_ID_1=0x%x,IC_ID_2=0x%x.\n", r_buf[0], r_buf[1]);
				return 0;
		    }
	    }
	}

	pr_buf_err("Soft Reset Style Check IC ID ERROR!\n");
	pr_buf_err("Abort upgrade\n");
	return -EIO;
}

static int ft5x06_leave_update(struct ft5x06_ts_data *data)
{
	/* reset */
	u8 w_buf[FT_MAX_WR_BUF] = {0};

	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(data->client, w_buf, 1);
	msleep(data->pdata->soft_rst_dly);

	return 0;
}

static int ft5x06_get_pannel_id_enter_upgrademode_if_needed(
									struct ft5x06_ts_data *ts_data, bool *enter_upgrade)
{
	u8 pannel_id = 0;
	int rc = 0;
	struct device *dev = &(ts_data->client->dev);

	*enter_upgrade =  false;
	pr_buf_info("In normal mode, try get pannel id....\n");
	dev_info(dev, "In normal mode, try get pannel id....\n");
	rc = ft5x0x_read_reg(ts_data->client, FT_RGE_PANNEL_ID, &pannel_id);
	pr_buf_err("a rc = %d, pannel_id = %d\n",rc,pannel_id);

	if (rc < 0 || pannel_id == 0 || pannel_id == 239 || pannel_id == 255) {
		pr_buf_err("Can't get pannel id in normal mode; Try to enter upgrade mode....\n");
		if (FT_FT5446_FAMILY_ID_0x54 == ts_data->family_id) {
			if (ft5x46_enter_update(ts_data) == 0) {
				*enter_upgrade = true;
				ft5x06_i2c_write(ts_data->client, ts_data->pdata->panelid_command,
								sizeof(ts_data->pdata->panelid_command));
				msleep(5);
				rc = ft5x06_i2c_read(ts_data->client, ts_data->pdata->panelid_command,
									sizeof(ts_data->pdata->panelid_command), &pannel_id, 1);
				pr_buf_err("panelid_command addr = [%d %d %d %d]\n",ts_data->pdata->panelid_command[3],ts_data->pdata->panelid_command[2],ts_data->pdata->panelid_command[1],ts_data->pdata->panelid_command[0]);
				if (rc < 0) {
					pr_buf_err("Ft5x46 in upgrade mode, Get pannel id failed\n");
					return rc;
				}
				
				pr_buf_info("Get pannel id = 0x%X\n", pannel_id);
	
				ts_data->pannel_id = pannel_id;

			} else
				return -EIO;
		} else {
			pr_buf_err("b rc = %d, pannel_id = %d\n",rc,pannel_id);
			if (ft5x06_enter_update(ts_data) == 0) {
				*enter_upgrade = true;
				pr_buf_err("Ft5x06 in upgrade mode, try get pannel id....\n");
				rc = ft5x06_i2c_read(ts_data->client, ts_data->pdata->panelid_command,
									 sizeof(ts_data->pdata->panelid_command), &pannel_id, 1);
				if (rc < 0) {
					pr_buf_err("Ft5x06 in upgrade mode, Get pannel id failed\n");
					return rc;
				}
			} else
				return -EIO;
		}
	}

	//ts_data->pannel_id = pannel_id;

	pr_buf_info("ts_data->pannel id = 0x%X\n", ts_data->pannel_id);
	return 0;
}

static int ft5x06_get_fw_filename(struct ft5x06_ts_data *data, u8 pannel_id, char *buf, int size)
{
	char tmp[16] = {0};
	const char *str = "NULL";

	if (pannel_id != 0) {
		strlcpy(buf, "fw_", size);
		strlcat(buf, CONFIG_HISENSE_PRODUCT_NAME, size);
		strlcat(buf, "_", size);
		if (!strcmp(str, data->pdata->factory_info))
			strlcat(buf, data->pdata->name, size);
		else
			strlcat(buf, data->pdata->factory_info, size);
		snprintf(tmp, ARRAY_SIZE(tmp), "_0x%02x", pannel_id);
		strlcat(buf, tmp, size);
		strlcat(buf, ".bin", size);
	} else {
		strlcpy(buf, data->pdata->fw_name_pref, size);
	}

	return 0;
}

static int ft5x06_fetch_fw(struct ft5x06_ts_data *data, const struct firmware **ppfw)
{
	int rc;
	size_t len;
	const struct firmware *fw = NULL;
	struct device *dev = &data->client->dev;
	char fw_file_name[FT_FW_NAME_MAX_LEN] = {0};
	char fw_path[FT_FW_NAME_MAX_LEN] = {0};

	if (ppfw == NULL)
		return -EINVAL;

	if (data->fw_name[0]) {
		strlcpy(fw_file_name, data->fw_name, sizeof(fw_file_name));
		dev_info(dev, "%s: with ori fw_name = %s\n",
				__func__, data->fw_name);
	} else {
		ft5x06_get_fw_filename(data, data->pannel_id, fw_file_name, sizeof(fw_file_name));
		dev_info(dev, "%s: with ori fw_name=null, fw_file_name=%s\n",
				__func__, fw_file_name);
	}

	if (data->fw_path[0]) {
		strlcpy(fw_path, data->fw_path, sizeof(fw_path));
		len = strlen(fw_path);
		if (fw_path[len-1] != '/')
			strlcat(fw_path, "/", sizeof(fw_path));
	}

	strlcat(fw_path, fw_file_name, sizeof(fw_path));
	pr_buf_info("%s: fw_path=%s\n", __func__, fw_path);
	rc = request_firmware(&fw, fw_path, dev);

	if (rc < 0) {
		pr_buf_err("Request firmware failed (%d)\n", rc);
		return rc;
	}
	*ppfw = fw;

	return 0;
}

static int ft5x06_get_fw_upgrade_config(struct ft5x06_ts_data *data,
										struct upgrade_config *config)
{
	int rc;
	const struct firmware *fw;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min;
	bool need_update  = false;

	rc = ft5x06_get_pannel_id_enter_upgrademode_if_needed(data, &config->enter_upgrade_mode);

	if (rc < 0)
		return -EIO;

	rc = ft5x06_fetch_fw(data, &config->firmware);
	fw = config->firmware;
	if (rc != 0 || fw == NULL) {
		return -EIO;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		pr_buf_err("Invalid firmware size (%zd)\n", fw->size);
		return -EIO;
	}
	if (FT_FT6436_FAMILY_ID_0x36 == data->family_id || FT_FT5822_FAMILY_ID_0x58 == data->family_id)
		fw_file_maj = FT_6436FW_FILE_MAJ_VER(fw);
    else if (FT_FT5446_FAMILY_ID_0x54 == data->family_id){
		if(0x53 == data->pannel_id || 0x54 == data->pannel_id)
		   fw_file_maj = FT_5452FW_FILE_MAJ_VER(fw);
		else if (0x5C == data->pannel_id || 0x5E == data->pannel_id)
		   fw_file_maj = FT_5422uFW_FILE_MAJ_VER(fw);
		else
			fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
		}
	else if (FT_FT8006_FAMILY_ID_0x80 == data->family_id)
		fw_file_maj = FT_8006FW_FILE_MAJ_VER(fw);
	else if (FT_FT8006P_FAMILY_ID_0x86 == data->family_id)
		fw_file_maj = FT_8006PFW_FILE_MAJ_VER(fw);
	else if (FT_FT8719_FAMILY_ID_0x87 == data->family_id)
		fw_file_maj = FT_8719FW_FILE_MAJ_VER(fw);
	else
		fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	pr_buf_info("Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	pr_buf_info("New firmware: %d.%d.%d\n", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	need_update = (fw_file_maj > data->fw_ver[0]) ? true :
									(fw_file_maj < data->fw_ver[0]) ? false :
										(fw_file_min > data->fw_ver[1]) ? true :
											(fw_file_min < data->fw_ver[1]) ? false :
												(fw_file_sub_min > data->fw_ver[2]) ? true : false;
	if(data->fw_ver[0] >= 0xEF)
		need_update = true;

	config->need_upgrade = need_update;
	return 0;
}

static void ft5x06_triger_update(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, force);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
}

static struct ft5x06_touch_event *ft5x06_allocate_touch_event_seq(
									struct device *dev, u32 max_touches)
{
	struct ft5x06_touch_event *event = NULL;
	event = devm_kzalloc(dev, sizeof(struct ft5x06_touch_event), GFP_KERNEL);
	return event;
}

static struct ft5x06_touch_event *ft5x06_new_touchdata_index(struct ft5x06_ts_data *data)
{
	struct ft5x06_touch_event *event;

	u8 index = (++data->last_touch_event_index)%2;
	event = data->event[index];
	memset(event, 0, sizeof(struct ft5x06_touch_event));

	return event;
}

static struct ft5x06_touch_event *ft5x06_pri_touchdata_event(struct ft5x06_ts_data *data)
{
	u8 index = (data->last_touch_event_index+1)%2;
	return  data->event[index];
}

static struct ft5x06_touch_event *ft5x06_cur_touchdata_event(struct ft5x06_ts_data *data)
{
	u8 index = (data->last_touch_event_index)%2;
	return data->event[index];
}

static int ft5x06_fetch_touchdata(struct ft5x06_ts_data *data)
{
	struct ft5x06_touch_event *event;
	int rc, i;
	u8 id;
	u8 reg = 0x00;
	u8 *buf = data->tch_data;

	rc = ft5x06_i2c_read(data->client, &reg, 1, buf, data->tch_data_len);
	if (rc < 0) {
		pr_buf_err("%s: read data fail\n", __func__);
		return rc;
	}

	event = ft5x06_new_touchdata_index(data);

	for (i = 0; i < data->pdata->num_max_touches; i++) {

	   id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
	   if (id >= FT_MAX_ID)
		   break;

	   event->x[i] = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
		   (buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
	   event->y[i] = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
		   (buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);
	   event->id[i] = id;
	   event->status[i] = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;
	   event->weight[i] = buf[FT_TOUCH_WEIGHT_POS + FT_ONE_TCH_LEN * i];
	   event->area[i] = (data->pdata->touch_area_param) * (buf[FT_TOUCH_AREA_POS + FT_ONE_TCH_LEN * i] >> 4);
	}
	event->count = i;

#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
	if (data->ps_en) {
		buf[1] = buf[1]>>5;
		if (last_ps_val != buf[1]) {
			/* far */
			if (buf[1] == 0x07) {
				psensor_report_dist(data, FAR_CODE);
				pr_buf_err("PS_NEAR_TO_FAR INT occurred.\n");
			}
			/* near */
			if (buf[1] == 0x06) {
				psensor_report_dist(data, NEAR_CODE);
				pr_buf_err("PS_FAR_TO_NEAR INT occurred.\n");
			}
			last_ps_val = buf[1];
		}
	}
#endif

	return 0;
}


static void ft5x06_adjust_touchdata(struct ft5x06_ts_data *data)
{
	int i, j;
	u8 pre_id;
	struct ft5x06_touch_event *cur_event = ft5x06_cur_touchdata_event(data);
	struct ft5x06_touch_event *pre_event = ft5x06_pri_touchdata_event(data);

	for (i = 0; i < pre_event->count; i++) {
		pre_id = pre_event->id[i];
		if (pre_event->status[i] == 0 || pre_event->status[i] == 2) {

			for (j = 0; j < cur_event->count; j++) {
				if (cur_event->id[j] == pre_id)
					break;
			}
			if ((j >= cur_event->count)
				&& (cur_event->count < data->pdata->num_max_touches)) {
				cur_event->id[cur_event->count] = pre_id;
				cur_event->status[cur_event->count] = 1;
				cur_event->count++;
			}
		}
	}
}

static void ft5x06_report_touchdata(struct ft5x06_ts_data *data)
{
	int i;
	struct ft5x06_touch_event *event = ft5x06_cur_touchdata_event(data);
	struct input_dev *input_dev =  data->input_dev;

	mutex_lock(&data->input_dev->mutex);
	for (i = 0; i < event->count; i++) {
		input_mt_slot(input_dev, event->id[i]);
		if (event->status[i] == FT_TOUCH_DOWN || event->status[i] == FT_TOUCH_CONTACT) {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, event->x[i]);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, event->y[i]);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
			/* input_report_abs(input_dev, ABS_MT_PRESSURE, event->weight[i]); */
			if(event->y[i] > data->pdata->y_max) {
				dev_info(&data->client->dev,
					"%s: [x, y]=[%d, %d],id = %d\n", __func__, event->x[i], event->y[i], event->id[i]);
			}
			dev_info(&data->client->dev, "%s: [x, y]=[%d, %d]\n", __func__, event->x[i], event->y[i]);
		} else {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		}
	}

	if (event->count) {
		input_mt_report_pointer_emulation(input_dev, false);
		input_sync(input_dev);
	}
	mutex_unlock(&data->input_dev->mutex);
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
static int ft5x06_init_gesture(struct ft5x06_ts_data *data)
{
	return 0;
}

#else
static int ft5x06_init_gesture(struct ft5x06_ts_data *data)
{
	int i, j;
	u8 reg_value;

	for (i = 0; i < data->pdata->gesture_reg_num; i++) {
		for (j = 0; j < GESTURE_WRITE_LOOP; j++) {
			ft5x0x_write_reg(data->client, data->pdata->gesture_reg_map[i], data->pdata->gesture_reg_value_map[i]);
			ft5x0x_read_reg(data->client, data->pdata->gesture_reg_map[i], &reg_value);
			if (data->pdata->gesture_reg_value_map[i] == reg_value) {
				dev_info(&data->client->dev, "Write reg 0x%2x success.loop = %d.(%s,%d)\n", 
					data->pdata->gesture_reg_map[i], j, __func__, __LINE__);
				break;
			}
		}
		if (GESTURE_WRITE_LOOP == j) {
			pr_buf_err("Write reg 0x%2x failed.loop = %d.(%s,%d)\n", data->pdata->gesture_reg_map[i], j, __func__, __LINE__);
		}
	}
	return 0;
}
#endif
static bool ft5x06_report_gesture(struct ft5x06_ts_data *data)
{
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FTS_GESTURE_POINTS * 3] = {0};
	int ret = 0;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
	unsigned short pointnum = 0;
#endif
	int gesture_id = 0;
	int index = 0;

	w_buf[0] = 0xd3;

	ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0) {
		 pr_buf_err("ft5x06_i2c_read() read data error!\n");
		 return false;
	}
	gesture_id = r_buf[0];
	data->gesture_track_pointnum = r_buf[1] & 0xff;

	if (data->gesture_track_pointnum > FTS_GESTURE_POINTS_NUM) {
		pr_buf_err("pointnum(%d) is out of range!\n", data->gesture_track_pointnum);
		return false;
	}

	if (FT_FT5446_FAMILY_ID_0x54 == data->family_id
	|| FT_FT5822_FAMILY_ID_0x58 == data->family_id
	|| FT_FT8006_FAMILY_ID_0x80 == data->family_id
	|| FT_FT8006P_FAMILY_ID_0x86 == data->family_id
	|| FT_FT8719_FAMILY_ID_0x87 == data->family_id) {
		if ((data->gesture_track_pointnum * 4 + 2) < 255) {
			ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, (data->gesture_track_pointnum * 4 + 2));
		} else {
			ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, 255);
			ret = ft5x06_i2c_read(data->client, w_buf, 0, r_buf + 255, (data->gesture_track_pointnum * 4 + 2) - 255);
		}
		if (ret < 0) {
			pr_buf_err("read touchdata failed.\n");
			return ret;
		}

		for (index = 0; index < data->gesture_track_pointnum; index++) {
			data->gesture_track_x[index] = (r_buf[2 + 4 * index] & 0x0F) << 8 | (r_buf[3 + 4 * index] & 0xFF);
			data->gesture_track_y[index] = (r_buf[4 + 4 * index] & 0x0F) << 8 | (r_buf[5 + 4 * index] & 0xFF);
		}
		goto gesture_report;
	} else {
		if (GESTURE_DOUBLECLICK == r_buf[0]) {
			gesture_id = GESTURE_DOUBLECLICK;
			if (data->gesture_state & 0x0002) {
				input_report_key(data->input_dev, data->pdata->gesture_func_map[0], 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, data->pdata->gesture_func_map[0], 0);
				input_sync(data->input_dev);
			}
			return true;
		}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
		pointnum = (short)(r_buf[1]) & 0xff;
		if ((pointnum * 4 + 8) < 255) {
			ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, (pointnum * 4 + 8));
		} else {
			ret = ft5x06_i2c_read(data->client, w_buf, 1, r_buf, 255);
			ret = ft5x06_i2c_read(data->client, w_buf, 0, r_buf + 255, (pointnum * 4 + 8) - 255);
		}
		if (ret < 0) {
			pr_buf_err("read touchdata failed.\n");
			return ret;
		}

		gesture_id = fetch_object_sample(r_buf, pointnum);

		for (index = 0; index < data->gesture_track_pointnum; index++) {
			data->gesture_track_x[index] = (r_buf[0 + 4 * index] & 0x0F) << 8 | (r_buf[1 + 4 * index] & 0xFF);
			data->gesture_track_y[index] = (r_buf[2 + 4 * index] & 0x0F) << 8 | (r_buf[3 + 4 * index] & 0xFF);
		}
#endif
	}

gesture_report:
	pr_buf_err("%s:gestrue_id = %x\n", __func__, gesture_id);
	for (index = 0; index < data->pdata->gesture_num; index++) {
		if (gesture_id == data->pdata->gesture_figure_map[index])
			break;
	}
	if (index >= data->pdata->gesture_num) {
		pr_buf_err("%s couldn't find the matched gesture.\n", __func__);
		return true;
	} else {
		if ((data->gesture_state) >> (index + 1) & 1) {
			input_report_key(data->input_dev, data->pdata->gesture_func_map[index], 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, data->pdata->gesture_func_map[index], 0);
			input_sync(data->input_dev);
		} else {
			pr_buf_err("%s not open the gesture switch.\n", __func__);
		}
	}

	return true;

}
#endif

static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x06_ts_data *data = dev_id;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	int	count = 10;
#endif

	if (!data) {
		pr_buf_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}
	if (data->suspended) {
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
		if (data->gesture_en) {
			do {
				if (ft5x06_report_gesture(data))
					break;
				msleep(GESTURE_LOOP_DELAY);
			} while (count--);
		}
#endif
	} else {
		if (data->irq_can_wake == true)
		{
			//disable_irq_wake(data->client->irq);

			input_report_key(data->input_dev, FT_IRQAWAKE_KEY, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, FT_IRQAWAKE_KEY, 0);
			input_sync(data->input_dev);
			data->irq_can_wake = false;
		}

		ft5x06_fetch_touchdata(data);
		ft5x06_adjust_touchdata(data);
		ft5x06_report_touchdata(data);
    }

	return IRQ_HANDLED;
}


static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc = 0;

	if (!on)
		goto power_off;

	#ifndef CONFIG_FT_INCELL_CHIP
       rc = regulator_set_load(data->vdd, (100*1000));
       if (rc) {
               dev_err(&data->client->dev,
                       "Regulator vdd set load  failed rc=%d\n", rc);
               return rc;
       }

	rc = regulator_enable(data->vdd);
	if (rc) {
		pr_buf_err("Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}
	#endif

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		pr_buf_err("Regulator vcc_i2c enable failed rc=%d\n", rc);
		#ifndef CONFIG_FT_INCELL_CHIP
		regulator_disable(data->vdd);
		#endif
	}

	return rc;

power_off:
#if 0
	#ifndef CONFIG_FT_INCELL_CHIP
	rc = regulator_disable(data->vdd);
	if (rc) {
		pr_buf_err("Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}
	#endif

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		pr_buf_err("Regulator vcc_i2c disable failed rc=%d\n", rc);
		#ifndef CONFIG_FT_INCELL_CHIP
		rc = regulator_enable(data->vdd);
		if (rc) {
			pr_buf_err("Regulator vdd enable failed rc=%d\n", rc);
		}
		#endif
	}
#endif
	return rc;
}

static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	#ifndef CONFIG_FT_INCELL_CHIP
	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		pr_buf_err("Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			pr_buf_err("Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}
	#endif

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		pr_buf_err("Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			pr_buf_err("Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
#ifndef CONFIG_FT_INCELL_CHIP
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
#endif
	return rc;

pwr_deinit:
	#ifndef CONFIG_FT_INCELL_CHIP
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);
	#endif
	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int ft5x06_ts_pinctrl_init(struct ft5x06_ts_data *ft5x06_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		pr_buf_info("Target does not use pinctrl\n");
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

#ifdef CONFIG_TOUCHSCREEN_FT5X06_SUB
	ft5x06_data->gpio_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			"pmx_sub_ts_active");
#else
	ft5x06_data->gpio_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			"pmx_ts_active");
#endif
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_active)) {
		pr_buf_err("Can not get ts default pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_active);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

#ifdef CONFIG_TOUCHSCREEN_FT5X06_SUB
	ft5x06_data->gpio_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			"pmx_sub_ts_suspend");
#else
	ft5x06_data->gpio_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			"pmx_ts_suspend");
#endif
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_suspend)) {
		pr_buf_err("Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_suspend);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int ft5x06_ts_pinctrl_select(struct ft5x06_ts_data *ft5x06_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ft5x06_data->gpio_state_active
		: ft5x06_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ft5x06_data->ts_pinctrl, pins_state);
		if (ret) {
#ifdef CONFIG_TOUCHSCREEN_FT5X06_SUB
			pr_buf_err("can not set %s pins\n",
				on ? "pmx_sub_ts_active" : "pmx_sub_ts_suspend");
#else
			pr_buf_err("can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
#endif
			return ret;
		}
	} else {
#ifdef CONFIG_TOUCHSCREEN_FT5X06_SUB
		pr_buf_err("not a valid '%s' pinstate\n",
				on ? "pmx_sub_ts_active" : "pmx_sub_ts_suspend");
#else
		pr_buf_err("not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
#endif
	}

	return 0;
}


#ifdef CONFIG_PM
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;
	u8 i = 0;
#ifdef CONFIG_TOUCHSCREEN_FT5X06_GESTURE
	u8 gesture_switch;
#endif

	if (data->loading_fw) {
		pr_buf_info("Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		pr_buf_info("%s:Already in suspend state\n", __func__);
		return 0;
	}
#ifdef CONFIG_TOUCHSCREEN_FACE_DETECTION
	if (data->ps_en)
		return 0;
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06_GESTURE
	data->gesture_suspend_en = data->gesture_en;
	if (data->gesture_suspend_en) {
		for (i = 0; i < GESTURE_WRITE_LOOP; i++) {
			/* open ctp's gesture function */
			ft5x0x_write_reg(data->client, GESTURE_REG, GESTURE_ON);
			ft5x0x_read_reg(data->client, GESTURE_REG, &gesture_switch);

			if (GESTURE_ON == gesture_switch) {
				pr_buf_info("open ctp's gesture_fun.i = %d.(%s,%d)\n",
					i, __func__, __LINE__);
				break;
			}

			pr_buf_err("ctp's gesture_switch = %d\n", gesture_switch);
		}
		ft5x06_init_gesture(data);
	} else
#endif
	{
		disable_irq(data->client->irq);

		if (gpio_is_valid(data->pdata->reset_gpio)) {
#ifdef CONFIG_FT_INCELL_CHIP
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
			msleep(data->pdata->soft_rst_dly);
#endif
			ft5x0x_write_reg(data->client, FT_REG_PMODE,FT_PMODE_HIBERNATE);
		}

		if (data->pdata->need_poweroff) {
			if (data->pdata->power_on) {
				err = data->pdata->power_on(false);
				if (err) {
					pr_buf_err("power off failed");
					goto pwr_off_fail;
				}
			} else {
				err = ft5x06_power_on(data, false);
				if (err) {
					pr_buf_err("power off failed");
					goto pwr_off_fail;
				}
			}
		}
	}

	data->suspended = true;
	data->resume_is_running = false;
	/* release all touches */
	mutex_lock(&data->input_dev->mutex);
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);
	mutex_unlock(&data->input_dev->mutex);
	pr_buf_info("%s suspend success.\n", __func__);
	return 0;

pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->client->irq);
	return err;
}

#ifndef CONFIG_TOUCHSCREEN_FT5X06_SUB
static void ft5x06_ts_resume_work(struct work_struct *work)
{
	struct ft5x06_ts_data *ts_data = container_of(work,
			struct ft5x06_ts_data, resume_work);

	ft5x06_ts_resume(&ts_data->client->dev);
}
#endif

static int ft5x06_ts_resume(struct device *dev)
{
	return ft5x06_ts_resume_force(dev, false);
}
static int ft5x06_ts_resume_force(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	int err;
	int i;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	u8 gesture_switch;
#endif

	data->resume_is_running = true;
	if (!force && !data->suspended) {
		pr_buf_info("%s:Already in awake state\n", __func__);
		return 0;
	}
#ifdef CONFIG_TOUCHSCREEN_FACE_DETECTION
	if (data->ps_en) {
		if (data->tp_ps_en) {
			if (data->pdata->need_poweroff) {
				if (data->pdata->power_on) {
					err = data->pdata->power_on(true);
					if (err) {
						pr_buf_err("power on failed");
						return err;
					}
				} else {
					err = ft5x06_power_on(data, true);
					if (err) {
						pr_buf_err("power on failed");
						return err;
					}
				}
			}
			if (gpio_is_valid(data->pdata->reset_gpio)) {
				gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
				msleep(data->pdata->hard_rst_dly);
				gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
			}
			msleep(data->pdata->soft_rst_dly);
			enable_irq(data->client->irq);
			pr_buf_info("%s: reset ic.\n", __func__);
			data->tp_ps_en = 0;
			err = ft5x0x_write_reg(data->client, 0XB0, 1);
			if (err < 0) {
				pr_buf_err(" enable tp ps function failed\n");
				data->ps_en = 0;
			}
			pr_buf_info("%s: resend face state %d.\n", __func__, data->ps_en);
			/*err = enable_irq_wake(data->client->irq);
			if (0 != err) {
				pr_buf_err(" enable_irq_wake failed for tp_ps_irq_handler\n");
				ft5x0x_write_reg(data->client, 0XB0, 0);
				data->ps_en = 0;
			}*/
			data->suspended = false;
		}
		return 0;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	if (data->gesture_suspend_en) {
		for (i = 0; i < GESTURE_WRITE_LOOP; i++) {
			/* close ctp's gesture function */
			ft5x0x_write_reg(data->client, GESTURE_REG, GESTURE_OFF);
			ft5x0x_read_reg(data->client, GESTURE_REG, &gesture_switch);
			if (GESTURE_OFF == gesture_switch) {
				pr_buf_info("close ctp's gesture_fun.i = %d.\n", i);
				break;
			}
		}
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}
		msleep(data->pdata->soft_rst_dly);
	} else
#endif
	{
		if (data->pdata->need_poweroff) {
			if (data->pdata->power_on) {
				err = data->pdata->power_on(true);
				if (err) {
					pr_buf_err("power on failed");
					return err;
				}
			} else {
				err = ft5x06_power_on(data, true);
				if (err) {
					pr_buf_err("power on failed");
					return err;
				}
				err = ft5x06_ts_pinctrl_select(data, true);
				if (err < 0)
					pr_buf_err("Cannot get idle pinctrl state\n");
			}
		}

		if (gpio_is_valid(data->pdata->reset_gpio)) {
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}
		msleep(data->pdata->soft_rst_dly);
		enable_irq(data->client->irq);
	}
	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);
	data->suspended = false;

#if defined(CONFIG_TOUCHSCREEN_FT5X06_FH)
	if (usb_flag)
		ft_ctp_work_with_ac_usb_plugin(true);
	else
		ft_ctp_work_with_ac_usb_plugin(false);
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06_GLOVE
	if (data->glove_enable) {
		err = ft5x0x_write_reg(data->client, 0xC0, 1);
			if (err < 0) {
				pr_buf_err("%s: write glove reg failed,with result=%d\n", __func__, err);
				return -EFAULT;
			}
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06_HALL
	if (data->hall_enable) {
		err = ft5x0x_write_reg(data->client, 0xC1, 1);
			if (err < 0) {
				pr_buf_err("%s: write glove reg failed,with result=%d\n", __func__, err);
				return -EFAULT;
			}
	}
#endif
	pr_buf_info("%s resume success.\n", __func__);
	return 0;
}
#ifndef CONFIG_TOUCHSCREEN_FT5X06_SUB
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		pr_buf_info("fb_notifier_callback:ft5x06_tp blank=%d\n", *blank);
		if (*blank == FB_BLANK_UNBLANK) {
			if (!work_pending(&ft5x06_data->resume_work))
				schedule_work(&ft5x06_data->resume_work);
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
		ft5x06_data->ps_lcd_state = *blank;
#endif
		}
		#if 0
		else if (*blank == FB_BLANK_POWERDOWN) {
			if(ft5x06_data->irq_can_wake == false)
			{
				cancel_work_sync(&ft5x06_data->resume_work);
				ft5x06_ts_suspend(&ft5x06_data->client->dev);
			}
		#endif

	}else if(evdata && evdata->data && event == FB_EARLY_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client){
		blank = evdata->data;
		pr_buf_info("fb_notifier_callback:ft5x06_tp blank=%d\n", *blank);
		if (*blank == FB_BLANK_POWERDOWN) {
			if(ft5x06_data->irq_can_wake == false)
			{
				cancel_work_sync(&ft5x06_data->resume_work);
				ft5x06_ts_suspend(&ft5x06_data->client->dev);
			}
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
		ft5x06_data->ps_lcd_state = *blank;
#endif
		}

	}
	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif
#endif /*end CONFIG_TOUCHSCREEN_FT5X06_SUB*/

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
static int ft5x06_ts_suspend_gesture(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (data->gesture_suspend_en) {
		pr_info("ft5x06_ts_suspend_gesture\n");
		//disable_irq(data->client->irq);
		/* make tp can wake the system */
		//enable_irq_wake(data->client->irq);
	}

	return 0;
}
static int ft5x06_ts_resume_gesture(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (data->gesture_suspend_en) {
		pr_info("ft5x06_ts_resume_gesture\n");
		/* make tp cannot wake the system */
		//disable_irq_wake(data->client->irq);
		//enable_irq(data->client->irq);
	}

	return 0;
}
#endif

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	.suspend = ft5x06_ts_suspend_gesture,
	.resume = ft5x06_ts_resume_gesture,
#endif
};
#endif

static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}

static int ft8006_fw_upgrade_start(struct i2c_client * client,const u8* pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0, j = 0;
	u32 packet_number = 0;
	u32 temp, lenght;
	u8 packet_buf[FT_FW_PKT_LEN + 6];
	u8 auc_i2c_write_buf[10];
	u8 upgrade_ecc;

	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	if (ft5x46_enter_update(ts_data) != 0) {
		pr_buf_info("enter update mode failed and Reset the CTP.(%s,%d)\n", __func__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}
	/*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
	auc_i2c_write_buf[0] = 0x09;
	auc_i2c_write_buf[1] = 0x0B;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 2);

	/*Step 4:erase app and panel paramenter area*/
	pr_buf_info("[UPGRADE]: erase app and panel paramenter area!!");
	auc_i2c_write_buf[0] = 0x61;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(2000);

	for (i = 0; i < 15; i++) {
	    auc_i2c_write_buf[0] = 0x6a;
	    reg_val[0] = reg_val[1] = 0x00;
	    ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
	    if (0xF0==reg_val[0] && 0xAA==reg_val[1])
	        break;
	    msleep(50);
	}
	pr_buf_info("[UPGRADE]: erase app area reg_val[0] = %x reg_val[1] = %x!!", reg_val[0], reg_val[1]);

	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	ft5x06_i2c_write(client, auc_i2c_write_buf, 4);

	/*write FW to ctpm flash*/
	upgrade_ecc = 0;
	pr_buf_info("[UPGRADE]: write FW to ctpm flash!!");
	temp = 0;
	packet_number = (dw_lenth) / FT_FW_PKT_LEN;
	packet_buf[0] = 0xBF;

	for (j = 0; j < packet_number; j++)
	{
	    temp = 0x5000 + j * FT_FW_PKT_LEN;
	    packet_buf[1] = (u8) (temp >> 16);
	    packet_buf[2] = (u8) (temp >> 8);
	    packet_buf[3] = (u8) temp;
	    lenght = FT_FW_PKT_LEN;
	    packet_buf[4] = (u8) (lenght >> 8);
	    packet_buf[5] = (u8) lenght;
	    for (i = 0; i < FT_FW_PKT_LEN; i++) {
	        packet_buf[6 + i] = pbt_buf[j * FT_FW_PKT_LEN + i];
	        upgrade_ecc ^= packet_buf[6 + i];
	    }
	    ft5x06_i2c_write(client, packet_buf, FT_FW_PKT_LEN + 6);

	    for (i = 0; i < 30; i++) {
	        auc_i2c_write_buf[0] = 0x6a;
	        reg_val[0] = reg_val[1] = 0x00;
	        ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

	        if ((j + 0x1000 + (0x5000/FT_FW_PKT_LEN)) == (((reg_val[0]) << 8) | reg_val[1]))
	            break;

	        if (i > 15) {
	            msleep(1);
	            pr_buf_err("[UPGRADE]: write flash: host : %x status : %x!", (j + 0x1000 + (0x5000/FT_FW_PKT_LEN)), (((reg_val[0]) << 8) | reg_val[1]));
	        }
		msleep(1);
	    }
	}

	if ((dw_lenth) % FT_FW_PKT_LEN > 0) {
	    temp = 0x5000 + packet_number * FT_FW_PKT_LEN;
	    packet_buf[1] = (u8) (temp >> 16);
	    packet_buf[2] = (u8) (temp >> 8);
	    packet_buf[3] = (u8) temp;
	    temp = (dw_lenth) % FT_FW_PKT_LEN;
	    packet_buf[4] = (u8) (temp >> 8);
	    packet_buf[5] = (u8) temp;
	    for (i = 0; i < temp; i++) {
	        packet_buf[6 + i] = pbt_buf[packet_number * FT_FW_PKT_LEN + i];
	        upgrade_ecc ^= packet_buf[6 + i];
	    }
	    ft5x06_i2c_write(client, packet_buf, temp + 6);

	    for (i = 0; i < 30; i++) {
	        auc_i2c_write_buf[0] = 0x6a;
	        reg_val[0] = reg_val[1] = 0x00;
	        ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if ((0x1000 + ((0x5000 + packet_number * FT_FW_PKT_LEN)/((dw_lenth) % FT_FW_PKT_LEN))) == (((reg_val[0]) << 8) | reg_val[1]))
			break;
	        if (i > 15) {
	            msleep(1);
	            pr_buf_err("[UPGRADE]: write flash: host : %x status : %x!!", (j + 0x1000 + (0x5000/FT_FW_PKT_LEN)), (((reg_val[0]) << 8) | reg_val[1]));
	       	}
	        msleep(1);
	    }
	}
	msleep(50);
	/*Step 6: read out checksum*/
	/*send the opration head */
	pr_buf_info("[UPGRADE]: read out checksum!!");
	auc_i2c_write_buf[0] = 0x64;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	temp = 0x5000;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = (64*1024-1);
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	ft5x06_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth/256);

	temp = (0x5000+(64*1024-1));
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = (dw_lenth-(64*1024-1));
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	ft5x06_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth/256);

	for (i = 0; i < 100; i++) {
	    auc_i2c_write_buf[0] = 0x6a;
	    reg_val[0] = reg_val[1] = 0x00;
	    ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
	    if (0xF0==reg_val[0] && 0x55==reg_val[1]) {
	        pr_buf_err("[UPGRADE]: reg_val[0]=%02x reg_val[0]=%02x!!", reg_val[0], reg_val[1]);
	        break;
	    }
	    msleep(1);
	}
	auc_i2c_write_buf[0] = 0x66;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != upgrade_ecc) {
	    pr_buf_err("[UPGRADE]: ecc error! FW=%02x upgrade_ecc=%02x!!",reg_val[0],upgrade_ecc);
	    return -EIO;
	}
	pr_buf_err("[UPGRADE]: checksum %x %x!!",reg_val[0],upgrade_ecc);

	pr_buf_info("[UPGRADE]: reset the new FW!!");
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1000);
	hidi2c_to_stdi2c(client);

	pr_buf_info("%s: Firmware upgrade finish.\n", __func__);
	return 0;
}

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	/* u8 reset_reg; */
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;

	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;

	if (ft5x06_enter_update(ts_data) != 0) {
		pr_buf_info("enter update mode failed and Reset the CTP.(%s,%d)\n", __func__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft5x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
						FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		pr_buf_err("ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	pr_buf_info("%s: Firmware upgrade finish.\n", __func__);
	return 0;
}

static int hidi2c_to_stdi2c(struct i2c_client *client)
{
	u8 w_buf[5] = {0};
	int ret = 0;

	w_buf[0] = 0xeb;
	w_buf[1] = 0xaa;
	w_buf[2] = 0x09;
	ret = ft5x06_i2c_write(client, w_buf, 3);
	msleep(10);
	w_buf[0] = w_buf[1] = w_buf[2] = 0;
	ft5x06_i2c_read(client, w_buf, 0, w_buf, 3);

	if (0xeb == w_buf[0] && 0xaa == w_buf[1] && 0x08 == w_buf[2]) {
		pr_buf_info("hidi2c_to_stdi2c successful.\n");
		ret = 1;
	} else {
		pr_buf_info("hidi2c_to_stdi2c error.\n");
		ret = 0;
	}

	return ret;
}

static int ft5x46_enter_update(struct ft5x06_ts_data *ts_data)
{
	struct fw_upgrade_info info = ts_data->pdata->info;
	struct i2c_client *client = ts_data->client;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	int i, j, ret;

	ret = hidi2c_to_stdi2c(client);
	if (ret == 0) {
		pr_buf_info("%s: [FTS] hid change to i2c fail ! \n", __func__);
	}


	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		for (j = 0; j < FT_UPGRADE_LOOP; j++) {
			if (FT6X06_ID == ts_data->family_id || FT_FT6436_FAMILY_ID_0x36 == ts_data->family_id) {
				reset_reg = FT_RST_CMD_REG2;
			} else {
				reset_reg = FT_RST_CMD_REG1;
			}

			/* Reset the Ctp */
			dev_info(&ts_data->client->dev, " Step 1:Reset the ctp\n");
			dev_info(&ts_data->client->dev, " i=%d,info.delay_aa=%d\n", i, (info.delay_aa + 5*i));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
			mdelay(info.delay_aa + 5*i);
			dev_info(&ts_data->client->dev, " j=%d,info.delay_55=%d\n", j, (info.delay_55 + 10*j));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
			mdelay(info.delay_55 + 5*j);

			ret = hidi2c_to_stdi2c(client);
			if (ret == 0) {
				dev_err(&client->dev, "[FTS] hid change to i2c fail ! \n");
			}
			msleep(10);
			/* Enter upgrade mode */
			dev_info(&ts_data->client->dev, " Step 2:Enter upgrade mode.\n");
			w_buf[0] = FT_UPGRADE_55;
			w_buf[1] = FT_UPGRADE_AA;
			ft5x06_i2c_write(client, w_buf, 2);

			/* check READ_ID */
			dev_info(&ts_data->client->dev, " Step 3: Check IC ID.\n");
			mdelay(info.delay_readid);
			w_buf[0] = FT_READ_ID_REG;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
			if ((r_buf[0] == info.upgrade_id_1) && (r_buf[1] == info.upgrade_id_2)) {
				pr_buf_info(" Upgrade ID(%u,%u) match (%u,%u)(%d)\n",
						r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2, i);
				break;
			} else if((r_buf[0] == info.upgrade_id_1) && (r_buf[1] == info.upgrade_id_3)){
				pr_buf_info(" Upgrade ID(%u,%u) match (%u,%u)(%d)\n",
						r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_3, i);
				break;
			} else if((r_buf[0] == info.upgrade_id_1) && (r_buf[1] == info.upgrade_id_5)){
				pr_buf_info(" Upgrade ID(%u,%u) match (%u,%u)(%d)\n",
						r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_5, i);
				break;
			} else{
				pr_buf_info(" Upgrade ID(%u,%u) mismatch (%u,%u) or (%u,%u) or (%u,%u) (%d)\n",
						r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2, info.upgrade_id_1, info.upgrade_id_3,info.upgrade_id_1, info.upgrade_id_5,i);
			}
		}
		if (j != FT_UPGRADE_LOOP)
			break;
	}

	if (i == FT_UPGRADE_LOOP) {
		pr_buf_info("Soft Reset Style Check IC ID ERROR!\n");
		pr_buf_err("Abort upgrade\n");
		return -EIO;
	}
	pr_buf_info(" F5x46 enter update success.\n");
	msleep(50);
	return 0;
}

static int ft8719_enter_update(struct ft5x06_ts_data *ts_data)
{
	struct fw_upgrade_info info = ts_data->pdata->info;
	struct i2c_client *client = ts_data->client;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	int i, j, ret;

	ret = hidi2c_to_stdi2c(client);
	if (ret == 0) {
		pr_buf_info("%s: [FTS] hid change to i2c fail ! \n", __func__);
	}


	for (i = 0; i < FT_UPGRADE_LOOP; i++) {
		for (j = 0; j < FT_UPGRADE_LOOP; j++) {
			if (FT6X06_ID == ts_data->family_id || FT_FT6436_FAMILY_ID_0x36 == ts_data->family_id) {
				reset_reg = FT_RST_CMD_REG2;
			} else {
				reset_reg = FT_RST_CMD_REG1;
			}

			/* Reset the Ctp */
			dev_info(&ts_data->client->dev, " Step 1:Reset the ctp\n");
			dev_info(&ts_data->client->dev, " i=%d,info.delay_aa=%d\n", i, (info.delay_aa + 5*i));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
			mdelay(info.delay_aa + 5*i);
			dev_info(&ts_data->client->dev, " j=%d,info.delay_55=%d\n", j, (info.delay_55 + 10*j));
			ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
			mdelay(info.delay_55 + 5*j);

			/*ret = hidi2c_to_stdi2c(client);
			if (ret == 0) {
				dev_err(&client->dev, "[FTS] hid change to i2c fail ! \n");
			}
			msleep(10);
			*/
			/* Enter upgrade mode */
			dev_info(&ts_data->client->dev, " Step 2:Enter upgrade mode.\n");
			w_buf[0] = FT_UPGRADE_55;
			w_buf[1] = FT_UPGRADE_AA;
			ft5x06_i2c_write(client, w_buf, 2);

			/* check READ_ID */
			dev_info(&ts_data->client->dev, " Step 3: Check IC ID.\n");
			mdelay(info.delay_readid);
			w_buf[0] = FT_READ_ID_REG;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
			if ((r_buf[0] != info.upgrade_id_1) || (r_buf[1] != info.upgrade_id_2)) {
				pr_buf_info(" Upgrade ID(%u,%u) mismatch (%u,%u)(%d)\n",
						r_buf[0], r_buf[1], info.upgrade_id_1, info.upgrade_id_2, i);
			} else {
				pr_buf_info("IC_ID_1=0x%x,IC_ID_2=0x%x.\n", r_buf[0], r_buf[1]);
				break;
			}
		}
		if (j != FT_UPGRADE_LOOP)
			break;
	}

	if (i == FT_UPGRADE_LOOP) {
		pr_buf_info("Soft Reset Style Check IC ID ERROR!\n");
		pr_buf_err("Abort upgrade\n");
		return -EIO;
	}
	pr_buf_info(" Ft8719 enter update success.\n");
	msleep(50);
	return 0;
}

static int ft5x46_leave_update(struct ft5x06_ts_data *data)
{
	/* reset */
	u8 w_buf[FT_MAX_WR_BUF] = {0};
	int ret;

	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(data->client, w_buf, 1);
	msleep(data->pdata->soft_rst_dly);
	ret = hidi2c_to_stdi2c(data->client);
	if (ret == 0) {
		pr_buf_err("%s: [FTS] hid change to i2c fail ! \n", __func__);
	}
	pr_buf_info("Ft5x46 leave update!\n");
	return 0;
}

static int ft5x46_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i = 0, j = 0, temp = 0;
	u32 pkt_num = 0, pkt_len = 0;
	u8 fw_ecc = 0;
	if (ft5x46_enter_update(ts_data) != 0) {
		pr_buf_info("enter update mode failed and Reset the CTP.(%s,%d)\n", __FUNCTION__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(1350);

	for (i = 0; i < 15; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = 0x00;
		r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

		if (0xF0 == r_buf[0] && 0xAA == r_buf[1])
			break;
		else
			pr_buf_err("%d != 0xF0,%d != 0xAA\n", r_buf[0], r_buf[1]);
		mdelay(50);
	}

	w_buf[0] = 0xB0;
	w_buf[1] = (u8)((data_len >> 16) & 0xFF);
	w_buf[2] = (u8)((data_len >> 8) & 0xFF);
	w_buf[3] = (u8)(data_len & 0xFF);
	ft5x06_i2c_write(client, w_buf, 4);
	/*write firmware*/
	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		/* msleep(FT_FW_PKT_DLY_MS); */

		for (j = 0; j < 30; j++) {
			w_buf[0] = 0x6a;
			r_buf[0] = 0x00;
			r_buf[0] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
			if ((i + 0x1000) == (((r_buf[0]) << 8) | r_buf[1]))
				break;
			mdelay(5);
		}
	}

	/*send the remaining bytes*/
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);

		/* msleep(FT_FW_PKT_DLY_MS); */
		for (i = 0; i < 30; i++) {
			w_buf[0] = 0x6a;
			r_buf[0] = 0x00;
			r_buf[0] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
			if ((j + 0x1000) == (((r_buf[0]) << 8) | r_buf[1]))
				break;
			mdelay(5);
		}
	}

	/*read out checksum*/
	w_buf[0] = 0x64;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(100);

	temp = 0;
	w_buf[0] = 0x65;
	w_buf[1] = (u8)(temp >> 16);
	w_buf[2] = (u8)(temp >> 8);
	w_buf[3] = (u8)(temp);
	temp = data_len;
	w_buf[4] = (u8)(temp >> 8);
	w_buf[5] = (u8)(temp);
	ft5x06_i2c_write(client, w_buf, 6);
	msleep(200);
	for (i = 0; i < 100; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = 0x00;
		r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
		if (0xF0 == r_buf[0] && 0x55 == r_buf[1])
			break;
		mdelay(5);
	}

	w_buf[0] = 0x66;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] != fw_ecc) {
			pr_buf_err("ECC error! dev_ecc=%02x fw_ecc=%02x\n",
						r_buf[0], fw_ecc);
			return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	pr_buf_info("%s: Firmware upgrade finish.\n", __func__);

	return 0;

}
static int ft5452_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FTS_FLASH_PACKET_LENGTH + FT_FW_PKT_META_LEN];
	int i = 0, j = 0, temp = 0;
	u32 pkt_num = 0, pkt_len = 0;
	u8 fw_ecc = 0;
	u8 auc_i2c_write_buf[10];
	u32 remainder = 0;
	if (ft5x46_enter_update(ts_data) != 0) {
		dev_err(&client->dev, "enter update mode failed and Reset the CTP.(%s,%d)\n", __FUNCTION__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}

	/*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
	auc_i2c_write_buf[0] = 0x09;
	auc_i2c_write_buf[1] = 0x0B;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 2);

	w_buf[0] = 0xB0;
	w_buf[1] = (u8)((data_len >> 16) & 0xFF);
	w_buf[2] = (u8)((data_len >> 8) & 0xFF);
	w_buf[3] = (u8)(data_len & 0xFF);
	ft5x06_i2c_write(client, w_buf, 4);

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(1350);

	for (i = 0; i < 50; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = 0x00;
		r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);

		if (0xF0 == r_buf[0] && 0xAA == r_buf[1])
			break;
		else
			dev_err(&client->dev, "ft5x06 %s: %d != 0xF0,%d != 0xAA\n", __func__, r_buf[0], r_buf[1]);
		mdelay(400);
	}

	/*write firmware*/
	pkt_num = (data_len) / FTS_FLASH_PACKET_LENGTH;
	remainder = data_len % FTS_FLASH_PACKET_LENGTH;
	if (remainder > 0)
		pkt_num++;
	pkt_len = FTS_FLASH_PACKET_LENGTH;
	pkt_buf[0] = FT_FW_START_REG;

	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FTS_FLASH_PACKET_LENGTH;
		pkt_buf[1] = (u8) (temp >> FT_16BIT_SHIFT);
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;

		/* last packet */
		if ((i == (pkt_num - 1)) && remainder)
			pkt_len = remainder;

		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < pkt_len; j++) {
			pkt_buf[6 + j] = data[i * FTS_FLASH_PACKET_LENGTH + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				pkt_len + FT_FW_PKT_META_LEN);
		msleep(5);

		for (j = 0; j < 100; j++) {
			w_buf[0] = 0x6a;
			r_buf[0] = 0x00;
			r_buf[0] = 0x00;
			ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
			if ((i + 0x1000) == (((r_buf[0]) << 8) | r_buf[1]))
				break;
			mdelay(5);
		}
	}

	/*read out checksum*/
	w_buf[0] = 0x64;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(100);

	temp = 0;
	w_buf[0] = 0x65;
	w_buf[1] = (u8)(temp >> 16);
	w_buf[2] = (u8)(temp >> 8);
	w_buf[3] = (u8)(temp);
	temp = data_len;
	w_buf[4] = (u8)(temp >> 16);
	w_buf[5] = (u8)(temp >> 8);
	w_buf[6] = (u8)(temp);
	ft5x06_i2c_write(client, w_buf, 7);
	msleep(200);
	for (i = 0; i < 10; i++) {
		w_buf[0] = 0x6a;
		r_buf[0] = 0x00;
		r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 1, r_buf, 2);
		if (0xF0 == r_buf[0] && 0x55 == r_buf[1])
			break;
		mdelay(50);
	}

	w_buf[0] = 0x66;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] != fw_ecc) {
			dev_err(&client->dev, "%s: ECC error! dev_ecc=%02x fw_ecc=%02x\n",
						__func__, r_buf[0], fw_ecc);
			return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(280);

	dev_err(&client->dev, "%s: Firmware upgrade finish.\n", __func__);

	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
static u16 fts_pram_ecc_calc_host(u8 *pbuf, u16 length)
{
	u16 ecc = 0;
	u16 i = 0;
	u16 j = 0;

	for ( i = 0; i < length; i += 2 ) {
		ecc ^= ((pbuf[i] << 8) | (pbuf[i + 1]));
		for (j = 0; j < 16; j ++) {
			if (ecc & 0x01)
				ecc = (u16)((ecc >> 1) ^ ((1 << 15) + (1 << 10) + (1 << 3)));
			else
				ecc >>= 1;
		}
	}

	return ecc;
}

static int fts_pram_ecc_cal_algo(
	struct i2c_client *client,
	u32 start_addr,
	u32 ecc_length)
{
	int ret = 0;
	int i = 0;
	int ecc = 0;
	u8 val[2] = { 0 };
	u8 cmd[7] = { 0 };

	pr_buf_info("read out pramboot checksum");
	cmd[0] = 0xCC;
	cmd[1] = BYTE_OFF_16(start_addr);
	cmd[2] = BYTE_OFF_8(start_addr);
	cmd[3] = BYTE_OFF_0(start_addr);
	cmd[4] = BYTE_OFF_16(ecc_length);
	cmd[5] = BYTE_OFF_8(ecc_length);
	cmd[6] = BYTE_OFF_0(ecc_length);
	ret = ft5x06_i2c_write(client, cmd, 7);
	if (ret < 0) {
		pr_buf_err("write pramboot ecc cal cmd fail!\n");
	}

	cmd[0] = 0xCE;
	for (i = 0; i < 100; i++) {
		msleep(1);
		ret = ft5x06_i2c_read(client, cmd, 1, val, 1);
		if (ret < 0) {
			pr_buf_err("ecc_finish read cmd fail!\n");
			return ret;
		}
		if (0 == val[0])
			break;
	}
	if (i >= 100) {
		pr_buf_err("wait ecc finish fail!\n");
		return -EIO;
	}

	cmd[0] = 0xCD;
	ret = ft5x06_i2c_read(client, cmd, 1, val, 2);
	if (ret < 0) {
		pr_buf_err("read pramboot ecc fail");
		return ret;
	}

	ecc = ((u16)(val[0] << 8) + val[1]) & 0x0000FFFF;
	return ecc;
}
#endif

static int fts_pram_init(struct i2c_client *client)
{
	int ret = 0;
	u8 reg_val = 0;
	u8 wbuf[3] = { 0 };

	pr_buf_info("pramboot initialization!\n");

	/* read flash ID */
	wbuf[0] = 0x05;
	ret = ft5x06_i2c_read(client, wbuf, 1, &reg_val, 1);
	if (ret < 0) {
		pr_buf_info("read flash type fail!\n");
		return ret;
	}

	/* set flash clk */
	wbuf[0] = 0x05;
	wbuf[1] = reg_val;
	wbuf[2] = 0x00;
	ret = ft5x06_i2c_write(client, wbuf, 3);
	if (ret < 0) {
		pr_buf_err("write flash type fail!\n");
		return ret;
	}

	return 0;
}

static bool fts_fwupg_check_flash_status(
	struct i2c_client *client,
	u16 flash_status,
	int retries,
	int retries_delay)
{
	int ret = 0;
	int i = 0;
	u8 cmd = 0;
	u8 val[2] = { 0 };
	u16 read_status = 0;

	for (i = 0; i < retries; i++) {
		cmd = 0x6A;
		ret = ft5x06_i2c_read(client, &cmd , 1, val, 2);
		read_status = (((u16)val[0]) << 8) + val[1];
		if (flash_status == read_status) {
			return true;
		}
		msleep(retries_delay);
	}

	return false;
}

static int ft8719_fw_upgrade_start(struct i2c_client * client,const u8* pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0, j = 0;
	u32 packet_number = 0;
	u32 temp, lenght;
	u8 packet_buf[FT_FW_PKT_LEN + 6];
	u8 auc_i2c_write_buf[10];
	u8 upgrade_ecc;
	int pram_ecc = 0;
	u32 remainder = 0;
	int ret = 0;

	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	if (ft8719_enter_update(ts_data) != 0) {
		pr_buf_info("enter update mode failed and Reset the CTP.(%s,%d)\n", __func__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}
	/* write pramboot to pram */
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
	ts_data->pramboot = pb_file_ft8719;
	ts_data->pb_length = sizeof(pb_file_ft8719);
	packet_number = ts_data->pb_length / FT_FW_PKT_LEN;
	remainder = ts_data->pb_length % FT_FW_PKT_LEN;
#endif

	if (remainder > 0)
		packet_number++;
	lenght = FT_FW_PKT_LEN;

	packet_buf[0] = 0xAE;
	for (i = 0; i < packet_number; i++) {
		temp = i * FT_FW_PKT_LEN;
		packet_buf[1] = BYTE_OFF_16(temp);
		packet_buf[2] = BYTE_OFF_8(temp);
		packet_buf[3] = BYTE_OFF_0(temp);

		/* last packet */
		if ((i == (packet_number - 1)) && remainder)
		lenght = remainder;

		packet_buf[4] = BYTE_OFF_8(lenght);
		packet_buf[5] = BYTE_OFF_0(lenght);
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
		for (j = 0; j < lenght; j++) {
			packet_buf[6 + j] = ts_data->pramboot[temp + j];
		}
#endif
		ret = ft5x06_i2c_write(client, packet_buf, lenght + 6);
		if (ret < 0) {
			pr_buf_err("pramboot write data(%d) fail!\n", i);
			return ret;
		}
	}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
	pram_ecc = (int)fts_pram_ecc_calc_host(ts_data->pramboot,
		ts_data->pb_length);
#endif
	if (pram_ecc < 0) {
		pr_buf_err( "write pramboot fail!\n");
	}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
	ret = fts_pram_ecc_cal_algo(client, 0, ts_data->pb_length);
#endif
	if (ret < 0) {
		pr_buf_err( "read pramboot ecc fail!\n");
	}
	pr_buf_info("pram ecc in tp:%x, host:%x\n", ret, pram_ecc);
	/*  pramboot checksum != fw checksum, upgrade fail */
	if (ret != pram_ecc) {
		pr_buf_err("pramboot ecc check fail!\n");
		//return -EIO;
	}

	pr_buf_info("remap to start pramboot!\n");
	auc_i2c_write_buf[0] = 0x08;
	ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	if (ret < 0) {
		pr_buf_err("write start pram cmd fail!\n");
		return ret;
	}
	mdelay(10);

	ts_data->pdata->info.upgrade_id_1 = 0x87;
	ts_data->pdata->info.upgrade_id_2 = 0xA9;
	if (ft8719_enter_update(ts_data) != 0) {
		pr_buf_err("enter pramboot mode failed and Reset the CTP.(%s,%d)\n",
			__func__, __LINE__);
		return -EIO;
	}
	ts_data->pdata->info.upgrade_id_1 = 0x87;
	ts_data->pdata->info.upgrade_id_2 = 0x19;
	ret = fts_pram_init(client);
	if (ret < 0) {
		pr_buf_err("pramboot init fail!\n");
		return ret;
	}

	/*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
	auc_i2c_write_buf[0] = 0x09;
	auc_i2c_write_buf[1] = 0x0B;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 2);

	mdelay(800);
	/*Step 4:erase app and panel paramenter area*/
	pr_buf_info("[UPGRADE]: erase app and panel paramenter area!!\n");
	auc_i2c_write_buf[0] = 0x61;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(ts_data->pdata->info.delay_erase_flash);

	for (i = 0; i < 15; i++) {
		auc_i2c_write_buf[0] = 0x6A;
		reg_val[0] = reg_val[1] = 0x00;
		ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if (0xF0==reg_val[0] && 0xAA==reg_val[1])
			break;
		mdelay(50);
	}
	pr_buf_info("[UPGRADE]: erase app area reg_val[0] = %x reg_val[1] = %x!!\n", reg_val[0], reg_val[1]);


	/*write FW to ctpm flash*/
	upgrade_ecc = 0;
	pr_buf_info("[UPGRADE]: write FW to ctpm flash!!\n");
	temp = 0;
	packet_number = (dw_lenth) / FT_FW_PKT_LEN;
	remainder = (dw_lenth) % FT_FW_PKT_LEN;
	if (remainder > 0)
		packet_number++;
	lenght = FT_FW_PKT_LEN;

	packet_buf[0] = 0xBF;
	for (j = 0; j < packet_number; j++) {
		temp = 0x2000 + j * FT_FW_PKT_LEN;
		packet_buf[1] = BYTE_OFF_16(temp);
		packet_buf[2] = BYTE_OFF_8(temp);
		packet_buf[3] = BYTE_OFF_0(temp);

		/* last packet */
		if ((j == (packet_number - 1)) && remainder)
			lenght = remainder;

		packet_buf[4] = BYTE_OFF_8(lenght);
		packet_buf[5] = BYTE_OFF_0(lenght);
		for (i = 0; i < FT_FW_PKT_LEN; i++) {
			packet_buf[6 + i] = pbt_buf[j * FT_FW_PKT_LEN + i];
			upgrade_ecc ^= packet_buf[6 + i];
		}
		ft5x06_i2c_write(client, packet_buf, FT_FW_PKT_LEN + 6);

		mdelay(1);
		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6A;
			reg_val[0] = reg_val[1] = 0x00;
			ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((0x1000 + (0x2000/FT_FW_PKT_LEN)) == (((reg_val[0]) << 8) | reg_val[1]))
				break;
		}
		mdelay(1);
	}

	msleep(50);
	/*Step 6: read out checksum*/
	/*send the opration head */
	pr_buf_info("[UPGRADE]: read out checksum!!\n");
	auc_i2c_write_buf[0] = 0x64;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	packet_number = dw_lenth / 0xFFFE;
	remainder = dw_lenth % 0xFFFE;
	if (remainder)
		packet_number++;
	lenght = 0xFFFE;

	auc_i2c_write_buf[0] = 0x65;
	for (i = 0; i < packet_number; i++) {
		temp = 0xFFFE * i + 0x2000;
		auc_i2c_write_buf[1] = (u8)(temp >> 16);
		auc_i2c_write_buf[2] = (u8)(temp >> 8);
		auc_i2c_write_buf[3] = (u8)(temp);
		if ((i == (packet_number - 1)) && remainder)
			lenght = remainder;
		auc_i2c_write_buf[4] = (u8)(lenght >> 8);
		auc_i2c_write_buf[5] = (u8)(lenght);
		ft5x06_i2c_write(client, auc_i2c_write_buf, 6);
		msleep(lenght / 256);
	}

	/* read status if check sum is finished */
	ret = fts_fwupg_check_flash_status(client, 0xF055, 10, 50);
	if (ret < 0) {
		pr_buf_err("ecc flash status read fail\n");
		return ret;
	}

	auc_i2c_write_buf[0] = 0x66;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != upgrade_ecc) {
		pr_buf_err("[UPGRADE]: ecc error! FW=%02x upgrade_ecc=%02x!!\n", reg_val[0], upgrade_ecc);
		//return -EIO;
	}
	pr_buf_info("[UPGRADE]: checksum %x %x!!\n", reg_val[0], upgrade_ecc);

	pr_buf_info("[UPGRADE]: reset the new FW!!\n");
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);
	//hidi2c_to_stdi2c(client);

	pr_buf_info("%s: Firmware upgrade finish.\n", __func__);
	return 0;
}

static int ft8006p_fw_upgrade_start(struct i2c_client * client,const u8* pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0, j = 0;
	u32 packet_number = 0;
	u32 temp, lenght;
	u8 packet_buf[FTS_FLASH_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 upgrade_ecc;
	int pram_ecc = 0;
	u32 remainder = 0;
	int ret = 0;

	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	if (ft8719_enter_update(ts_data) != 0) {
		pr_buf_info("enter update mode failed and Reset the CTP.(%s,%d)\n", __func__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}
	/* write pramboot to pram */
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
	ts_data->pramboot = pb_file_ft8006p;
	ts_data->pb_length = sizeof(pb_file_ft8006p);
	packet_number = ts_data->pb_length / FTS_FLASH_PACKET_LENGTH;
	remainder = ts_data->pb_length % FTS_FLASH_PACKET_LENGTH;
#endif

	if (remainder > 0)
		packet_number++;
	lenght = FTS_FLASH_PACKET_LENGTH;

	packet_buf[0] = 0xAE;
	for (i = 0; i < packet_number; i++) {
		temp = i * FTS_FLASH_PACKET_LENGTH;
		packet_buf[1] = BYTE_OFF_16(temp);
		packet_buf[2] = BYTE_OFF_8(temp);
		packet_buf[3] = BYTE_OFF_0(temp);

		/* last packet */
		if ((i == (packet_number - 1)) && remainder)
		lenght = remainder;

		packet_buf[4] = BYTE_OFF_8(lenght);
		packet_buf[5] = BYTE_OFF_0(lenght);
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
		for (j = 0; j < lenght; j++) {
			packet_buf[6 + j] = ts_data->pramboot[temp + j]; // ECC_CHECK_MODE_XOR
		}
#endif
		ret = ft5x06_i2c_write(client, packet_buf, lenght + 6);
		if (ret < 0) {
			pr_buf_err("pramboot write data(%d) fail!\n", i);
			return ret;
		}
	}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
	pram_ecc = (int)fts_pram_ecc_calc_host(ts_data->pramboot,
		ts_data->pb_length);
#endif
	if (pram_ecc < 0) {
		pr_buf_err( "write pramboot fail!\n");
	}
#if defined(CONFIG_TOUCHSCREEN_FT5X06_PRAMUPDATE)
	ret = fts_pram_ecc_cal_algo(client, 0, ts_data->pb_length);
#endif
	if (ret < 0) {
		pr_buf_err( "read pramboot ecc fail!\n");
	}
	pr_buf_info("pram ecc in tp:%x, host:%x\n", ret, pram_ecc);
	/*  pramboot checksum != fw checksum, upgrade fail */
	if (ret != pram_ecc) {
		pr_buf_err("pramboot ecc check fail!\n");
		//return -EIO;
	}

	pr_buf_info("remap to start pramboot!\n");
	auc_i2c_write_buf[0] = 0x08;
	ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	if (ret < 0) {
		pr_buf_err("write start pram cmd fail!\n");
		return ret;
	}
	mdelay(10);

	ts_data->pdata->info.upgrade_id_1 = 0x86;
	ts_data->pdata->info.upgrade_id_2 = 0xA2; // download完pram后id2改变是正常的
	if (ft8719_enter_update(ts_data) != 0) {
		pr_buf_err("enter pramboot mode failed and Reset the CTP.(%s,%d)\n",
			__func__, __LINE__);
		return -EIO;
	}
	ts_data->pdata->info.upgrade_id_1 = 0x86; //
	ts_data->pdata->info.upgrade_id_2 = 0x22; // id2改回原始值
	ret = fts_pram_init(client);
	if (ret < 0) {
		pr_buf_err("pramboot init fail!\n");
		return ret;
	}

	/*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
	auc_i2c_write_buf[0] = 0x09;
	auc_i2c_write_buf[1] = 0x0B;
	ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 2);
	if (ret < 0) {
		pr_buf_err("upgrade mode(09) cmd write fail");
		//goto fw_reset;
	}

	// mdelay(800); // FT8006P del

	// FT8006P add:
	auc_i2c_write_buf[0] = 0x7A;
	auc_i2c_write_buf[1] = BYTE_OFF_16(dw_lenth);
	auc_i2c_write_buf[2] = BYTE_OFF_8(dw_lenth);
	auc_i2c_write_buf[3] = BYTE_OFF_0(dw_lenth);
	ret = ft5x06_i2c_write(client, auc_i2c_write_buf, 4);
	if (ret < 0) {
	    pr_buf_err("data len cmd write fail");
	    //goto fw_reset;
	}
	/*Step 4:erase app and panel paramenter area*/
	pr_buf_info("[UPGRADE]: erase app and panel paramenter area!!\n");
	auc_i2c_write_buf[0] = 0x61;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(ts_data->pdata->info.delay_erase_flash);

	for (i = 0; i < 15; i++) {
		auc_i2c_write_buf[0] = 0x6A;
		reg_val[0] = reg_val[1] = 0x00;
		ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if (0xF0==reg_val[0] && 0xAA==reg_val[1])
			break;
		mdelay(50);
	}
	pr_buf_info("[UPGRADE]: erase app area reg_val[0] = %x reg_val[1] = %x!!\n", reg_val[0], reg_val[1]);

	/*write FW to ctpm flash*/
	upgrade_ecc = 0;
	pr_buf_info("[UPGRADE]: write FW to ctpm flash!! dw_lenth = %d \n", dw_lenth);
	temp = 0;
	packet_number = (dw_lenth) / FTS_FLASH_PACKET_LENGTH;
	remainder = (dw_lenth) % FTS_FLASH_PACKET_LENGTH;
	if (remainder > 0)
		packet_number++;
	lenght = FTS_FLASH_PACKET_LENGTH;

	packet_buf[0] = 0xBF;
	for (j = 0; j < packet_number; j++) {
		temp = 0x2000 + j * FTS_FLASH_PACKET_LENGTH;
		packet_buf[1] = BYTE_OFF_16(temp);
		packet_buf[2] = BYTE_OFF_8(temp);
		packet_buf[3] = BYTE_OFF_0(temp);

		/* last packet */
		if ((j == (packet_number - 1)) && remainder)
			lenght = remainder;

		packet_buf[4] = BYTE_OFF_8(lenght);
		packet_buf[5] = BYTE_OFF_0(lenght);
		for (i = 0; i < lenght; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_FLASH_PACKET_LENGTH + i];
			upgrade_ecc ^= packet_buf[6 + i];
		}
		ft5x06_i2c_write(client, packet_buf, lenght + 6);

		mdelay(1);
		for (i = 0; i < 100; i++) {
			auc_i2c_write_buf[0] = 0x6A;
			reg_val[0] = reg_val[1] = 0x00;
			ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((0x1000 + (temp/lenght)) == (((reg_val[0]) << 8) | reg_val[1]))
				break;
		}
		mdelay(1);
	}

	// not ECC_CHECK_MODE_CRC16
	// upgrade_ecc = (int)fts_crc16_calc_host(pbt_buf, dw_lenth);
	msleep(50);
	/*Step 6: read out checksum*/
	/*send the opration head */
	pr_buf_info("[UPGRADE]: read out checksum!!\n");
	auc_i2c_write_buf[0] = 0x64;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	packet_number = dw_lenth / 0xFFFE;
	remainder = dw_lenth % 0xFFFE;
	if (remainder)
		packet_number++;
	lenght = 0xFFFE;

	auc_i2c_write_buf[0] = 0x65;
	for (i = 0; i < packet_number; i++) {
		temp = 0xFFFE * i + 0x2000;
		auc_i2c_write_buf[1] = (u8)(temp >> 16);
		auc_i2c_write_buf[2] = (u8)(temp >> 8);
		auc_i2c_write_buf[3] = (u8)(temp);
		if ((i == (packet_number - 1)) && remainder)
			lenght = remainder;
		auc_i2c_write_buf[4] = (u8)(lenght >> 8);
		auc_i2c_write_buf[5] = (u8)(lenght);
		ft5x06_i2c_write(client, auc_i2c_write_buf, 6);
		msleep(lenght / 256);
	}

	/* read status if check sum is finished */
	ret = fts_fwupg_check_flash_status(client, 0xF055, 10, 50);
	if (ret < 0) {
		pr_buf_err("ecc flash status read fail\n");
		return ret;
	}

	auc_i2c_write_buf[0] = 0x66;
	ft5x06_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);// not ECC_CHECK_MODE_CRC16
	if (reg_val[0] != upgrade_ecc) {
		pr_buf_err("[UPGRADE]: ecc error! FW=%02x upgrade_ecc=%02x!!\n", reg_val[0], upgrade_ecc);
		//return -EIO;
	}
	pr_buf_info("[UPGRADE]: checksum %x %x!!\n", reg_val[0], upgrade_ecc);

	pr_buf_info("[UPGRADE]: reset the new FW!!\n");
	auc_i2c_write_buf[0] = 0x07;
	ft5x06_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);
	//hidi2c_to_stdi2c(client);

	pr_buf_info("%s: Firmware upgrade finish.\n", __func__);
	return 0;
}

static int ft6x36_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	/* u8 reset_reg; */
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 fw_ecc;

	/*check fw first byte*/
	if (data[0] != 0x02) {
		pr_buf_err("FW first byte is not 0x02,not valid.");
		return -EIO;
	}
	/*check fw lenght*/
	if (data_len > 0x11f) {
		pkt_len = ((u32)data[0x100] << 8) + data[101];
		if (data_len < pkt_len) {
			pr_buf_err("FW lenght is invalid.");
			return -EIO;
		}
	} else {
		pr_buf_err("FW lenght is invalid.");
		return -EIO;
	}

	/*Enter update mode*/
	if (ft5x06_enter_update(ts_data) != 0) {
		pr_buf_info("enter update mode failed and Reset the CTP.(%s,%d)\n", __func__, __LINE__);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		mdelay(ts_data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		mdelay(ts_data->pdata->soft_rst_dly);

		return -EIO;
	}

	/*erase app and panel paramenter area*/
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	for (i = 0; i < 200; i++) {
		w_buf[0] = 0x6a;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;
		r_buf[0] = 0x00;
		r_buf[1] = 0x00;
		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
		if (0xb0 == r_buf[0] && 0x02 == r_buf[1]) {
			pr_buf_err("erase app finished.");
			break;
		}
		msleep(50);
	}

	/*write firmware to ctpm flash*/
	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);

		for (j = 0; j < 30; j++) {
			w_buf[0] = 0x6a;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			r_buf[0] = 0x00;
			r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
			if (0xb0 == (r_buf[0] & 0xf0) && (0x03 + (i % 0x0ffd))
				== (((r_buf[0] & 0x0f) << 8) | r_buf[1]))
				break;
			msleep(1);
		}
	}

	/*send the remaining bytes*/
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		for (j = 0; j < 30; j++) {
			w_buf[0] = 0x6a;
			w_buf[1] = 0x00;
			w_buf[2] = 0x00;
			w_buf[3] = 0x00;
			r_buf[0] = 0x00;
			r_buf[1] = 0x00;
			ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);
			if (0xb0 == (r_buf[0] & 0xf0) && (0x03 + (i % 0x0ffd))
				== (((r_buf[0] & 0x0f) << 8) | r_buf[1]))
				break;
			msleep(1);
		}
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		pr_buf_err("ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	pr_buf_info("%s: Firmware upgrade finish.\n", __func__);
	return 0;
}

static int ft_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	int ret;

	dev_err(&client->dev, "ft5x06 ft_fw_upgrade_start");
	dev_err(&client->dev, "family_id=%02x pannel_id=%02x\n", ts_data->family_id, ts_data->pannel_id);
	if (FT_FT5446_FAMILY_ID_0x54 == ts_data->family_id || FT_FT5822_FAMILY_ID_0x58 == ts_data->family_id){
		if (0x53 == ts_data->pannel_id || 0x54 == ts_data->pannel_id || 0x5C == ts_data->pannel_id  || 0x5E == ts_data->pannel_id )
			/*the fw_upgrade of 5446-P03 and 5446-Q03 is the same */
			ret = ft5452_fw_upgrade_start(client, data, data_len);
		else
			ret = ft5x46_fw_upgrade_start(client, data, data_len);
	}else if (FT_FT6436_FAMILY_ID_0x36 == ts_data->family_id)
		ret = ft6x36_fw_upgrade_start(client, data, data_len);
	else if (FT_FT8006_FAMILY_ID_0x80 == ts_data->family_id)
		ret = ft8006_fw_upgrade_start(client, data, data_len);
	else if (FT_FT8006P_FAMILY_ID_0x86 == ts_data->family_id)
		ret = ft8006p_fw_upgrade_start(client, data, data_len);
	else if (FT_FT8719_FAMILY_ID_0x87 == ts_data->family_id)
		ret = ft8719_fw_upgrade_start(client, data, data_len);
	else
		ret = ft5x06_fw_upgrade_start(client, data, data_len);

	return ret;
}

static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	struct upgrade_config config = {
		.enter_upgrade_mode = false,
		.need_upgrade = false,
		.firmware = NULL,
	};

	rc = ft5x06_get_fw_upgrade_config(data, &config);
	fw = config.firmware;

	if (rc < 0)
		goto rel_fw;

	if (!force && !config.need_upgrade) {
		pr_buf_info("Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	/* if (FT_FW_CHECK(fw))  { */
	if (1) {
		rc = ft_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0) {
			pr_buf_err("update failed (%d). try later...\n", rc);
			goto rel_fw;
		} else if (data->pdata->info.auto_cal) {
			ft5x06_auto_cal(data->client);
		}
	} else {
		pr_buf_err("FW format error\n");
		rc = -EIO;
	}

	ft5x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

	ft5x06_ts_register_productinfo(data);

rel_fw:
	if (config.enter_upgrade_mode == true) {
		if (FT_FT5446_FAMILY_ID_0x54 == data->family_id)
			ft5x46_leave_update(data);
		else
			ft5x06_leave_update(data);
	}
	release_firmware(fw);
	return rc;
}

static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/* struct ft5x06_ts_data *data = dev_get_drvdata(dev); */
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

#if 1
	if (val)
		ft5x06_triger_update(dev, false);
#else
	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);
#endif
	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/* struct ft5x06_ts_data *data = dev_get_drvdata(dev); */
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;
#if 1
	if (val)
		ft5x06_triger_update(dev, true);
#else
	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);
#endif
	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);

static ssize_t ft5x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_buf_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		pr_buf_err("invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		pr_buf_err("unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc = 0;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val = 0, num_buttons = 0;
	u32 button_map[MAX_BUTTONS];
	u8 read_flash_cmd[] = {0x3, 0x00, 0x07, 0xb4};
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	int i = 0;
	u32 gesture_map[MAX_GESTURE];
#ifndef CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER
	u32 gesture_reg_map[MAX_GESTURE_REG];
#endif
#endif

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		printk("Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
#if defined(CONFIG_TP_MATCH_HW)
	pdata->tp_match_hw_gpio = of_get_named_gpio_flags(np, "focaltech,tp-match-hw-gpio",
					0, &pdata->tp_match_hw_gpio_flags);
#endif

	pdata->fw_name_pref = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name_pref);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id3", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw upgrade id3\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_3 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id5", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw upgrade id5\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_5 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

	pdata->support_usb_check = of_property_read_bool(np,
							"focaltech,support-usb-check");
	pdata->need_poweroff = of_property_read_bool(np,
							"focaltech,need-poweroff");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			pr_buf_err("Unable to read key codes\n");
			return rc;
		}
	}

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	prop = of_find_property(np, "focaltech,gesture-func-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,gesture-func-map", gesture_map,
			num_buttons);
		if (rc) {
			pr_buf_err("Unable to read gesture func map\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_func_map[i] = gesture_map[i];
		pdata->gesture_num = num_buttons;
	}
	prop = of_find_property(np, "focaltech,gesture-figure-map", NULL);
	if (prop) {
		rc = of_property_read_u32_array(np,
			"focaltech,gesture-figure-map", gesture_map,
			num_buttons);
		if (rc) {
			pr_buf_err("Unable to read gesture figure map\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_figure_map[i] = gesture_map[i];
	}
#ifndef CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER
	prop = of_find_property(np, "focaltech,gesture-reg-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE_REG)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,gesture-reg-map", gesture_reg_map,
			num_buttons);
		if (rc) {
			pr_buf_err("Unable to read gesture reg map\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_reg_map[i] = gesture_reg_map[i];
		pdata->gesture_reg_num = num_buttons;
	}
	prop = of_find_property(np, "focaltech,gesture-reg-value-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE_REG)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,gesture-reg-value-map", gesture_reg_map,
			num_buttons);
		if (rc) {
			pr_buf_err("Unable to read gesture reg value map\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			pdata->gesture_reg_value_map[i] = gesture_reg_map[i];
	}
#endif
#endif

	prop = of_find_property(np, "focaltech,fw-rawdata-range", NULL);
	if (prop) {
		rc = of_property_read_u32_array(np,
			"focaltech,fw-rawdata-range", pdata->rawdata_range, ARRAY_SIZE(pdata->rawdata_range));
	}

	prop = of_find_property(np, "focaltech,fw-panelid-command", NULL);
	if (prop && prop->value)
		memcpy(pdata->panelid_command, prop->value, sizeof(pdata->panelid_command));
	else
		memcpy(pdata->panelid_command, read_flash_cmd, sizeof(pdata->panelid_command));

	prop = of_find_property(np, "focaltech,fw-panelid-command1", NULL);
	if (prop && prop->value)
		memcpy(pdata->panelid_command_1, prop->value, sizeof(pdata->panelid_command_1));
	else
		memcpy(pdata->panelid_command_1, read_flash_cmd, sizeof(pdata->panelid_command_1));

	rc = of_property_read_u32(np, "focaltech,touch-area-param", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pr_buf_err("Unable to read touch area param\n");
		return rc;
	} else if (rc) {
		pdata->touch_area_param = 1;
	} else {
		pdata->touch_area_param = temp_val;
	}

	rc = of_property_read_string(np, "focaltech,factory-info", &pdata->factory_info);
	if (rc < 0) {
		pr_buf_err("factory info read failed!\n");
		pdata->factory_info = "NULL";
	}
#ifdef CONFIG_FT_INCELL_CHIP
	pdata->keep_lcd_suspend_reset_high = of_property_read_bool(np,
						"focaltech,need-lcd-suspend-reset-keep-high");
#endif
	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static void ft5x06_ts_register_productinfo(struct ft5x06_ts_data *ts_data)
{
	/* format as flow: version:0x01 Module id:0x57 */
	char deviceinfo[64];
#if defined(CONFIG_TP_MATCH_HW)
	bool match;
	struct i2c_client *client = ts_data->client;
	struct device *dev = &client->dev;
	//char *buf = NULL;
	match = factory_tp_match_hw(dev);
	snprintf(deviceinfo, ARRAY_SIZE(deviceinfo), "FW version:0x%2x Module id:0x%2x MATCH:%d",
			ts_data->fw_ver[0], ts_data->pannel_id, match);
#else /* CONFIG_TP_MATCH_HW */
	snprintf(deviceinfo, ARRAY_SIZE(deviceinfo), "FW version:0x%2x Module id:0x%2x",
			ts_data->fw_ver[0], ts_data->pannel_id);
#endif /* CONFIG_TP_MATCH_HW */
#if defined(CONFIG_TOUCHSCREEN_FT5X06_SUB)
	productinfo_register(PRODUCTINFO_SUB_CTP_ID, NULL, deviceinfo);
#else
	productinfo_register(PRODUCTINFO_CTP_ID, NULL, deviceinfo);
#endif
}
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
static void psensor_report_dist(struct ft5x06_ts_data *data, int dist_code)
{
	input_report_abs(data->ps_input_dev, ABS_DISTANCE, dist_code ? 1000 : 1);
	input_report_abs(data->ps_input_dev, ABS_DISTANCE, dist_code ? 1023 : 0);
	input_sync(data->ps_input_dev);
	if (dist_code == FAR_CODE) {
		if (!wake_lock_active(&data->ps_wake_lock)) {
			pr_buf_err("wake_lock PROX_NEAR_TO_FAR_WLOCK not be locked, and lock it!\n");
			wake_lock_timeout(&data->ps_wake_lock, HZ);
		} else {
			pr_buf_err("wake_lock PROX_NEAR_TO_FAR_WLOCK be locked, do nothing!\n");
		}
	}
}

static int tp_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ft5x06_ts_data *data =
		container_of(sensors_cdev, struct ft5x06_ts_data, ps_cdev);
	int ret = 0;
	if ((enable < 0) || (enable > 1)) {
		pr_buf_err("%s(): It is an illegal para %d!\n", __func__, enable);
		return -EINVAL;
	}

	if (data->ps_en == enable)
		return 0;

	if (data->suspended) {
		/* send in resume */
		data->tp_ps_en = 1;
		data->ps_en = enable;
		/*  permit the last val diff with the current ps state. */
		last_ps_val = 0;
		pr_buf_err("%s: tp is in suspend, ps_en = %d, resend in resume.\n", __func__, enable);
		return 0;
	}
	pr_buf_err("%s: ps_en = %d\n", __func__, enable);

	ret = ft5x0x_write_reg(data->client, 0XB0, enable);
	if (ret < 0) {
		pr_buf_err(" enable tp ps function failed\n");
		enable = 0;
		/* Add set ps func off fail process */
		goto quit;
	}
/*	ret = enable ?
		enable_irq_wake(data->client->irq) :
		disable_irq_wake(data->client->irq);*/
	if (0 != ret) {
		pr_buf_err(" enable_irq_wake failed for tp_ps_irq_handler\n");
		ft5x0x_write_reg(data->client, 0XB0, 0);
		enable = 0;
	}
quit:
	data->ps_en = enable;
	last_ps_val = enable ? 0x07 : 0;
	/* Force report a FAR value when excute this function. */
	psensor_report_dist(data, FAR_CODE);
	return 0;
}

static int tp_ps_input_init(struct ft5x06_ts_data *data)
{
	int ret = 0;

	data->ps_input_dev = input_allocate_device();
	if (!data->ps_input_dev) {
		pr_buf_err("[PS][FocalTeck TP error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	data->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, data->ps_input_dev->evbit);
	input_set_abs_params(data->ps_input_dev, ABS_DISTANCE, 0, 1023, 0, 0);
	input_set_drvdata(data->ps_input_dev, data);

	ret = input_register_device(data->ps_input_dev);
	if (ret < 0) {
		pr_buf_err("[PS][CM36686 error]%s: could not register ps input device\n", __func__);
		goto err_free_ps_input_device;
	}
	return ret;

err_free_ps_input_device:
	input_free_device(data->ps_input_dev);
	return ret;
}

int powerkey_close_face_detect(void)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(s_ctp_dev);
	if (data->ps_en)	{
		pr_buf_err("%s lcd state:%d.\n", __func__, data->ps_lcd_state);
		if (data->ps_lcd_state == FB_BLANK_POWERDOWN && data->tp_ps_en == 0) {
			input_report_key(data->input_dev, KEY_POWER, 1);
			input_report_key(data->input_dev, KEY_POWER, 0);
			input_sync(data->input_dev);
			pr_buf_err("%s: powerkey send FAR state.\n", __func__);
			psensor_report_dist(data, FAR_CODE);
		}
	}
	return 0;
}
EXPORT_SYMBOL(powerkey_close_face_detect);
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06_GET_PANNEL_ID
static u8 pannel_id_value = 0;
u8 fts_get_panel_id(void)
{
	return pannel_id_value;
}
EXPORT_SYMBOL(fts_get_panel_id);
#endif

static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
#if defined(CONFIG_DEBUG_FS)
	struct dentry *temp;
#endif
	u8 reg_value;
	u8 reg_addr;
	int err, len, i,j;
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	int index;
#endif

	pr_buf_info("%s enter!\n", __func__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			pr_buf_err("Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			pr_buf_err("DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		pr_buf_err("Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_buf_err("I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!data) {
		pr_buf_err("Not enough memory\n");
		return -ENOMEM;
	}

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			pr_buf_err("Invalid firmware name\n");
			return -EINVAL;
		}
		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);

	if (!data) {
		pr_buf_err("Not enough memory\n");
		return -ENOMEM;
	}

	data->event[0] = ft5x06_allocate_touch_event_seq(&client->dev, pdata->num_max_touches);
	data->event[1] = ft5x06_allocate_touch_event_seq(&client->dev, pdata->num_max_touches);
	if (!data->event[0] || !data->event[1]) {
		pr_buf_err("Not enough memory\n");
		return -ENOMEM;
	}

	data->client = client;
	data->pdata = pdata;
	data->input_dev = NULL;
	i2c_set_clientdata(client, data);

	err = (pdata->power_init != NULL) ?
		pdata->power_init(true) :
		ft5x06_power_init(data, true);
	if (err) {
		pr_buf_err("power init failed");
		return err;
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			pr_buf_err("power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			pr_buf_err("power on failed");
			goto pwr_deinit;
		}
	}

	err = ft5x06_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(data, true);
		if (err < 0)
			goto pwr_off;
	}
#if defined(CONFIG_TP_MATCH_HW)
	if (gpio_is_valid(pdata->tp_match_hw_gpio)) {
		err = gpio_request(pdata->tp_match_hw_gpio, "ft5x06_tp_match_hw_gpio");
		if (err) {
			pr_buf_err("tp_match_hw gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->tp_match_hw_gpio);
		if (err) {
			pr_buf_err("set_direction for tp_match_hw gpio failed\n");
			goto free_tp_match_hw_gpio;
		}
	}
#endif
	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft5x06_irq_gpio");
		if (err) {
			pr_buf_err("irq gpio request failed");
#if defined(CONFIG_TP_MATCH_HW)
			goto free_tp_match_hw_gpio;
#else
			goto pwr_off;
#endif
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			pr_buf_err("set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft5x06_reset_gpio");
		if (err) {
			pr_buf_err("reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			pr_buf_err("set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_value = 0;
	reg_addr = FT_REG_ID;
	for (i = 0; i < 5; i++) {
		err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
		if (err >= 0) {
			pr_buf_err("version read success, time:%d, Device ID = 0x%02x\n", i, reg_value);
			break;
		}
		if (gpio_is_valid(data->pdata->reset_gpio))
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		ft5x06_power_on(data, false);
		msleep(10);
		ft5x06_power_on(data, true);
		msleep(5);
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}
		msleep(data->pdata->soft_rst_dly);
	}
	if (i >= 5) {
		pr_buf_err("version read failed");
		goto free_reset_gpio;
	}
	/*if the IC is ft5446_p03,the device_id should be 0x52;if the IC is ft5446DQS,the device_id should be 0x22*/
	reg_addr = 0x9F;
	for (j = 0; j < 5; j++) {
		err = ft5x06_i2c_read(client, &reg_addr, 1, &data->pdata->fw_device_id, 1);
		if (err >= 0) {
			pr_buf_err("version read success, time:%d, Device ID = 0x%02x\n", i, data->pdata->fw_device_id);
			break;
		}
		if (gpio_is_valid(data->pdata->reset_gpio))
			gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		ft5x06_power_on(data, false);
		msleep(10);
		ft5x06_power_on(data, true);
		msleep(5);
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}
		msleep(data->pdata->soft_rst_dly);
	}
	if (j >= 5) {
		pr_buf_err("version read failed");
		goto free_reset_gpio;
	}
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_buf_err("failed to allocate input device\n");
		return -ENOMEM;
	}
	data->input_dev = input_dev;
	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(FT_IRQAWAKE_KEY, input_dev->keybit);

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	for (index = 0; index < data->pdata->gesture_num; index++)
		__set_bit(data->pdata->gesture_func_map[index], input_dev->keybit);
#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE_DRIVER)
	init_para(data->pdata->x_max, data->pdata->y_max, 100, 0, 0);
#endif
	data->gesture_state = 0x00;
#endif

	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    /* input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0); */
	err = input_register_device(input_dev);
	if (err) {
		pr_buf_err("Input device registration failed\n");
		goto free_inputdev;
	}
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
	__set_bit(KEY_POWER, input_dev->keybit);
	err = tp_ps_input_init(data);
	if (err) {
		pr_buf_err("PS function Input device registration failed\n");
		goto unreg_inputdev;
	}

	data->ps_cdev = tp_proximity_cdev;
	data->ps_cdev.sensors_enable = tp_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL,

	err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if (err) {
		pr_buf_err("%s: Unable to register to sensors class: %d\n",
				__func__, err);
		goto unreg_ps_inputdev;
	}

	wake_lock_init(&(data->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity_tp");
#endif

	/* if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) { */
	if (pdata->family_id != reg_value) {
		pr_buf_err("%s:Unsupported controller 0x%02x\n", __func__, reg_value);
		/* goto free_reset_gpio; */
	}
	data->family_id = pdata->family_id;
	pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	err = request_threaded_irq(client->irq, NULL,
				ft5x06_ts_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				client->dev.driver->name, data);
	if (err) {
		pr_buf_err("request irq failed\n");
		goto free_reset_gpio;
	}

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		pr_buf_err("sys file creation failed\n");
		goto irq_free;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		pr_buf_err("sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		pr_buf_err("sys file creation failed\n");
		goto free_update_fw_sys;
	}
#if defined(CONFIG_DEBUG_FS)
	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_buf_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_buf_err("debugfs_create_file addr failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_buf_err("debugfs_create_file data failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_buf_err("debugfs_create_file suspend failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_buf_err("debugfs_create_file dump_info failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}
#endif

	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		pr_buf_err("Not enough memory\n");
#if defined(CONFIG_DEBUG_FS)
		goto free_debug_dir;
#else
		goto free_force_update_fw_sys;
#endif
	}

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		pr_buf_err("report rate read failed");

	pr_buf_info("report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		pr_buf_err("threshold read failed");

	pr_buf_info("touch threshold = %d\n", reg_value * 4);

	ft5x06_update_fw_ver(data);

	reg_addr = FT_RGE_PANNEL_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		pr_buf_err("Pannel Id version read failed");
	pr_buf_info("Pannel Id version = 0x%x\n", reg_value);
	if(data->pdata->fw_device_id == 0x52){
		/*The device id of FT5446-P03 is 0x52, the upgrade_id_2 is 0x5C*/
		if(reg_value == 0xAA){
			pr_buf_info("it is the third TP IC FT5446_P03, TXD module & BOE 0x53.");
			data->pannel_id = 0x53;
			data->pdata->info.upgrade_id_3 = 0x5C;

		}else{
			pr_buf_info("it is the forth TP IC FT5446_P03, GuoXian module &chipone 0x54.");
			data->pannel_id = 0x54;
			data->pdata->info.upgrade_id_3 = 0x5C;
		}
	}else if(data->pdata->fw_device_id == 0x22){
		/*The device id of FT5446-Q03 is 0x22, the upgrade_id_2 is 0x2E*/
		if(reg_value == 0x5C || 0x5E == reg_value){
			pr_buf_info("it is the fifth TP IC FT5446_P03, TXD module & BOE 0x55.");
			data->pannel_id = reg_value;
			data->pdata->info.upgrade_id_5 = 0x2E;
#ifdef CONFIG_TOUCHSCREEN_FT5X06_GET_PANNEL_ID
			pannel_id_value = reg_value;
#endif
		}else{
			pr_buf_info("it is the first TP IC FT5446_DQS, GuoXian module & BOE 0x51.");
			data->pannel_id =  reg_value;
		}
	}

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

	factory_ts_func_test_register(data);

	ft5x06_ts_register_productinfo(data);
#ifndef CONFIG_TOUCHSCREEN_FT5X06_SUB
#if defined(CONFIG_FB)

	INIT_WORK(&data->resume_work, ft5x06_ts_resume_work);

	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		pr_buf_err("Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
#endif
	ctp_register_device(&data->client->dev);
	G_Client = data->client;

#ifdef CONFIG_TOUCHSCREEN_FT5X06_APK_TEST
	err = fts_create_apk_debug_channel(client);
	if (err)
		pr_buf_err("Unable to create apk debug node: %d\n", err);

	err = fts_rw_iic_drv_init(client);
	if (err)
		pr_buf_err("%s:[FTS] create fts control iic driver failed\n", __func__);
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X06_GESTURE)
	data->gesture_en = false;
#endif
	enable_irq_wake(data->client->irq);
	data->resume_is_running = true;
	pr_buf_info("%s success!!\n", __func__);

	return 0;

#if defined(CONFIG_DEBUG_FS)
free_debug_dir:
	debugfs_remove_recursive(data->dir);
#endif
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
irq_free:
	free_irq(client->irq, data);
#if defined(CONFIG_TOUCHSCREEN_FACE_DETECTION)
unreg_ps_inputdev:
	sensors_classdev_unregister(&data->ps_cdev);
	input_unregister_device(data->ps_input_dev);
#endif
free_inputdev:
	input_free_device(input_dev);
	input_unregister_device(input_dev);
	input_dev = NULL;
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (data->ts_pinctrl)
		if (ft5x06_ts_pinctrl_select(data, false) < 0)
			pr_buf_err("Cannot get idle pinctrl state\n");
	/* pinctrl_disable_state(data->ts_pinctrl); */
#if defined(CONFIG_TP_MATCH_HW)
free_tp_match_hw_gpio:
	if (gpio_is_valid(pdata->tp_match_hw_gpio))
		gpio_free(pdata->tp_match_hw_gpio);
#endif
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
	pr_buf_err("%s failed!!!\n", __func__);
	return err;
}

static int ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	int rc;

#ifdef CONFIG_TOUCHSCREEN_FT5X06_APK_TEST
	fts_rw_iic_drv_exit();
	fts_release_apk_debug_channel();
#endif

	ctp_unregister_device(&data->client->dev);

#if defined(CONFIG_DEBUG_FS)
	debugfs_remove_recursive(data->dir);
#endif
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		pr_buf_err("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->ts_pinctrl) {
		rc = ft5x06_ts_pinctrl_select(data, false);
		if (rc < 0)
			pr_buf_err("Cannot get idle pinctrl state\n");
	}
	/* pinctrl_disable_state(data->ts_pinctrl); */

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	.driver = {
		   .name = "ft5x06_ts",
		   .owner = THIS_MODULE,
		.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x06_ts_pm_ops,
#endif
		   },
	.id_table = ft5x06_ts_id,
};

static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
