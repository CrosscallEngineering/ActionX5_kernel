/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd., all rights reserved.
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
/*****************************************************************************
 *
 * File Name: focaltech_core.c
 *
 * Author: Focaltech Driver Team
 *
 * Created: 2016-08-08
 *
 * Abstract: entrance for focaltech ts driver
 *
 * Version: V1.0
 *
 *****************************************************************************/

/*****************************************************************************
 * Included header files
 *****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1	/* Early-suspend level */
#endif
#include "focaltech_core.h"

/*****************************************************************************
 * Private constant and macro definitions using #define
 *****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define INTERVAL_READ_REG                   200	/* unit:ms */
#define TIMEOUT_READ_REG                    1000	/* unit:ms */
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV                      2800000
#define FTS_VTG_MAX_UV                      3300000
#define FTS_I2C_VTG_MIN_UV                  1800000
#define FTS_I2C_VTG_MAX_UV                  1800000
#endif

//#define attrify(propname) (&dev_attr_##propname.attr)
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_FH)
extern int usb_flag;
#endif

/*****************************************************************************
 * Global variable or extern global variabls/functions
 *****************************************************************************/
struct fts_ts_data *fts_data;
static bool fts_probed = false;

/*****************************************************************************
 * Static function prototypes
 *****************************************************************************/
static int fts_ts_suspend(struct device *dev);
int fts_ts_resume(struct device *dev);

/*****************************************************************************
 *  Name: fts_wait_tp_to_valid
 *  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
 *         need call when reset/power on/resume...
 *  Input:
 *  Output:
 *  Return: return 0 if tp valid, otherwise return error code
 *****************************************************************************/
int fts_wait_tp_to_valid(void)
{
	int ret = 0;
	int cnt = 0;
	u8 idh = 0;
	u8 idl = 0;
	u8 chip_idh = fts_data->ic_info.ids.chip_idh;
	u8 chip_idl = fts_data->ic_info.ids.chip_idl;

	do {
		ret = fts_read_reg(FTS_REG_CHIP_ID, &idh);
		ret = fts_read_reg(FTS_REG_CHIP_ID2, &idl);
		if ((ret < 0) || (idh != chip_idh) || (idl != chip_idl)) {
			FTS_DEBUG("TP Not Ready,ReadData:0x%02x%02x", idh, idl);
		} else if ((idh == chip_idh) && (idl == chip_idl)) {
			FTS_INFO("TP Ready,Device ID:0x%02x%02x", idh, idl);
			return 0;
		}
		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	return -EIO;
}

/*****************************************************************************
 *  Name: fts_tp_state_recovery
 *  Brief: Need execute this function when reset
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
void fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	/* wait tp stable */
	fts_wait_tp_to_valid();
	/* recover TP charger state 0x8B */
	/* recover TP glove state 0xC0 */
	/* recover TP cover state 0xC1 */
	fts_ex_mode_recovery(ts_data);
	/* recover TP gesture state 0xD0 */
#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE
	fts_gesture_recovery(ts_data);
#endif/*CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE*/
	FTS_FUNC_EXIT();
}

int fts_reset_proc(int hdelayms)
{
	FTS_DEBUG("tp reset");
	gpio_direction_output(fts_data->pdata->reset_gpio, 0);
	msleep(5);
	gpio_direction_output(fts_data->pdata->reset_gpio, 1);
	if (hdelayms)
		msleep(hdelayms);

	return 0;
}

void fts_irq_disable(void)
{
	unsigned long irqflags;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&fts_data->irq_lock, irqflags);

	if (!fts_data->irq_disabled) {
		disable_irq_nosync(fts_data->irq);
		fts_data->irq_disabled = true;
	}

	spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

void fts_irq_enable(void)
{
	unsigned long irqflags = 0;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&fts_data->irq_lock, irqflags);

	if (fts_data->irq_disabled) {
		enable_irq(fts_data->irq);
		fts_data->irq_disabled = false;
	}

	spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

void fts_hid2std(void)
{
	int ret = 0;
	u8 buf[3] = { 0xEB, 0xAA, 0x09 };

	ret = fts_write(buf, 3);
	if (ret < 0) {
		FTS_ERROR("hid2std cmd write fail");
	} else {
		msleep(10);
		buf[0] = buf[1] = buf[2] = 0;
		ret = fts_read(NULL, 0, buf, 3);
		if (ret < 0) {
			FTS_ERROR("hid2std cmd read fail");
		} else if ((buf[0] == 0xEB) && (buf[1] == 0xAA)
			   && (buf[2] == 0x08)) {
			FTS_DEBUG("hidi2c change to stdi2c successful");
		} else {
			FTS_DEBUG
			    ("hidi2c change to stdi2c not support or fail");
		}
	}
}

static int fts_get_chip_types(struct fts_ts_data *ts_data,
			      u8 id_h, u8 id_l, bool fw_valid)
{
	int i = 0;
	struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
	u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

	if ((id_h == 0x0) || (id_l == 0x0)) {
		FTS_ERROR("id_h/id_l is 0");
		return -EINVAL;
	}

	FTS_DEBUG("verify id:0x%02x%02x", id_h, id_l);
	for (i = 0; i < ctype_entries; i++) {
		if (fw_valid == VALID) {
			if ((id_h == ctype[i].chip_idh)
			    && (id_l == ctype[i].chip_idl))
				break;
		} else {
			if (((id_h == ctype[i].rom_idh)
			     && (id_l == ctype[i].rom_idl))
			    || ((id_h == ctype[i].pb_idh)
				&& (id_l == ctype[i].pb_idl))
			    || ((id_h == ctype[i].bl_idh)
				&& (id_l == ctype[i].bl_idl)))
				break;
		}
	}

	if (i >= ctype_entries)
		return -ENODATA;

	ts_data->ic_info.ids = ctype[i];
	return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
	int ret = 0;
	u8 chip_id[2] = { 0 };
	u8 id_cmd[4] = { 0 };
	u32 id_cmd_len = 0;

	id_cmd[0] = FTS_CMD_START1;
	id_cmd[1] = FTS_CMD_START2;
	ret = fts_write(id_cmd, 2);
	if (ret < 0) {
		FTS_ERROR("start cmd write fail");
		return ret;
	}

	msleep(FTS_CMD_START_DELAY);
	id_cmd[0] = FTS_CMD_READ_ID;
	id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
	if (ts_data->ic_info.is_incell)
		id_cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
	else
		id_cmd_len = FTS_CMD_READ_ID_LEN;
	ret = fts_read(id_cmd, id_cmd_len, chip_id, 2);
	if ((ret < 0) || (chip_id[0] == 0x0) || (chip_id[1] == 0x0)) {
		FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0],
			  chip_id[1]);
		return -EIO;
	}

	id[0] = chip_id[0];
	id[1] = chip_id[1];
	return 0;
}

/*****************************************************************************
 * Name: fts_get_ic_information
 * Brief: read chip id to get ic information, after run the function, driver w-
 *        ill know which IC is it.
 *        If cant get the ic information, maybe not focaltech's touch IC, need
 *        unregister the driver
 * Input:
 * Output:
 * Return: return 0 if get correct ic information, otherwise return error code
 *****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int cnt = 0;
	u8 chip_id[2] = { 0 };

	ts_data->ic_info.is_incell = FTS_CHIP_IDC;
	ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;

	for (cnt = 0; cnt < 3; cnt++) {
		fts_reset_proc(0);
		mdelay(FTS_CMD_START_DELAY);

		ret = fts_read_bootid(ts_data, &chip_id[0]);
		if (ret < 0) {
			FTS_ERROR("read boot id fail,retry:%d", cnt);
			continue;
		}

		ret =
		    fts_get_chip_types(ts_data, chip_id[0], chip_id[1],
				       INVALID);
		if (ret < 0) {
			FTS_ERROR("can't get ic informaton,retry:%d", cnt);
			continue;
		}

		break;
	}

	if (cnt >= 3) {
		FTS_ERROR("get ic informaton fail");
		return -EIO;
	}

	FTS_INFO("get ic information, chip id = 0x%02x%02x",
		 ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl);

	return 0;
}

/*****************************************************************************
 *  Reprot related
 *****************************************************************************/
static void fts_show_touch_buffer(u8 *data, int datalen)
{
	int i = 0;
	int count = 0;
	char *tmpbuf = NULL;

	tmpbuf = kzalloc(1024, GFP_KERNEL);
	if (!tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return;
	}

	for (i = 0; i < datalen; i++) {
		count +=
		    snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
		if (count >= 1024)
			break;
	}
	FTS_DEBUG("point buffer:%s", tmpbuf);

	if (tmpbuf) {
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
}

void fts_release_all_finger(void)
{
	struct input_dev *input_dev = fts_data->input_dev;
#if FTS_MT_PROTOCOL_B_EN
	u32 finger_count = 0;
	u32 max_touches = fts_data->pdata->max_touch_number;
#endif

	FTS_FUNC_ENTER();
	mutex_lock(&fts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
	for (finger_count = 0; finger_count < max_touches; finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
#else
	input_mt_sync(input_dev);
#endif
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	fts_data->touchs = 0;
	fts_data->key_state = 0;
	mutex_unlock(&fts_data->report_mutex);
	FTS_FUNC_EXIT();
}

/*****************************************************************************
 * Name: fts_input_report_key
 * Brief: process key events,need report key-event if key enable.
 *        if point's coordinate is in (x_dim-50,y_dim-50) ~ (x_dim+50,y_dim+50),
 *        need report it to key event.
 *        x_dim: parse from dts, means key x_coordinate, dimension:+-50
 *        y_dim: parse from dts, means key y_coordinate, dimension:+-50
 * Input:
 * Output:
 * Return: return 0 if it's key event, otherwise return error code
 *****************************************************************************/
static int fts_input_report_key(struct fts_ts_data *data, int index)
{
	int i = 0;
	int x = data->events[index].x;
	int y = data->events[index].y;
	int *x_dim = &data->pdata->key_x_coords[0];
	int *y_dim = &data->pdata->key_y_coords[0];

	if (!data->pdata->have_key)
		return -EINVAL;

	for (i = 0; i < data->pdata->key_number; i++) {
		if ((x >= x_dim[i] - FTS_KEY_DIM)
		    && (x <= x_dim[i] + FTS_KEY_DIM)
		    && (y >= y_dim[i] - FTS_KEY_DIM)
		    && (y <= y_dim[i] + FTS_KEY_DIM)) {
			if (EVENT_DOWN(data->events[index].flag)
			    && !(data->key_state & (1 << i))) {
				input_report_key(data->input_dev,
						 data->pdata->keys[i], 1);
				data->key_state |= (1 << i);
				FTS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
			} else if (EVENT_UP(data->events[index].flag)
				   && (data->key_state & (1 << i))) {
				input_report_key(data->input_dev,
						 data->pdata->keys[i], 0);
				data->key_state &= ~(1 << i);
				FTS_DEBUG("Key%d(%d,%d) Up!", i, x, y);
			}
			return 0;
		}
	}
	return -EINVAL;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_report_b(struct fts_ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;
	bool va_reported = false;
	u32 max_touch_num = data->pdata->max_touch_number;
	struct ts_event *events = data->events;

	for (i = 0; i < data->touch_point; i++) {
		if (fts_input_report_key(data, i) == 0)
			continue;

		va_reported = true;
		input_mt_slot(data->input_dev, events[i].id);

		if (EVENT_DOWN(events[i].flag)) {
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, true);

#if FTS_REPORT_PRESSURE_EN
			if (events[i].p <= 0)
				events[i].p = 0x3f;

			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					 events[i].p);
#endif
			if (events[i].area <= 0)
				events[i].area = 0x09;

			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 events[i].area);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 events[i].x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 events[i].y);

			touchs |= BIT(events[i].id);
			data->touchs |= BIT(events[i].id);

			if ((data->log_level > 2) || ((data->log_level == 2)
						       && (FTS_TOUCH_DOWN ==
							   events[i].flag))) {
				FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
					  events[i].id, events[i].x,
					  events[i].y, events[i].p,
					  events[i].area);
			}
		} else {
			uppoint++;
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(events[i].id);
			if (data->log_level >= 2)
				FTS_DEBUG("[B]P%d UP!", events[i].id);
		}
	}

	if (unlikely(data->touchs ^ touchs)) {
		for (i = 0; i < max_touch_num; i++) {
			if (BIT(i) & (data->touchs ^ touchs)) {
				if (data->log_level >= 2)
					FTS_DEBUG("[B]P%d UP!", i);
				va_reported = true;
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev,
							   MT_TOOL_FINGER,
							   false);
			}
		}
	}
	data->touchs = touchs;

	if (va_reported) {
		/* touchs==0, there's no point but key */
		if (EVENT_NO_DOWN(data) || (!touchs)) {
			if (data->log_level >= 2)
				FTS_DEBUG("[B]Points All Up!");
			input_report_key(data->input_dev, BTN_TOUCH, 0);
		} else {
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	}

	input_sync(data->input_dev);
	return 0;
}

#else
static int fts_input_report_a(struct fts_ts_data *data)
{
	int i = 0;
	int touchs = 0;
	bool va_reported = false;
	struct ts_event *events = data->events;

	for (i = 0; i < data->touch_point; i++) {
		if (fts_input_report_key(data, i) == 0)
			continue;

		va_reported = true;
		if (EVENT_DOWN(events[i].flag)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 events[i].id);
#if FTS_REPORT_PRESSURE_EN
			if (events[i].p <= 0)
				events[i].p = 0x3f;

			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					 events[i].p);
#endif
			if (events[i].area <= 0)
				events[i].area = 0x09;

			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 events[i].area);

			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 events[i].x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 events[i].y);

			input_mt_sync(data->input_dev);

			if ((data->log_level > 2) || ((data->log_level == 2)
						       && (FTS_TOUCH_DOWN ==
							   events[i].flag))) {
				FTS_DEBUG("[A]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
					  events[i].id, events[i].x,
					  events[i].y, events[i].p,
					  events[i].area);
			}
			touchs++;
		}
	}

	/* last point down, current no point but key */
	if (data->touchs && !touchs)
		va_reported = true;

	data->touchs = touchs;

	if (va_reported) {
		if (EVENT_NO_DOWN(data)) {
			if (data->log_level >= 2)
				FTS_DEBUG("[A]Points All Up!");

			input_report_key(data->input_dev, BTN_TOUCH, 0);
			input_mt_sync(data->input_dev);
		} else {
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	}

	input_sync(data->input_dev);
	return 0;
}
#endif

static int fts_read_touchdata(struct fts_ts_data *data)
{
	int ret = 0;
	u8 *buf = data->point_buf;

	memset(buf, 0xFF, data->pnt_buf_size);
	buf[0] = 0x01;

	ret = fts_read(buf, 1, buf + 1, data->pnt_buf_size - 1);
	if (((buf[2] == 0xEF) && (buf[3] == 0xEF) && (buf[4] == 0xEF))
	    || ((ret < 0) && (buf[1] == 0xEF))) {
		/* check if need recovery fw */
		fts_fw_recovery();
		return 1;
	} else if ((ret < 0) || ((buf[1] & 0xF0) != 0x90)) {
		FTS_ERROR("touch data(%x) abnormal,ret:%d", buf[1], ret);
		return -EIO;
	}

//	if (data->gesture_mode) {
#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE
    if (data->gesture_en) {
        ret = fts_gesture_readdata(data, buf + FTS_TOUCH_DATA_LEN);
        if (0 == ret) {
            FTS_INFO("succuss to get gesture data in irq handler");
            return 1;
        }
    }
#endif

	if (data->log_level >= 3)
		fts_show_touch_buffer(buf, data->pnt_buf_size);

	return 0;
}

static int fts_read_parse_touchdata(struct fts_ts_data *data)
{
	int ret = 0;
	int i = 0;
	u8 pointid = 0;
	int base = 0;
	struct ts_event *events = data->events;
	int max_touch_num = data->pdata->max_touch_number;
	u8 *buf = data->point_buf;

	ret = fts_read_touchdata(data);
	if (ret)
		return ret;

	data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	data->touch_point = 0;

	if (data->ic_info.is_incell) {
		if ((data->point_num == 0x0F) && (buf[2] == 0xFF)
		    && (buf[3] == 0xFF) && (buf[4] == 0xFF) &&
		    (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
			FTS_DEBUG("touch buff is 0xff, need recovery state");
			fts_release_all_finger();
			fts_tp_state_recovery(data);
			return -EIO;
		}
	}

	if (data->point_num > max_touch_num) {
		FTS_INFO("invalid point_num(%d)", data->point_num);
		return -EIO;
	}

	for (i = 0; i < max_touch_num; i++) {
		base = FTS_ONE_TCH_LEN * i;
		pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else if (pointid >= max_touch_num) {
			FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
			return -EINVAL;
		}

		data->touch_point++;
		events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
		    (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
		events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
		    (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
		events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
		events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
		events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
		events[i].p = buf[FTS_TOUCH_PRE_POS + base];

		if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
			FTS_INFO("abnormal touch data from fw");
			return -EIO;
		}
	}

	if (data->touch_point == 0) {
		FTS_INFO("no touch point information");
		return -EIO;
	}

	return 0;
}

static void fts_irq_read_report(void)
{
	int ret = 0;
	struct fts_ts_data *ts_data = fts_data;

#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(1);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_prc_queue_work(ts_data);
#endif

	ret = fts_read_parse_touchdata(ts_data);
	if (ret == 0) {
		if(ts_data->suspended)
			return;
		mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
		fts_input_report_b(ts_data);
#else
		fts_input_report_a(ts_data);
#endif
		mutex_unlock(&ts_data->report_mutex);
	}
#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(0);
#endif
}

static irqreturn_t fts_irq_handler(int irq, void *data)
{
	fts_irq_read_report();
	return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
	int ret = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;

	ts_data->irq = gpio_to_irq(pdata->irq_gpio);
	pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	FTS_INFO("irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
	ret = request_threaded_irq(ts_data->irq, NULL, fts_irq_handler,
				   pdata->irq_gpio_flags,
				   FTS_DRIVER_NAME, ts_data);

	return ret;
}

static int fts_input_init(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int key_num = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;
	struct input_dev *input_dev;

	FTS_FUNC_ENTER();
	input_dev = input_allocate_device();
	if (!input_dev) {
		FTS_ERROR("Failed to allocate memory for input device");
		return -ENOMEM;
	}

	/* Init and register Input device */
	input_dev->name = FTS_DRIVER_NAME;
	input_dev->id.bustype = BUS_SPI;

	input_dev->dev.parent = ts_data->dev;

	input_set_drvdata(input_dev, ts_data);

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	if (pdata->have_key) {
		FTS_INFO("set key capabilities");
		for (key_num = 0; key_num < pdata->key_number; key_num++)
			input_set_capability(input_dev, EV_KEY,
					     pdata->keys[key_num]);
	}
#if FTS_MT_PROTOCOL_B_EN
	input_mt_init_slots(input_dev, pdata->max_touch_number,
			    INPUT_MT_DIRECT);
#else
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

	ret = input_register_device(input_dev);
	if (ret) {
		FTS_ERROR("Input device registration failed");
		input_set_drvdata(input_dev, NULL);
		input_free_device(input_dev);
		input_dev = NULL;
		return ret;
	}

	ts_data->input_dev = input_dev;

	FTS_FUNC_EXIT();
	return 0;
}

static int fts_report_buffer_init(struct fts_ts_data *ts_data)
{
	int point_num = 0;
	int events_num = 0;

	point_num = FTS_MAX_POINTS_SUPPORT;
	ts_data->pnt_buf_size = FTS_TOUCH_DATA_LEN + FTS_GESTURE_DATA_LEN;

	ts_data->point_buf =
	    (u8 *)kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
	if (!ts_data->point_buf) {
		FTS_ERROR("failed to alloc memory for point buf");
		return -ENOMEM;
	}

	events_num = point_num * sizeof(struct ts_event);
	ts_data->events = (struct ts_event *)kzalloc(events_num, GFP_KERNEL);
	if (!ts_data->events) {
		FTS_ERROR("failed to alloc memory for point events");
		kfree_safe(ts_data->point_buf);
		return -ENOMEM;
	}

	return 0;
}

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
 * Power Control
 *****************************************************************************/
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
	int ret = 0;

	ts->pinctrl = devm_pinctrl_get(ts->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		FTS_ERROR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(ts->pinctrl);
		goto err_pinctrl_get;
	}

	ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pins_active)) {
		FTS_ERROR("Pin state[active] not found");
		ret = PTR_ERR(ts->pins_active);
		goto err_pinctrl_lookup;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		FTS_ERROR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
		goto err_pinctrl_lookup;
	}

	ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(ts->pins_release)) {
		FTS_ERROR("Pin state[release] not found");
		ret = PTR_ERR(ts->pins_release);
	}

	return 0;
err_pinctrl_lookup:
	if (ts->pinctrl)
		devm_pinctrl_put(ts->pinctrl);

err_pinctrl_get:
	ts->pinctrl = NULL;
	ts->pins_release = NULL;
	ts->pins_suspend = NULL;
	ts->pins_active = NULL;
	return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_active) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
		if (ret < 0)
			FTS_ERROR("Set normal pin state error:%d", ret);
	}

	return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0)
			FTS_ERROR("Set suspend pin state error:%d", ret);
	}

	return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl) {
		if (IS_ERR_OR_NULL(ts->pins_release)) {
			devm_pinctrl_put(ts->pinctrl);
			ts->pinctrl = NULL;
		} else {
			ret =
			    pinctrl_select_state(ts->pinctrl, ts->pins_release);
			if (ret < 0)
				FTS_ERROR("Set gesture pin state error:%d",
					  ret);
		}
	}

	return ret;
}
#endif /* FTS_PINCTRL_EN */

static int fts_power_source_ctrl(struct fts_ts_data *ts_data, int enable)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(ts_data->vdd)) {
		FTS_ERROR("vdd is invalid");
		return -EINVAL;
	}

	FTS_FUNC_ENTER();
	if (enable) {
		if (ts_data->power_disabled) {
			FTS_DEBUG("regulator enable !");
			gpio_direction_output(ts_data->pdata->reset_gpio, 0);
			msleep(1);
			ret = regulator_enable(ts_data->vdd);
			if (ret) {
				FTS_ERROR("enable vdd regulator failed,ret=%d",
					  ret);
			}

			if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
				ret = regulator_enable(ts_data->vcc_i2c);
				if (ret) {
					FTS_ERROR
						("enable vcc_i2c regulator failed,ret=%d",
						ret);
				}
			}
			ts_data->power_disabled = false;
		}
	} else {
		if (!ts_data->power_disabled) {
			FTS_DEBUG("regulator disable !");
			gpio_direction_output(ts_data->pdata->reset_gpio, 0);
			msleep(1);
			ret = regulator_disable(ts_data->vdd);
			if (ret)
				FTS_ERROR("disable vdd regulator failed,ret=%d",
					  ret);

			if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
				ret = regulator_disable(ts_data->vcc_i2c);
				if (ret)
					FTS_ERROR
						("disable vcc_i2c regulator failed,ret=%d",
						ret);

			}
			ts_data->power_disabled = true;
		}
	}

	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
 * Name: fts_power_source_init
 * Brief: Init regulator power:vdd/vcc_io(if have), generally, no vcc_io
 *        vdd---->vdd-supply in dts, kernel will auto add "-supply" to parse
 *        Must be call after fts_gpio_configure() execute,because this function
 *        will operate reset-gpio which request gpio in fts_gpio_configure()
 * Input:
 * Output:
 * Return: return 0 if init power successfully, otherwise return error code
 *****************************************************************************/
static int fts_power_source_init(struct fts_ts_data *ts_data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ts_data->vdd = regulator_get(ts_data->dev, "vdd");
	if (IS_ERR_OR_NULL(ts_data->vdd)) {
		ret = PTR_ERR(ts_data->vdd);
		FTS_ERROR("get vdd regulator failed,ret=%d", ret);
		return ret;
	}

	if (regulator_count_voltages(ts_data->vdd) > 0) {
		ret = regulator_set_voltage(ts_data->vdd, FTS_VTG_MIN_UV,
					    FTS_VTG_MAX_UV);
		if (ret) {
			FTS_ERROR("vdd regulator set_vtg failed ret=%d", ret);
			regulator_put(ts_data->vdd);
			return ret;
		}
	}

	ts_data->vcc_i2c = regulator_get(ts_data->dev, "vcc_i2c");
	if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
		if (regulator_count_voltages(ts_data->vcc_i2c) > 0) {
			ret = regulator_set_voltage(ts_data->vcc_i2c,
						    FTS_I2C_VTG_MIN_UV,
						    FTS_I2C_VTG_MAX_UV);
			if (ret) {
				FTS_ERROR
				    ("vcc_i2c regulator set_vtg failed,ret=%d",
				     ret);
				regulator_put(ts_data->vcc_i2c);
			}
		}
	}
#if FTS_PINCTRL_EN
	fts_pinctrl_init(ts_data);
	fts_pinctrl_select_normal(ts_data);
#endif

	ts_data->power_disabled = true;
	ret = fts_power_source_ctrl(ts_data, ENABLE);
	if (ret)
		FTS_ERROR("fail to enable power(regulator)");

	FTS_FUNC_EXIT();
	return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
#if FTS_PINCTRL_EN
	fts_pinctrl_select_release(ts_data);
#endif

	fts_power_source_ctrl(ts_data, DISABLE);

	if (!IS_ERR_OR_NULL(ts_data->vdd)) {
		if (regulator_count_voltages(ts_data->vdd) > 0)
			regulator_set_voltage(ts_data->vdd, 0, FTS_VTG_MAX_UV);
		regulator_put(ts_data->vdd);
	}

	if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
		if (regulator_count_voltages(ts_data->vcc_i2c) > 0)
			regulator_set_voltage(ts_data->vcc_i2c, 0,
					      FTS_I2C_VTG_MAX_UV);
		regulator_put(ts_data->vcc_i2c);
	}

	return 0;
}

static int fts_power_source_suspend(struct fts_ts_data *ts_data)
{
	int ret = 0;

#if FTS_PINCTRL_EN
	fts_pinctrl_select_suspend(ts_data);
#endif

	ret = fts_power_source_ctrl(ts_data, DISABLE);
	if (ret < 0)
		FTS_ERROR("power off fail, ret=%d", ret);

	return ret;
}

static int fts_power_source_resume(struct fts_ts_data *ts_data)
{
	int ret = 0;

#if FTS_PINCTRL_EN
	fts_pinctrl_select_normal(ts_data);
#endif

	ret = fts_power_source_ctrl(ts_data, ENABLE);
	if (ret < 0)
		FTS_ERROR("power on fail, ret=%d", ret);

	return ret;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

static int fts_gpio_configure(struct fts_ts_data *data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
#if 0
	ret = gpio_request(83, "fts_1.8v_gpio");
	if (ret) {
		FTS_ERROR("[GPIO]1.8V gpio request failed");
		//goto err_irq_gpio_dir;
	}
	
	ret = gpio_direction_output(83, 1);
	if (ret) {
		FTS_ERROR("[GPIO]set_direction for 1.8V gpio failed");
		//goto err_irq_gpio_dir;
	}
#endif
	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		ret = gpio_direction_input(data->pdata->irq_gpio);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	/* request reset gpio */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		ret = gpio_direction_output(data->pdata->reset_gpio, 1);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}
	}

	FTS_FUNC_EXIT();
	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	#if 0
	if (gpio_is_valid(83))
	gpio_free(83);
	#endif
	FTS_FUNC_EXIT();
	return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
			     struct fts_ts_platform_data *pdata)
{
	int ret = 0;
	u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FTS_COORDS_ARR_SIZE) {
		FTS_ERROR("invalid:%s, size:%d", name, coords_size);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, name, coords, coords_size);
	if (ret < 0) {
		FTS_ERROR("Unable to read %s, please check dts", name);
		pdata->x_min = FTS_X_MIN_DISPLAY_DEFAULT;
		pdata->y_min = FTS_Y_MIN_DISPLAY_DEFAULT;
		pdata->x_max = FTS_X_MAX_DISPLAY_DEFAULT;
		pdata->y_max = FTS_Y_MAX_DISPLAY_DEFAULT;
		return -ENODATA;
	} else {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	}

	FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
		 pdata->y_min, pdata->y_max);
	return 0;
}

static int factory_get_calibration_ret(struct device *dev)
{
	return 1;
}

static int factory_get_chip_type(struct device *dev, char *buf)
{
	return sprintf(buf, "focaltech\n");
}
static int factory_proc_hibernate_test(struct device *dev)
{
	int err = 0;
	int i;

	struct fts_ts_data *data = dev_get_drvdata(dev);

	gpio_direction_output(data->pdata->reset_gpio, 1);
	mdelay(1);
	gpio_direction_output(data->pdata->reset_gpio, 0);
	mdelay(5);

	err = fts_read_reg(FTS_REG_FW_VER, &data->fw_ver[0]);
	if (err >= 0) {
		dev_err(dev, "%s: error:read spi ok,with result=%d\n", __func__, err);
		goto out;
		}

	gpio_direction_output(data->pdata->reset_gpio, 1);
	mdelay(400);
	err = fts_read_reg(FTS_REG_FW_VER, &data->fw_ver[0]);

	if (err < 0) {
		FTS_ERROR("read fwversion fail,read:0x%02x", data->fw_ver[0]);
		goto out;
	}

	printk("%s reset test success!\n", __func__);
	/* release all touches */
	for (i = 0; i < FTS_MAX_POINTS_SUPPORT; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 1;
out:
	/* release all touches */
	for (i = 0; i < FTS_MAX_POINTS_SUPPORT; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 0;
}

static bool get_tp_enable_switch(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	return !data->suspended;
}
static int set_tp_enable_switch(struct device *dev, bool enable)
{
	static bool is_the_first_set = 1;
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
	static bool gesture_switch;
	struct fts_ts_data *data = dev_get_drvdata(dev);
#endif

	if (is_the_first_set) {
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
		gesture_switch = data->gesture_en;
#endif
		is_the_first_set = 0;
	}
	FTS_INFO("%s: %d.\n", __func__, enable);

	if (enable) {
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
		data->gesture_en = gesture_switch;
#endif
		fts_reset_proc(200);
		fts_ts_resume(dev);
		return 0;
	} else {
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
		gesture_switch = data->gesture_en;
		data->gesture_en = 0;
#endif
		fts_ts_suspend(dev);
		return 0;
	}
}

static int factory_get_ic_fw_version(struct device *dev, char *buf)
{
	int ret = 0;
	struct fts_ts_data *data = dev_get_drvdata(dev);
	ret = fts_read_reg(FTS_REG_FW_VER, &data->fw_ver[0]);
	FTS_INFO("read chip id-version:0x%02x", data->fw_ver[0]);
	return sprintf(buf, "0x%02X\n", data->fw_ver[0]);
}
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_FH)
static int ctp_work_with_ac_usb_plugin_ftp(int plugin)
{
	struct fts_ts_data *ts_data = fts_data;
	if ((fts_data == NULL) || (!fts_probed))
		return -ENODEV;
	
	if ((!ts_data->suspended) && ts_data->spi) {
		if (0 == plugin)
			printk("USB is plugged Out(%d,%s)\n", __LINE__, __func__);
		else
			printk("USB is plugged In(%d,%s)\n", __LINE__, __func__);

		fts_write_reg(0x8B, plugin);
	}
	return 0;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
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
	struct fts_ts_data *data = dev_get_drvdata(dev);
	unsigned char gesture[10], len;

	strlcpy(gesture, buf, sizeof(gesture));
	len = strlen(gesture);
	if (len > 0)
		if ((gesture[len-1] == '\n') || (gesture[len-1] == '\0'))
			len--;

	FTS_INFO("%s len: %d gtp_state: %d,%d,%d.\n",
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
		FTS_INFO("[set_gesture_switch]write wrong cmd.");
		return 0;
	}
	if (!data->gesture_state)
		data->gesture_en = false;
	else
		data->gesture_en = true;

	FTS_INFO("%s is %x.\n", __func__, data->gesture_state);

	return 0;
}

static bool get_gesture_switch(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	return data->gesture_en;
}

static int get_gesture_pos(struct device *dev, char *buf)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	char temp[20] = {0};
	char gesture_pos[512] = {0};
	int i = 0;

	for (i = 0; i < data->gesture_track_pointnum; i++) {
		snprintf(temp, 20, "%u,%u;", (unsigned int)data->gesture_track_x[i], (unsigned int)data->gesture_track_y[i]);
		strlcat(gesture_pos, temp, 512);
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", gesture_pos);
}
#endif/*CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE*/
#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GLOVE
static bool get_glove_switch(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	printk("%s: glove_enable = %d\n", __func__, data->glove_enable);
	return data->glove_enable;
}

static int set_glove_switch(struct device *dev, bool enable)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	u8 reg_val = enable;
	u8 reg_addr = 0xC0;
	int err = 0;

	data->glove_enable = enable;
	printk("%s: glove_enable = %d\n", __func__, data->glove_enable);
	err = fts_write_reg(reg_addr, reg_val);
	if (err < 0) {
		FTS_ERROR("write glove reg failed,with result=%d\n",err);
		return -EFAULT;
	}

	return 0;
}
#endif
#ifdef CONFIG_TOUCHSCREEN_WORK_MODE
static int get_tp_work_mode(struct device *dev, char *buf)
{
	u8 reg_val = 0;
	u8 reg_addr = 0xA5;
	int err = 0;
	struct fts_ts_data *data = dev_get_drvdata(dev);

	if(data->suspended)
	{
		return snprintf(buf, PAGE_SIZE, "%s\n", "IDLE");
	} else {
		err = fts_read_reg(reg_addr, &reg_val);
		if (err < 0)
			return snprintf(buf, PAGE_SIZE, "%s\n", "READ MODE ERROR");

		printk("%s: read 0xA5 reg value = %#x)\n", __func__, reg_val);

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
	u8 reg_val = 0;

	u8 reg_val_readback = 0;
	int err = 0;
	struct fts_ts_data *data = dev_get_drvdata(dev);

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

		err = fts_write_reg(reg_addr, reg_val);
		if (err < 0) {
			FTS_ERROR("%s: write reg failed, [err]=%d\n", __func__, err);
			return -EFAULT;
		}

		err = fts_read_reg(reg_addr, &reg_val_readback);
		if (err < 0 || reg_val_readback!=reg_val) {
			FTS_ERROR("%s: set operating mode failed, write val==%d, read back val==%d, [read err]=%d, []=\n", 
				__func__, reg_val, reg_val_readback, err);
			return -EFAULT;
		}
	}
	printk("%s: tp set to %s work mode\n", __func__, mode);

	return 0;
}
#endif/*CONFIG_TOUCHSCREEN_WORK_MODE*/

#ifdef CONFIG_FT_INCELL_CHIP
static int ft_ts_provide_reset_control(void)
{
	struct fts_ts_data *ts_data = fts_data;

	if(ts_data->spi == NULL)
		return -ENODEV;

	ts_data = dev_get_drvdata(&ts_data->spi->dev);
	if (gpio_is_valid(ts_data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		//msleep(ts_data->pdata->hard_rst_dly);
		msleep(5);
		gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
	}
	return 0;
}

#ifdef CONFIG_PM
static int ft8xxx_ts_suspend_for_lcd_async_use(void)
{
	struct fts_ts_data *ts_data = fts_data;
	if(ts_data->spi == NULL)
		return -ENODEV;

	fts_data = dev_get_drvdata(&ts_data->spi->dev);
	FTS_INFO("%s Enter!", __func__);
	cancel_work_sync(&fts_data->resume_work);
	fts_ts_suspend(&fts_data->client->dev);

	return 0;
}
static int ft8xxx_ts_resume_for_lcd_async_use(void)
{
	struct fts_ts_data *ts_data = fts_data;
	if(ts_data->spi == NULL)
		return -ENODEV;

	ts_data = dev_get_drvdata(&ts_data->spi->dev);
	FTS_INFO("%s Enter!", __func__);
	if (!work_pending(&ts_data->resume_work))
		schedule_work(&ts_data->resume_work);

	return 0;
}
static int ft8xxx_suspend_need_lcd_reset_high(void)
{
	struct fts_ts_data *ts_data = fts_data;
	if((ts_data == NULL) || (ts_data->spi == NULL))
		return -ENODEV;

	ts_data = dev_get_drvdata(&ts_data->spi->dev);
	return ts_data->pdata->keep_lcd_suspend_reset_high;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE
int ft_need_lcd_power_reset_keep_flag_get(void)
{
	struct fts_ts_data *ts_data = fts_data;

	if(ts_data->spi == NULL)
		return -ENODEV;

	ts_data = dev_get_drvdata(&ts_data->spi->dev);
	return ts_data->gesture_en;
}
#endif
#endif
#ifdef CONFIG_TOUCHSCREEN_FT8006S_SPI_HALL
static bool get_hall_switch(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	return data->hall_enable;
}

static int set_hall_switch(struct device *dev, const char *buf)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	u8 reg_addr = 0xC1;
	int err = 0;
	unsigned char hall[5];
	strlcpy(hall, buf, sizeof(hall));
	if (hall[0] == '0')
		data->hall_enable = false;
	else
		data->hall_enable = true;

	err = fts_write_reg(fts_i2c_client, reg_addr, data->hall_enable);
	if (err < 0) {
		dev_err(dev, "%s: write glove reg failed,with result=%d\n", __func__, err);
		return -EFAULT;
	}

	return 0;
}
#endif /*CONFIG_TOUCHSCREEN_FT8006S_SPI_HALL*/
static int factory_ts_func_test_register(struct fts_ts_data* data)
{
	ts_gen_func_test_init();
	data->ts_test_dev.dev = &data->spi->dev;
	data->ts_test_dev.get_chip_type = factory_get_chip_type;
	data->ts_test_dev.get_calibration_ret = factory_get_calibration_ret;
	data->ts_test_dev.get_module_id = factory_get_module_id;
	data->ts_test_dev.proc_hibernate_test = factory_proc_hibernate_test;
	data->ts_test_dev.get_tp_enable_switch = get_tp_enable_switch;//
	data->ts_test_dev.set_tp_enable_switch = set_tp_enable_switch;//
	data->ts_test_dev.get_fs_fw_version = factory_get_fs_fw_version;
	data->ts_test_dev.get_ic_fw_version = factory_get_ic_fw_version;
	data->ts_test_dev.get_diff = factory_get_diff;
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
	data->ts_test_dev.get_gesture_switch = get_gesture_switch;
	data->ts_test_dev.set_gesture_switch = set_gesture_switch;
	data->ts_test_dev.get_gesture_pos = get_gesture_pos;
#endif

#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GLOVE)
	data->ts_test_dev.get_glove_switch = get_glove_switch;
	data->ts_test_dev.set_glove_switch = set_glove_switch;
#endif

#ifdef CONFIG_FT_INCELL_CHIP
	data->ts_test_dev.ts_reset_for_lcd_use = ft_ts_provide_reset_control;
	data->ts_test_dev.ts_async_suspend_for_lcd_use = ft8xxx_ts_suspend_for_lcd_async_use;
	data->ts_test_dev.ts_async_resume_for_lcd_use = ft8xxx_ts_resume_for_lcd_async_use;
	data->ts_test_dev.ts_suspend_need_lcd_reset_high = ft8xxx_suspend_need_lcd_reset_high;
#endif
#if defined(CONFIG_FT_INCELL_CHIP) && defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
	data->ts_test_dev.ts_suspend_need_lcd_power_reset_high = ft_need_lcd_power_reset_keep_flag_get;
#endif

#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_FH)
	data->ts_test_dev.ts_usb_plugin_out_status_get= ctp_work_with_ac_usb_plugin_ftp;
#endif

#if defined(CONFIG_TOUCHSCREEN_WORK_MODE)
	data->ts_test_dev.get_tp_work_mode = get_tp_work_mode;
	data->ts_test_dev.set_tp_work_mode = set_tp_work_mode;
#endif
#if defined(CONFIG_TOUCHSCREEN_FT8006S_SPI_HALL)
	data->ts_test_dev.get_hall_switch = get_hall_switch;
	data->ts_test_dev.set_hall_switch = set_hall_switch;
#endif
#if 0
/*
	data->ts_test_dev.get_factory_info = factory_get_factory_info;
	data->ts_test_dev.get_short_test = factory_short_test_sy;
	data->ts_test_dev.get_short_test = factory_open_test_sy;
	data->ts_test_dev.get_rawdata = factory_get_rawdata_sy;
	data->ts_test_dev.get_rawdata_info = factory_get_rawdata_info_sy;
	data->ts_test_dev.check_fw_update_need = factory_check_fw_update_need;
	data->ts_test_dev.get_fw_update_progress = factory_get_fw_update_progress;
	data->ts_test_dev.proc_fw_update = factory_proc_fw_update;
	data->ts_test_dev.proc_fw_update_with_given_file = factory_proc_fw_bin_update;
#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_HALL)
	data->ts_test_dev.get_hall_switch = get_hall_switch;
	data->ts_test_dev.set_hall_switch = set_hall_switch;
#endif
#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_FACTORYINFO)
	data->ts_test_dev.get_factory_info = factory_get_factory_info;
#endif
*/
#endif
	register_ts_func_test_device(&data->ts_test_dev);
	return 0;
}
static int fts_parse_dt(struct device *dev, struct fts_ts_data *data)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	u32 temp_val = 0;
	struct fts_ts_platform_data *pdata = data->pdata;
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
	struct property *prop;
	int i = 0;
	u32 gesture_map[MAX_GESTURE];
	u32 num_buttons = 0;
#endif

	FTS_FUNC_ENTER();
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
	prop = of_find_property(np, "focaltech,gesture-func-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE)
			return -EINVAL;
		ret = of_property_read_u32_array(np,
			"focaltech,gesture-func-map", gesture_map, num_buttons);
		if (ret) {
			FTS_ERROR("Unable to read gesture func map\n");
			return ret;
		}
		for (i = 0; i < num_buttons; i++)
			data->gesture_func_map[i] = gesture_map[i];
		data->gesture_num = num_buttons;
	}
	prop = of_find_property(np, "focaltech,gesture-figure-map", NULL);
	if (prop) {
		ret = of_property_read_u32_array(np, "focaltech,gesture-figure-map", gesture_map,
			num_buttons);
		if (ret) {
			FTS_ERROR("Unable to read gesture figure map\n");
			return ret;
		}
		for (i = 0; i < num_buttons; i++){
			data->gesture_figure_map[i] = gesture_map[i];
		}
	}
#endif

	ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (ret < 0)
		FTS_ERROR("Unable to get display-coords");

	/* key */
	pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
	if (pdata->have_key) {
		ret =
		    of_property_read_u32(np, "focaltech,key-number",
					 &pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key number undefined!");

		ret = of_property_read_u32_array(np, "focaltech,keys",
						 pdata->keys,
						 pdata->key_number);
		if (ret < 0) {
			FTS_ERROR("Keys undefined!");
		} else if (pdata->key_number > FTS_MAX_KEYS)
			pdata->key_number = FTS_MAX_KEYS;

		ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
						 pdata->key_x_coords,
						 pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key Y Coords undefined!");

		ret = of_property_read_u32_array(np, "focaltech,key-y-coords",
						 pdata->key_y_coords,
						 pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key X Coords undefined!");

		FTS_INFO("VK Number:%d, key:(%d,%d,%d), "
			 "coords:(%d,%d),(%d,%d),(%d,%d)",
			 pdata->key_number,
			 pdata->keys[0], pdata->keys[1], pdata->keys[2],
			 pdata->key_x_coords[0], pdata->key_y_coords[0],
			 pdata->key_x_coords[1], pdata->key_y_coords[1],
			 pdata->key_x_coords[2], pdata->key_y_coords[2]);
	}

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
						    0,
						    &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		FTS_ERROR("Unable to get reset_gpio");

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
						  0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		FTS_ERROR("Unable to get irq_gpio");

	ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
	if (ret < 0) {
		FTS_ERROR("Unable to get max-touch-number, please check dts");
		pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
	} else {
		/* max_touch_number must >= 2 */
		if (temp_val < 2)
			pdata->max_touch_number = 2;
		else if (temp_val > FTS_MAX_POINTS_SUPPORT)
			pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
		else
			pdata->max_touch_number = temp_val;
	}

#ifdef CONFIG_FT_INCELL_CHIP
	pdata->keep_lcd_suspend_reset_high = of_property_read_bool(np,
			"focaltech,need-lcd-suspend-reset-keep-high");
#endif
	FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
		 pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);

	FTS_FUNC_EXIT();
	return 0;
}

#if defined(CONFIG_FB)
static void fts_resume_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
						   resume_work);

	fts_ts_resume(ts_data->dev);
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
						   fb_notif);

	if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
		FTS_INFO("event(%lu) do not need process\n", event);
		return 0;
	}

	blank = evdata->data;
	FTS_INFO("FB event:%lu,blank:%d", event, *blank);
	switch (*blank) {
	case FB_BLANK_UNBLANK:
		if (event == FB_EARLY_EVENT_BLANK) {
			FTS_INFO("resume: event = %lu, not care\n", event);
		} else if (event == FB_EVENT_BLANK) {
			queue_work(fts_data->ts_workqueue,
				   &fts_data->resume_work);
		}
		break;
	case FB_BLANK_POWERDOWN:
		if (event == FB_EARLY_EVENT_BLANK) {
			cancel_work_sync(&fts_data->resume_work);
			fts_ts_suspend(ts_data->dev);
		} else if (event == FB_EVENT_BLANK) {
			FTS_INFO("suspend: event = %lu, not care\n", event);
		}
		break;
	default:
		FTS_INFO("FB BLANK(%d) do not need process\n", *blank);
		break;
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void fts_ts_early_suspend(struct early_suspend *handler)
{
	struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
						   early_suspend);

	fts_ts_suspend(ts_data->dev);
}

static void fts_ts_late_resume(struct early_suspend *handler)
{
	struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
						   early_suspend);

	fts_ts_resume(ts_data->dev);
}
#endif
#if 0
static struct device *ts_dev;

static ssize_t fts_light_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	dev = ts_dev;

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (input == 1)
		fts_ts_suspend(dev);
	else if (input == 0)
		fts_ts_resume(dev);
	else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(ts_suspend, 0664, NULL, fts_light_suspend_store);

static struct attribute *TP_sysfs_attrs[] = {
	attrify(ts_suspend),
	NULL,
};

static struct kobject *TP_ctrl_kobj;
static struct attribute_group TP_attr_group = {
	.attrs = TP_sysfs_attrs,
};

static int ctp_sysfs_init(struct i2c_client *client)
{
	TP_ctrl_kobj = kobject_create_and_add("touchscreen", NULL);
	if (!TP_ctrl_kobj) {
		dev_err(&client->dev, "Create TP_sysfs_init failed!\n");
		return -ENOMEM;
	}
	return sysfs_create_group(TP_ctrl_kobj, &TP_attr_group);
}

static void ctp_sysfs_exit(void)
{
	sysfs_remove_group(TP_ctrl_kobj, &TP_attr_group);
	kobject_put(TP_ctrl_kobj);
}
#endif
static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int pdata_size = sizeof(struct fts_ts_platform_data);

	FTS_FUNC_ENTER();
	FTS_INFO("%s", FTS_DRIVER_VERSION);
	ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
	if (!ts_data->pdata) {
		FTS_ERROR("allocate memory for platform_data fail");
		return -ENOMEM;
	}

	if (ts_data->dev->of_node) {
		ret = fts_parse_dt(ts_data->dev, ts_data);
		if (ret)
			FTS_ERROR("device-tree parse fail");
	} else {
		if (ts_data->dev->platform_data) {
			memcpy(ts_data->pdata, ts_data->dev->platform_data,
			       pdata_size);
		} else {
			FTS_ERROR("platform_data is null");
			return -ENODEV;
		}
	}

	ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (!ts_data->ts_workqueue)
		FTS_ERROR("create fts workqueue fail");

	spin_lock_init(&ts_data->irq_lock);
	mutex_init(&ts_data->report_mutex);
	mutex_init(&ts_data->bus_lock);

	/* Init communication interface */
	ret = fts_bus_init(ts_data);
	if (ret) {
		FTS_ERROR("bus initialize fail");
		goto err_bus_init;
	}

	ret = fts_input_init(ts_data);
	if (ret) {
		FTS_ERROR("input initialize fail");
		goto err_input_init;
	}

	ret = fts_report_buffer_init(ts_data);
	if (ret) {
		FTS_ERROR("report buffer init fail");
		goto err_report_buffer;
	}

	ret = fts_gpio_configure(ts_data);
	if (ret) {
		FTS_ERROR("configure the gpios fail");
		goto err_gpio_config;
	}
#if FTS_POWER_SOURCE_CUST_EN
	ret = fts_power_source_init(ts_data);
	if (ret) {
		FTS_ERROR("fail to get power(regulator)");
		goto err_power_init;
	}
#endif

#if (!FTS_CHIP_IDC)
	fts_reset_proc(200);
#endif

	ret = fts_get_ic_information(ts_data);
	if (ret) {
		FTS_ERROR("not focal IC, unregister driver");
		goto err_irq_req;
	}

	ret = fts_create_apk_debug_channel(ts_data);
	if (ret)
		FTS_ERROR("create apk debug node fail");
#if 0
	ret = ctp_sysfs_init(ts_data->client);
	if (ret < 0)
		dev_err(&ts_data->client->dev, "Fail to create debug files!");

	ts_dev =  &ts_data->client->dev;
#endif
	ret = fts_create_sysfs(ts_data);
	if (ret)
		FTS_ERROR("create sysfs node fail");

#if FTS_POINT_REPORT_CHECK_EN
	ret = fts_point_report_check_init(ts_data);
	if (ret)
		FTS_ERROR("init point report check fail");
#endif

	ret = fts_ex_mode_init(ts_data);
	if (ret)
		FTS_ERROR("init glove/cover/charger fail");

#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE
	ret = fts_gesture_init(ts_data);
#endif/*CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE*/
	if (ret)
		FTS_ERROR("init gesture fail");

#if FT8006P_TEST_EN
	ret = fts_test_init(ts_data);
	if (ret) {
		FTS_ERROR("init production test fail");
	}
#endif
#if FTS_ESDCHECK_EN
	ret = fts_esdcheck_init(ts_data);
	if (ret)
		FTS_ERROR("init esd check fail");
#endif

	ret = fts_irq_registration(ts_data);
	if (ret) {
		FTS_ERROR("request irq failed");
		goto err_irq_req;
	}

	ret = fts_fwupg_init(ts_data);
	if (ret)
		FTS_ERROR("init fw upgrade fail");
#if defined(CONFIG_FB)
	if (ts_data->ts_workqueue)
		INIT_WORK(&ts_data->resume_work, fts_resume_work);

	ts_data->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts_data->fb_notif);
	if (ret)
		FTS_ERROR("[FB]Unable to register fb_notifier: %d", ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts_data->early_suspend.level =
	    EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
	ts_data->early_suspend.suspend = fts_ts_early_suspend;
	ts_data->early_suspend.resume = fts_ts_late_resume;
	register_early_suspend(&ts_data->early_suspend);
#endif

	FTS_FUNC_EXIT();
	return 0;

err_irq_req:
#if FTS_POWER_SOURCE_CUST_EN
err_power_init:
	fts_power_source_exit(ts_data);
#endif
	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);
	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);
err_gpio_config:
	kfree_safe(ts_data->point_buf);
	kfree_safe(ts_data->events);
err_report_buffer:
	input_unregister_device(ts_data->input_dev);
err_input_init:
	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);
err_bus_init:
	kfree_safe(ts_data->bus_tx_buf);
	kfree_safe(ts_data->bus_rx_buf);
	kfree_safe(ts_data->pdata);

	FTS_FUNC_EXIT();
	return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_exit(ts_data);
#endif

	fts_release_apk_debug_channel(ts_data);
	fts_remove_sysfs(ts_data);
	fts_ex_mode_exit(ts_data);

	fts_fwupg_exit(ts_data);

#if FT8006P_TEST_EN
	fts_test_exit(ts_data);
#endif
#if FTS_ESDCHECK_EN
	fts_esdcheck_exit(ts_data);
#endif

#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE
	fts_gesture_exit(ts_data);
#endif/*CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE*/
	fts_bus_exit(ts_data);

	free_irq(ts_data->irq, ts_data);
	input_unregister_device(ts_data->input_dev);

	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts_data->fb_notif))
		FTS_ERROR("Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts_data->early_suspend);
#endif

	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);

	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);

#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_exit(ts_data);
#endif

	kfree_safe(ts_data->point_buf);
	kfree_safe(ts_data->events);

	kfree_safe(ts_data->pdata);
	kfree_safe(ts_data);

	FTS_FUNC_EXIT();
#if 0
	ctp_sysfs_exit();
#endif
	return 0;
}

static int fts_ts_suspend(struct device *dev)
{
	int ret = 0;
	struct fts_ts_data *ts_data = fts_data;

	FTS_FUNC_ENTER();
	if (ts_data->suspended) {
		FTS_INFO("Already in suspend state");
		return 0;
	}

	if (ts_data->fw_loading) {
		FTS_INFO("fw upgrade in process, can't suspend");
		return 0;
	}
#if FTS_ESDCHECK_EN
	fts_esdcheck_suspend();
#endif
#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GLOVE
	if (ts_data->glove_enable) {
		int err = 0;
		err = fts_write_reg(0xC0, 0);
		if (err < 0) {
			FTS_ERROR("write glove reg failed,with result=%d\n",err);
		}
		fts_reset_proc(200);
	}
#endif/*CONFIG_TOUCHSCREEN_FT8006P_SPI_GLOVE*/

#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE
	ts_data->gesture_suspend_en = ts_data->gesture_en;
	if(ts_data->gesture_suspend_en)
	{
		if (fts_gesture_suspend(ts_data) == 0) {
			/* Enter into gesture mode(suspend) */
			ts_data->suspended = true;
			return 0;
		}
	}
#endif
	fts_irq_disable();
		FTS_INFO("make TP enter into sleep mode");
		ret =
		    fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
		if (ret < 0)
			FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

		if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
			ret = fts_power_source_suspend(ts_data);
			if (ret < 0)
				FTS_ERROR("power enter suspend fail");
#endif
		}

	fts_release_all_finger();
	ts_data->suspended = true;
	FTS_FUNC_EXIT();
	return 0;
}

int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = fts_data;

	FTS_FUNC_ENTER();
	if (!ts_data->suspended) {
		FTS_DEBUG("Already in awake state");
		return 0;
	}

	fts_release_all_finger();

	if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
		fts_power_source_resume(ts_data);
#endif
		fts_reset_proc(200);
	}

	fts_wait_tp_to_valid();
	fts_ex_mode_recovery(ts_data);

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume();
#endif

//	if (ts_data->gesture_mode)
#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE
	if(ts_data->gesture_suspend_en)
		fts_gesture_resume(ts_data);
#endif/*CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE*/
#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GLOVE
	if (ts_data->glove_enable) {
		int err = 0;
		err = fts_write_reg(0xC0, 1);
		if (err < 0) {
			FTS_ERROR("write glove reg failed,with result=%d\n",err);
			//return ;
		}
	}
#endif/*CONFIG_TOUCHSCREEN_FT8006P_SPI_GLOVE*/

	fts_irq_enable();
	ts_data->suspended = false;
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_FH)
	if (usb_flag)
		ctp_work_with_ac_usb_plugin_ftp(true);
	else
		ctp_work_with_ac_usb_plugin_ftp(false);
#endif/*CONFIG_TOUCHSCREEN_FT8006P_SPI_FH*/
	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
 * TP Driver
 *****************************************************************************/
static int fts_ts_probe(struct spi_device *spi)
{
	int ret = 0;
	struct fts_ts_data *ts_data = NULL;

	FTS_INFO("Touch Screen(SPI BUS) driver prboe...");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	//ret = spi_setup(spi);
	if (ret) {
		FTS_ERROR("spi setup fail");
		return ret;
	}

	/* malloc memory for global struct variable */
	ts_data = (struct fts_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data) {
		FTS_ERROR("allocate memory for fts_data fail");
		return -ENOMEM;
	}

	fts_data = ts_data;
	ts_data->spi = spi;
	ts_data->dev = &spi->dev;
	ts_data->log_level = 2;
	spi_set_drvdata(spi, ts_data);

	ret = fts_ts_probe_entry(ts_data);
	if (ret) {
		FTS_ERROR("Touch Screen(SPI BUS) driver probe fail");
		kfree_safe(ts_data);
		return ret;
	}

	factory_ts_func_test_register(ts_data);
#ifdef CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE
	ts_data->gesture_en = false;
#endif
	FTS_INFO("Touch Screen(SPI BUS) driver prboe successfully");
	fts_probed = true;
	return 0;
}

#ifdef CONFIG_PM
static int pm_fts_ts_suspend(struct device *dev)
{
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
	if (fts_data->gesture_suspend_en && fts_probed) {
		FTS_INFO("fts_ts_suspend_gesture\n");
		fts_irq_disable();
		/* make tp can wake the system */
		enable_irq_wake(fts_data->irq);
	}
#endif

	return 0;
}
static int pm_fts_ts_resume(struct device *dev)
{
#if defined(CONFIG_TOUCHSCREEN_FT8006P_SPI_GESTURE)
	if (fts_data->gesture_suspend_en && fts_probed) {
		FTS_INFO("fts_ts_resume_gesture\n");
		/* make tp cannot wake the system */
		disable_irq_wake(fts_data->irq);
		fts_irq_enable();
	}
#endif

	return 0;
}

static const struct dev_pm_ops fts_ts_pm_ops = {
	.suspend = pm_fts_ts_suspend,
	.resume = pm_fts_ts_resume,
};
#endif

static int fts_ts_remove(struct spi_device *spi)
{
	return fts_ts_remove_entry(spi_get_drvdata(spi));
}

static const struct spi_device_id fts_ts_id[] = {
	{FTS_DRIVER_NAME, 0},
	{},
};

static const struct of_device_id fts_dt_match[] = {
	{.compatible = "focaltech,fts",},
	{},
};

MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct spi_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		   .name = FTS_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(fts_dt_match),
#ifdef CONFIG_PM
        .pm = &fts_ts_pm_ops,
#endif
		   },
	.id_table = fts_ts_id,
};

static int __init fts_ts_init(void)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ret = spi_register_driver(&fts_ts_driver);
	if (ret != 0)
		FTS_ERROR("Focaltech touch screen driver init failed!");

	FTS_FUNC_EXIT();
	return ret;
}

static void __exit fts_ts_exit(void)
{
	spi_unregister_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
