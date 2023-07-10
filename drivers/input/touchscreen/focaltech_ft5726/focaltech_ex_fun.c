/*
*
* FocalTech fts TouchScreen driver.
*
* Copyright (c) 2010-2016 Hisense All rights reserved.
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
* File Name: Focaltech_ex_fun.c
*
* Author: shao zuodong
*
* Created: 2016-02-22
*
* Abstract:
*
* Reference:
*
*/
 #define pr_fmt(fmt) "TP-focaltech-ex-func: " fmt
/*
* 1.Included header files
*/
#include "focaltech_ts.h"

/*
* Private constant and macro definitions using #define
*/
/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER		2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA			6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG		8
#define PROC_NAME	"ftxxxx-debug"

#define WRITE_BUF_SIZE		512
#define READ_BUF_SIZE		512

/* Private enumerations, structures and unions using typedef */
/* Static variables */
static unsigned char proc_operate_mode;
static struct proc_dir_entry *fts_proc_entry;
/* Global variable or extern global variabls/functions */
#define GTP_ESD_PROTECT 0
#if GTP_ESD_PROTECT
int apk_debug_flag = 0;
#endif
extern struct i2c_client *G_Client;
/* Static function prototypes */

/*interface of write proc*/
/*
* Name: fts_debug_write
* Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
*/
static ssize_t fts_debug_write(struct file *filp,
	const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;

	if (copy_from_user(&writebuf, buff, buflen)) {
		pr_buf_err("%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = ft5x06_i2c_write(G_Client, writebuf + 1, writelen);
		if (ret < 0) {
			pr_buf_err("%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = ft5x06_i2c_write(G_Client, writebuf + 1, writelen);
		if (ret < 0) {
			pr_buf_err("%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = ft5x06_i2c_write(G_Client, writebuf + 1, writelen);
		if (ret < 0) {
			pr_buf_err("%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}

	return count;
}

/*interface of read proc*/
/*
* Name: fts_debug_read
* Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use
* Output: page point to data
* Return: read char number
*/
static ssize_t fts_debug_read(struct file *filp,
	char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	unsigned char buf[READ_BUF_SIZE];

	switch (proc_operate_mode) {
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = ft5x06_i2c_read(G_Client, NULL, 0, buf, readlen);
		if (ret < 0) {
			pr_buf_err("%s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = ft5x06_i2c_read(G_Client, NULL, 0, buf, readlen);
		if (ret < 0) {
			pr_buf_err("%s:read iic error\n", __func__);
			return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

	if (copy_to_user(buff, buf, num_read_chars)) {
		pr_buf_err("%s:copy to user error\n", __func__);
		return -EFAULT;
	}

	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
		.owner = THIS_MODULE,
		.read = fts_debug_read,
		.write = fts_debug_write,
};

/*
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
*/
int fts_create_apk_debug_channel(struct i2c_client *client)
{
	fts_proc_entry = proc_create(PROC_NAME, 0644, NULL, &fts_proc_fops);
	if (NULL == fts_proc_entry) {
		pr_buf_err("Couldn't create proc entry!\n");
		return -ENOMEM;
	}

	return 0;
}
/*
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
*/
void fts_release_apk_debug_channel(void)
{
	if (fts_proc_entry)
		proc_remove(fts_proc_entry);
}

