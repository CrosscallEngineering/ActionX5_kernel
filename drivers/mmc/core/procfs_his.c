/*
 * Procfs support for hosts and cards
 *
 * Copyright (C) 2021 Hisense Corporation
 *
 * This program is modified from debugfs.c, 
 * contributed to instead debugfs in user version
 */

#include <linux/moduleparam.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/fault-inject.h>
#include <linux/his_debug_base.h>
#include <linux/proc_fs.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include "core.h"
#include "card.h"
#include "host.h"
#include "mmc_ops.h"
#include "../host/sdhci-msm.h"


/*add start: add device node for getting status of TFCard holder in factory mode */
extern int sdcard_status_global;
/*add end*/

/* The procfs functions are optimized away when CONFIG_HISENSE_DEBUG_CTRL isn't set and CONFIG_DEBUFS is set. */
static int mmc_ios_show(struct seq_file *s, void *data)
{
	static const char *vdd_str[] = {
		[8]	= "2.0",
		[9]	= "2.1",
		[10]	= "2.2",
		[11]	= "2.3",
		[12]	= "2.4",
		[13]	= "2.5",
		[14]	= "2.6",
		[15]	= "2.7",
		[16]	= "2.8",
		[17]	= "2.9",
		[18]	= "3.0",
		[19]	= "3.1",
		[20]	= "3.2",
		[21]	= "3.3",
		[22]	= "3.4",
		[23]	= "3.5",
		[24]	= "3.6",
	};
	struct mmc_host	*host = s->private;
	struct mmc_ios	*ios = &host->ios;
	const char *str;

    if(NULL == host || NULL == ios)
    {
        pr_err("[%s,%d]: Invalid pointer\r\n", __func__, __LINE__);
		return -EINVAL;
    }

	seq_printf(s, "clock:\t\t%u Hz\n", ios->clock);
	if (host->actual_clock)
		seq_printf(s, "actual clock:\t%u Hz\n", host->actual_clock);
	seq_printf(s, "vdd:\t\t%u ", ios->vdd);
	if ((1 << ios->vdd) & MMC_VDD_165_195)
		seq_printf(s, "(1.65 - 1.95 V)\n");
	else if (ios->vdd < (ARRAY_SIZE(vdd_str) - 1)
			&& vdd_str[ios->vdd] && vdd_str[ios->vdd + 1])
		seq_printf(s, "(%s ~ %s V)\n", vdd_str[ios->vdd],
				vdd_str[ios->vdd + 1]);
	else
		seq_printf(s, "(invalid)\n");

	switch (ios->bus_mode) {
	case MMC_BUSMODE_OPENDRAIN:
		str = "open drain";
		break;
	case MMC_BUSMODE_PUSHPULL:
		str = "push-pull";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "bus mode:\t%u (%s)\n", ios->bus_mode, str);

	switch (ios->chip_select) {
	case MMC_CS_DONTCARE:
		str = "don't care";
		break;
	case MMC_CS_HIGH:
		str = "active high";
		break;
	case MMC_CS_LOW:
		str = "active low";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "chip select:\t%u (%s)\n", ios->chip_select, str);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		str = "off";
		break;
	case MMC_POWER_UP:
		str = "up";
		break;
	case MMC_POWER_ON:
		str = "on";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "power mode:\t%u (%s)\n", ios->power_mode, str);
	seq_printf(s, "bus width:\t%u (%u bits)\n",
			ios->bus_width, 1 << ios->bus_width);

	switch (ios->timing) {
	case MMC_TIMING_LEGACY:
		str = "legacy";
		break;
	case MMC_TIMING_MMC_HS:
		str = "mmc high-speed";
		break;
	case MMC_TIMING_SD_HS:
		str = "sd high-speed";
		break;
	case MMC_TIMING_UHS_SDR12:
		str = "sd uhs SDR12";
		break;
	case MMC_TIMING_UHS_SDR25:
		str = "sd uhs SDR25";
		break;
	case MMC_TIMING_UHS_SDR50:
		str = "sd uhs SDR50";
		break;
	case MMC_TIMING_UHS_SDR104:
		str = "sd uhs SDR104";
		break;
	case MMC_TIMING_UHS_DDR50:
		str = "sd uhs DDR50";
		break;
	case MMC_TIMING_MMC_DDR52:
		str = "mmc DDR52";
		break;
	case MMC_TIMING_MMC_HS200:
		str = "mmc HS200";
		break;
	case MMC_TIMING_MMC_HS400:
		str = mmc_card_hs400es(host->card) ?
			"mmc HS400 enhanced strobe" : "mmc HS400";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "timing spec:\t%u (%s)\n", ios->timing, str);

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		str = "3.30 V";
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		str = "1.80 V";
		break;
	case MMC_SIGNAL_VOLTAGE_120:
		str = "1.20 V";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "signal voltage:\t%u (%s)\n", ios->signal_voltage, str);

	switch (ios->drv_type) {
	case MMC_SET_DRIVER_TYPE_A:
		str = "driver type A";
		break;
	case MMC_SET_DRIVER_TYPE_B:
		str = "driver type B";
		break;
	case MMC_SET_DRIVER_TYPE_C:
		str = "driver type C";
		break;
	case MMC_SET_DRIVER_TYPE_D:
		str = "driver type D";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "driver type:\t%u (%s)\n", ios->drv_type, str);

	return 0;
}

static int mmc_ios_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmc_ios_show, PDE_DATA(inode));
}

static const struct file_operations mmc_ios_fops = {
	.open           = mmc_ios_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

/*add start: add device node for getting status of TFCard holder in factory mode */
static ssize_t sdcard_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value;

	if (sdcard_status_global == -ENOSYS)
		value = 3;
	else
		value = sdcard_status_global;

	return sprintf(buf, "%d\n", value);
}

static DEVICE_ATTR(sdcard_status, S_IRUGO,
		sdcard_status_show, NULL);
/*add end*/

static int mmc_err_stats_show(struct seq_file *file, void *data)
{
	struct mmc_host *host = (struct mmc_host *)file->private;

	if (!host)
		return -EINVAL;

	seq_printf(file, "# Command Timeout Occurred:\t %d\n",
		   host->err_stats[MMC_ERR_CMD_TIMEOUT]);

	seq_printf(file, "# Command CRC Errors Occurred:\t %d\n",
		   host->err_stats[MMC_ERR_CMD_CRC]);

	seq_printf(file, "# Data Timeout Occurred:\t %d\n",
		   host->err_stats[MMC_ERR_DAT_TIMEOUT]);

	seq_printf(file, "# Data CRC Errors Occurred:\t %d\n",
		   host->err_stats[MMC_ERR_DAT_CRC]);

	seq_printf(file, "# Auto-Cmd Error Occurred:\t %d\n",
		   host->err_stats[MMC_ERR_ADMA]);

	seq_printf(file, "# ADMA Error Occurred:\t %d\n",
		   host->err_stats[MMC_ERR_ADMA]);

	seq_printf(file, "# Tuning Error Occurred:\t %d\n",
		   host->err_stats[MMC_ERR_TUNING]);

	seq_printf(file, "# CMDQ RED Errors:\t\t %d\n",
		   host->err_stats[MMC_ERR_CMDQ_RED]);

	seq_printf(file, "# CMDQ GCE Errors:\t\t %d\n",
		   host->err_stats[MMC_ERR_CMDQ_GCE]);

	seq_printf(file, "# CMDQ ICCE Errors:\t\t %d\n",
		   host->err_stats[MMC_ERR_CMDQ_ICCE]);

	seq_printf(file, "# Request Timedout:\t %d\n",
		   host->err_stats[MMC_ERR_REQ_TIMEOUT]);

	seq_printf(file, "# CMDQ Request Timedout:\t %d\n",
		   host->err_stats[MMC_ERR_CMDQ_REQ_TIMEOUT]);

	seq_printf(file, "# ICE Config Errors:\t\t %d\n",
		   host->err_stats[MMC_ERR_ICE_CFG]);
	return 0;
}

static int mmc_err_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmc_err_stats_show, PDE_DATA(inode));
}

static ssize_t mmc_err_stats_write(struct file *filp, const char __user *ubuf,
				   size_t cnt, loff_t *ppos)
{
	struct mmc_host *host = filp->f_mapping->host->i_private;

	if (!host)
		return -EINVAL;

	pr_debug("%s: Resetting MMC error statistics\n", __func__);
	memset(host->err_stats, 0, sizeof(host->err_stats));

	return cnt;
}

static const struct file_operations mmc_err_stats_fops = {
	.open	= mmc_err_stats_open,
	.read	= seq_read,
	.write	= mmc_err_stats_write,
};

/*******************************************************************************
 * Add local record for mmc0/mmc1 proc root node, to able to remove the node
 * *****************************************************************************/
typedef struct{
    struct device *dev;
    struct proc_dir_entry *procfs_root;
}mmc_procfs_t;

static mmc_procfs_t mmc_procfs_mmc0 = {0, 0};
static mmc_procfs_t mmc_procfs_mmc1 = {0, 0};

static void mmc_procfs_var_set(const char* host_name, struct device* dev, struct proc_dir_entry* root)
{
    if(!strncmp(host_name, "mmc0", sizeof("mmc0")))
    {   
        mmc_procfs_mmc0.dev = dev;
        mmc_procfs_mmc0.procfs_root = root;
    }
    else if(!strncmp(host_name, "mmc1", sizeof("mmc1")))
    {   
        mmc_procfs_mmc1.dev = dev;
        mmc_procfs_mmc1.procfs_root = root;
    }
    else if(NULL != host_name)
        pr_err("%s, %d: mmc_procfs_%s variable NOT declared\r\n", __func__, __LINE__, host_name);
}

static mmc_procfs_t* mmc_procfs_var_get(const char* host_name)
{   
    static mmc_procfs_t* mmc_procfs = NULL;

    if(!strncmp(host_name, "mmc0", sizeof("mmc0")))
        mmc_procfs = &mmc_procfs_mmc0;
    else if(!strncmp(host_name, "mmc1", sizeof("mmc1")))
        mmc_procfs = &mmc_procfs_mmc1;
    else
        mmc_procfs = NULL;
    
    return mmc_procfs;
}

/*******************************************************************************
 * Hisense Note:
 * mmc_add_host_debugfs(): create debug node through procfs, 
 * to replace debugfs if CONFIG_DEBUG_FS is not enabled in USER version.
 * *****************************************************************************/
void mmc_add_host_debugfs(struct mmc_host *host)
{
    mmc_procfs_t* mmc_procfs = NULL;

    if(NULL == host) {
        pr_err("[%s %d]: Invalid mmc_host pointer\r\n", __func__, __LINE__);
        return;
    }

    mmc_procfs = mmc_procfs_var_get(mmc_hostname(host));
	mmc_procfs->procfs_root = his_create_procfs_dir(mmc_hostname(host)); //node added under /proc/debug_control
	if (IS_ERR(mmc_procfs->procfs_root))
    {
        dev_err(&host->class_dev, "[%s, %d]:failed to his_create_procfs_dir\n", __func__, __LINE__);
		return;
    }
	if (!mmc_procfs->procfs_root)
		goto err_root;/* Complain -- procfs is enabled, but it failed to create the directory. */

    mmc_procfs->dev = &host->class_dev;

    if (!proc_create_data("ios", 0444, mmc_procfs->procfs_root, &mmc_ios_fops, host))
        goto err_node;

	if (!proc_create_data("err_stats", 0666, mmc_procfs->procfs_root, &mmc_err_stats_fops, host))
		goto err_node;

    /*add start: add device node for getting status of TFCard holder in factory mode */
    if(mmc_host_sd(host)) {
        if ( his_register_sysfs_attr(&dev_attr_sdcard_status.attr) < 0) {
            pr_err("Error creating sdcard_status sysfs node\n");
            goto err_node;
        }
    }
    /*add end*/

	return;

err_node:
	proc_remove(mmc_procfs->procfs_root);
	mmc_procfs_var_set(mmc_hostname(host), NULL, NULL);
err_root:
	dev_err(&host->class_dev, "failed to initialize procfs\n");
}

void mmc_remove_host_debugfs(struct mmc_host *host)
{
    mmc_procfs_t* mmc_procfs = mmc_procfs_var_get(mmc_hostname(host));
    if((NULL != mmc_procfs) && (NULL != mmc_procfs->dev) && (NULL != mmc_procfs->procfs_root))
        proc_remove(mmc_procfs->procfs_root);
    mmc_procfs_var_set(mmc_hostname(host), NULL, NULL); //clear procfs_root
}

