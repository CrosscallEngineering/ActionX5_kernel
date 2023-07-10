/**
 * The device control driver for Sunwave's fingerprint sensor.
 *
 * Copyright (C) 2016 Sunwave Corporation. <http://www.sunwavecorp.com>
 * Copyright (C) 2016 Langson L. <mailto: liangzh@sunwavecorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#if (defined CONFIG_FP_REPORT_WAKEUP_KEY) && (defined CONFIG_MACH_HISENSE_SMARTPHONE)
#include <linux/his_debug_base.h>
#endif /*(defined CONFIG_FP_REPORT_WAKEUP_KEY) && (defined CONFIG_MACH_HISENSE_SMARTPHONE)*/

#include "sf_ctl.h"

#if defined(CONFIG_FB_NOTUSE)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#else
#include <linux/msm_drm_notify.h>
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#if SF_BEANPOD_COMPATIBLE_V1
#include "nt_smc_call.h"
#endif

#if SF_INT_TRIG_HIGH
#include <linux/irq.h>
#endif

#ifdef CONFIG_RSEE
#include <linux/tee_drv.h>
#endif

#include <linux/productinfo.h>
#define SUNWAVEFP_PRODUCT_INFO "Sunwave SW9261"

#ifdef CONFIG_FINGERPRINT_VENDOR_CHECK
extern int get_fppin_val(void);
extern const char *fpname[];
#endif /*CONFIG_FINGERPRINT_VENDOR_CHECK*/


//---------------------------------------------------------------------------------
#define SF_DRV_VERSION "v2.2.10-2018-03-05"

#define MODULE_NAME "sunwave-sf_ctl"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME": "fmt, ##args)
#define SF_IRQ_NAME   "sf-irq"

//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
static long sf_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int sf_ctl_init_irq(void);
static int sf_ctl_init_input(void);
#ifdef CONFIG_COMPAT
static long sf_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif
#if SF_REE_PLATFORM
static ssize_t sf_ctl_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t sf_ctl_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
#endif
#if (SF_PROBE_ID_EN && !SF_RONGCARD_COMPATIBLE)
static int sf_read_sensor_id(void);
#endif

extern int sf_platform_init(struct sf_ctl_device *ctl_dev);
extern void sf_platform_exit(struct sf_ctl_device *ctl_dev);

//---------------------------------------------------------------------------------
#if MTK_6739_SPEED_MODE
static int sw_spi_max_speed = 1 * 1000 * 1000;
#endif

#ifdef CONFIG_RSEE
int rsee_client_get_fpid(int *vendor_id);
#endif
static struct file_operations sf_ctl_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = sf_ctl_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = sf_ctl_compat_ioctl,
#endif
#if SF_REE_PLATFORM
    .read           = sf_ctl_read,
    .write          = sf_ctl_write,
#endif
};

static struct sf_ctl_device sf_ctl_dev = {
    .miscdev = {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "sunwave_fp",
        .fops   = &sf_ctl_fops,
    },
    .rst_num = 0,
    .irq_pin = 0,
    .irq_num = 0,
};

static int sf_remove(sf_device_t *pdev);
static int sf_probe(sf_device_t *pdev);

#if SF_SPI_RW_EN
static struct of_device_id  sf_of_match[] = {
    { .compatible = COMPATIBLE_SW_FP, },
};

static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias = "sunwave-fp",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
    },
};

static int sf_ctl_spi_speed(unsigned int speed)
{
#if SF_MTK_CPU
#if MTK_6739_SPEED_MODE
    sw_spi_max_speed = speed;
#else
#define SPI_MODULE_CLOCK   (100*1000*1000)
    struct mt_chip_conf *config;
    unsigned int    time = SPI_MODULE_CLOCK / speed;
    config = (struct mt_chip_conf *)(sf_ctl_dev.pdev->controller_data);
    config->low_time = time / 2;
    config->high_time = time / 2;

    if (time % 2) {
        config->high_time = config->high_time + 1;
    }

#endif
#else
#if SF_REE_PLATFORM
    sf_ctl_dev.pdev->max_speed_hz = speed;
#endif
#endif
    return 0;
}
#else
#if (SF_PLATFORM_SEL == SF_TEE_QSEE)
static struct of_device_id  sf_of_match[] = {
#if 0
    { .compatible = "fp,fingerprint", },
#else
    { .compatible = "sunwave,fingerprint", },
#endif /*CONFIG_FINGERPRINT_VENDOR_CHECK*/
};
#endif
#endif

static sf_driver_t sf_driver = {
    .driver = {
        .name = "sunwave-fp",
#if SF_SPI_RW_EN
        .bus = &spi_bus_type,
#endif
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
#if SF_SPI_RW_EN
        .of_match_table = sf_of_match,
#else
#if (SF_PLATFORM_SEL == SF_TEE_QSEE)
		.of_match_table = sf_of_match,
#endif
#endif
#endif
    },
    .probe  = sf_probe,
    .remove = sf_remove,
};

//---------------------------------------------------------------------------------
#if SF_INT_TRIG_HIGH
static int sf_ctl_set_irq_type(unsigned long type)
{
    return irq_set_irq_type(sf_ctl_dev.irq_num, type | IRQF_NO_SUSPEND | IRQF_ONESHOT);
}
#endif
static void sf_ctl_device_event(struct work_struct *ws)
{
    char *uevent_env[2] = { SF_INT_EVENT_NAME, NULL };
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj,
                       KOBJ_CHANGE, uevent_env);

 #if (defined CONFIG_FP_REPORT_WAKEUP_KEY) && (defined CONFIG_MACH_HISENSE_SMARTPHONE)
     if (dev_bi.backlight_on == 0) {
         input_report_key(sf_ctl_dev.input, KEY_F18, 1);
         input_sync(sf_ctl_dev.input);
         msleep(10);

         input_report_key(sf_ctl_dev.input, KEY_F18, 0);
         input_sync(sf_ctl_dev.input);
         xprintk(KERN_ERR, "wxf: resume irq, report key event\n");
     }
 #endif /* (defined CONFIG_FP_REPORT_WAKEUP_KEY) && (defined CONFIG_MACH_HISENSE_SMARTPHONE) */

}

static irqreturn_t sf_ctl_device_irq(int irq, void *dev_id)
{
    disable_irq_nosync(irq);
    xprintk(KERN_INFO, "%s(irq = %d, ..) toggled.\n", __FUNCTION__, irq);
    schedule_work(&sf_ctl_dev.work_queue);

//    __pm_wakeup_event(&sf_ctl_dev.wakelock, msecs_to_jiffies(5000));

#if SF_INT_TRIG_HIGH
    sf_ctl_set_irq_type(IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#endif
    enable_irq(irq);
    return IRQ_HANDLED;
}

static int sf_ctl_report_key_event(struct input_dev *input, sf_key_event_t *kevent)
{
    int err = 0;
    unsigned int key_code = KEY_UNKNOWN;
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);

    switch (kevent->key) {
        case SF_KEY_HOME:
            key_code = KEY_HOME;
            break;

        case SF_KEY_MENU:
            key_code = KEY_MENU;
            break;

        case SF_KEY_BACK:
            key_code = KEY_BACK;
            break;

        case SF_KEY_F11:
            key_code = SF_NVKEY_CLICK;
            break;

        case SF_KEY_ENTER:
            key_code = SF_NVKEY_LONG_CLICK;
            break;

        case SF_KEY_UP:
            key_code = SF_NVKEY_MOVE_UP;
            break;

        case SF_KEY_LEFT:
            key_code = SF_NVKEY_MOVE_RIGHT;
            break;

        case SF_KEY_RIGHT:
            key_code = SF_NVKEY_MOVE_LEFT;
            break;

        case SF_KEY_DOWN:
            key_code = SF_NVKEY_MOVE_DOWN;
            break;

        case SF_KEY_WAKEUP:
            key_code = KEY_WAKEUP;
            break;

        default:
            break;
    }
	xprintk(SF_LOG_LEVEL, "%s(..), key_code = %d, kevent->value = %d.\n", input->name, key_code, kevent->value);

    input_report_key(input, key_code, kevent->value);
    input_sync(input);
    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static const char *sf_ctl_get_version(void)
{
    static char version[SF_DRV_VERSION_LEN] = {'\0', };
    strncpy(version, SF_DRV_VERSION, SF_DRV_VERSION_LEN);
    version[SF_DRV_VERSION_LEN - 1] = '\0';
    return (const char *)version;
}

////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.

static long sf_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    sf_key_event_t kevent;
    xprintk(SF_LOG_LEVEL, "%s(_IO(type,nr) nr= 0x%08x, ..)\n", __FUNCTION__, _IOC_NR(cmd));

    switch (cmd) {
        case SF_IOC_INIT_DRIVER: {
#if MULTI_HAL_COMPATIBLE
            sf_ctl_dev.gpio_init(&sf_ctl_dev);
#endif
            break;
        }

        case SF_IOC_DEINIT_DRIVER: {
#if MULTI_HAL_COMPATIBLE
            sf_ctl_dev.free_gpio(&sf_ctl_dev);
#endif
            break;
        }

        case SF_IOC_RESET_DEVICE: {
            sf_ctl_dev.reset();
            break;
        }

        case SF_IOC_ENABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_DISABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_REQUEST_IRQ: {
#if MULTI_HAL_COMPATIBLE
            sf_ctl_init_irq();
#endif
            break;
        }

        case SF_IOC_ENABLE_SPI_CLK: {
            sf_ctl_dev.spi_clk_on(true);
            break;
        }

        case SF_IOC_DISABLE_SPI_CLK: {
            sf_ctl_dev.spi_clk_on(false);
            break;
        }

        case SF_IOC_ENABLE_POWER: {
            sf_ctl_dev.power_on(true);
            break;
        }

        case SF_IOC_DISABLE_POWER: {
            sf_ctl_dev.power_on(false);
            break;
        }

        case SF_IOC_REPORT_KEY_EVENT: {
            if (copy_from_user(&kevent, (sf_key_event_t *)arg, sizeof(sf_key_event_t))) {
                xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            err = sf_ctl_report_key_event(sf_ctl_dev.input, &kevent);
            break;
        }

        case SF_IOC_SYNC_CONFIG: {
            // TODO:
            break;
        }

        case SF_IOC_SPI_SPEED: {
#if SF_SPI_RW_EN
            sf_ctl_spi_speed(arg);
#endif
            break;
        }

        case SF_IOC_ATTRIBUTE: {
            err = __put_user(sf_ctl_dev.attribute, (__u32 __user *)arg);
            break;
        }

        case SF_IOC_GET_VERSION: {
            if (copy_to_user((void *)arg, sf_ctl_get_version(), SF_DRV_VERSION_LEN)) {
                xprintk(KERN_ERR, "copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        default:
            err = (-EINVAL);
            break;
    }

    return err;
}

#ifdef CONFIG_COMPAT
static long sf_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return sf_ctl_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void sf_early_suspend(struct early_suspend *handler)
{
    char *screen[2] = { "SCREEN_STATUS=OFF", NULL };
    kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen);
#if SF_INT_TRIG_HIGH
    sf_ctl_set_irq_type(IRQF_TRIGGER_HIGH);
#endif
}

static void sf_late_resume(struct early_suspend *handler)
{
    char *screen[2] = { "SCREEN_STATUS=ON", NULL };
    kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen);
#if SF_INT_TRIG_HIGH
    sf_ctl_set_irq_type(IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#endif
}

#elif defined(CONFIG_FB_NOTUSE)

static int sf_fb_notifier_callback(struct notifier_block *self,
                                   unsigned long event, void *data)
{
    static char screen_status[64] = {'\0'};
    char *screen_env[2] = { screen_status, NULL };
    struct fb_event *evdata = data;
    unsigned int blank;
    int retval = 0;

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    blank = *(int *)evdata->data;

    switch (blank) {
        case FB_BLANK_UNBLANK:
            sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
            kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
#if SF_INT_TRIG_HIGH
            sf_ctl_set_irq_type(IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#endif
            break;

        case FB_BLANK_POWERDOWN:
            sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
            kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
#if SF_INT_TRIG_HIGH
            sf_ctl_set_irq_type(IRQF_TRIGGER_HIGH);
#endif
            break;

        default:
            break;
    }

    return retval;
}
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
static struct drm_panel *active_panel;

static int drm_check_dt(struct device_node *np)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if (count <= 0) {
        xprintk(KERN_ERR, "sf_fp find drm_panel count(%d) fail", count);
        return -ENODEV;
    }

    for (i = 0; i < count; i++) {
        node = of_parse_phandle(np, "panel", i);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) {
            xprintk(KERN_ERR, "sf_fp find drm_panel successfully");
            active_panel = panel;
            return 0;
        }
    }

    xprintk(KERN_ERR, "sf_fp no find drm_panel");
    return -ENODEV;
}

static int sf_drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
	static char screen_status[64] = {'\0'};
    char *screen_env[2] = { screen_status, NULL };
    struct drm_panel_notifier *evdata = data;
    int *blank = NULL;

	xprintk(KERN_ERR, "----------sf_drm_notifier_callback----------");
    if (!evdata) {
        xprintk(KERN_ERR, "sf_fp evdata is null");
        return 0;
    }

    if (!((event == DRM_PANEL_EARLY_EVENT_BLANK )
          || (event == DRM_PANEL_EVENT_BLANK))) {
        xprintk(KERN_ERR, "sf_fp event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    xprintk(KERN_ERR, "sf_fp DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
        case DRM_PANEL_BLANK_UNBLANK:
            sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
            kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
#if SF_INT_TRIG_HIGH
            sf_ctl_set_irq_type(IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#endif
            break;
        case DRM_PANEL_BLANK_POWERDOWN:
            sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
            kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
#if SF_INT_TRIG_HIGH
            sf_ctl_set_irq_type(IRQF_TRIGGER_HIGH);
#endif
            break;
        default:
            xprintk(KERN_ERR, "sf_fp DRM BLANK(%d) do not need process\n", *blank);
            break;
    }

    return 0;
}

static void drm_checkdt_delay_work(struct work_struct *work)
{
	int ret = 0;

	if (!sf_ctl_dev.pdev->dev.of_node) {
        xprintk(KERN_ERR, "sf_probe of_find_compatible_node(..) failed.\n");
        return;
    }
    ret = drm_check_dt(sf_ctl_dev.pdev->dev.of_node);
    if (ret) {
        xprintk(KERN_ERR, "sf_probe parse drm-panel fail");
    }

	if (active_panel) {
        ret = drm_panel_notifier_register(active_panel, &sf_ctl_dev.notifier);
        if (ret)
            xprintk(KERN_ERR, "[DRM]sf_fp drm_panel_notifier_register fail: %d\n", ret);
    }
}
#else
static int sf_drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
	static char screen_status[64] = {'\0'};
    char *screen_env[2] = { screen_status, NULL };
    struct drm_panel_notifier *evdata = data;
    int *blank = NULL;

	xprintk(KERN_ERR, "-------===sf_drm_notifier_callback===-------");
    if (!evdata) {
        xprintk(KERN_ERR, "evdata is null");
        return 0;
    }

    if (!((event == MSM_DRM_EARLY_EVENT_BLANK )
          || (event == MSM_DRM_EVENT_BLANK))) {
        xprintk(KERN_ERR, "event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    xprintk(KERN_ERR, "DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
        case MSM_DRM_BLANK_UNBLANK:
            sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
            kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
#if SF_INT_TRIG_HIGH
            sf_ctl_set_irq_type(IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#endif
            break;
        case MSM_DRM_BLANK_POWERDOWN:
            sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
            kobject_uevent_env(&sf_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
#if SF_INT_TRIG_HIGH
            sf_ctl_set_irq_type(IRQF_TRIGGER_HIGH);
#endif
            break;
        default:
            xprintk(KERN_ERR, "sf DRM BLANK(%d) do not need process\n", *blank);
            break;
    }

    return 0;
}
#endif
#endif //SF_CFG_HAS_EARLYSUSPEND
////////////////////////////////////////////////////////////////////////////////
static int sf_remove(sf_device_t *spi)
{
    int err = 0;
    return err;
}

static int sf_probe(sf_device_t *dev)
{
    int err = 0;
#ifdef CONFIG_FINGERPRINT_VENDOR_CHECK
	int fpvendor_v = 0;
#endif /*CONFIG_FINGERPRINT_VENDOR_CHECK*/
    sf_ctl_dev.pdev = dev;
    xprintk(KERN_INFO, "sunwave %s enter\n", __FUNCTION__);

#ifdef CONFIG_FINGERPRINT_VENDOR_CHECK
	fpvendor_v = get_fppin_val();
        if (strcmp(fpname[fpvendor_v],"sunwave")) {
		pr_err("fp vendor is not sunwave, v = %d\n", fpvendor_v);
		return -ENODEV;
	}
#endif /*CONFIG_FINGERPRINT_VENDOR_CHECK*/

    /* Initialize the platform config. */
    err = sf_platform_init(&sf_ctl_dev);

    if (err) {
        xprintk(KERN_ERR, "sf_platform_init failed with %d.\n", err);
        return err;
    }


//    wakeup_source_init(&sf_ctl_dev.wakelock, "sf_wakelock");

    /* Initialize the GPIO pins. */
#if MULTI_HAL_COMPATIBLE
    xprintk(KERN_INFO, " do not initialize the GPIO pins. \n");
#else
    err = sf_ctl_dev.gpio_init(&sf_ctl_dev);

    if (err) {
        xprintk(KERN_ERR, "gpio_init failed with %d.\n", err);
        sf_ctl_dev.free_gpio(&sf_ctl_dev);
        sf_platform_exit(&sf_ctl_dev);
        return err;
    }

//    sf_ctl_dev.reset();
#endif
#if SF_PROBE_ID_EN
#if SF_BEANPOD_COMPATIBLE_V2
    err = get_fp_spi_enable();

    if (err != 1) {
        xprintk(KERN_ERR, "get_fp_spi_enable ret=%d\n", err);
        sf_ctl_dev.free_gpio(&sf_ctl_dev);
        sf_platform_exit(&sf_ctl_dev);
        return -1;
    }

#endif
#if SF_RONGCARD_COMPATIBLE
#ifdef CONFIG_RSEE
    uint64_t vendor_id = 0x00;
    sf_ctl_dev.spi_clk_on(true);
    err = rsee_client_get_fpid(&vendor_id);
    sf_ctl_dev.spi_clk_on(false);
    xprintk(KERN_INFO, "rsee_client_get_fpid vendor id is 0x%x\n", vendor_id);

    if (err || !((vendor_id >> 8) == 0x82)) {
        xprintk(KERN_ERR, "rsee_client_get_fpid failed !\n");
        err = -1;
    }

#else
    err = -1;
    xprintk(KERN_INFO, "CONFIG_RSEE not define, skip rsee_client_get_fpid!\n");
#endif
#else
    sf_ctl_dev.spi_clk_on(true);
    err = sf_read_sensor_id();
    sf_ctl_dev.spi_clk_on(false);
#endif

    if (err < 0) {
        xprintk(KERN_ERR, "sunwave probe read chip id is failed\n");
        sf_ctl_dev.free_gpio(&sf_ctl_dev);
        sf_platform_exit(&sf_ctl_dev);
        return -1;
    }

#if SF_BEANPOD_COMPATIBLE_V2
    set_fp_vendor(FP_VENDOR_SUNWAVE);
#endif
#endif
    /* reset spi dma mode. */
#if (SF_REE_PLATFORM && SF_MTK_CPU)
    {
        struct mt_chip_conf *config = (struct mt_chip_conf *)(sf_ctl_dev.pdev->controller_data);
        config->cpol = 0;
        config->cpha = 0;
        config->com_mod = DMA_TRANSFER;
    }
#endif
    /* Initialize the input subsystem. */
    err = sf_ctl_init_input();

    if (err) {
        sf_ctl_dev.free_gpio(&sf_ctl_dev);
        sf_platform_exit(&sf_ctl_dev);
        xprintk(KERN_ERR, "sf_ctl_init_input failed with %d.\n", err);
        return err;
    }

    /* Register as a miscellaneous device. */
    err = misc_register(&sf_ctl_dev.miscdev);

    if (err) {
        sf_ctl_dev.free_gpio(&sf_ctl_dev);
        sf_platform_exit(&sf_ctl_dev);
        xprintk(KERN_ERR, "misc_register(..) = %d.\n", err);
        input_unregister_device(sf_ctl_dev.input);
        return err;
    }

    /* Initialize the interrupt callback. */
    INIT_WORK(&sf_ctl_dev.work_queue, sf_ctl_device_event);

#if MULTI_HAL_COMPATIBLE
    xprintk(KERN_INFO, " do not initialize the fingerprint interrupt. \n");
#else
    err = sf_ctl_init_irq();

    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_irq failed with %d.\n", err);
        sf_ctl_dev.free_gpio(&sf_ctl_dev);
        sf_platform_exit(&sf_ctl_dev);
        return err;
    }

#endif

/*#if defined(CONFIG_FB_NOTUSE)
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
    if (!sf_ctl_dev.pdev->dev.of_node) {
        xprintk(KERN_ERR, "sf_probe of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }
    err = drm_check_dt(sf_ctl_dev.pdev->dev.of_node);
    if (err) {
        xprintk(KERN_ERR, "sf_probe parse drm-panel fail");
    }
#endif
#endif*/

#if defined(CONFIG_HAS_EARLYSUSPEND)
    sf_ctl_dev.early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
    sf_ctl_dev.early_suspend.suspend = sf_early_suspend;
    sf_ctl_dev.early_suspend.resume = sf_late_resume;
    register_early_suspend(&sf_ctl_dev.early_suspend);
#elif defined(CONFIG_FB_NOTUSE)
    sf_ctl_dev.notifier.notifier_call = sf_fb_notifier_callback;
    fb_register_client(&sf_ctl_dev.notifier);
#elif defined(CONFIG_DRM)
    sf_ctl_dev.notifier.notifier_call = sf_drm_notifier_callback;
#if defined(CONFIG_DRM_PANEL)
/*    if (active_panel) {
        err = drm_panel_notifier_register(active_panel, &sf_ctl_dev.notifier);
        if (err)
            xprintk(KERN_ERR, "[DRM]drm_panel_notifier_register fail: %d\n", err);
    }*/
	sf_ctl_dev.checkdrm_workqueue = create_singlethread_workqueue("sf_drmcheck_wq");
	INIT_DELAYED_WORK(&sf_ctl_dev.checkdrm_delay_work, drm_checkdt_delay_work);
	queue_delayed_work(sf_ctl_dev.checkdrm_workqueue, &sf_ctl_dev.checkdrm_delay_work,
							usecs_to_jiffies(5000000));
#else
    err = msm_drm_register_client(&sf_ctl_dev.notifier);
    if (err) {
        xprintk(KERN_ERR, "[DRM]Unable to register fb_notifier: %d\n", err);
    }
#endif
#endif

	productinfo_register(PRODUCTINFO_FINGERPRINT_ID, SUNWAVEFP_PRODUCT_INFO, NULL);
	printk("%s: product info add\n", __func__);

    xprintk(KERN_ERR, "%s leave err = %d\n", __FUNCTION__, err);
    return err;
}

#if (SF_BEANPOD_COMPATIBLE_V2 || SF_REE_PLATFORM)
static int tee_spi_transfer(void *smt_conf, int cfg_len, const char *txbuf, char *rxbuf, int len)
{
    struct spi_transfer t;
    struct spi_message m;
    memset(&t, 0, sizeof(t));
#if SF_MTK_CPU
#if MTK_6739_SPEED_MODE
    t.speed_hz = sw_spi_max_speed;
#else
    sf_ctl_dev.pdev->controller_data = smt_conf;
#endif
#endif
    spi_message_init(&m);
    t.tx_buf = txbuf;
    t.rx_buf = rxbuf;
    t.bits_per_word = 8;
    t.len = len;
    spi_message_add_tail(&t, &m);
    return spi_sync(sf_ctl_dev.pdev, &m);
}
#endif

#if SF_REE_PLATFORM
static ssize_t sf_ctl_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    const size_t bufsiz = 25 * 1024;
    ssize_t status = 0;

    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz) {
        return (-EMSGSIZE);
    }

    if (sf_ctl_dev.spi_buffer == NULL) {
        sf_ctl_dev.spi_buffer = kmalloc(bufsiz, GFP_KERNEL);

        if (sf_ctl_dev.spi_buffer == NULL) {
            xprintk(KERN_ERR, " %s malloc spi_buffer failed.\n", __FUNCTION__);
            return (-ENOMEM);
        }
    }

    memset(sf_ctl_dev.spi_buffer, 0, bufsiz);

    if (copy_from_user(sf_ctl_dev.spi_buffer, buf, count)) {
        xprintk(KERN_ERR, "%s copy_from_user(..) failed.\n", __FUNCTION__);
        return (-EMSGSIZE);
    }

    {
#if SF_MTK_CPU
        struct mt_chip_conf *smt_conf = (struct mt_chip_conf *) (sf_ctl_dev.pdev->controller_data);
        int cfg_len = sizeof(struct mt_chip_conf);
#else
        /* not used */
        void *smt_conf;
        int cfg_len = 0;
#endif
        status = tee_spi_transfer(smt_conf, cfg_len, sf_ctl_dev.spi_buffer, sf_ctl_dev.spi_buffer, count);
    }

    if (status == 0) {
        status = copy_to_user(buf, sf_ctl_dev.spi_buffer, count);

        if (status != 0) {
            status = -EFAULT;
        }
        else {
            status = count;
        }
    }
    else {
        xprintk(KERN_ERR, " %s spi_transfer failed.\n", __FUNCTION__);
    }

    return status;
}

static ssize_t sf_ctl_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    const size_t bufsiz = 25 * 1024;
    ssize_t status = 0;

    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz) {
        return (-EMSGSIZE);
    }

    if (sf_ctl_dev.spi_buffer == NULL) {
        sf_ctl_dev.spi_buffer = kmalloc(bufsiz, GFP_KERNEL);

        if (sf_ctl_dev.spi_buffer == NULL) {
            xprintk(KERN_ERR, " %s malloc spi_buffer failed.\n", __FUNCTION__);
            return (-ENOMEM);
        }
    }

    memset(sf_ctl_dev.spi_buffer, 0, bufsiz);

    if (copy_from_user(sf_ctl_dev.spi_buffer, buf, count)) {
        xprintk(KERN_ERR, "%s copy_from_user(..) failed.\n", __FUNCTION__);
        return (-EMSGSIZE);
    }

    {
#if SF_MTK_CPU
        struct mt_chip_conf *smt_conf = (struct mt_chip_conf *) (sf_ctl_dev.pdev->controller_data);
        int cfg_len = sizeof(struct mt_chip_conf);
#else
        /* not used */
        void *smt_conf;
        int cfg_len = 0;
#endif
        status = tee_spi_transfer(smt_conf, cfg_len, sf_ctl_dev.spi_buffer, sf_ctl_dev.spi_buffer, count);
    }

    if (status == 0) {
        status = count;
    }
    else {
        xprintk(KERN_ERR, " %s spi_transfer failed.\n", __FUNCTION__);
    }

    return status;
}
#endif

#if (SF_PROBE_ID_EN && !SF_RONGCARD_COMPATIBLE)
static int sf_read_sensor_id(void)
{
    int ret = -1;
    int trytimes = 3;
    char readbuf[16]  = {0};
    char writebuf[16] = {0};
    //默认速度设置为1M, 不然8201/8211系列有可能读不到ID
#if SF_MTK_CPU
    static struct mt_chip_conf smt_conf = {
        .setuptime = 15,
        .holdtime = 15,
        .high_time = 60, // 10--6m 15--4m 20--3m 30--2m [ 60--1m 120--0.5m  300--0.2m]
        .low_time  = 60,
        .cs_idletime = 20,
        .ulthgh_thrsh = 0,
        .cpol = 0,
        .cpha = 0,
        .rx_mlsb = SPI_MSB,
        .tx_mlsb = SPI_MSB,
        .tx_endian = 0,
        .rx_endian = 0,
        .com_mod = FIFO_TRANSFER,
        .pause = 0,
        .finish_intr = 1,
        .deassert = 0,
        .ulthigh = 0,
        .tckdly = 0,
    };
    int cfg_len = sizeof(struct mt_chip_conf);
#if MTK_6739_SPEED_MODE
    sf_ctl_dev.pdev->max_speed_hz = 1 * 1000 * 1000;
    sf_ctl_dev.pdev->bits_per_word = 8;
    sf_ctl_dev.pdev->mode = SPI_MODE_0;
#endif
#else
    /* not used */
    int smt_conf;
    int cfg_len = 0;
    sf_ctl_dev.pdev->max_speed_hz = 1 * 1000 * 1000;
    sf_ctl_dev.pdev->bits_per_word = 8;
    sf_ctl_dev.pdev->mode = SPI_MODE_0;
#endif
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    msleep(10);

    do {
        /* 1.detect 8205, 8231, 8241 or 8271 */
        memset(readbuf,  0, sizeof(readbuf));
        memset(writebuf, 0, sizeof(writebuf));
        writebuf[0] = 0xA0;
        writebuf[1] = (uint8_t)(~0xA0);
        ret =  tee_spi_transfer(&smt_conf, cfg_len, writebuf, readbuf, 6);

        if (ret != 0) {
            xprintk(KERN_ERR, "SPI transfer failed\n");
            continue;
        }

        if ((0x53 == readbuf[2]) && (0x75 == readbuf[3]) && (0x6e == readbuf[4])
            && (0x57 == readbuf[5])) {
            xprintk(KERN_INFO, "read chip is ok\n");
            return 0;
        }

        /* 2.detect 8202, 8205 or 8231 */
        memset(readbuf,  0, sizeof(readbuf));
        memset(writebuf, 0, sizeof(writebuf));
        writebuf[0] = 0x60;
        writebuf[1] = (uint8_t)(~0x60);
        writebuf[2] = 0x28;
        writebuf[3] = 0x02;
        writebuf[4] = 0x00;
        ret =  tee_spi_transfer(&smt_conf, cfg_len, writebuf, readbuf, 7);

        if (ret != 0) {
            xprintk(KERN_ERR, "SPI transfer failed\n");
            continue;
        }

        if (readbuf[5] == 0x82) {
            xprintk(KERN_INFO, "read chip is ok\n");
            return 0;
        }

        /* 3.detect 8221 */
        memset(readbuf,  0, sizeof(readbuf));
        memset(writebuf, 0, sizeof(writebuf));
        writebuf[0] = 0x60;
        writebuf[1] = 0x28;
        writebuf[2] = 0x02;
        writebuf[3] = 0x00;
        ret =  tee_spi_transfer(&smt_conf, cfg_len, writebuf, readbuf, 6);

        if (ret != 0) {
            xprintk(KERN_ERR, "SPI transfer failed\n");
            continue;
        }

        if (readbuf[4] == 0x82) {
            xprintk(KERN_INFO, "read chip is ok\n");
            return 0;
        }

#if SF_REE_PLATFORM
        /* 4.detect 8201 or 8211 */
        {
            /* reset脚拉高后，需等 200ms 后方可读ID */
            msleep(200);
            memset(readbuf,  0, sizeof(readbuf));
            memset(writebuf, 0, sizeof(writebuf));
            writebuf[0] = 0x1c;
            writebuf[1] = 0x1c;
            writebuf[2] = 0x1c;
            ret =  tee_spi_transfer(&smt_conf, cfg_len, writebuf, readbuf, 3);

            if (ret != 0) {
                xprintk(KERN_ERR, "SPI transfer failed\n");
                continue;
            }

            msleep(5);
            memset(readbuf,  0, sizeof(readbuf));
            memset(writebuf, 0, sizeof(writebuf));
            writebuf[0] = 0x96;
            writebuf[1] = 0x69;
            writebuf[2] = 0x00;
            writebuf[3] = 0x00;
            writebuf[4] = 0x1e;
            writebuf[5] = 0x00;
            writebuf[6] = 0x02;
            writebuf[7] = 0x00;
            ret =  tee_spi_transfer(&smt_conf, cfg_len, writebuf, readbuf, 10);

            if (ret != 0) {
                xprintk(KERN_ERR, "SPI transfer failed\n");
                continue;
            }

            if ((readbuf[8] == 0xfa) || (readbuf[9] == 0xfa)) {
                xprintk(KERN_INFO, "read chip is ok\n");
                return 0;
            }
        }
#endif
    }
    while (trytimes--);

    return -1;
}

#endif

////////////////////////////////////////////////////////////////////////////////
static int sf_ctl_init_irq(void)
{
    int err = 0;
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    /* Register interrupt callback. */
    err = request_irq(sf_ctl_dev.irq_num, sf_ctl_device_irq,
                      IRQF_TRIGGER_FALLING | IRQF_ONESHOT, SF_IRQ_NAME, NULL);

    if (err) {
        xprintk(KERN_ERR, "request_irq(..) = %d.\n", err);
    }

    enable_irq_wake(sf_ctl_dev.irq_num);
    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int sf_ctl_init_input(void)
{
    int err = 0;
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    sf_ctl_dev.input = input_allocate_device();

    if (!sf_ctl_dev.input) {
        xprintk(KERN_ERR, "input_allocate_device(..) failed.\n");
        return (-ENOMEM);
    }

    sf_ctl_dev.input->name = "sf-keys";
    __set_bit(EV_KEY  ,  			sf_ctl_dev.input->evbit );
    __set_bit(KEY_HOME,   			sf_ctl_dev.input->keybit);
    __set_bit(KEY_MENU,   			sf_ctl_dev.input->keybit);
    __set_bit(KEY_BACK,   			sf_ctl_dev.input->keybit);
    __set_bit(SF_NVKEY_CLICK,    	sf_ctl_dev.input->keybit);
    __set_bit(SF_NVKEY_LONG_CLICK,  sf_ctl_dev.input->keybit);
    __set_bit(SF_NVKEY_MOVE_LEFT,	sf_ctl_dev.input->keybit);
    __set_bit(SF_NVKEY_MOVE_UP,   	sf_ctl_dev.input->keybit);
    __set_bit(SF_NVKEY_MOVE_DOWN,  	sf_ctl_dev.input->keybit);
    __set_bit(SF_NVKEY_MOVE_RIGHT,  sf_ctl_dev.input->keybit);
    __set_bit(KEY_WAKEUP, 			sf_ctl_dev.input->keybit);
    err = input_register_device(sf_ctl_dev.input);

    if (err) {
        xprintk(KERN_ERR, "input_register_device(..) = %d.\n", err);
        input_free_device(sf_ctl_dev.input);
        sf_ctl_dev.input = NULL;
        return (-ENODEV);
    }

    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int __init sf_ctl_driver_init(void)
{
    int err = 0;
    xprintk(KERN_INFO, "'%s' SW_BUS_NAME = %s\n", __FUNCTION__, SW_BUS_NAME);
#if SF_BEANPOD_COMPATIBLE_V1
    uint64_t fp_vendor_id = 0x00;
    get_t_device_id(&fp_vendor_id);
    xprintk(KERN_INFO, "'%s' fp_vendor_id = 0x%x\n", __FUNCTION__, fp_vendor_id);

    if (fp_vendor_id != 0x02) {
        return 0;
    }

#endif
#if SF_SPI_RW_EN
    /**register SPI device、driver***/
    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
    err = spi_register_driver(&sf_driver);

    if (err < 0) {
        xprintk(KERN_ERR, "%s, Failed to register SPI driver.\n", __FUNCTION__);
    }

#else

    err = platform_driver_register(&sf_driver);

    if (err) {
        xprintk(KERN_ERR, "%s, Failed to register platform driver.\n", __FUNCTION__);
        return -EINVAL;
    }

#endif
    xprintk(KERN_INFO, "sunwave fingerprint device control driver registered.\n");
    xprintk(KERN_INFO, "driver version: '%s'.\n", sf_ctl_get_version());
    return err;
}

static void __exit sf_ctl_driver_exit(void)
{
    if (sf_ctl_dev.input) {
        input_unregister_device(sf_ctl_dev.input);
    }

    if (sf_ctl_dev.irq_num >= 0) {
        free_irq(sf_ctl_dev.irq_num, (void *)&sf_ctl_dev);
    }

    misc_deregister(&sf_ctl_dev.miscdev);
#if SF_SPI_RW_EN
    spi_unregister_driver(&sf_driver);
#else
    platform_driver_unregister(&sf_driver);
#endif

//    wakeup_source_trash(&sf_ctl_dev.wakelock);

    sf_ctl_dev.free_gpio(&sf_ctl_dev);
    sf_platform_exit(&sf_ctl_dev);
    xprintk(KERN_INFO, "sunwave fingerprint device control driver released.\n");
}

module_init(sf_ctl_driver_init);
module_exit(sf_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for Sunwave's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Langson L. <liangzh@sunwavecorp.com>");

