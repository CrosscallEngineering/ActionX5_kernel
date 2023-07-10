#include "sitronix_ts_custom_func.h"
#include "sitronix_ts.h"
#include <linux/kthread.h>

#define DELAY_MONITOR_THREAD_START_PROBE 300

#define ST_ST1802_RAWTYPE_RAW 0x06
#define ST_ST1802_RAWTYPE_HEADER 0x09
//#define ST_ST1802_DIST_LENGTH (4 + 2 * 13)
//#define ST_ST1802_READ_COUNT 20

static struct task_struct *SitronixMonitorThread = NULL;
static int gMonitorThreadSleepInterval = 300; // 0.3 sec
static atomic_t iMonitorThreadPostpone = ATOMIC_INIT(0);
static int sitronix_ts_delay_monitor_thread_start = DELAY_MONITOR_THREAD_START_PROBE;
static int StatusDistErrCount = 0;

static int get_dist_value(void)
{
	signed short disv;
	unsigned char disbuf[128] = {0};
	int i;
	int index;
	stx_i2c_read(stx_gpts.client, disbuf, (stx_gpts.ts_dev_info.tx_chs * 2 + 4), 0x40);
	STX_DEBUG("%s disbuf[0] = 0x%x in ", __func__, disbuf[0]);

	if (disbuf[0] == ST_ST1802_RAWTYPE_RAW)
	{
		index = disbuf[2];
		if(index != 0)
		{
			for (i = 0; i < stx_gpts.ts_dev_info.tx_chs; i++)
			{
				disv = (signed short)((disbuf[4 + 2 * i]) * 0x100 + disbuf[5 + 2 * i]);
				if(disv >= 0x8000)
					disv = disv - 0xFFFF;
				if ((disv > 700) || (disv < (-700)))
				{
					StatusDistErrCount++;
					STX_ERROR("%s StatusDistErrCount = %d ", __func__, StatusDistErrCount);
					STX_ERROR("disv = %d,rx= %d,i = %d ",disv,index,i);
					break;
				}
				StatusDistErrCount = 0;
			}
		}
		if (StatusDistErrCount >= 3)
		{
			StatusDistErrCount = 0;
			return -1;
		}
	}
	else if (disbuf[0] == ST_ST1802_RAWTYPE_HEADER)
	{
		if (disbuf[8] & 0x10)
		{
			STX_ERROR("%s disbuf %x %x ", __func__, disbuf[8], (disbuf[8] & 0x10));
			return -1;
		}
		else
			return 0;
	}
	return 0;
}

static int sitronix_ts_monitor_thread(void *data)
{
	int ret = 0;
	uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int result = 1;
	int count = 0;
	int pre_count = 0;
	static int i2cErrorCount = 0;

	STX_DEBUG("delay %d ms", sitronix_ts_delay_monitor_thread_start);
	msleep(sitronix_ts_delay_monitor_thread_start);
	while (!kthread_should_stop())
	{
		if (atomic_read(&iMonitorThreadPostpone))
		{
			atomic_set(&iMonitorThreadPostpone, 0);
		}
		else if (stx_gpts.suspend_state || stx_gpts.is_upgrading || stx_gpts.is_testing || stx_gpts.fapp_in)
		{
			STX_ERROR("%s suspend state = %d ,is_upgrading = %d ,is_testing = %d,fapp_in=%d ", __func__, stx_gpts.suspend_state, stx_gpts.is_upgrading, stx_gpts.is_testing,stx_gpts.fapp_in);
		}
		else
		{
			result = 1;
			ret = stx_i2c_read(stx_gpts.client, buffer, 8, 0X01);
			if (ret < 0)
			{
				STX_ERROR("I2C read 0x01 Error ");
				goto exit_i2c_invalid;
			}
			if ((buffer[0] & 0x0F) == 0x06)
			{
				result = -1;
				STX_ERROR("%s IC status is buf0 = 0x%x ", __func__, buffer[0]);
				goto exit_i2c_invalid;
			}
			else if ((buffer[0] != 0xFF) && (buffer[2] == 0xFE))
			{
//				kthread_stop(SitronixMonitorThread);
				STX_ERROR("%s:", "Sitronix_ts_monitoring Idle time 0xFE");
				SitronixMonitorThread = NULL;
				result = 0;
				return 0;
			}
			else
			{
				count = buffer[6] * 0x100 + buffer[7];
				/*如果数值一样，记录错误一次*/
				if (count != pre_count)
				{
					pre_count = count;
					result = 0;
				}else{
					STX_ERROR("sensing ERROR ");
					goto exit_i2c_invalid;
				}
				result = get_dist_value();
			}
		exit_i2c_invalid:
			if ((result == 1) || (result == -1))
			{
				i2cErrorCount++;
				if ((2 <= i2cErrorCount) || (result == -1))
				{
					STX_ERROR("I2C abnormal, reset it!");
					st_reset_ic();
//#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GESTURE
//					sitronix_swk_enable();
//					st_power_down(&stx_gpts);
//#endif
					//设置手套，皮套模式
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_GLOVE
					if (stx_gpts.glove_mode)
						st_enter_glove_mode(&stx_gpts);
#endif
					i2cErrorCount = 0;
				}
			}
			else
			{
				i2cErrorCount = 0;
			}
		}
		msleep(gMonitorThreadSleepInterval);
	}
	STX_FUNC_EXIT();
	return 0;
}

int sitronix_monitor_start(void)
{
	STX_INFO("%s ENTER ",__func__);
	//== Add thread to monitor chip
	atomic_set(&iMonitorThreadPostpone, 1);
	if(NULL == SitronixMonitorThread)
	{
		SitronixMonitorThread = kthread_run(sitronix_ts_monitor_thread, "Sitronix", "Monitorthread");
		if (IS_ERR(SitronixMonitorThread))
		{
			SitronixMonitorThread = NULL;
			STX_ERROR("sitronix_monitor_start err");
			return 1;
		}
		STX_INFO("%s success ",__func__);
	}else{
		STX_ERROR("%s already start ",__func__);
	}
	return 0;
}

int sitronix_monitor_stop(void)
{
	STX_INFO("%s ENTER ",__func__);
	if (SitronixMonitorThread)
	{
		kthread_stop(SitronixMonitorThread);
		SitronixMonitorThread = NULL;
		STX_INFO("%s success ",__func__);
	}
	else
	{
		STX_ERROR("sitronix_monitor_stop already stop");
	}
	return 0;
}

int sitronix_monitor_delay(void)
{
	atomic_set(&iMonitorThreadPostpone, 1);

	return 0;
}