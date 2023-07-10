#include "sitronix_ts.h"
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_L762
#include "sitronix_ts_short_test_cfg_l752.h"
#else
#include "sitronix_ts_short_test_cfg.h"
#endif

/* #define ST_TEST_RAW */

#ifdef ST_TEST_RAW

#define ST_RAW_LOG_PATH "/data/httu/ST_RAW_LOG.txt"
#define MAX_RX_COUNT 32
#define MAX_TX_COUNT 16
#define MAX_KEY_COUNT 4
#define MAX_SENSOR_COUNT (MAX_RX_COUNT + MAX_TX_COUNT + MAX_KEY_COUNT)
#define MAX_RAW_LIMIT 10000
#define MIN_RAW_LIMIT 1000
/* #define ST_RAWTEST_LOGFILE */
//#define ST_SHORT_TEST_IS_1T2R		1
#define ST_SHORT_TEST_CFG_OFFSET 0xEFC0

#endif /* ST_TEST_RAW */

#ifdef ST_TEST_RAW
static st_int st_drv_Get_2D_Length(st_int tMode[])
{
	if (tMode[0] == 0)
		return tMode[1];
	else
		return tMode[2];
}
static st_int st_drv_Get_2D_Count(st_int tMode[])
{
	if (tMode[0] == 0)
		return tMode[2];
	else
		return tMode[1];
}

#ifdef ST_RAWTEST_LOGFILE
static st_int st_drv_Get_2D_RAW(st_int tMode[], st_int rawJ[], st_int gsMode, st_u8 *rtbuf, struct file *filp)
#else
static st_int st_drv_Get_2D_RAW(st_int tMode[], st_int rawJ[], st_int gsMode, st_u8 *rtbuf)
#endif /* ST_RAWTEST_LOGFILE */
{
	st_int count = st_drv_Get_2D_Count(tMode);
	st_int length = st_drv_Get_2D_Length(tMode);
	st_int maxTimes = 60;
	st_int dataCount = 0;
	st_int times = maxTimes;
	st_int readLength = 8 + 2 * length;
	st_u8 raw[0x40];
	st_int i = 0;
	st_int index;
	st_int keyAddCount = (tMode[3] > 0) ? 1 : 0;
	short rawI;
	st_int errorCount = 0;
	st_u8 isFillData[MAX_SENSOR_COUNT];
#ifdef ST_RAWTEST_LOGFILE
	char data1[80];
#endif
	memset(isFillData, 0, count + keyAddCount);
	memset(raw, 0, 0x40);

	//hfst add for jump 2 frame
		times = maxTimes*2;
		while (dataCount < (2*(count + keyAddCount+1)) && times-- > 0)
		{
			stx_i2c_read_bytes(0x40, raw, readLength);
			if (raw[0] == 6)
			{
				index = raw[2];
//				if (isFillData[index] == 0)
//				{
//					isFillData[index] = 1;
					dataCount++;
//				}
			}
			else if (raw[0] == 7)
			{
//				if (isFillData[count] == 0)
//				{
//					isFillData[count] = 1;
					dataCount++;
//				}
			}
		}

		dataCount = 0;
		times = maxTimes;
		memset(isFillData, 0, count + keyAddCount);
		memset(raw, 0, 0x40);
	//hfst modify end

	/* STX_DEBUG("isFill 0 : %d",isFillData[0]); */
	while (dataCount != (count + keyAddCount) && times-- > 0)
	{
		stx_i2c_read_bytes(0x40, raw, readLength);
		STX_DEBUG("%X %X %X data:%d key:%d",raw[0],raw[1],raw[2],dataCount,tMode[3]);
		if (raw[0] == 6)
		{
			index = raw[2];
			/* STX_DEBUG("isFill %d : %d %d , %d",index,isFillData[index],dataCount,count+keyAddCount); */
			if (isFillData[index] == 0)
			{
				/* STX_DEBUG("index %d",index); */
				isFillData[index] = 1;
				dataCount++;
				/* index = index*length; */
				if (index != 0)
				{
					for (i = 0; i < length; i++)
					{
						rawI = (short)((raw[4 + 2 * i]) * 0x100 + raw[5 + 2 * i]);
						/* STX_DEBUG("Sensor RAW %d,%d = %d",index,i,rawI); */
						
						if (rawI > MAX_RAW_LIMIT || rawI < MIN_RAW_LIMIT)
						{
							STX_ERROR("Error: Sensor RAW %d,%d = %d out of limity (%d,%d)", index, i, rawI, MIN_RAW_LIMIT, MAX_RAW_LIMIT);
#ifdef ST_RAWTEST_LOGFILE
							snprintf(data1, 80, "Error: Sensor RAW %d,%d = %d out of limity (%d,%d)\n", index, i, rawI, MIN_RAW_LIMIT, MAX_RAW_LIMIT);
							filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
							rtbuf[index * length + i] = 1;
							errorCount++;
						}
						/* rawI[index+i] = raw[4+2*i]*0x100 + raw[5+2*i]; */
					}
				}
			}
		}
		else if (raw[0] == 7)
		{
			/* key */
			STX_INFO("key");
			if (isFillData[count] == 0)
			{
				isFillData[count] = 1;
				dataCount++;
				for (i = 0; i < tMode[3]; i++)
				{
					rawI = (short)((raw[4 + 2 * i]) * 0x100 + raw[5 + 2 * i]);
					/* STX_DEBUG("Key RAW %d = %d",i,rawI); */

					if (rawI > MAX_RAW_LIMIT || rawI < MIN_RAW_LIMIT)
					{
						STX_ERROR("Error: Key RAW %d = %d out of limity (%d,%d)", i, rawI, MIN_RAW_LIMIT, MAX_RAW_LIMIT);
#ifdef ST_RAWTEST_LOGFILE
						snprintf(data1, 80, "Error: Key RAW %d = %d out of limity (%d,%d)\n", i, rawI, MIN_RAW_LIMIT, MAX_RAW_LIMIT);
						filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
						rtbuf[count * length + i] = 1;
						errorCount++;
					}
					/* rawI[count*length+i] = raw[4+2*i]*0x100 + raw[5+2*i]; */
				}
			}
		}
	}
	if (times <= 0)
	{
		STX_ERROR("Get 2D RAW fail!");
		return -1;
	}
	return errorCount;
}

static st_int st_drv_Get_2D_RAW_add(st_int tMode[], st_int rawJ[], st_int gsMode, st_u8 *rtbuf,char *buf)
{
	st_int count = st_drv_Get_2D_Count(tMode);
	st_int length = st_drv_Get_2D_Length(tMode);
	st_int maxTimes = 60;
	st_int dataCount = 0;
	st_int times = maxTimes;
	st_int readLength = 8 + 2 * length;
	st_u8 raw[0x40];
	st_int i = 0;
	st_int index;
	st_int keyAddCount = (tMode[3] > 0) ? 1 : 0;
	short rawI;
	st_int errorCount = 0;
	st_u8 isFillData[MAX_SENSOR_COUNT];
	st_int size = 0;
#ifdef ST_RAWTEST_LOGFILE
	char data1[80];
#endif

	memset(isFillData, 0, count + keyAddCount);
	memset(raw, 0, 0x40);

	//hfst add for jump 2 frame
		times = maxTimes*2;
		while (dataCount < (2*(count + keyAddCount+1)) && times-- > 0)
		{
			stx_i2c_read_bytes(0x40, raw, readLength);
			if (raw[0] == 6)
			{
				index = raw[2];
//				if (isFillData[index] == 0)
//				{
//					isFillData[index] = 1;
					dataCount++;
//				}
			}
			else if (raw[0] == 7)
			{
//				if (isFillData[count] == 0)
//				{
//					isFillData[count] = 1;
					dataCount++;
//				}
			}
		}

		dataCount = 0;
		times = maxTimes;
		memset(isFillData, 0, MAX_SENSOR_COUNT);
		memset(raw, 0, 0x40);
	//hfst modify end

	/* STX_DEBUG("isFill 0 : %d",isFillData[0]); */
	while (dataCount != (count + keyAddCount) && times-- > 0)
	{
		stx_i2c_read_bytes(0x40, raw, readLength);
		STX_DEBUG("%X %X %X data:%d key:%d",raw[0],raw[1],raw[2],dataCount,tMode[3]);
		if (raw[0] == 6)
		{
			index = raw[2];
			/* STX_DEBUG("isFill %d : %d %d , %d",index,isFillData[index],dataCount,count+keyAddCount); */
			if (isFillData[index] == 0)
			{
				/* STX_DEBUG("index %d",index); */
				isFillData[index] = 1;
				/* index = index*length; */
				if (index != 0)
				{
					dataCount++;
					for (i = 0; i < length; i++)
					{
						rawI = (short)((raw[4 + 2 * i]) * 0x100 + raw[5 + 2 * i]);
						/* STX_DEBUG("Sensor RAW %d,%d = %d",index,i,rawI); */
						printk(KERN_ERR "%d",rawI);
						size += snprintf(buf + size, PAGE_SIZE,
							i < length - 1 ? "%u " : "%u\n",rawI);
						
						if (rawI > MAX_RAW_LIMIT || rawI < MIN_RAW_LIMIT)
						{
							STX_ERROR("Error: Sensor RAW %d,%d = %d out of limity (%d,%d)", index, i, rawI, MIN_RAW_LIMIT, MAX_RAW_LIMIT);
#ifdef ST_RAWTEST_LOGFILE
							snprintf(data1, 80, "Error: Sensor RAW %d,%d = %d out of limity (%d,%d)\n", index, i, rawI, MIN_RAW_LIMIT, MAX_RAW_LIMIT);
							filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
							rtbuf[index * length + i] = 1;
							errorCount++;
						}
						/* rawI[index+i] = raw[4+2*i]*0x100 + raw[5+2*i]; */
					}
//					printk(KERN_ERR "\n");
				}
			}
		}
		else if (raw[0] == 7)
		{
			/* key */
			STX_INFO("key");
			if (isFillData[count] == 0)
			{
				isFillData[count] = 1;
				dataCount++;
				for (i = 0; i < tMode[3]; i++)
				{
					rawI = (short)((raw[4 + 2 * i]) * 0x100 + raw[5 + 2 * i]);
					/* STX_DEBUG("Key RAW %d = %d",i,rawI); */
					printk(KERN_ERR "%d",rawI);
					//rawJ[gsMode] = rawI;
					//gsMode++; 
					if (rawI > MAX_RAW_LIMIT || rawI < MIN_RAW_LIMIT)
					{
						STX_ERROR("Error: Key RAW %d = %d out of limity (%d,%d)", i, rawI, MIN_RAW_LIMIT, MAX_RAW_LIMIT);
#ifdef ST_RAWTEST_LOGFILE
						snprintf(data1, 80, "Error: Key RAW %d = %d out of limity (%d,%d)\n", i, rawI, MIN_RAW_LIMIT, MAX_RAW_LIMIT);
						filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
						rtbuf[count * length + i] = 1;
						errorCount++;
					}
					/* rawI[count*length+i] = raw[4+2*i]*0x100 + raw[5+2*i]; */
//					printk(KERN_ERR "\n"); 
				}
			}
		}
	}
//	STX_DEBUG("\n");
	if (times <= 0)
	{
		STX_ERROR("Get 2D RAW fail!");
		return -1;
	}
	return size;
}

static int Get1DRaw(bool fSelectTXMode, int nRawData[])
{
	unsigned char mutBuf[0xFF];
	int count = 0;
	int maxtime = 0;
	int id;

	mutBuf[0] = 0xF1;
	mutBuf[1] = 0xC0;

	stx_i2c_write_bytes(mutBuf, 2);

	while (count < 5 && maxtime++ < 1000)
	{
		/* msleep(20); */
		if (count < 4)
			stx_i2c_read_bytes(0x40, mutBuf, 4);
		else
			stx_i2c_read_bytes(0x40, mutBuf, 0x50);

		if (((!fSelectTXMode) && (mutBuf[0] == 0x04) && (mutBuf[2] == 0x00) && (mutBuf[3] == 0x00)) || ((fSelectTXMode) && (mutBuf[0] == 0x04) && (mutBuf[2] == 0x02)))
		{
			count++;
			if (count == 5)
			{
				for (id = 0; id < (36); id++)
				{
					if (mutBuf[4 + 2 * id] & 0x80)
						nRawData[id] = ((mutBuf[4 + 2 * id] << 8) + mutBuf[5 + 2 * id]) - 65536;
					else
						nRawData[id] = ((mutBuf[4 + 2 * id] << 8) + mutBuf[5 + 2 * id]);
				}
			}
		}
	}
	if (count >= 5)
		STX_DEBUG("Get 1d RAW ok %d %d %d %d %d %d %d %d", nRawData[0], nRawData[1], nRawData[2], nRawData[3], nRawData[4], nRawData[5], nRawData[6], nRawData[7]);
	else
		STX_ERROR("Get 1d RAW fail");
	return (count >= 5) ? 0 : -1;
}

#if 0
static void WriteFilterRam(void)
{
	/* FilterRam */
	unsigned char pFilterRam[2] = {0x00, 0x30};
	unsigned char pCMDData[1024] = {0};
	int i;

	for (i = 0; i < 1024; i++)
	{
		if ((i % 2) == 0)
			pCMDData[i] = pFilterRam[0];
		else
			pCMDData[i] = pFilterRam[1];
	}
	stx_cmdio_write(3, 0, pCMDData, 0x9A);
}
#endif

static void WriteTestCFG(void)
{
	unsigned char buf[8];

	stx_cmdio_write(2, ST_SHORT_TEST_CFG_OFFSET, st_short_test_cfg, sizeof(st_short_test_cfg));
	/* #DoPower Down Wake Up */
	buf[0] = 0x2;
	buf[1] = 0x2;
	stx_i2c_write_bytes(buf, 2);

	msleep(150);
	buf[0] = 0x2;
	buf[1] = 0x0;
	stx_i2c_write_bytes(buf, 2);
	msleep(150);
//	WriteFilterRam();
}
static int RunFindShortStart(bool fSelectTXMode, int xNum, int yNum, unsigned char pMaxPatternCount)
{
	int nScanChannel = 0;
	int nMaxShortRecordRaw = 0;
	int nShortMaxRawDefine = 0;
	unsigned char pTX_OSB = 0x03, pRX_OSB = 0x03;
	unsigned char pShortSkipArray[40] = {0};

	int nMaxRx = xNum = 36;
	int nMaxTx = yNum = 20;

	bool fMayBeFilterRamError = false;
	int nRet = 1, pMutulRaw[40];
	unsigned char pCMDData[64] = {0};
	unsigned char Pattern[13][5]; /* ={0}; */
	unsigned char pListArray[13][37] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
		{0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
		{0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1},
		{0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
		{0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
		{0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
		{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
		{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0}};
	unsigned char pSkipArray[37] = {0};
	unsigned char pTempArray_1[37] = {0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
	unsigned char pTempArray_2[37] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

	int nSum = 0;
	int i, j;

	int nTxStartRun = 0;
	int nRun;

	unsigned char fFindShort = false;
	int nRaw = 0;

	int nAddTx = 0;
	bool fFind = false;
	int nCheckStart = 0;
	/* /////////////////// */
	/* ////////// start work */
	if (fSelectTXMode)
		nScanChannel = yNum;
	else
		nScanChannel = xNum;

	if (fSelectTXMode)
		nScanChannel = nMaxTx;
	else
		nScanChannel = nMaxRx;

	if (fSelectTXMode == false)
		memcpy(pSkipArray, pTempArray_1, 37);
	else
		memcpy(pSkipArray, pTempArray_2, 37);

	for (i = 1; i < 13; i++)
		for (j = 0; j < 36; j++)
			if (pSkipArray[j] == 1)
				pListArray[i][j] = 1;

	/* memset(pResult,1,40*40); */
	for (i = 0; i < 13; i++)
	{
		nSum = 0;
		for (j = 0; j < 37; j++)
		{
			nSum += ((pListArray[i][j]) << (j % 0x08));
			if ((j + 1) % 8 == 0)
			{
				Pattern[i][j / 8] = nSum;
				nSum = 0;
			}
		}
		Pattern[i][37 / 8] = nSum + 0x20;
	}

	/* /////////// */
	if (fSelectTXMode)
		nTxStartRun = 0;
	for (nRun = nTxStartRun; (nRun < pMaxPatternCount) && (fMayBeFilterRamError == false); nRun++)
	{
		pCMDData[0] = 0x01;
		pCMDData[1] = 0x07;
		pCMDData[2] = 0x04;
		pCMDData[3] = 0x00;
		pCMDData[4] = 0x1D;
		pCMDData[5] = 0x02;
		pCMDData[6] = 0x2D;
		pCMDData[7] = 0x2D;
		stx_cmdio_write(4, 0x1D, pCMDData + 6, 2);

		pCMDData[0] = 0x01;
		pCMDData[1] = 0x07;
		pCMDData[2] = 0x04;
		pCMDData[3] = 0x00;
		pCMDData[4] = 0x28;
		pCMDData[5] = 0x02;
		pCMDData[6] = 0x24;
		pCMDData[7] = 0x00;
		stx_cmdio_write(4, 0x28, pCMDData + 6, 2);

		pCMDData[0] = 0x01;
		pCMDData[1] = 0x07;
		pCMDData[2] = 0x04;
		pCMDData[3] = 0x00;
		pCMDData[4] = 0x11;
		pCMDData[5] = 0x02;
		pCMDData[6] = 0x00;
		pCMDData[7] = 0x00;
		stx_cmdio_write(4, 0x11, pCMDData + 6, 2);

		pCMDData[0] = 0x01;
		pCMDData[1] = 0x07;
		pCMDData[2] = 0x04;
		pCMDData[3] = 0x00;
		pCMDData[4] = 0x12;
		pCMDData[5] = 0x02;
		pCMDData[6] = 0x00;
		pCMDData[7] = 0x00;
		stx_cmdio_write(4, 0x12, pCMDData + 6, 2);

		/* Set Addr :0x00D1 STOG[15:1] */
		pCMDData[0] = 0x01;
		pCMDData[1] = 0x07;
		pCMDData[2] = 0x04;
		pCMDData[3] = 0x00;
		pCMDData[4] = 0xD1;
		pCMDData[5] = 0x02;
		pCMDData[6] = Pattern[nRun][1];
		pCMDData[7] = Pattern[nRun][0];
		stx_cmdio_write(4, 0xD1, pCMDData + 6, 2);

		/* Set Addr :0x00D2 STOG[31:16] */
		pCMDData[0] = 0x01;
		pCMDData[1] = 0x07;
		pCMDData[2] = 0x04;
		pCMDData[3] = 0x00;
		pCMDData[4] = 0xD2;
		pCMDData[5] = 0x02;
		pCMDData[6] = Pattern[nRun][3];
		pCMDData[7] = Pattern[nRun][2];
		stx_cmdio_write(4, 0xD2, pCMDData + 6, 2);

		/* Set Addr :0x00D3 STOG[35:32] , OSB:10 <- 0.8uA */
		pCMDData[0] = 0x01;
		pCMDData[1] = 0x07;
		pCMDData[2] = 0x04;
		pCMDData[3] = 0x00;
		pCMDData[4] = 0xD3;
		pCMDData[5] = 0x02;
		pCMDData[6] = 0x03;
		if (fSelectTXMode)
			pCMDData[7] = ((Pattern[nRun][4] & 0x0F) | (pTX_OSB << 4));
		else
			pCMDData[7] = ((Pattern[nRun][4] & 0x0F) | (pRX_OSB << 4));

		STX_DEBUG("Pattern0-3 %x %x %x %x,pCMDData[6][7] %x %x", Pattern[nRun][0], Pattern[nRun][1], Pattern[nRun][2], Pattern[nRun][3], pCMDData[6], pCMDData[7]);
		stx_cmdio_write(4, 0xD3, pCMDData + 6, 2);

		if (Get1DRaw(fSelectTXMode, pMutulRaw) < 0)
		{
			STX_ERROR("Get1DRaw(fSelectTXMode,pMutulRaw) < 0");
			nRet = -2;
			return nRet;
		}

		for (i = 0; i < nScanChannel; i++)
		{
			nRaw = pMutulRaw[i];
			if (fSelectTXMode)
				nAddTx = 1;
			fFind = true;
			if (fSelectTXMode)
				nCheckStart = -1;
			if (pListArray[nRun][i + nAddTx] == 0 && fFind)
			{
				if ((nRaw > nShortMaxRawDefine) && (i > nCheckStart))
				{
					if (fSelectTXMode)
					{
						if (pShortSkipArray[i] != 0x01)
						{
							fFindShort = true;
							nRet = -1;
							if (nRaw > nMaxShortRecordRaw)
								nMaxShortRecordRaw = nRaw;
						}
					}
					else
					{
						if (pShortSkipArray[i] != 0x01)
						{
							STX_INFO("i %d , raw %d", i, nRaw);
							if (nRaw > nMaxShortRecordRaw)
								nMaxShortRecordRaw = nRaw;
							fFindShort = true;
							nRet = -1;
						}
					}
				}
			}
		}
		STX_INFO("short test result %d  nRun %d", nRet, nRun);
	}
	return nRet;
}

/*
	rtbuf is a u8 buf[X * Y + K]
	X:sensor count of X
	Y:sensor count of Y
	K:sensor count of K
	buf[?] = 0 means pass
	buf[?] = 1 means fail
	bug[20] means TX: 20/RX_count , RX: 20%RX_count
	RX_count : maybe X or Y , according layout..

	return :
	<0 error
	0  success
	>0 failed sensor count
*/

static int st_drv_get_rawdata(st_u8 *rtbuf, int length,char *buff)
{
	st_int result;
	st_u8 buf[8];
	st_int sensorCount = 0;
	st_int raw_J[MAX_SENSOR_COUNT];
	st_int tMode[4];

	result = 0;
	memset(rtbuf, 0, length);
	STX_INFO("start of st_drv_test_raw");
	/* check status */
	memset(buf, 0, 8);
	result = stx_i2c_read_bytes(1, buf, 8);
	if (result < 0)
	{
		STX_ERROR("ST I2C error (%d)", result);
		result = -1;
		goto ST_RAW_CLOSE_FILE;
	}

	STX_INFO("status :0x%X", buf[0]);
	if ((buf[0] & 0xf) == 6)
	{
		STX_ERROR("ST IC in boot code , can't do test !");
		result = -1;
		goto ST_RAW_CLOSE_FILE;
	}
	buf[0] = 0xF1;
	buf[1] = 0x40;
	stx_i2c_write_bytes(buf, 2);
	st_msleep(100);
	/* ///////////////////////////// */
	/* get tmode */
	stx_i2c_read_bytes(0xF0, buf, 1);
	tMode[0] = (buf[0] & 0x04) >> 2; /* tx is ? */
	STX_INFO("ST TX flag = %d", tMode[0]);
	stx_i2c_read_bytes(0xF5, buf, 3);
	tMode[1] = buf[0];		 /* x */
	tMode[2] = buf[1];		 /* y */
	tMode[3] = buf[2] & 0xf; /* key */

	sensorCount = tMode[1] + tMode[2] + tMode[3];

	STX_INFO("sensor count:%d %d %d", tMode[1], tMode[2], tMode[3]);

	memset(rtbuf, 0, tMode[1] * tMode[2] + tMode[3]);
	/* //////////////////////////// */
	/* get raw and judge */

	result = st_drv_Get_2D_RAW_add(tMode, raw_J, 0, rtbuf,buff);
	if (result < 0)
	{
		STX_ERROR("Error: Test fail with %d sensor", result);
		result = -1;
		goto ST_RAW_CLOSE_FILE;
	}else{
		STX_INFO("Test open/rawdata successed!");
	}
	/* st_irq_on(); */
	/* /////////////////////////// */
	buf[0] = 2;
	buf[1] = 1;
	stx_i2c_write_bytes(buf, 2);
	st_msleep(150);

	stx_clr_crash_msg();

ST_RAW_CLOSE_FILE:
	return result;
}
static int st_drv_test_short(st_u8 *rtbuf, int length)
{
	st_int txr, rxr;
	st_u8 buf[8];
	st_int result;
	
	result = 0;
	memset(rtbuf, 0, length);
	STX_INFO("start of st_drv_test_short");
	/* check status */
	memset(buf, 0, 8);
	result = stx_i2c_read_bytes(1, buf, 8);
	if (result < 0)
	{
		STX_ERROR("ST I2C error (%d)", result);
		result = -1;
		goto ST_RAW_CLOSE_FILE;
	}
	STX_INFO("status :0x%X", buf[0]);
	if ((buf[0] & 0xf) == 6)
	{
		STX_ERROR("ST IC in boot code , can't do test !");
		result = -1;
		goto ST_RAW_CLOSE_FILE;
	}
	buf[0] = 0xF1;
	buf[1] = 0x40;
	stx_i2c_write_bytes(buf, 2);
	st_msleep(100);


	WriteTestCFG();

	rxr = RunFindShortStart(false, 0x14, 0x24, 3);
	if (rxr < 0)
	{
		STX_ERROR("Error: Test Rx Short fail");
		result++;
	}

	txr = RunFindShortStart(true, 0x14, 0x24, 3);
	if (txr < 0)
	{
		STX_ERROR("Error: Test Tx Short fail");
		result++;
	}
	if (rxr >= 0 && txr >= 0)
	{
		STX_INFO("Test short successed!");
		result = 0;
	}

	/* st_irq_on(); */
	/* /////////////////////////// */
	buf[0] = 2;
	buf[1] = 1;
	stx_i2c_write_bytes(buf, 2);
	st_msleep(150);

	stx_clr_crash_msg();
ST_RAW_CLOSE_FILE:
	return result;


}

static int st_drv_test_raw(st_u8 *rtbuf, int length)
{
	st_int result;
	st_u8 buf[8];
	st_int sensorCount = 0;
	st_int raw_J[MAX_SENSOR_COUNT];
	st_int tMode[4];
	st_int txr, rxr;
#ifdef ST_RAWTEST_LOGFILE
	struct file *filp;
	char data1[50];
	mm_segment_t fs;

	filp = filp_open(ST_RAW_LOG_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (IS_ERR(filp))
	{
		STX_ERROR("ST open %s error...", ST_RAW_LOG_PATH);
		return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	filp->f_op->llseek(filp, 0, 0);
#endif
	result = 0;
	memset(rtbuf, 0, length);
	STX_INFO("start of st_drv_test_raw");
	/* /////////////////////////// */
	/* check status */
	memset(buf, 0, 8);
	result = stx_i2c_read_bytes(1, buf, 8);
	if (result < 0)
	{
		STX_ERROR("ST I2C error (%d)", result);
#ifdef ST_RAWTEST_LOGFILE
		snprintf(data1, 50, "ST I2C error (%d)\n", result);
		filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
		result = -1;
		goto ST_RAW_CLOSE_FILE;
	}

	STX_INFO("status :0x%X", buf[0]);
	if ((buf[0] & 0xf) == 6)
	{
		STX_ERROR("ST IC in boot code , can't do test !");
#ifdef ST_RAWTEST_LOGFILE
		snprintf(data1, 50, "ST IC in boot code , can't do test !\n");
		filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
		result = -1;
		goto ST_RAW_CLOSE_FILE;
	}

	buf[0] = 0xF1;
	buf[1] = 0x40;
	stx_i2c_write_bytes(buf, 2);
	st_msleep(100);
	/* ///////////////////////////// */
	/* get tmode */
	stx_i2c_read_bytes(0xF0, buf, 1);
	tMode[0] = (buf[0] & 0x04) >> 2; /* tx is ? */
	STX_INFO("ST TX flag = %d", tMode[0]);
	stx_i2c_read_bytes(0xF5, buf, 3);
	tMode[1] = buf[0];		 /* x */
	tMode[2] = buf[1];		 /* y */
	tMode[3] = buf[2] & 0xf; /* key */

	sensorCount = tMode[1] + tMode[2] + tMode[3];

	STX_INFO("sensor count:%d %d %d", tMode[1], tMode[2], tMode[3]);

	memset(rtbuf, 0, tMode[1] * tMode[2] + tMode[3]);
	/* //////////////////////////// */
	/* get raw and judge */
#ifdef ST_RAWTEST_LOGFILE
	result = st_drv_Get_2D_RAW(tMode, raw_J, 0, rtbuf, filp);
#else
	result = st_drv_Get_2D_RAW(tMode, raw_J, 0, rtbuf);
#endif
	if (result != 0)
	{
		STX_ERROR("Error: Test fail with %d sensor", result);
#ifdef ST_RAWTEST_LOGFILE
		snprintf(data1, 50, "Error: Test fail with %d sensor\n", result);
		filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
	}
	else
	{
		STX_INFO("Test open successed!");
#ifdef ST_RAWTEST_LOGFILE
		snprintf(data1, 50, "Test open successed!\n");
		filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
		result = 0;
	}
	/* //////////////////////////// */
	/* int nRawData[36]; */
	/* Get1DRaw(false,nRawData); */
	/* st_irq_off(); */
	WriteTestCFG();

	rxr = RunFindShortStart(false, 0x14, 0x24, 3);
	if (rxr < 0)
	{
		STX_ERROR("Error: Test Rx Short fail");
#ifdef ST_RAWTEST_LOGFILE
		snprintf(data1, 50, "Error: Test Rx Short fail\n");
		filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
		result++;
	}

	txr = RunFindShortStart(true, 0x14, 0x24, 3);
	if (txr < 0)
	{
		STX_ERROR("Error: Test Tx Short fail");
#ifdef ST_RAWTEST_LOGFILE
		snprintf(data1, 50, "Error: Test Tx Short fail\n");
		filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
		result++;
	}
	if (rxr >= 0 && txr >= 0)
	{
		STX_INFO("Test short successed!");
#ifdef ST_RAWTEST_LOGFILE
		snprintf(data1, 50, "Test short successed!\n");
		filp->f_op->write(filp, data1, strlen(data1), &filp->f_pos);
#endif
	}

	/* st_irq_on(); */
	/* /////////////////////////// */
	buf[0] = 2;
	buf[1] = 1;
	stx_i2c_write_bytes(buf, 2);
	st_msleep(150);

	stx_clr_crash_msg();
ST_RAW_CLOSE_FILE:

#ifdef ST_RAWTEST_LOGFILE
	set_fs(fs);
	filp_close(filp, NULL);
#endif
	STX_INFO("Test result file	: %s", ST_RAW_LOG_PATH);

	return result;
}
#endif /* ST_TEST_RAW */

int sitronix_ts_get_rawdata(struct device *dev,  char *buf)
{
	st_int size = -1;
#ifdef ST_TEST_RAW
	st_u8 rtbuf[0x14 * 0x24];
	int length = sizeof(rtbuf) / sizeof(st_u8);
	STX_INFO("%s:enter.",__func__);
	stx_gpts.is_testing = true;
	size = st_drv_get_rawdata(rtbuf, length,buf);
	stx_gpts.is_testing = false;
#endif /* ST_TEST_RAW */
	STX_INFO("%s:exit.",__func__);

	if(size >= 0)
		return size;
	else{
		return snprintf(buf, PAGE_SIZE, "%s\n", "NOT SUPPORTED");
	}
}
int sitronix_ts_get_short_test(struct device *dev,  char *buf)
{
	int ret;
#ifdef ST_TEST_RAW
	st_u8 rtbuf[0x14 * 0x24];
	int length = sizeof(rtbuf) / sizeof(st_u8);
	STX_INFO("%s:enter.",__func__);
	stx_gpts.is_testing = true;
	ret = st_drv_test_short(rtbuf, length);
	stx_gpts.is_testing = false;
#endif /* ST_TEST_RAW */

	STX_INFO("%s:exit.",__func__);
	if(ret == 0)
		return 1;
	else{
		return 0;
	}

}
#if 0

int sitronix_ts_get_open_test(struct device *dev,  char *buf)
{
	int ret;
	char * buff;
#ifdef ST_TEST_RAW
	st_u8 rtbuf[0x14 * 0x24];
	int length = sizeof(rtbuf) / sizeof(st_u8);
	STX_INFO("%s:enter.",__func__);
	stx_gpts.is_testing = true;
	ret = st_drv_get_rawdata(rtbuf, length,buff);
	stx_gpts.is_testing = false;
#endif /* ST_TEST_RAW */
	if(ret >= 0)
		return 1;
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "NOT SUPPORTED");

}
#endif


int st_testraw_invoke(void)
{
	st_int ret = -1;
#ifdef ST_TEST_RAW
	st_u8 rtbuf[0x14 * 0x24];
	int length = sizeof(rtbuf) / sizeof(st_u8);
	stx_gpts.is_testing = true;
	ret = st_drv_test_raw(rtbuf, length);
	stx_gpts.is_testing = false;
#endif /* ST_TEST_RAW */

	return ret;
}
