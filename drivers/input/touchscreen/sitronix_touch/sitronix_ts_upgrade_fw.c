#include "sitronix_ts_custom_func.h"
#include "sitronix_ts.h"
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_L762
#include "sitronix_ts_upgrade_fw_bin_l762.h"
#else
#include "sitronix_ts_upgrade_fw_bin.h"

#endif
#include <linux/firmware.h>


#define ST_FW_LEN 0xEFC0
#define ST_CFG_LEN 0x840
#define ST_CFG_OFFSET 0xEFC0
#define ST_FLASH_PAGE_SIZE 4096
#define ST_FLASH_SIZE 0x10000
#define ST_FLASH_READ_SIZE 0xF800
#define ST_CHECK_FW_OFFSET 0xF000

#define ST_ISP_RETRY_MAX 5 //hfst
#define ST_ISP_MAX_TRANS_LEN 8
#define ST_ISP_MAX_WRITE_LEN ST_ISP_MAX_TRANS_LEN - 2
#define ST_ISP_BLOCK_SIZE 256


#define ISP_PACKET_SIZE 8
#define ST_FW_INFO_LEN 0x10

#define ISP_CMD_ERASE 0x80
#define ISP_CMD_SEND_DATA 0x81
#define ISP_CMD_WRITE_FLASH 0x82
#define ISP_CMD_READ_FLASH 0x83
#define ISP_CMD_RESET 0x84
#define ISP_CMD_UNLOCK 0x87
#define ISP_CMD_READY 0x8F

#define ISPID_OFFSET 0x1BF00
#define ISPID_READ_LENGTH 100

#define STX_AFE_REG_TYPE	4

#ifdef STX_UPGRADE_REQUEST_FW_FILE
unsigned char fw_buf[ST_FW_LEN];
unsigned char cfg_buf[ST_CFG_LEN];
#else//STX_UPGRADE_REQUEST_FW_FILE
unsigned char fw_buf[] = SITRONIX_FW;
unsigned char cfg_buf[] = SITRONIX_CFG;

#if defined(ST_UPGRADE_BY_ISPID) || defined(ST_UPGRADE_BY_FWID)
unsigned char fw_buf0[] = SITRONIX_FW;
unsigned char cfg_buf0[] = SITRONIX_CFG;
unsigned char fw_buf1[] = SITRONIX_FW1;
unsigned char cfg_buf1[] = SITRONIX_CFG1;
unsigned char fw_buf2[] = SITRONIX_FW2;
unsigned char cfg_buf2[] = SITRONIX_CFG2;

static void st_replace_fw_by_id(int id)
{
	if (id == 0)
	{
		STX_INFO("Found id by SITRONIX_FW and SITRONIX_CFG");
		memcpy(fw_buf, fw_buf0, sizeof(fw_buf0));
		memcpy(cfg_buf, cfg_buf0, sizeof(cfg_buf0));
	}
	else if (id == 1)
	{
		STX_INFO("Found id by SITRONIX_FW1 and SITRONIX_CFG1");
		memcpy(fw_buf, fw_buf1, sizeof(fw_buf1));
		memcpy(cfg_buf, cfg_buf1, sizeof(cfg_buf1));
	}
	else if (id == 2)
	{
		STX_INFO("Found id by SITRONIX_FW2 and SITRONIX_CFG2");
		memcpy(fw_buf, fw_buf2, sizeof(fw_buf2));
		memcpy(cfg_buf, cfg_buf2, sizeof(cfg_buf2));
	}
}
#endif
#endif//STX_UPGRADE_REQUEST_FW_FILE

static int st_check_chipid(void)
{
	int ret = 0;
	unsigned char buffer[1];

	ret = stx_i2c_read_bytes(0xF4, buffer, 1);
	if (ret < 0)
	{
		STX_ERROR("read status reg error (%d)", ret);
		return ret;
	}
	else
	{
		STX_INFO("ChipID = %d", buffer[0]);
	}

	if (buffer[0] != 0xC)
	{
		STX_ERROR("This IC is not A1802 , cancel upgrade");
		return -1;
	}
	return 0;
}

static int st_get_device_status(void)
{
	int ret = 0;
	unsigned char buffer[8];

	ret = stx_i2c_read_bytes(1, buffer, 8);
	if (ret < 0)
	{
		STX_ERROR("read status reg error (%d)", ret);
		return ret;
	}
	else
	{
		STX_INFO("status reg = %d", buffer[0]);
	}
	return buffer[0] & 0xF;
}

static int st_check_device_status(int ck1, int ck2, int delay)
{
	int maxTimes = 3;
	int isInStauts = 0;
	int status = -1;
	while (maxTimes-- > 0 && isInStauts == 0)
	{
		status = st_get_device_status();
		if (status == ck1 || status == ck2)
			isInStauts = 1;
		msleep(delay);
	}
	if (isInStauts == 0)
		return -1;
	else
		return 0;
}

static int st_v2_power_up(void)
{
	unsigned char reset[2];
	reset[0] = 2;
	reset[1] = 0;
	return stx_i2c_write_bytes(reset, 2);
}

static int st_isp_on(void)
{
	unsigned char IspKey[] = {0, 'S', 0, 'T', 0, 'X', 0, '_', 0, 'F', 0, 'W', 0, 'U', 0, 'P'};
	unsigned char i;
	int icStatus = st_get_device_status();
	STX_INFO("ISP on");

	if (icStatus < 0)
		return -1;
	if (icStatus == 0x6)
		return 0;
	else if (icStatus == 0x5)
		st_v2_power_up();

	for (i = 0; i < sizeof(IspKey); i += 2)
	{
		if (stx_i2c_write_bytes(&IspKey[i], 2) < 0)
		{
			STX_ERROR("Entering ISP fail.");
			return -1;
		}
	}
	msleep(150); //This delay is very important for ISP mode changing.
		//Do not remove this delay arbitrarily.
	return st_check_device_status(6, 99, 10);
}

static int st_compare_array(unsigned char *b1, unsigned char *b2, int len)
{
	int i = 0;
	for (i = 0; i < len; i++)
	{
		if (b1[i] != b2[i])
			return -1;
	}
	return 0;
}

static int st_get_fw_info_offset(int fwSize, unsigned char *data)
{
	int cksOffset;
	int i = 0;
	cksOffset = data[0x84] * 0x100 + data[0x85];

	for (i = cksOffset - 4; i >= 4; i--)
	{
		if (data[i] == 0x54 && data[i + 1] == 0x46 && data[i + 2] == 0x49 && data[i + 3] == 0x33)
		{
			STX_INFO("TOUCH_FW_INFO offset = 0x%X", i + 4);
			return i + 4;
		}
	}
	STX_ERROR("can't find TOUCH_FW_INFO offset");
	return -1;
}

static void ChecksumCalculation(unsigned short *pChecksum, unsigned char *pInData, unsigned long Len)
{
	unsigned long i;
	unsigned char LowByteChecksum;
	for (i = 0; i < Len; i++)
	{
		*pChecksum += (unsigned short)pInData[i];
		LowByteChecksum = (unsigned char)(*pChecksum & 0xFF);
		LowByteChecksum = (LowByteChecksum) >> 7 | (LowByteChecksum) << 1;
		*pChecksum = (*pChecksum & 0xFF00) | LowByteChecksum;
	}
}

static int st_v2_format(st_u16 addr, st_u8 isRead, unsigned char *txbuf, unsigned char *rxbuf, int len)
{
	int result;

	unsigned char isp_tbuf[ST_ISP_MAX_WRITE_LEN + 2] = {0};

	isp_tbuf[0] = (addr >> 8);
	isp_tbuf[1] = (addr & 0xFF);
	if (isRead)
	{
		result = stx_i2c_write_bytes(isp_tbuf, 2);
		st_usleep(50);
		result = stx_i2c_read_direct(rxbuf, len);

		if (result < 0)
			return result;
		else
			return len;
	}
	else
	{
		memcpy(isp_tbuf + 2, txbuf, len);
		result = stx_i2c_write_bytes(isp_tbuf, len + 2);

		if (result < 0)
			return result;
		else
			return len;
	}
	return 0;
}

static int st_v2_len_check(st_u16 addr, st_u8 isRead, unsigned char *txbuf, unsigned char *rxbuf, int len)
{
	int nowlen = len;
	int nowoff = 0;
	int ret = 0;

	if (isRead)
	{
		while (nowlen > 0)
		{
			if (nowlen > ST_ISP_MAX_TRANS_LEN)
			{
				ret += st_v2_format(addr + nowoff, isRead, txbuf + nowoff, rxbuf + nowoff, ST_ISP_MAX_TRANS_LEN);
				nowoff += ST_ISP_MAX_TRANS_LEN;
				nowlen -= ST_ISP_MAX_TRANS_LEN;
			}
			else
			{
				ret += st_v2_format(addr + nowoff, isRead, txbuf + nowoff, rxbuf + nowoff, nowlen);
				nowlen = 0;
			}
		}
	}
	else
	{
		while (nowlen > 0)
		{
			if (nowlen > ST_ISP_MAX_WRITE_LEN)
			{
				ret += st_v2_format(addr + nowoff, isRead, txbuf + nowoff, rxbuf + nowoff, ST_ISP_MAX_WRITE_LEN);
				nowoff += ST_ISP_MAX_WRITE_LEN;
				nowlen -= ST_ISP_MAX_WRITE_LEN;
			}
			else
			{
				ret += st_v2_format(addr + nowoff, isRead, txbuf + nowoff, rxbuf + nowoff, nowlen);
				nowlen = 0;
			}
		}
	}
	return ret;
}

static int st_v2_read_bytes(st_u16 addr, unsigned char *rxbuf, int len)
{
	//unsigned char tmp_buf[len];
	int ret;
	unsigned char *tmp_buf = NULL;
	tmp_buf = kzalloc(len, GFP_KERNEL);
	if (tmp_buf == NULL) {
		STX_ERROR("txbuf malloc fail");
		goto out;
	}
	ret = st_v2_len_check(addr,1,tmp_buf, rxbuf, len);

out:
	if (tmp_buf) {
		kfree(tmp_buf);
		tmp_buf = NULL;
	}
	return ret;	
}

static int st_v2_write_bytes(st_u16 addr, unsigned char *txbuf, int len)
{
	//unsigned char tmp_buf[len];
	int ret;
	unsigned char *tmp_buf = NULL;
	tmp_buf = kzalloc(len, GFP_KERNEL);
	if (tmp_buf == NULL) {
		STX_ERROR("txbuf malloc fail");
		goto out;
	}
	ret = st_v2_len_check(addr,0,txbuf, tmp_buf, len);	
out:
	if (tmp_buf) {
		kfree(tmp_buf);
		tmp_buf = NULL;
	}
	return ret;	
}

static int st_v2_set_2B_mode(void)
{
	unsigned char data[2];
	int rt = 0;
	data[0] = 0xF1;
	data[1] = 0x20;
	rt += stx_i2c_write_bytes(data, 2);
	if (rt < 0)
	{
		STX_ERROR("Set 2 byte mode error");
		return -1;
	}
	st_msleep(1);

	return 0;
}

int stx_clr_crash_msg(void)
{
	int addr;
	unsigned char buf[2];
	int len;

	addr = 0x72;
	len = 2;
	stx_cmdio_read(STX_AFE_REG_TYPE, addr,buf,len);
	STX_INFO("before buf0-1 = 0x%x,0x%x ",buf[0],buf[1]);

	addr = 0x8E;
	len = 2;
	buf[0] = 0x85;
	buf[1] = 0x01;
	stx_cmdio_write(STX_AFE_REG_TYPE,addr,buf,len);
	msleep(5);

	addr = 0x72;
	len = 2;
	stx_cmdio_read(STX_AFE_REG_TYPE, addr,buf,len);
	STX_INFO("after write buf0-1 = 0x%x,0x%x ",buf[0],buf[1]);

	return 0;
}

static int st_v2_isp_off(void)
{
	unsigned char data[1];
	int rt = 0;
	data[0] = 1;
	rt = st_v2_write_bytes(0x2, data, 1);
	if (rt < 0)
	{
		STX_ERROR("ISP off error");
		return -1;
	}
	st_msleep(300);

	stx_clr_crash_msg();
	return st_check_device_status(0, 4, 10);
}

static int st_v2_cmd_write(unsigned char *buf, int len, int delay)
{
	unsigned short pCheckSum = 0;
	unsigned char pStatus[8] = {0};
	int retryCount = 0;
	int isSuccess = 0;

	ChecksumCalculation(&pCheckSum, buf, len);
	buf[len] = pCheckSum & 0xFF; //CheckSum

	if (st_v2_write_bytes(0xD0, buf, len + 1) != len + 1)
	{
		STX_ERROR("ST_SPIW_1 Send 0xD0 fail.");
		return -1;
	}

	st_msleep(1);
	pStatus[0] = 1;
	if (st_v2_write_bytes(0xF8, pStatus, 1) != 1)
	{
		STX_ERROR("ST_SPIW_2 Send 0xF8 fail.");
		return -1;
	}
	st_msleep(delay);

	retryCount = 0;
	isSuccess = 0;

	while (isSuccess == 0 && retryCount++ < ST_ISP_RETRY_MAX)
	{
		if (st_v2_read_bytes(0xF8, pStatus, 1) != 1)
		{
			STX_ERROR("ST_SPIW_3 Read 0xF8  fail. retry %d", retryCount);
			isSuccess = 0;
		}
		else
			isSuccess = 1;

		if (isSuccess == 1)
		{
			if (pStatus[0] != 0)
			{
				STX_ERROR("ST_SPIW_4 status of 0xF8 error ,error:%x.", pStatus[0]);
				isSuccess = 0;
				st_msleep(10);
			}
			else
				isSuccess = 1;
		}
	}
	return isSuccess - 1;
}

static int st_v2_flash_read_offset(unsigned char *Buf, st_u32 off, unsigned short len)
{
	unsigned char PacketData[8];
	st_u32 offset;
	st_u32 readLen;
	st_u32 nowLen;

	offset = off;
	nowLen = len;
	while (nowLen > 0)
	{
		if (offset % ST_ISP_BLOCK_SIZE != 0)
			readLen = ST_ISP_BLOCK_SIZE - offset % ST_ISP_BLOCK_SIZE;
		else
			readLen = ST_ISP_BLOCK_SIZE;

		if (offset + readLen > off + len)
			readLen = off + len - offset;

		//read
		memset(PacketData, 0, 8);
		PacketData[0] = 0x15;
		PacketData[1] = 6;
		PacketData[2] = (offset >> 16) & 0xFF;
		PacketData[3] = (offset >> 8) & 0xFF;
		PacketData[4] = (offset)&0xFF;
		PacketData[5] = (readLen >> 8) & 0xff;
		PacketData[6] = (readLen)&0xFF;

		if (st_v2_cmd_write(PacketData, 7, 5) < 0)
			return -1;

		if (st_v2_read_bytes(0x200, Buf + (offset - off), readLen) != readLen)
		{
			STX_ERROR("ST_FR_2 Read Flash Data fail.");
			return -1;
		}

		nowLen -= readLen;
		offset += readLen;
	}
	return 0;
}

static int st_v2_flash_read_page(unsigned char *Buf, unsigned short PageNumber)
{
	unsigned char PacketData[8];
	int i;
	st_u32 offset;

	for (i = 0; i < ST_FLASH_PAGE_SIZE / ST_ISP_BLOCK_SIZE; i++)
	{
		memset(PacketData, 0, 8);
		PacketData[0] = 0x15;
		PacketData[1] = 6;
		offset = PageNumber * ST_FLASH_PAGE_SIZE + i * ST_ISP_BLOCK_SIZE;
		PacketData[2] = (offset >> 16) & 0xFF;
		PacketData[3] = (offset >> 8) & 0xFF;
		PacketData[4] = (offset)&0xFF;
		PacketData[5] = 0x1;
		PacketData[6] = 0x0;

		if (st_v2_cmd_write(PacketData, 7, 5) < 0)
			return -1;

		if (st_v2_read_bytes(0x200, Buf + (i * ST_ISP_BLOCK_SIZE), ST_ISP_BLOCK_SIZE) != ST_ISP_BLOCK_SIZE)
		{
			STX_ERROR("ST_FR_2 Read Flash Data fail.");
			return -1;
		}
	}
	return 0;
}

static int st_v2_flash_unlock(void)
{
	unsigned char PacketData[12];
	int retryCount = 0;
	int isSuccess = 0;

	PacketData[0] = 0x10;
	PacketData[1] = 9;
	PacketData[2] = 0x55;
	PacketData[3] = 0xAA;
	PacketData[4] = 0x55;
	PacketData[5] = 0x6E;
	PacketData[6] = 0x4C;
	PacketData[7] = 0x7F;
	PacketData[8] = 0x83;
	PacketData[9] = 0x9B;

	while (isSuccess == 0 && retryCount++ < ST_ISP_RETRY_MAX)
	{
		if (st_v2_cmd_write(PacketData, 10, 3) == 0)
			isSuccess = 1;

		if (isSuccess == 0)
		{
			STX_ERROR("Read ISP_Unlock_Ready packet fail retry : %d", retryCount);
		}
	}

	if (isSuccess == 0)
	{
		STX_ERROR("st_flash_unlock fail.");
		return -1;
	}
	return 0;
}

static int st_v2_flash_erase_page(unsigned short PageNumber)
{
	unsigned char PacketData[8];
	st_u32 offset;
	int retryCount = 0;
	int isSuccess = 0;

	PacketData[0] = 0x13;
	PacketData[1] = 4;
	offset = PageNumber * ST_FLASH_PAGE_SIZE;
	PacketData[2] = (offset >> 16) & 0xFF;
	PacketData[3] = (offset >> 8) & 0xFF;
	PacketData[4] = (offset)&0xFF;

	while (isSuccess == 0 && retryCount++ < ST_ISP_RETRY_MAX)
	{
		if (st_v2_cmd_write(PacketData, 5, 40) == 0)
			isSuccess = 1;

		if (isSuccess == 0)
		{
			STX_ERROR("Read ISP_Erase_Ready packet fail with page %d retry : %d", PageNumber, retryCount);
		}
	}
	if (isSuccess == 0)
	{
		STX_ERROR("st_flash_erase_page fail.");
		return -1;
	}
	return 0;
}

static int st_v2_flash_write_page(unsigned char *Buf, unsigned short PageNumber)
{
	unsigned char PacketData[10];
	unsigned char RetryCount;
	unsigned short DataCheckSum = 0;
	int i;
	st_u32 offset;

	for (i = 0; i < ST_FLASH_PAGE_SIZE / ST_ISP_BLOCK_SIZE; i++)
	{
		if (st_v2_flash_unlock() < 0)
			return -1;

		RetryCount = 0;
		PacketData[0] = 0x14;
		PacketData[1] = 7;
		offset = PageNumber * ST_FLASH_PAGE_SIZE + i * ST_ISP_BLOCK_SIZE;
		PacketData[2] = (offset >> 16) & 0xFF;
		PacketData[3] = (offset >> 8) & 0xFF;
		PacketData[4] = (offset)&0xFF;
		PacketData[5] = 0x1;
		PacketData[6] = 0x0;
		DataCheckSum = 0;
		ChecksumCalculation(&DataCheckSum, Buf + (i * ST_ISP_BLOCK_SIZE), ST_ISP_BLOCK_SIZE);

		PacketData[7] = DataCheckSum;

		if (st_v2_write_bytes(0x200, Buf + (i * ST_ISP_BLOCK_SIZE), ST_ISP_BLOCK_SIZE) != ST_ISP_BLOCK_SIZE)
		{
			STX_ERROR("ST_FW_1 Write Flash Data fail.");
			return -1;
		}
		if (st_v2_cmd_write(PacketData, 8, 5) < 0)
			return -1;
	}
	return 0;
}

static int st_v2_flash_write(unsigned char *Buf, int Offset, int NumByte)
{
	unsigned short StartPage;
	unsigned short PageOffset;
	int WriteNumByte;
	short WriteLength;
	unsigned char *TempBuf;
	int retry = 0;
	int isSuccess = 0;

	TempBuf = kzalloc(ST_FLASH_PAGE_SIZE, GFP_KERNEL);
	STX_DEBUG("Write flash offset:0x%X , length:0x%X", Offset, NumByte);

	WriteNumByte = 0;
	if (NumByte == 0)
		return WriteNumByte;

	if ((Offset + NumByte) > ST_FLASH_SIZE)
		NumByte = ST_FLASH_SIZE - Offset;

	StartPage = Offset / ST_FLASH_PAGE_SIZE;
	PageOffset = Offset % ST_FLASH_PAGE_SIZE;
	while (NumByte > 0)
	{
		if ((PageOffset != 0) || (NumByte < ST_FLASH_PAGE_SIZE))
		{
			if (st_v2_flash_read_page(TempBuf, StartPage) < 0)
				return -1;
		}

		WriteLength = ST_FLASH_PAGE_SIZE - PageOffset;
		if (NumByte < WriteLength)
			WriteLength = NumByte;
		memcpy(&TempBuf[PageOffset], Buf, WriteLength);

		retry = 0;
		isSuccess = 0;
		while (retry++ < 2 && isSuccess == 0)
		{
			if (st_v2_flash_unlock() >= 0 && st_v2_flash_erase_page(StartPage) >= 0)
			{
				STX_INFO("write page:%d", StartPage);
				if (st_v2_flash_write_page(TempBuf, StartPage) >= 0)
					isSuccess = 1;
			}
			if (isSuccess == 0)
				STX_INFO("FIOCTL_IspPageWrite write page %d retry: %d", StartPage, retry);
		}
		if (isSuccess == 0)
		{
			STX_ERROR("FIOCTL_IspPageWrite write page %d error", StartPage);
			return -1;
		}
		else
			StartPage++;

		NumByte -= WriteLength;
		Buf += WriteLength;
		WriteNumByte += WriteLength;
		PageOffset = 0;
	}
	kfree(TempBuf);
	return WriteNumByte;
}

unsigned char fw_check[ST_FLASH_PAGE_SIZE];

#ifdef ST_UPGRADE_BY_ISPID
unsigned char ispid_buf[] = SITRONIX_ISPIDS;

static int st_select_ispid(void)
{
	int ret = 0;
	unsigned int id_cnt;
	int len;
	unsigned int i;
	unsigned int j;
	unsigned int count = 0;

	ret = st_v2_flash_read_offset(fw_check,ISPID_OFFSET,ISPID_READ_LENGTH);
	if(ret < 0)
	{
		STX_ERROR("read ispid fail ");
		return ret;
	}
	len = fw_check[3] - 2 ;
	if((fw_check[0] == 0x50) &&(fw_check[1] == 0x49) &&(fw_check[2] == 0x44) && (len > 0))
	{
		id_cnt = (sizeof(ispid_buf) / sizeof(unsigned char)) / len;
		STX_INFO("id_cnt = 0x%x,len = 0x%x",id_cnt,len);
		if(id_cnt > 1)
		{
			for (i = 0; i < id_cnt; i++)
			{
				for(j = 0;j < len ;j++)
				{
					if(fw_check[6+j] == ispid_buf[i*len + j])
						count++;
				}
				if(count == len)
					return i;
				else
					count = 0;
			}
		}else{
			STX_ERROR("id_cnt is error ");
			return -1;
		}
	}else{
		STX_ERROR("ispid fail 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x",\
			fw_check[0],fw_check[1],fw_check[2],fw_check[3],fw_check[6],fw_check[7],fw_check[8],fw_check[9],fw_check[10]);
		return -1;
	}
	return -1;
}

#elif defined(ST_UPGRADE_BY_FWID)
unsigned char id_buf[] = SITRONIX_IDS;
st_u16 id_off[] = SITRONIX_ID_OFF;

static int st_select_fw_by_id(void)
{
	int ret = 0;
	unsigned char buf[8];
	unsigned char id[4];
	int i = 0;
	int idlen = sizeof(id_buf) / 4;
	int isFindID = 0;
	int status = st_get_device_status();
	if (status < 0)
	{
		return -1;
	}
	else if (status != 0x6)
	{
		stx_i2c_read_bytes(0xC, buf, 4);
		stx_i2c_read_bytes(0xF1, buf + 4, 1);
		buf[6] = buf[4];
		buf[5] = (buf[4] & 0xFC) | 1;
		buf[4] = 0xF1;
		stx_i2c_write_bytes(buf + 4, 2);
		st_msleep(1);

		stx_i2c_read_bytes(0xF1, id, 1);
		STX_DEBUG("customer bank: %x ", id[0]);

		stx_i2c_read_bytes(0xC, id, 4);
		buf[5] = buf[6];
		stx_i2c_write_bytes(buf + 4, 2);
		st_msleep(1);
		//STX_DEBUG("customer ids: %x %x %x %x ,buf %x %x %x %x ",id[0],id[1],id[2],id[3],buf[0],buf[1],buf[2],buf[3]);
		if (id[0] == buf[0] && id[1] == buf[1] && id[2] == buf[2] && id[3] == buf[3])
		{
			STX_ERROR("read customer id fail ");
			return -1;
		}
		else
			STX_INFO("customer ids: %x %x %x %x", id[0], id[1], id[2], id[3]);

		for (i = 0; i < idlen; i++)
		{
			if (id[0] == id_buf[i * 4] && id[1] == id_buf[i * 4 + 1] && id[2] == id_buf[i * 4 + 2] && id[3] == id_buf[i * 4 + 3])
			{
				isFindID = 1;
//				st_replace_fw_by_id(i);
				return i;
			}
		}
		if (0 == isFindID)
			return -1;
	}
	else
	{
		STX_INFO("IC's status : boot code ");
		// if bootcode
		if (0 == st_v2_isp_off())
		{
			//could ispoff
			STX_INFO("IC's could go normat status");
			return st_select_fw_by_id();
		}
		else
		{
			STX_INFO("IC really bootcode mode");
			ret = st_v2_flash_read_offset(fw_check, ST_CFG_OFFSET, ST_CFG_LEN);
			if (ret < 0)
			{
				STX_ERROR("read CFG fail! (%x)", ret);
				return -1;
			}

			for (i = 0; i < idlen; i++)
			{
				STX_DEBUG("customer ids: %x %x %x %x", fw_check[0 + id_off[i]], fw_check[1 + id_off[i]], fw_check[2 + id_off[i]], fw_check[3 + id_off[i]]);
				if (fw_check[0 + id_off[i]] == id_buf[i * 4] && fw_check[1 + id_off[i]] == id_buf[i * 4 + 1] && fw_check[2 + id_off[i]] == id_buf[i * 4 + 2] && fw_check[3 + id_off[i]] == id_buf[i * 4 + 3])
				{
					isFindID = 1;
//					st_replace_fw_by_id(i);
					return i;
				}
			}

			if (0 == isFindID)
			{
				//read from page 0
				STX_ERROR("Find ID in CFG fail , try to find in page 0");

				ret = st_v2_flash_read_offset(fw_check, 0, ST_CFG_LEN);
				if (ret < 0)
				{
					STX_ERROR("read flash fail! (%x)", ret);
					return -1;
				}
				for (i = 0; i < idlen; i++)
				{
					STX_DEBUG("customer ids: %x %x %x %x ", fw_check[0 + id_off[i]], fw_check[1 + id_off[i]], fw_check[2 + id_off[i]], fw_check[3 + id_off[i]]);
					if (fw_check[0 + id_off[i]] == id_buf[i * 4] && fw_check[1 + id_off[i]] == id_buf[i * 4 + 1] && fw_check[2 + id_off[i]] == id_buf[i * 4 + 2] && fw_check[3 + id_off[i]] == id_buf[i * 4 + 3])
					{
						STX_INFO("Find ID in page 0 ,customer ids: %x %x %x %x ", id_buf[i * 4], id_buf[i * 4 + 1], id_buf[i * 4 + 2], id_buf[i * 4 + 3]);
						isFindID = 1;
//						st_replace_fw_by_id(i);

						return i;
					}
				}
				if (0 == isFindID)
					return -1;
			}
		}
	}
	return -1;
}
#endif

#ifdef STX_UPGRADE_REQUEST_FW_FILE
static int stx_request_fw_file(char *path)
{
	const struct firmware *fw = NULL;
	int len;
	int ret;

	len = strlen(path);
	if (path[len - 1] == 10)
		path[len - 1] = 0;

	STX_DEBUG("%s", __func__);

	ret = request_firmware(&fw, path, &stx_gpts.client->dev);
	if (ret < 0)
	{
		STX_ERROR("fail to request_firmware fwpath: %s (ret:%d)", path, ret);
		return ret;
	}

	if (fw != NULL)
	{
		if (fw->size < (ST_FW_LEN + ST_CFG_LEN))
			STX_ERROR("upgrade fw_cfg size : 0x%x ", (unsigned int)fw->size);

		memcpy(fw_buf, fw->data, sizeof(uint8_t) * ST_FW_LEN);
		memcpy(cfg_buf, &fw->data[ST_CFG_OFFSET], sizeof(uint8_t) * ST_CFG_LEN);
		STX_INFO("cfg_buf[0~3] = 0x%x, 0x%x, 0x%x, 0x%x",cfg_buf[0],cfg_buf[1],cfg_buf[2],cfg_buf[3]);
	}
	else
	{
		STX_ERROR("%s: error fw = NULL", __func__);
		return -1;
	}

	STX_INFO("fw size %x ", (unsigned int)fw->size);
	release_firmware(fw);
	return 0;
}
#endif
int sitronix_get_fs_fw_version(struct device *dev, char *buf)
{
	//struct sitronix_ts_data *ts = dev_get_drvdata(dev);
	u8 fs_fw_version = 0;
	int fwSize = 0;
	int cfgSize = 0;
#ifdef ST_UPGRADE_VERSION_CTRL
	uint8_t customer_ver = 0;
#endif
	pr_buf_info(" %s enter \n",  __func__);

	fwSize = sizeof(fw_buf);
	cfgSize = sizeof(cfg_buf);
	STX_INFO("fwSize 0x%X,cfgsize 0x%X", fwSize, cfgSize);

	if (cfgSize != 0)
	{
#ifdef ST_UPGRADE_VERSION_CTRL
		customer_ver = cfg_buf[319];
		STX_INFO("The dump_file version:%d",customer_ver);

#endif
	}

	fs_fw_version = customer_ver;
	pr_buf_info(" %s 0x%02X \n",  __func__, fs_fw_version);
	return snprintf(buf, PAGE_SIZE, "0x%02X\n", fs_fw_version);
}
int sitronix_check_fw_update_need(struct device *dev)
{
	struct sitronix_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	char buffer[8];
	u8 fs_fw_version = 0;
	int fwSize = 0;
	int cfgSize = 0;
#ifdef ST_UPGRADE_VERSION_CTRL
	uint8_t customer_ver = 0;
#endif

	ret = stx_i2c_read(ts->client, buffer, 1, FIRMWARE_VERSION);
	if (ret != 1)
	{
		STX_ERROR(" i2c communcate error in getting FW version : 0x%x", ret);
	}
	STX_INFO("Touch Panel Firmware version %x", buffer[0]);
	ts->ts_dev_info.fw_version[0] = buffer[0];

	fwSize = sizeof(fw_buf);
	cfgSize = sizeof(cfg_buf);
	STX_INFO("fwSize 0x%X,cfgsize 0x%X", fwSize, cfgSize);

	if (cfgSize != 0)
	{
#ifdef ST_UPGRADE_VERSION_CTRL
		customer_ver = cfg_buf[319];
		STX_INFO("The dump_file version:%d",customer_ver);
#endif
	}
	fs_fw_version = customer_ver;
	STX_INFO("IC_FW_VERSION = 0x%02X, CFG_BUFF FW_VERSION = 0x%02X.",ts->ts_dev_info.fw_version[0],fs_fw_version);
#ifdef CONFIG_TOUCHSCREEN_SITRONIX_TS_L762
	if(fs_fw_version != ts->ts_dev_info.fw_version[0])
#else
	if(fs_fw_version > ts->ts_dev_info.fw_version[0])
#endif
	{
		STX_INFO("IC_FW_VERSION is Older then CFG_BUFF FW_VERSION, NEED UPDATE.");
		return true;
	}
	else
	{
		STX_INFO("IC_FW_VERSION is not Older then CFG_BUFF FW_VERSION, NOT NEED UPDATE.");
		return false;
	}
}


int st_upgrade_fw(void)
{
	int rt = 0;
	int fwSize = 0;
	int cfgSize = 0;
	int fwInfoOff = 0;
	int fwInfoLen = ST_FW_INFO_LEN;
	int powerfulWrite = 0;
	int idStatus;
#ifdef STX_UPGRADE_REQUEST_FW_FILE
	char path[100];
#endif

#ifdef ST_UPGRADE_VERSION_CTRL
	uint8_t customer_ver = 0;
#endif

	STX_FUNC_ENTER();
	st_print_version(&stx_gpts);
	idStatus = 0;

#ifdef ST_UPGRADE_BY_ISPID
	STX_INFO("UPGRADE BY ISPID");
#elif defined(ST_UPGRADE_BY_FWID)
	idStatus = st_select_fw_by_id();
	if (idStatus < 0)
	{
		STX_ERROR("find id fail , cancel upgrade");
		return -1;
	}
#endif

	if (st_get_device_status() == 0x6)
		powerfulWrite = 1;

	st_irq_off();

	rt = st_isp_on();
	if (rt != 0)
	{
		STX_ERROR("ISP on fail");
		goto ST_IRQ_ON;
	}

	if (st_check_chipid() < 0)
	{
		STX_ERROR("Check ChipId fail");
		rt = -1;
		goto ST_ISP_OFF;
	}
	rt = st_v2_set_2B_mode();
	if(rt < 0)
	{
		STX_ERROR("set 2B mode fail");
		rt = -1;
		goto ST_ISP_OFF;
	}

#ifdef ST_UPGRADE_BY_ISPID
	idStatus = st_select_ispid();
	if(idStatus < 0)
	{
		STX_ERROR("select ispid fail");
		goto ST_ISP_OFF;
	}
#elif defined(ST_UPGRADE_BY_FWID)
	STX_INFO("UPGRADE BY FWID");
#endif

#if defined(ST_UPGRADE_BY_ISPID) || defined(ST_UPGRADE_BY_FWID)

#ifdef STX_UPGRADE_REQUEST_FW_FILE
	snprintf(path, sizeof(path), "sitronix_%d.dump", idStatus);
	STX_INFO("get dumpfile path : %s",path);
	rt = stx_request_fw_file(path);
	if(rt < 0)
	{
		STX_ERROR("request fw file fail");
		goto ST_ISP_OFF;
	}
#else
	st_replace_fw_by_id(idStatus);
#endif

#else//defined(ST_UPGRADE_BY_ISPID) || defined(ST_UPGRADE_BY_FWID)

#ifdef STX_UPGRADE_REQUEST_FW_FILE
	snprintf(path, sizeof(path), "sitronix_%d.dump", idStatus);
	STX_INFO("get dumpfile path : %s",path);
	rt = stx_request_fw_file(path);
	if(rt < 0)
	{
		STX_ERROR("request fw file fail");
		goto ST_ISP_OFF;
	}
#endif

#endif//defined(ST_UPGRADE_BY_ISPID) || defined(ST_UPGRADE_BY_FWID)

	fwSize = sizeof(fw_buf);
	cfgSize = sizeof(cfg_buf);
	STX_INFO("fwSize 0x%X,cfgsize 0x%X", fwSize, cfgSize);

	if (fwSize != 0)
	{
		fwInfoOff = st_get_fw_info_offset(fwSize, fw_buf);
#ifdef ST_UPGRADE_VERSION_CTRL
		if (fwInfoOff < 0)
		{
			STX_ERROR("fail to get dumpFile fwInfoOff");
			goto ST_ISP_OFF;
		}
		customer_ver = cfg_buf[319];
		STX_INFO("The dump_file version:%d ,the TP version:%d ",customer_ver,stx_gpts.ts_dev_info.fw_version[0]);
		if (customer_ver <= stx_gpts.ts_dev_info.fw_version[0])
		{
			STX_ERROR("The dump_file version:%d is lower than the TP:%d ,cancel upgrade",customer_ver,stx_gpts.ts_dev_info.fw_version[0]);
			//goto ST_ISP_OFF;
		}
#endif
		//		fwInfoOff = st_get_fw_info_offset(fwSize,fw_buf);
		if (fwInfoOff < 0)
		{
			fwSize = 0;
			STX_ERROR("check fwInfoOff Len error (%x), cancel upgrade", fwInfoOff);
			goto ST_ISP_OFF;
		}
		fwInfoLen = fw_buf[fwInfoOff] * 0x100 + fw_buf[fwInfoOff + 1] + 2 + 3;
		STX_INFO("fwInfoOff 0x%X , fwInfoLen 0x%x", fwInfoOff, fwInfoLen);
	}

	cfgSize = min(cfgSize, ST_CFG_LEN);
	if (cfgSize != 0)
	{
		if (cfg_buf[0] != 0x43 || cfg_buf[1] != 0x46 || cfg_buf[2] != 0x54 || cfg_buf[3] != 0x31)
		{
			STX_ERROR("cfg_buf error (invalid CFG)");
			goto ST_ISP_OFF;
		}
	}

	if (fwSize == 0 && cfgSize == 0)
	{
		STX_ERROR("can't find FW or CFG , cancel upgrade");
		goto ST_ISP_OFF;
	}

	if (powerfulWrite == 0 && (fwSize != 0 || cfgSize != 0))
	{
		//check fw and cfg
		//int checkOff = (fwInfoOff / ST_FLASH_PAGE_SIZE) *ST_FLASH_PAGE_SIZE;
		if (st_v2_flash_read_offset(fw_check, fwInfoOff, fwInfoLen) < 0)
		{
			STX_ERROR("read flash fail , cancel upgrade");
			rt = -1;
			goto ST_ISP_OFF;
		}
		if (fwSize != 0)
		{
			if (0 == st_compare_array(fw_check, fw_buf + fwInfoOff, fwInfoLen))
			{
				STX_INFO("fw compare :same");
				fwSize = 0;
			}
			else
			{
				STX_INFO("fw compare :different");
			}
		}
		if (cfgSize != 0)
		{
			st_v2_flash_read_offset(fw_check, ST_CFG_OFFSET, ST_CFG_LEN);
			if (0 == st_compare_array(fw_check, cfg_buf, cfgSize))
			{
				STX_INFO("cfg compare :same");
				cfgSize = 0;
			}
			else
			{
				STX_INFO("cfg compare : different");
			}
		}
	}

#ifdef ST_UPGRADE_BY_ISPID
	STX_INFO("UPGRADE BY ISPID");
#elif defined(ST_UPGRADE_BY_FWID)
	if (cfgSize != 0 || fwSize != 0)
	{
		if (idStatus == 1)
			st_v2_flash_write(cfg_buf, ST_CFG_OFFSET, cfgSize);

		//backup CFG in page 0
		memcpy(fw_check, fw_buf, ST_FLASH_PAGE_SIZE);
		memcpy(fw_check, cfg_buf, cfgSize);
		st_v2_flash_write(fw_check, 0, ST_FLASH_PAGE_SIZE);
	}
#endif

	if (cfgSize != 0)
		st_v2_flash_write(cfg_buf, ST_CFG_OFFSET, cfgSize);

	if (fwSize != 0)
	{
		//write page 1 ~ N
		st_v2_flash_write(fw_buf + ST_FLASH_PAGE_SIZE, ST_FLASH_PAGE_SIZE, fwSize - ST_FLASH_PAGE_SIZE);
		//write page 0
		st_v2_flash_write(fw_buf, 0, ST_FLASH_PAGE_SIZE);
	}
#ifdef ST_UPGRADE_BY_ISPID
	STX_INFO("UPGRADE BY ISPID");
#elif defined(ST_UPGRADE_BY_FWID)
	//restore FW in page 0
	if (cfgSize != 0 && fwSize == 0)
		st_v2_flash_write(fw_buf, 0, ST_FLASH_PAGE_SIZE);
#endif

ST_ISP_OFF:
	rt = st_v2_isp_off();
ST_IRQ_ON:
	st_irq_on();

	if (cfgSize != 0 || fwSize != 0)
		return 1;
	else
		return rt;
}
int sitronix_proc_fw_update(struct device *dev, bool force)
{
	struct sitronix_ts_data *data = dev_get_drvdata(dev);

	if(!data->loading_fw)
	{
		data->loading_fw = true;
		STX_INFO("enter update process.");
		st_upgrade_fw();
		data->loading_fw = false;
	}else
	{
		STX_INFO("fw already enter updating!");
	}
	return 0;
}

static int stx_upgrade_fw_direct(unsigned char *fw_buf, unsigned char *cfg_buf, int fwSize, int cfgSize)
{
	int rt = 0;
	int fwInfoOff = 0;
	int fwInfoLen = ST_FW_INFO_LEN;
	//	int powerfulWrite = 0;

	STX_FUNC_ENTER();

	STX_DEBUG("fwSize 0x%X,cfgsize 0x%X", fwSize, cfgSize);
	if (fwSize != 0)
	{
		fwInfoOff = st_get_fw_info_offset(fwSize, fw_buf);
		if (fwInfoOff < 0)
			fwSize = 0;
		fwInfoLen = fw_buf[fwInfoOff] * 0x100 + fw_buf[fwInfoOff + 1] + 2 + 3;
		if (fwInfoOff == -1)
		{
			STX_ERROR("check fwInfoOff Len error (%x), cancel upgrade", fwInfoOff);
			return -1;
		}
		STX_INFO("fwInfoOff 0x%X , fwInfoLen 0x%x", fwInfoOff, fwInfoLen);
	}
	cfgSize = min(cfgSize, ST_CFG_LEN);
	if (cfgSize != 0)
	{
		if (cfg_buf[0] != 0x43 || cfg_buf[1] != 0x46 || cfg_buf[2] != 0x54 || cfg_buf[3] != 0x31)
		{
			STX_ERROR("cfg_buf error (invalid CFG)");
			return -1;
		}
	}
	if (fwSize == 0 && cfgSize == 0)
	{
		STX_ERROR("can't find FW or CFG , cancel upgrade");
		return -1;
	}

	if (st_get_device_status() == 0x0)
	{
		//		if(powerfulWrite ==0 &&(fwSize !=0 || cfgSize!=0))
		if ((fwSize != 0 || cfgSize != 0))
		{
			//check fw and cfg
			if (stx_cmdio_read(2, fwInfoOff, fw_check, fwInfoLen) < 0)
			{
				STX_ERROR("read flash fail , cancel upgrade");
				//				rt = -1;
				return -1;
				//				goto ST_ISP_OFF;
			}
			if (fwSize != 0)
			{
				if (0 == st_compare_array(fw_check, fw_buf + fwInfoOff, fwInfoLen))
				{
					STX_INFO("fw compare :same");
					fwSize = 0;
				}
				else
				{
					STX_INFO("fw compare :different");
				}
			}
			if (cfgSize != 0)
			{
				//			st_v2_flash_read_offset(fw_check,ST_CFG_OFFSET,ST_CFG_LEN);
				if (stx_cmdio_read(2, ST_CFG_OFFSET, fw_check, ST_CFG_LEN) < 0)
				{
					STX_ERROR("read flash fail , cancel upgrade");
					//					rt = -1;
					return -1;
					//					goto ST_ISP_OFF;
				}
				if (0 == st_compare_array(fw_check, cfg_buf, cfgSize))
				{
					STX_INFO("cfg compare :same");
					cfgSize = 0;
				}
				else
				{
					STX_INFO("cfg compare : different");
				}
			}
		}
	}

	if (fwSize == 0 && cfgSize == 0)
	{
		STX_ERROR(" FW and CFG are all same , cancel upgrade");
		return -1;
	}

	if (cfgSize != 0 || fwSize != 0)
	{
		//	else if(st_get_device_status() == 0x6)
		//    	powerfulWrite = 1;

		st_irq_off();

		rt = st_isp_on();
		if (rt != 0)
		{
			STX_ERROR("ISP on fail");
			goto ST_IRQ_ON;
		}

		if (st_check_chipid() < 0)
		{
			STX_ERROR("Check ChipId fail");
			rt = -1;
			goto ST_ISP_OFF;
		}
		st_v2_set_2B_mode();
	}

	if (cfgSize != 0)
		st_v2_flash_write(cfg_buf, ST_CFG_OFFSET, cfgSize);

	if (fwSize != 0)
	{
		//write page 1 ~ N
		st_v2_flash_write(fw_buf + ST_FLASH_PAGE_SIZE, ST_FLASH_PAGE_SIZE, fwSize - ST_FLASH_PAGE_SIZE);
		//write page 0
		st_v2_flash_write(fw_buf, 0, ST_FLASH_PAGE_SIZE);
	}

ST_ISP_OFF:
	rt = st_v2_isp_off();
ST_IRQ_ON:
	st_irq_on();

	STX_FUNC_EXIT();
	if (cfgSize != 0 || fwSize != 0)
		return 1;
	else
		return rt;
}

uint8_t *pFw_Buf = NULL;
uint8_t *pCfg_Buf = NULL;

int stx_force_upgrade_fw(char *path)
{
	const struct firmware *fw = NULL;
	//	char file_name[128] = "sitronix_upg.dump";
	int len;
	int ret;

	len = strlen(path);
	if (path[len - 1] == 10)
		path[len - 1] = 0;

	STX_DEBUG("%s", __func__);

	ret = request_firmware(&fw, path, &stx_gpts.client->dev);
	if (ret < 0)
	{
		STX_ERROR("fail to request_firmware fwpath: %s (ret:%d)", path, ret);
		return ret;
	}

	if (fw != NULL)
	{
		if (fw->size < (ST_FW_LEN + ST_CFG_LEN))
			STX_ERROR("upgrade fw_cfg size : 0x%x ", (unsigned int)fw->size);

		pFw_Buf = kzalloc((sizeof(uint8_t) * ST_FW_LEN), GFP_KERNEL);
		if (pFw_Buf == NULL)
		{
			STX_ERROR("Output buffer memory allocate fail");
			return -1;
		}
		memcpy(pFw_Buf, fw->data, sizeof(uint8_t) * ST_FW_LEN);

		pCfg_Buf = kzalloc((sizeof(uint8_t) * ST_CFG_LEN), GFP_KERNEL);
		if (pCfg_Buf == NULL)
		{
			STX_ERROR("Output buffer memory allocate fail");
			return -1;
		}
		memcpy(pCfg_Buf, &fw->data[ST_CFG_OFFSET], sizeof(uint8_t) * ST_CFG_LEN);
		STX_INFO("%s: pCfg_Buf [0] = %02X, [1] = %02X, [2] = %02X, [3] = %02X", __func__, pCfg_Buf[0], pCfg_Buf[1], pCfg_Buf[2], pCfg_Buf[3]);
	}
	else
	{
		STX_ERROR("%s: error fw = NULL", __func__);
		return -1;
	}

	STX_INFO("fw size %x ", (unsigned int)fw->size);
	release_firmware(fw);
	stx_upgrade_fw_direct(pFw_Buf, pCfg_Buf, ST_FW_LEN, ST_CFG_LEN);

	kfree(pFw_Buf);
	kfree(pCfg_Buf);

	return 0;
}

int st_upgrade_fw_handler(void *unused)
{
	int status = 0;
	status = st_upgrade_fw();
	st_print_version(&stx_gpts);
	return status;
}
