/* 
* vbat convert to capacity..
* Copyright (c) 2018, 2021, Hisense HCMT All rights reserved.
*/
#ifndef __QG_VBAT2CAP_H__
#define	__QG_VBAT2CAP_H__

#define MAX_BAT_CAP_LENGTH		(8)
struct vbat2Cap {
	int			vbat_uv;
	int			capacitymAh;
};
const struct vbat2Cap		bat_cap_array[MAX_BAT_CAP_LENGTH]={
	{4340000,	5263},
	{4300000,	5098},
	{4250000,	4879},
	{4200000,	4645},
	{4150000,	4404},
	{4100000,	4153},
	{4050000,	3896},
	{4000000,	3641}	
};

#endif /*__QG_VBAT2CAP_H__*/
