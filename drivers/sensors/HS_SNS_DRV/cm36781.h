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

#ifndef __CM36781_H
#define __CM36781_H

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/sensors.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>

#define CM36781_I2C_NAME "cm36781"
#define ALS_U_DELAY_PER_ATIME 50000
/* cm36781 Identifier  */
#define CM36781_ID 0x1058

#define CM36781_VDD_MIN_UV 4800000
#define CM36781_VDD_MAX_UV 5000000


/* Define Slave Address*/
#define	CM36781_slave_add	(0xA2>>1)

/*Define Command Code*/
#define		ALS_CONF	  0x00
#define		ALS_THDH  	  0x01
#define		ALS_THDL	  0x02
#define		PS_CONF1_2    0x03
#define		PS_CONF3      0x04
#define		PS_CANC       0x05
#define		PS_THDL       0x06
#define		PS_THDH       0x07
#define		PS_DATA       0x08
#define		ALS_DATA      0x09
#define		INT_FLAG      0x0D
#define		ID_REG        0x0E

/*cm36781*/
/*for ALS CONF command*/
#define CM36781_ALS_IT_50MS 	(0 << 6)
#define CM36781_ALS_IT_100MS 	(1 << 6)
#define CM36781_ALS_IT_200MS 	(2 << 6)
#define CM36781_ALS_IT_400MS 	(3 << 6)
#define CM36781_ALS_IT_MASK     (~CM36781_ALS_IT_400MS)

#define CM36781_ALS_PERS_1 		(0 << 2)
#define CM36781_ALS_PERS_2 		(1 << 2)
#define CM36781_ALS_PERS_4 		(2 << 2)
#define CM36781_ALS_PERS_8 		(3 << 2)
#define CM36781_ALS_PERS_MASK   (~CM36781_ALS_PERS_8)

#define CM36781_ALS_INT_EN	 	(1 << 1) /*enable/disable Interrupt*/
#define CM36781_ALS_INT_MASK	 0xFFFD
#define CM36781_ALS_SD			(1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define CM36781_ALS_SD_MASK		 0xFFFE

/*for PS CONF1_2 command*/
#define CM36781_PS_INT_OFF	       (0 << 8) /*enable/disable Interrupt*/
#define CM36781_PS_INT_IN          (1 << 8)
#define CM36781_PS_INT_OUT         (2 << 8)
#define CM36781_PS_INT_IN_AND_OUT  (3 << 8)

#define CM36781_PS_INT_MASK        0xFCFF

#define CM36781_PS_DR_1_160   (0 << 6)
#define CM36781_PS_DR_1_320   (1 << 6)
#define CM36781_PS_DR_1_640   (2 << 6)
#define CM36781_PS_DR_1_1280  (3 << 6)
#define CM36781_PS_DR_1_MASK  (~CM36781_PS_DR_1_1280)

#define CM36781_PS_PERS_1     (0 << 4)
#define CM36781_PS_PERS_2     (1 << 4)
#define CM36781_PS_PERS_3     (2 << 4)
#define CM36781_PS_PERS_4     (3 << 4)
#define CM36781_PS_PERS_MASK  (~CM36781_PS_PERS_4)

#define CM36781_PS_IT_1T      (0 << 1)
#define CM36781_PS_IT_1_5T    (1 << 1)
#define CM36781_PS_IT_2T      (2 << 1)
#define CM36781_PS_IT_4T      (3 << 1)
#define CM36781_PS_IT_8T      (4 << 1)
#define CM36781_PS_IT_9T      (5 << 1)
#define CM36781_PS_IT_MASK    (~(7 << 1))

#define CM36781_PS_SD	      (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define CM36781_PS_SD_MASK     0xFFFE

#define CM36781_PS_HD_16      (1 << 11)
#define CM36781_PS_HD_12      (~CM36781_PS_HD_16)

/*for PS CONF3 command*/
#define CM36781_PS_MS_NORMAL          (0 << 13)
#define CM36781_PS_MS_LOGIC_ENABLE    (1 << 13)
#define CM36781_LED_I_50              (0 << 8)
#define CM36781_LED_I_75              (1 << 8)
#define CM36781_LED_I_100             (2 << 8)
#define CM36781_LED_I_120             (3 << 8)
#define CM36781_LED_I_140             (4 << 8)
#define CM36781_LED_I_160             (5 << 8)
#define CM36781_LED_I_180             (6 << 8)
#define CM36781_LED_I_200             (7 << 8)
#define CM36781_LED_I_MASK            (~CM36781_LED_I_200)

#define CM36781_PS_MP_1               (0 << 5) //before is 0 << 6, as for the below 3 items
#define CM36781_PS_MP_2               (1 << 5)
#define CM36781_PS_MP_4               (2 << 5)
#define CM36781_PS_MP_8               (3 << 5)
#define CM36781_PS_MP_MASK            (~CM36781_PS_MP_8)

#define CM36781_PS_SMART_PERS_ENABLE  (1 << 4)
#define CM36781_PS_ACTIVE_FORCE_MODE  (1 << 3)
#define CM36781_PS_ACTIVE_FORCE_TRIG  (1 << 2)

/*for INT FLAG*/
#define INT_FLAG_PS_SAFLAG           (1 << 15)
#define INT_FLAG_PS_SPFLAG           (1 << 14)
#define INT_FLAG_ALS_IF_L            (1 << 13)
#define INT_FLAG_ALS_IF_H            (1 << 12)
#define INT_FLAG_PS_IF_CLOSE         (1 << 9)
#define INT_FLAG_PS_IF_AWAY          (1 << 8)
#define INT_FLAG_MASK                (0xFF << 8)

struct cm36781_als_data_t {
	u16 raw;
	int lux;
};

struct cm36781_ps_data_t {
	bool state_changed;
	u16 raw;
	int state;
	int last_state;
	int near_last_count;
	int far_last_count;
};


struct cm36781_chip {
	struct i2c_client *client;
	struct mutex mlock;
    struct wakeup_source wake_src;
	struct workqueue_struct *workqueue;
	struct work_struct ps_work;

	atomic_t in_suspend;
	int wake_irq;
	unsigned int irq_gpio;

	u32 vdd_3v_gpio;
	u32 vdd_3v_gpio_flags;
	u32 vdd_5v_gpio;
	u32 vdd_5v_gpio_flags;

	bool unpowered;
	int als_take_effect;
	int ps_take_effect;

	struct input_dev *p_idev;
	struct sensors_classdev ps_cdev;
	struct cm36781_ps_data_t ps_inf;
	struct delayed_work ps_poll_dwork;
	int ps_enabled;
	bool ps_functioned;
	bool detected_change;

	atomic_t ps_poll_work_on;
	atomic_t engtest_mode;
	u16 fac_canc;

	u16 ps_th_min;
	u16 ps_th_max;
	u32 ps_thd_away;
	u32 ps_thd_close;
	u16 ps_thd_more_close;
	u16 ps_thd_more_close_base;
	u16 ps_thd_covered;
	u16 poffset;
	u16 ps_crosstalk_max;
	
	u16 ps_duty;
	u16 ptime;
	u16 ps_pers;  //persistence
	u16 ps_mp;    //multi_pulse
	u16 ps_led_drv;
	bool ps_si;   //sunlight_immunity
	bool first_als_value;
	u16 ps_conf1_2_val;
	u16 ps_conf3_val;

	struct input_dev *a_idev;
	struct sensors_classdev als_cdev;
	struct cm36781_als_data_t als_inf;
	struct delayed_work als_dwork;
	int als_enabled;
	bool als_functioned;
	int als_u_delay;
	int als_raw_max;
	u16 atime;
	u16 ls_cmd;
	int als_resolution;
	int als_fittness;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;

	struct regulator *vdd;
	struct regulator *vio;
};

#endif /* __CM36781_H */
