/* $RTSos: , v1.1 2021/09/23 21:44:00 anders Exp $ */

/* All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @(#)i2c_drv.c
 */
#include <sys.h>
#include <io.h>
#include <devices.h>
#include <gpio_drv.h>
#include <stm32f407.h>
#include <config.h>

#include "i2c_drv.h"

struct i2c_data {
	struct blocker_list rblocker_list;
	struct blocker_list wblocker_list;
};

struct user_data {
	struct device_handle	dh;
	struct i2c_data		*drv_data;
	DRV_CBH			callback;
	void			*userdata;
	int			events;
};

static struct user_data i2c_data0;

#define MAX_USERS 4
static struct user_data udata[MAX_USERS];

/*********************************************************/



/********************************************************/

void I2C3_EV_IRQHandler(void) {
	sys_printf("I2C3_ev_irq: sr1=%x, sr2=%x\n", I2C3->SR1, I2C3->SR2);
}

void I2C3_ER_IRQHandler(void) {
	sys_printf("I2C3_er_irq: sr1=%x, sr2=%x\n", I2C3->SR1, I2C3->SR2);
}


static struct device_handle *i2c_open(void *driver_instance, DRV_CBH cb_handler, void *userdata) {

	int i;
	for (i=0;i<MAX_USERS;i++) {
		if (udata[i].drv_data==0) break;
	}
	if (i>=MAX_USERS) return 0;
	udata[i].drv_data=driver_instance;
	udata[i].callback=cb_handler;
	udata[i].userdata=userdata;
	return &udata[i].dh;
}

static int i2c_close(struct device_handle *dh) {
	struct user_data *udata=(struct user_data *)dh;
	udata->drv_data=0;
	return 0;
}

static int i2c_control(struct device_handle *dh,
			int   cmd,
			void  *arg1,
			int   arg2) {
	struct user_data *udata=(struct user_data *)dh;
	struct i2c_data *i2c_d=udata->drv_data;

	switch(cmd) {
		case RD_CHAR: {
			sys_printf("got driver read\n");
			return 0;
		}
		case WR_CHAR: {
			sys_printf("got driver write\n");
			return 0;
		}
		case IO_POLL: {
			sys_printf("got IO_POLL\n");
			return 0;
		}
		default: {
			sys_printf("got an unhandled driver cmd %d\n", cmd);
			return -1;
		}
	}
	return 0;
}

static int i2c_init(void *instance) {
	sys_printf("driver init called\n");

	/* start clocks for this i2c chip */
	RCC->APB1ENR|=RCC_APB1ENR_I2C3EN;

	/* set bus clock to 42 Mhz, but on the MB1075 it is 45 */
	I2C3->CR2|=0x2a;
	I2C3->CCR=0xd2;    /* 1/42Mhz/5000 ns --> 100Khz SCL */
	I2C3->TRISE=0x02b;

	return 0;
}

static int i2c_start(void *instance) {
	sys_printf("driver start called\n");

	NVIC_SetPriority(I2C3_EVn, 0xd);
	NVIC_EnableIRQ(I2C3_EVn);

	NVIC_SetPriority(I2C3_ERn, 0xd);
	NVIC_EnableIRQ(I2C3_ERn);

	I2C3->CR2|=(I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

//	I2C3->CR1=I2C_CR1_SWRST|I2C_CR1_PE;
	I2C3->CR1=I2C_CR1_PE;
	return 0;
}

static struct driver_ops i2c_ops = {
	i2c_open,
	i2c_close,
	i2c_control,
	i2c_init,
	i2c_start
};

static struct driver i2c_0 = {
	"i2c_0",
	&i2c_data0,
	&i2c_ops,
};

void init_i2c_drv(void) {
	driver_publish(&i2c_0);
}

INIT_FUNC(init_i2c_drv);
