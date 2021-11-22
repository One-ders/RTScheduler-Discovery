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
 * @(#)tsc_drv.c
 */
#include <sys.h>
#include <io.h>
#include <devices.h>
#include <gpio_drv.h>
#include <stm32f407.h>
#include <config.h>

#include "tsc_drv.h"

static struct device_handle *i2c3_sda_dh;
static struct device_handle *i2c3_scl_dh;
static struct driver	    *pindrv;

struct tsc_data {
	struct blocker_list rblocker_list;
	struct blocker_list wblocker_list;
};

struct user_data {
	struct device_handle	dh;
	struct tsc_data		*drv_data;
	DRV_CBH			callback;
	void			*userdata;
	int			events;
};

static struct user_data tsc_data0;

#define MAX_USERS 4
static struct user_data udata[MAX_USERS];

/*********************************************************/



/********************************************************/

static struct device_handle *tsc_open(void *driver_instance, DRV_CBH cb_handler, void *userdata) {

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

static int tsc_close(struct device_handle *dh) {
	struct user_data *udata=(struct user_data *)dh;
	udata->drv_data=0;
	return 0;
}

static int tsc_control(struct device_handle *dh,
			int   cmd,
			void  *arg1,
			int   arg2) {
	struct user_data *udata=(struct user_data *)dh;
	struct tsc_data *tsc_d=udata->drv_data;

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

/* connected to I2C3 :  SDA PC9
 * 			INT PA15
 * 			SCL PA8
 */

/* chip: STMPE811QTR */


static int tsc_init(void *instance) {
	sys_printf("tsc driver init called\n");
	return 0;
}

static int tsc_start(void *instance) {
	int rc;
	unsigned int flags;
	int i2c3_sda=GPIO_PIN(GPIO_PC,9);
	int i2c3_scl=GPIO_PIN(GPIO_PA,8);
	sys_printf("tsc driver start called\n");

	/* connect tsc to i2c inputs */

	pindrv=driver_lookup(GPIO_DRV);
	if (!pindrv) return 0;
	i2c3_sda_dh=pindrv->ops->open(pindrv->instance,0,0);
	if (!i2c3_sda_dh) {
		sys_printf("tsc driver failed to open pin drv\n");
		return -1;
	}

	i2c3_scl_dh=pindrv->ops->open(pindrv->instance,0,0);
	if (!i2c3_scl_dh) {
		sys_printf("tsc driver failed to open pin drv\n");
		return -1;
	}

	rc=pindrv->ops->control(i2c3_sda_dh,GPIO_BIND_PIN,&i2c3_sda,sizeof(i2c3_sda));
	if (rc<0) {
		goto closeGPIO;
	}

	rc=pindrv->ops->control(i2c3_scl_dh,GPIO_BIND_PIN,&i2c3_scl,sizeof(i2c3_scl));
	if (rc<0) {
		goto closeGPIO;
	}

	flags=GPIO_DIR(0,GPIO_ALTFN_PIN);
	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_ALTFN(flags,4);
	rc=pindrv->ops->control(i2c3_sda_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		goto closeGPIO;
	}

	rc=pindrv->ops->control(i2c3_scl_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		goto closeGPIO;
	}

	return 0;

closeGPIO:
	sys_printf("failed to start tsc driver\n");
	pindrv->ops->close(i2c3_sda_dh);
	pindrv->ops->close(i2c3_scl_dh);
	return 0;
}

static struct driver_ops tsc_ops = {
	tsc_open,
	tsc_close,
	tsc_control,
	tsc_init,
	tsc_start
};

static struct driver tsc_0 = {
	"tsc_0",
	&tsc_data0,
	&tsc_ops,
};

void init_tsc_drv(void) {
	driver_publish(&tsc_0);
}

INIT_FUNC(init_tsc_drv);
