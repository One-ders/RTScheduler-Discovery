/* $RTOs: , v1.1 2023/03/12 13:44:00 anders Exp $ */

/*
 * Copyright (c) 2023, Anders Franzen.
 * All rights reserved.
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
 * @(#)pwr_mgr_drv.c
 */


#include <sys.h>
#include <system_params.h>

#include "pwr_mgr_drv.h"

static struct device_handle my_dh;

static int set_clock_high() {
	driver_prep_clk_update(SYS_CLK_HI);
	sys_clk_high_speed();
	config_sys_tic(10);
	driver_do_clk_update(SYS_CLK_HI);
	return 0;
}

static int set_clock_low() {
	driver_prep_clk_update(SYS_CLK_LO);
	sys_clk_low_speed();
	config_sys_tic(10);
	driver_do_clk_update(SYS_CLK_LO);
	return 0;
}

//////////////////// Driver user api /////////////////////

static struct device_handle *pwr_mgr_open(void *driver_instance, DRV_CBH cb_handler, void *dum) {
	return &my_dh;
}

static int pwr_mgr_close(struct device_handle *dh) {
	return 0;
}


static int pwr_mgr_control(struct device_handle *dh,
				int cmd,
				void *arg1,
				int arg2) {

	switch(cmd) {
		case GET_POWER_MODE: {
			*(unsigned int *)arg1=power_mode;
			return 0;
			break;
		}
		case GET_SYS_CLOCK: {
			*(unsigned int *)arg1=SystemCoreClock;
			return 0;
			break;
		}
		case SET_POWER_MODE: {
			unsigned int pmode=*(unsigned int *)arg1;
			power_mode&=~0xE;
			if (pmode&1) {
				if ((power_mode&1)==0) {
					power_mode|=1;
					set_clock_high();
				}
			} else {
				if ((power_mode&1)) {
					power_mode&=~1;
					set_clock_low();
				}
			}

			if (pmode&POWER_MODE_WAIT_WFI) {
				power_mode|=POWER_MODE_WAIT_WFI;
			} else if (pmode&POWER_MODE_WAIT_WFE) {
				power_mode|=POWER_MODE_WAIT_WFE;
			}

			if (pmode&POWER_MODE_DEEP_SLEEP) {
				power_mode|=POWER_MODE_DEEP_SLEEP;
			}
			return 0;
			break;
		}
	}
	sys_printf("pwr_mgr: got unknown control %d\n", cmd);
	return -1;
}

static int pwr_mgr_init(void *instance) {
	return 0;
}

static int pwr_mgr_start(void *instance) {
	return 0;
}

static struct driver_ops pwr_mgr_ops = {
	pwr_mgr_open,
	pwr_mgr_close,
	pwr_mgr_control,
	pwr_mgr_init,
	pwr_mgr_start,
};


static struct driver pwr_mgr = {
	"pwr_mgr",
	0,
	&pwr_mgr_ops,
};

void init_pwr_mgr(void) {
	driver_publish(&pwr_mgr);
}

INIT_FUNC(init_pwr_mgr);
