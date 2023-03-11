/* $RTS: , v1.1 2014/04/07 21:44:00 anders Exp $ */

/*
 * Copyright (c) 2014, Anders Franzen.
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
 * @(#)led_drv.c
 */
#include "stm32f407.h"
#include "devices.h"
#include "sys.h"
#include <gpio_drv.h>
#include "led_drv.h"

static struct driver *pindrv;
static struct device_handle *pin_dh;
static int started=0;
static unsigned int led_map;  // Led 1 bit, defined 0x0000000f
			      // Led 2 bit, defined 0x000000f0
			      // ...

/*************************  Led driver ***************************/

// Blackpill is active low, so invert all on off actions

static int led_control(struct device_handle *dh, int cmd, void *arg1, int arg2) {
        switch(cmd) {
                case LED_CTRL_STAT: {
			unsigned int leds=0;
			unsigned int bits=0;
			int i;

			if (arg2<4) return -1;
			pindrv->ops->control(pin_dh, GPIO_BUS_READ_BITS, &bits, arg2);
			for(i=0;i<USER_LEDS;i++) {
				leds|=(bits&(1<<((led_map>>(4*i))&0xf)))?(1<<i):0;
			}
#ifdef BLACKPILL
			*((unsigned int *)arg1)=~leds;    // for blackpill, led is active low
#else
			*((unsigned int *)arg1)=leds;

#endif
                        return 0;
		}
                case LED_CTRL_ACTIVATE: {
			unsigned int leds=*(unsigned int *)arg1;
			int i;
			int pins=0;
			if (arg2<4) return -1;

			for(i=0;i<USER_LEDS;i++) {
				if (leds&(1<<i)) {
					pins|=1<<((led_map>>(i*4))&0xf);
				}
			}

#ifdef BLACKPILL
			pindrv->ops->control(pin_dh, GPIO_BUS_CLR_BITS, &pins, arg2);
#else
			pindrv->ops->control(pin_dh, GPIO_BUS_SET_BITS, &pins, arg2);
#endif
			break;
		}
                case LED_CTRL_DEACTIVATE: {
			unsigned int leds=*(unsigned int *)arg1;
			int i;
			int pins=0;

			if (arg2<4) return -1;

			for(i=0;i<USER_LEDS;i++) {
				if (leds&(1<<i)) {
					pins|=1<<((led_map>>(i*4))&0xf);
				}
			}

#ifdef BLACKPILL
			pindrv->ops->control(pin_dh, GPIO_BUS_SET_BITS, &pins, arg2);
#else
			pindrv->ops->control(pin_dh, GPIO_BUS_CLR_BITS, &pins, arg2);
#endif
			break;
		}
                default:
                        return -1;
        }
        return 0;
}

static int led_close(struct device_handle *dh) {
        return 0;
}

static int led_start(void *inst);
static struct device_handle *led_open(void *instance, DRV_CBH cb_handler, void *dum) {
	if (!started) led_start(0);
        return pin_dh;
}


static int led_init(void *inst) {
#if 0
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // enable the clock to GPIOD
	GPIOD->MODER |= (1 << 24); // set pin 12 to be general purpose output
	GPIOD->MODER |= (1 << 26); // set pin 13 to be general purpose output
	GPIOD->MODER |= (1 << 28); // set pin 14 to be general purpose output
	GPIOD->MODER |= (1 << 30); // set pin 15 to be general purpose output
#endif
	return 0;
};

static int led_start(void *inst) {
	int rc;
	struct pin_spec ps;
	unsigned int flags;
	unsigned int pins=LED_PORT_PINS;
	int i;
	led_map=LED_PORT_PINS;
	if (started) return 0;
	pindrv=driver_lookup(GPIO_DRV);
	if (!pindrv) return 0;
	pin_dh=pindrv->ops->open(pindrv->instance,0,0);
	if (!pin_dh) return 0;
	flags=GPIO_DIR(0,GPIO_OUTPUT);
	ps.pins=0;
	ps.port=LED_PORT;
	ps.flags=flags;
	for(i=0;i<USER_LEDS;i++) {
		ps.pins|=(1<<((pins>>(i*4))&0xf));
	}
	rc=pindrv->ops->control(pin_dh,GPIO_BUS_ASSIGN_PINS,&ps,sizeof(ps));

	// start with leds off
#ifdef BLACKPILL
	pindrv->ops->control(pin_dh, GPIO_BUS_SET_BITS, &ps.pins, sizeof(ps.pins));
#else
	pindrv->ops->control(pin_dh, GPIO_BUS_CLR_BITS, &ps.pins, sizeof(ps.pins));
#endif
	started=1;
	return rc;
}

static struct driver_ops led_drv_ops = {
        led_open,
        led_close,
        led_control,
	led_init,
	led_start,
};

static struct driver led_drv = {
	LED_DRV,
	0,
	&led_drv_ops,
};

void init_led_drv(void) {
	driver_publish(&led_drv);
}

INIT_FUNC(init_led_drv);
