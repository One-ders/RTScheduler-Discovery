/* $RTSOs: , v1.1 2014/04/07 21:44:00 anders Exp $ */

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
 * @(#)gpio_drv.c
 */
#include <devices.h>
#include <stm32f407.h>
#include <sys.h>
#include <io.h>
#include <string.h>

#include "gpio_drv.h"

struct GPIO_REG {
	volatile unsigned int moder;
	volatile unsigned int otyper;
	volatile unsigned int ospeedr;
	volatile unsigned int pupdr;
	volatile unsigned int idr;
	volatile unsigned int odr;
	volatile unsigned int bsrr;
	volatile unsigned int lckr;
	volatile unsigned int afrl;
	volatile unsigned int afrh;
};

struct GPIO_REG *GPIO[9];

#define PORT_A	0x0001
#define PORT_B	0x0002
#define PORT_C	0x0004
#define PORT_D	0x0008
#define PORT_E	0x0010
#define PORT_F	0x0020
#define PORT_G  0x0040
#define PORT_H  0x0080
#define PORT_I  0x0100

#if defined(BLACKPILL)
const unsigned int ports_mask=PORT_A|PORT_B|PORT_C|PORT_D|PORT_E|PORT_H;
static unsigned short int pinmap[PORT_H+1];
#else
const unsigned int ports_mask=PORT_A|PORT_B|PORT_C|PORT_D|
				PORT_E|PORT_F|PORT_G|PORT_H|
				PORT_I;
static unsigned short int pinmap[PORT_I+1];
#endif


#define PIN_FLAGS_BUS		0x20000000
#define PIN_FLAGS_ASSIGNED	0x40000000
#define PIN_FLAGS_IN_USE	0x80000000

// pin_flags 0xaabbccdd
// aa == ASSIGNED and IN_USE and BUS
// if !BUS
// 	cc altfnc
// 	dd pinflags
//
// if BUS
// 	bb numpins in in bus
// 	cc altfnc
// 	dd pinflags

struct pin_data {
	struct device_handle dh;
	unsigned int flags;
	unsigned short port;
	unsigned short pins;
	/*=======================*/
	void		*userdata;
	DRV_CBH 	callback;
};

struct exti_regs {
	volatile unsigned int imr;
	volatile unsigned int emr;
	volatile unsigned int rtsr;
	volatile unsigned int ftsr;
	volatile unsigned int swier;
	volatile unsigned int pr;
};

struct exti_regs * const exti_regs=(struct exti_regs *)(APB2+0x3c00);

#define MAX_USERS 32
static struct pin_data pd[MAX_USERS];

static struct pin_data *exti2pd[16];

unsigned int * const exti_cr=(unsigned int *)(APB2+0x3808);

static unsigned int get_user(struct pin_data **pdpptr) {
	int i;
	for(i=0;i<MAX_USERS;i++) {
		if (!pd[i].flags) {
			pd[i].flags=PIN_FLAGS_IN_USE;
			(*pdpptr)=&pd[i];
			return i;
		}
	}
	return -1;
}

/*************** The Irqers *************************/

void EXTI0_IRQHandler(void) {
	enable_interrupts();
	if (exti2pd[0]) {
		if (exti2pd[0]->callback) {
			exti2pd[0]->callback(&exti2pd[0]->dh,EV_STATE,exti2pd[0]->userdata);
		}
	}
	exti_regs->pr=(1<<0);
}

void EXTI1_IRQHandler(void) {
	enable_interrupts();
	if (exti2pd[1]) {
		if (exti2pd[1]->callback) {
			exti2pd[1]->callback(&exti2pd[1]->dh,EV_STATE,exti2pd[1]->userdata);
		}
	}
	exti_regs->pr=(1<<1);
}

void EXTI2_IRQHandler(void) {
	enable_interrupts();
	if (exti2pd[2]) {
		if (exti2pd[2]->callback) {
			exti2pd[2]->callback(&exti2pd[2]->dh,EV_STATE,exti2pd[2]->userdata);
		}
	}
	exti_regs->pr=(1<<2);
}

void EXTI3_IRQHandler(void) {
	enable_interrupts();
	if (exti2pd[3]) {
		if (exti2pd[3]->callback) {
			exti2pd[3]->callback(&exti2pd[3]->dh,EV_STATE,exti2pd[3]->userdata);
		}
	}
	exti_regs->pr=(1<<3);
}

void EXTI4_IRQHandler(void) {
	enable_interrupts();
	if (exti2pd[4]) {
		if (exti2pd[4]->callback) {
			exti2pd[4]->callback(&exti2pd[4]->dh,EV_STATE,exti2pd[4]->userdata);
		}
	}
	exti_regs->pr=(1<<4);
}

void EXTI9_5_IRQHandler(void) {
	unsigned short int stat=exti_regs->pr&0x03e0;
	int i;
	enable_interrupts();
	for (i=5;i<10;i++) {
		if (stat&&(1<<i)) {
			if (exti2pd[i] && exti2pd[i]->callback) {
				exti2pd[i]->callback(&exti2pd[i]->dh,EV_STATE,exti2pd[i]->userdata);
			}
		}
	}
	exti_regs->pr=stat;
}

void EXTI15_10_IRQHandler(void) {
	enable_interrupts();
	sys_printf("exti15_10_irq\n");
	exti_regs->pr=0xfc00;
}


/**************** Support functions **********************/

static int assign_pin(struct pin_data *pdp, int bpin) {
	int pin=bpin&0xf;
	int port=(bpin>>4)&0xf;

	if (pdp->flags&PIN_FLAGS_ASSIGNED) {
		return -1;
	}

	if (pinmap[port]&(1<<pin)) {
		sys_printf("pin in use %x:%x\n", port,pin);
		return -1;
	}

	if (!pdp->pins) {
		pdp->port=port;
	} else {
		if (pdp->port!=port) return -1;
	}

	if (!pinmap[port]) {
		RCC->AHB1ENR|=(1<<port);
	}

	pinmap[port]|=(1<<pin);
	pdp->pins|=(1<<pin);
	return 0;
}

static int deassign_pin(struct pin_data *pdp, int bpin) {
	int pin=bpin&0xf;
	int port=(bpin>>4)&0xf;

	if (!(pdp->flags&PIN_FLAGS_ASSIGNED)) {
		return 0;
	}
//	sys_printf("lookup pin %d at bus %d\n",bpin,bus);

	if (!(pdp->pins&(1<<pin))) {
		return 0;
	}

	pdp->pins&=~(1<<pin);
	pinmap[pdp->port]&=~(1<<pin);
	// make pin input?
	if (!pinmap[port]) {
		RCC->AHB1ENR&=~(1<<port);
	}
	return 0;
}

static int read_pin(struct pin_data *pdp) {
	if (!(pdp->flags&PIN_FLAGS_ASSIGNED)) {
		return -1;
	}
	return (pdp->port<<4)|(ffs(pdp->pins)-1);
}

static int sense_pin(struct pin_data *pdp) {
	unsigned int rc;
	if ((rc=(GPIO[pdp->port]->idr&pdp->pins))!=0) {
		return rc;
	}
	return 0;
}

static int out_pin(struct pin_data *pdp, unsigned int val) {
	if (val) {
		GPIO[pdp->port]->bsrr=pdp->pins;
	} else {
		GPIO[pdp->port]->bsrr=pdp->pins<<16;
	}
	return 0;
}

static int sink_pin(struct pin_data *pdp) {
	GPIO[pdp->port]->moder|=(1<<(((ffs(pdp->pins)-1)<<1)));
	return 0;
}

static int release_pin(struct pin_data *pdp) {
	exti_regs->pr=(1<<pdp->pins);
	GPIO[pdp->port]->moder&=~(3<<((ffs(pdp->pins)-1)<<1));
	return 0;
}

static int set_flags(struct pin_data *pdp, unsigned int flags, unsigned int bpin) {
	int dir=flags&GPIO_DIR_MASK;
	int drive=(flags&GPIO_DRIVE_MASK)>>GPIO_DRIVE_SHIFT;
	int speed=(flags&GPIO_SPEED_MASK)>>GPIO_SPEED_SHIFT;
	int altfn=(flags&GPIO_ALTFN_MASK)>>GPIO_ALTFN_SHIFT;
	int bus,pin;
	bus=(bpin>>4)&0xf;
	pin=bpin&0xf;

	if ((dir==GPIO_OUTPUT)||(dir==GPIO_BUSPIN)) {
//		sys_printf("pin is output or bus (%x:%x)\n", bus,pin);
		GPIO[bus]->moder&=~(3<<(pin<<1));
		if (dir==GPIO_OUTPUT) {
			GPIO[bus]->moder|=(1<<(pin<<1));
		}

		if ((drive==GPIO_OPENDRAIN)||(dir==GPIO_BUSPIN)) {
			GPIO[bus]->otyper|=(1<<pin);
		} else if (drive==GPIO_PUSHPULL) {
			GPIO[bus]->otyper&=~(1<<pin);
		}

		GPIO[bus]->ospeedr&=~(3<<(pin<<1));
		GPIO[bus]->ospeedr|=(speed<<(pin<<1));
	}

	if (dir==GPIO_BUSPIN) {    /* make sure output is low */
		GPIO[bus]->bsrr=(1<<(pin+16));
	}

	if ((dir==GPIO_INPUT)||(dir==GPIO_BUSPIN)) {
//		sys_printf("pin is input or bus (%x:%x)\n", bus,pin);
		GPIO[bus]->moder&=~(3<<(pin<<1));

		switch(drive) {
			case GPIO_FLOAT: {
//				sys_printf("gpio float (%x:%x)\n",bus,pin);
				GPIO[bus]->pupdr&=~(3<<(pin<<1));
				break;
			}
			case GPIO_PULLUP: {
//				sys_printf("gpio pullup (%x:%x)\n",bus,pin);
				GPIO[bus]->pupdr&=~(3<<(pin<<1));
				GPIO[bus]->pupdr|=(1<<(pin<<1));
				break;
			}
			case GPIO_PULLDOWN: {
//				sys_printf("gpio_pulldown (%x:%x)\n",bus,pin);
				GPIO[bus]->pupdr&=~(3<<(pin<<1));
				GPIO[bus]->pupdr|=(2<<(pin<<1));
				break;
			}
			default: {
				sys_printf(" gpio unknown drive mode %d\n",drive);
			}
		}
	}

	if (dir==GPIO_ALTFN_PIN) {
//		sys_printf("pin is altfn (%x:%x)=%x\n",bus,pin,altfn);
		GPIO[bus]->moder&=~(3<<(pin<<1));
		GPIO[bus]->moder|=(2<<(pin<<1));
		if (speed) {
			GPIO[bus]->ospeedr&=~(3<<(pin<<1));
			GPIO[bus]->ospeedr|=(speed<<(pin<<1));
		}

//		if ((drive==GPIO_OPENDRAIN)||(dir==GPIO_BUSPIN)) {

		if (drive==GPIO_PUSHPULL) {
			GPIO[bus]->otyper&=~(1<<pin);
		} else {
			GPIO[bus]->otyper|=(1<<pin);
		}

#if 0
		switch(drive) {
			case GPIO_FLOAT: {
				sys_printf("gpio float\n");
				GPIO[bus]->pupdr&=~(3<<(pin<<1));
				break;
			}
			case GPIO_PULLUP: {
				sys_printf("gpio pullup\n");
				GPIO[bus]->pupdr&=~(3<<(pin<<1));
				GPIO[bus]->pupdr|=(1<<(pin<<1));
				break;
			}
			case GPIO_PULLDOWN: {
				sys_printf("gpio_pulldown\n");
				GPIO[bus]->pupdr&=~(3<<(pin<<1));
				GPIO[bus]->pupdr|=(2<<(pin<<1));
				break;
			}
			default: {
				sys_printf(" gpio drive mode %d\n",drive);
			}
		}
#endif
	}

	if (altfn) {
		if (pin<8) {
			GPIO[bus]->afrl&=~(0xf<<(pin<<2));
			GPIO[bus]->afrl|=(altfn<<(pin<<2));
		} else {
			GPIO[bus]->afrh&=~(0xf<<((pin-8)<<2));
			GPIO[bus]->afrh|=(altfn<<((pin-8)<<2));
		}
	}

	if (flags&GPIO_IRQ) {
		int exticr=pin>>2;
		int exticrshift=(pin&0x3)<<2;

		if ((exti2pd[pin]) && (exti2pd[pin]!=pdp)) {
			sys_printf("interrupt line cannot not be assigned, in use\n");
			return -1;
		}

		exti2pd[pin]=pdp;

		if ((pin>=0)&&(pin<5)) {
			NVIC_SetPriority(pin+6,0x1);
			NVIC_EnableIRQ(pin+6);
		} else if ((pin>=5)&&(pin<10)) {
			NVIC_SetPriority(23,0x1);
			NVIC_EnableIRQ(23);
		} else if ((pin>=10)&&(pin<16)) {
			NVIC_SetPriority(40,0x1);
			NVIC_EnableIRQ(40);
		}


		/* Syscfg exticr */
		exti_cr[exticr]&=~(0xf<<exticrshift);
		exti_cr[exticr]|=(bus<<exticrshift);

		exti_regs->rtsr|=(1<<pin);
		exti_regs->ftsr|=(1<<pin);
		exti_regs->imr|=(1<<pin);
	}
	return 0;
}

static int clr_flags(struct pin_data *pdp, unsigned int flags, unsigned int bpin) {

	unsigned int pin=bpin&0xf;

	if (flags&GPIO_IRQ) {
		exti_regs->imr&=~pdp->pins;
	}
	GPIO[pdp->port]->moder&=~(3<<(pin<<1));
	return 0;
}



/**********************************************************/
static int gpio_init(void *inst);

static struct device_handle *gpio_open(void *instance, DRV_CBH callback, void *userdata) {
	struct pin_data *pd=0;
	int ix=get_user(&pd);
//	sys_printf("gpio_open rc=%d\n",ix);
	if (ix<0) return 0;
	pd->userdata=userdata;
	pd->callback=callback;
	return &pd->dh;
}

static int gpio_close(struct device_handle *dh) {
	struct pin_data *pdp=(struct pin_data *)dh;
	int i;
//	sys_printf("gpio_close: port %x, pins %x\n", pdp->port, pdp->pins);
	for(i=0;i<16;i++) {
		if (pdp->pins&(1<<i)) {
			int flags=0;
			flags=GPIO_DIR(0,GPIO_INPUT);
			flags=GPIO_DRIVE(flags,GPIO_FLOAT);
			set_flags(pdp,flags,(pdp->port<<4)|i);
			deassign_pin(pdp,(pdp->port<<4)|i);
		}
	}
//	pdp->flags&=~PIN_FLAGS_ASSIGNED;
//	pdp->flags&=~PIN_FLAGS_IN_USE;
	pdp->port=0;
	pdp->flags=0;
	return 0;
}

static int gpio_control(struct device_handle *dh, int cmd, void *arg1, int arg2) {
	struct pin_data *pdp=(struct pin_data *)dh;
	if (!(pdp->flags&PIN_FLAGS_IN_USE)) return -1;
	switch(cmd) {
		case GPIO_BIND_PIN: {
			unsigned int pin;
			int rc=-1;
			if (arg2!=4) return rc;
			pin=*((unsigned int *)arg1);
			rc=assign_pin(pdp,pin);
			if (rc<0) return rc;
			pdp->flags|=PIN_FLAGS_ASSIGNED;
			break;
		}
		case GPIO_UNBIND_PIN: {
			unsigned int pin;
			int rc=-1;
			if (arg2!=4) return rc;
			pin=*((unsigned int *)arg1);
			rc=deassign_pin(pdp,pin);
			if (rc<0) return rc;
			pdp->flags&=~PIN_FLAGS_ASSIGNED;
			pdp->port=0;
			break;
		}
		case GPIO_GET_BOUND_PIN: {
			if (arg2!=4) return -1;
			*((unsigned int *)arg1)=read_pin(pdp);
			return 0;
			break;
		}
		case GPIO_SET_FLAGS: {
			unsigned int flags;
			if (arg2!=4) return -1;
			flags=*((unsigned int *)arg1);
			set_flags(pdp,flags,(pdp->port<<4)|(ffs(pdp->pins)-1));
			return 0;
		}
		case GPIO_CLR_FLAGS: {
			unsigned int flags;
			if (arg2!=4) return -1;
			flags=*((unsigned int *)arg1);
			clr_flags(pdp,flags,(pdp->port<<4)|(ffs(pdp->pins)-1));
			return 0;
		}
		case GPIO_SENSE_PIN: {
			if (arg2!=4) return -1;
			*((unsigned int *)arg1)=sense_pin(pdp);
			return 0;
			break;
		}
		case GPIO_SET_PIN: {
			if (arg2!=4) return -1;
			out_pin(pdp,*((unsigned int *)arg1));
			return 0;
			break;
		}
		case GPIO_SINK_PIN: {
			return sink_pin(pdp);
			break;
		}
		case GPIO_RELEASE_PIN: {
			return release_pin(pdp);
			break;
		}
		case GPIO_BUS_ASSIGN_PINS: {
			struct pin_spec *ps=(struct pin_spec *)arg1;
			int i;
			pdp->flags|=PIN_FLAGS_BUS;
			pdp->pins=ps->pins;
			pdp->port=ps->port;
			for(i=0;i<16;i++) {
				if (ps->pins&(1<<i)) {
					assign_pin(pdp,(pdp->port<<4)|i);
					set_flags(pdp,ps->flags,(pdp->port<<4)|i);
				}
			}
			pdp->flags|=PIN_FLAGS_ASSIGNED;
			break;
		}
		case GPIO_BUS_DEASSIGN_PINS: {
			int i;
			for(i=0;i<16;i++) {
				if (pdp->pins&(1<<i)) {
					deassign_pin(pdp,(pdp->port<<4)|i);
				}
			}
			pdp->flags&=~PIN_FLAGS_ASSIGNED;
			break;
		}
		case GPIO_BUS_UPDATE_FLAGS: {
			unsigned int flags;
			if (arg2!=4) return -1;
			flags=*((unsigned int *)arg1);
			int i;
			for(i=0;i<16;i++) {
				if (pdp->pins&(1<<i)) {
					set_flags(pdp,flags,(pdp->port<<4)|i);
				}
			}
			break;
		}
		case GPIO_BUS_READ_BUS: {
			unsigned int *bits=(unsigned int *)arg1;
			int port=pdp->port;
			int pv;
			if (!(pdp->flags&PIN_FLAGS_BUS)) return -1;
			pv=GPIO[port]->idr;
			*bits=pv&pdp->pins;
			break;
		}
		case GPIO_BUS_WRITE_BUS: {
			unsigned int *bits=(unsigned int *)arg1;
			int port=pdp->port;
			int pv;
			if (!(pdp->flags&PIN_FLAGS_BUS)) return -1;
			pv=GPIO[port]->idr&~pdp->pins;
			GPIO[port]->odr=pv|*bits;
			break;
		}
		case GPIO_BUS_SET_BITS: {
			unsigned int bits=*(unsigned int *)arg1;
			int port=pdp->port;
			int pins;
			if (!(pdp->flags&PIN_FLAGS_BUS)) return -1;
			pins=bits&pdp->pins;
			GPIO[port]->bsrr=pins;
			break;
		}
		case GPIO_BUS_CLR_BITS: {
			unsigned int bits=*(unsigned int *)arg1;
			int port=pdp->port;
			int pins;
			if (!(pdp->flags&PIN_FLAGS_BUS)) return -1;
			pins=bits&pdp->pins;
			GPIO[port]->bsrr=(pins<<16);
			break;
		}
	}
	return 0;
}

static int gpio_init(void *inst) {
	int i;
	// 0 1 2 3 4 5 6 7 8 9
	// A B C D E F G H I J
	for(i=0;i<9;i++) {
		if (ports_mask&(1<<i)) {
			GPIO[i]=(struct GPIO_REG *)(AHB1+(i*0x400));
		}
	}
	RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;
	return 0;
}

static int gpio_start(void *inst) {
	return 0;
}

struct driver_ops gpio_drv_ops = {
	gpio_open,
	gpio_close,
	gpio_control,
	gpio_init,
	gpio_start,
};

static struct driver gpio_drv = {
	GPIO_DRV,
	0,
	&gpio_drv_ops,
};

void init_gpio_drv(void) {
	driver_publish(&gpio_drv);
}



INIT_FUNC(init_gpio_drv);
