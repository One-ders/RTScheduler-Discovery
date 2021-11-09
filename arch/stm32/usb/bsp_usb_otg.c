#include <sys.h>
#include <io.h>
#include <devices.h>
#include <stm32f407.h>
#include <gpio_drv.h>

#include "usb_core.h"

static struct usb_dev_handle *usb_dev;

void OTG_FS_IRQHandler(void) {
	sys_irqs++;
	handle_device_irq(usb_dev);
}

void OTG_HS_IRQHandler(void) {
	sys_irqs++;
	handle_device_irq(usb_dev);
}

int bsp_usb_enable_interrupt(struct usb_dev_handle *pdev) {
#ifdef USB_OTG_HS_ADDR
	if (pdev->regs==USB_OTG_HS_ADDR) {
		NVIC_SetPriority(OTG_HS_IRQn,0xc);
		NVIC_EnableIRQ(OTG_HS_IRQn);
	} else {
		NVIC_SetPriority(OTG_FS_IRQn,0xc);
		NVIC_EnableIRQ(OTG_FS_IRQn);
	}
#else
	NVIC_SetPriority(OTG_FS_IRQn,0xc);
	NVIC_EnableIRQ(OTG_FS_IRQn);
#endif
	return 0;
}


int bsp_usb_init(struct usb_dev_handle *pdev) {
	struct driver *pindrv=driver_lookup(GPIO_DRV);
	struct device_handle *pin_dh;
	struct pin_spec ps[4];
	int pin_no=0;
	unsigned int flags;
	int altfn;

	usb_dev=pdev;

	if (!pindrv) return 0;
	pin_dh=pindrv->ops->open(pindrv->instance,0,0);
	if(!pin_dh) return 0;

#ifdef USB_OTG_HS_ADDR
	if (pdev->regs==USB_OTG_HS_ADDR) {
		altfn=12;
	} else {
		altfn=10;
	}
#else
	altfn=10;
#endif


#ifdef USB_ID
	ps[pin_no].pin=USB_ID;
	flags=GPIO_DIR(0,GPIO_ALTFN_PIN);
	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_ALTFN(flags,altfn);
	ps[pin_no].flags=flags;
	pin_no++;
#endif
#ifdef USB_VBUS
	ps[pin_no].pin=USB_VBUS;
	flags=GPIO_DIR(0,GPIO_INPUT);
	ps[pin_no].flags=flags;
	pin_no++;
#endif
	ps[pin_no].pin=USB_DM;
	flags=GPIO_DIR(0,GPIO_ALTFN_PIN);
	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_ALTFN(flags,altfn);
	ps[pin_no].flags=flags;
	pin_no++;
	ps[pin_no].pin=USB_DP;
	flags=GPIO_DIR(0,GPIO_ALTFN_PIN);
	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_ALTFN(flags,altfn);
	ps[pin_no].flags=flags;
	pin_no++;

	pindrv->ops->control(pin_dh,GPIO_BUS_ASSIGN_PINS,ps,pin_no*sizeof(struct pin_spec));

#ifdef USB_OTG_HS_ADDR
	if (pdev->regs==USB_OTG_HS_ADDR) {
		RCC->AHB1ENR|=RCC_AHB1ENR_OTGHSEN;
	} else {
		RCC->AHB2ENR|=RCC_AHB2ENR_OTGFSEN;
	}
#else
	RCC->AHB2ENR|=RCC_AHB2ENR_OTGFSEN;
#endif

	return 0;
}

void init_usb_core_drv(void) {

}

INIT_FUNC(init_usb_core_drv);
