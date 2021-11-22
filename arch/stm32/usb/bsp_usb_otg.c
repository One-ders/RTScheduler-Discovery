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


#define PORT(a) ((a>>4)&0xf)
#define PIN(a)	(a&0xf)

int bsp_usb_init(struct usb_dev_handle *pdev) {
	struct driver *pindrv=driver_lookup(GPIO_DRV);
	struct device_handle *pinO_dh;
	struct device_handle *pinI_dh;
	struct pin_spec pso;
	struct pin_spec psi;
	unsigned int flagso;
	int altfn;

	usb_dev=pdev;

	if (!pindrv) return 0;
	pinO_dh=pindrv->ops->open(pindrv->instance,0,0);
	if(!pinO_dh) return 0;

#ifdef USB_VBUS
	pinI_dh=pindrv->ops->open(pindrv->instance,0,0);
	if(!pinI_dh) return 0;
#endif

#ifdef USB_OTG_HS_ADDR
	if (pdev->regs==USB_OTG_HS_ADDR) {
		altfn=12;
	} else {
		altfn=10;
	}
#else
	altfn=10;
#endif

	pso.pins=0;

	flagso=GPIO_DIR(0,GPIO_ALTFN_PIN);
	flagso=GPIO_SPEED(flagso,GPIO_SPEED_HIGH);
	flagso=GPIO_DRIVE(flagso,GPIO_PUSHPULL);
	flagso=GPIO_ALTFN(flagso,altfn);
	pso.flags=flagso;
	pso.port=PORT(USB_DM);

#ifdef USB_ID
	pso.pins|=(1<<PIN(USB_ID));
#endif
#ifdef USB_VBUS
	psi.port=PORT(USB_VBUS);
	psi.pins=0;
	psi.flags=GPIO_DIR(0,GPIO_INPUT);
	psi.pins|=(1<<PIN(USB_VBUS));
#endif
	pso.pins|=(1<<PIN(USB_DM));
	pso.pins|=(1<<PIN(USB_DP));

	pindrv->ops->control(pinO_dh,GPIO_BUS_ASSIGN_PINS,&pso,sizeof(struct pin_spec));
	pindrv->ops->control(pinI_dh,GPIO_BUS_ASSIGN_PINS,&psi,sizeof(struct pin_spec));

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
