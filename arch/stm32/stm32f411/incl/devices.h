
#include <busses.h>
#include <usart_reg.h>
#include <rcc_reg.h>
#include <gpio_reg.h>
#include <timer_reg.h>
#include <power_reg.h>
#include <flash_reg.h>
#include <iwdg_reg.h>
#include <usb_otg_reg.h>
#include <i2c_reg.h>


#define TIM2	((struct TIMER *)(APB1+0x0000))
#define TIM3	((struct TIMER *)(APB1+0x0400))
#define TIM4	((struct TIMER *)(APB1+0x0800))
#define TIM5	((struct TIMER *)(APB1+0x0c00))
//define RTC_BKP ((struct whatever *)(APB1+0x2800))
//define WWDG   ((struct WWDG *)(APB1+0x2C00))
#define IWDG	((struct IWDG *)(APB1+0x3000))
//define I2S2ex ((xxxx        *)(APB1+0x3400))
//define SPI2   ((SPI         *)(APB1+0x3800))
//deifne SPI3   ((SPI         *)(APB1+0x3C00))
//define I2S3ex ((xxxx        *)(APB1+0x4000))
#define USART2	((struct USART *)(APB1+0x4400))
#define I2C1	((struct I2C *)(APB1+0x5400))
#define I2C2	((struct I2C *)(APB1+0x5800))
#define I2C3	((struct I2C *)(APB1+0x5C00))
#define PWR	((struct POWER *)(APB1+0x7000))


#define TIM1	((struct TIMER *)(APB2+0x0000))
#define USART1	((struct USART *)(APB2+0x1000))
#define USART6	((struct USART *)(APB2+0x1400))
//define ADC1   ((struct xxxx  *)(APB2+0x2000))
//define SDIO   ((struct xxxx  *)(APB2+0x2c00))
//define SPI1   ((SPI         *)(APB2+0x3000))
//define SPI4   ((SPI         *)(APB2+0x3400))
//define SYSCFG ((SYSCFG      *)(APB2+0x3800))
//define EXTI   ((EXTRI       *)(APB2+0x3C00))
#define TIM9	((struct TIMER *)(APB2+0x4000))
#define TIM10	((struct TIMER *)(APB2+0x4400))
#define TIM11	((struct TIMER *)(APB2+0x4800))
//deifne SPI5   ((SPI         *)(APB2+0x5000))


#define GPIOA	((struct GPIO *)(AHB1+0x0000))
#define GPIOB	((struct GPIO *)(AHB1+0x0400))
#define GPIOC	((struct GPIO *)(AHB1+0x0800))
#define GPIOD	((struct GPIO *)(AHB1+0x0C00))
#define GPIOE	((struct GPIO *)(AHB1+0x1000))
#define GPIOH	((struct GPIO *)(AHB1+0x1C00))
#define RCC	((struct RCC *)(AHB1+0x3800))
#define FLASH	((struct FLASH *)(AHB1+0x3c00))
//define DMA1   ((struct DMA *)(AHB1+0x6000))
//define DMA2   ((struct DMA *)(AHB1+0x6400))

#define USB_OTG_FS_ADDR	((struct usb_otg_regs *)(AHB2))

