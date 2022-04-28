

//#define SYS_CLOCK	100000000
#define SYS_CLOCK	84000000   // PLL config


#define MAX_TASKS	256
#define TQ_SIZE		1024

#define USB_TX_BSIZE	128 // lower than 128 will result in a bootup crash
#define USB_RX_BSIZE	512 //  multiples of 64

#define USART_TX_BSIZE  1024
#define USART_RX_BSIZE  16

/* define board */
#define BLACKPILL

#define SDRAM_SIZE	128*1024
#define BYTES_PER_LONG	4

//#define USB_VENDOR	0x24,0x04
//#define USB_PRODUCT	0x4e,0x27
#define USB_VENDOR	0x48,0x25
#define USB_PRODUCT	0x01,0x10

/* Usart */
#define USE_USART	2
#define USART_TX_PIN	GPIO_PIN(GPIO_PA,2)
#define USART_RX_PIN	GPIO_PIN(GPIO_PA,3)

#if 1
/* Usb */
#define USB_DM		GPIO_PIN(GPIO_PA,11)
#define USB_DP		GPIO_PIN(GPIO_PA,12)
#endif

//define USB_VBUS	GPIO_PIN(GPIO_PA,9)
//define USB_ID		GPIO_PIN(GPIO_PA,10)


/* Macros defined by the LED drv
 * User leds */
#define USER_LEDS	1
#define LED_PORT	GPIO_PC
#define LED_PORT_PINS 	13
#define LED_BLUE	1

#ifndef SYS_CONSOLE_DEV
#define SYS_CONSOLE_DEV		"usart0"
//#define SYS_CONSOLE_DEV		"usb_serial0"
#endif
#ifndef USER_CONSOLE_DEV
#define USER_CONSOLE_DEV	"usart0"
//#define USER_CONSOLE_DEV	"usb_serial0"
#endif

#if 0
/* Cec */
#define CEC_PIN         GPIO_PIN(GPIO_PC,4)

/* Sony A1 */
// previous use of PB0 changed, because of burned gpio pin
#define A1_PIN          GPIO_PIN(GPIO_PC,5)

// OBD1 GM ALDL
#define OBD160_PIN          GPIO_PIN(GPIO_PC,5)

/* Timer 1 Ch1 & Ch 2, AF1 */
#define TIM1_CH1_PIN	GPIO_PIN(GPIO_PE,9)
#define TIM1_CH2_PIN	GPIO_PIN(GPIO_PE,11)

/* Timer 8 ch1, AF3 */
#define TIM8_CH1_PIN	GPIO_PIN(GPIO_PC,6)
#endif
