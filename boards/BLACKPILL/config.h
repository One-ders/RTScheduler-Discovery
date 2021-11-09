

//#define SYS_CLOCK	100000000
#define SYS_CLOCK	84000000   // PLL config


#define MAX_TASKS	256
#define TQ_SIZE		1024

#define USB_TX_BSIZE	16
#define USB_RX_BSIZE	64

#define USART_TX_BSIZE  1024
#define USART_RX_BSIZE  16

#define GPIO_PINMEM	256

/* define board */
#define BLACKPILL

#define SDRAM_SIZE	128*1024
#define BYTES_PER_LONG	4


/* Usart */
#define USE_USART	2
#define USART_TX_PIN	GPIO_PIN(PA,2)
#define USART_RX_PIN	GPIO_PIN(PA,3)

#if 1
/* Usb */
#define USB_DM		GPIO_PIN(PA,11)
#define USB_DP		GPIO_PIN(PA,12)
#endif

//define USB_VBUS	GPIO_PIN(PA,9)
//define USB_ID		GPIO_PIN(PA,10)


/* User leds */
#define USER_LEDS	1
#define LED_PINS 	GPIO_PIN(PC,13)
#define LED_BLUE	1


#define SYS_CONSOLE_DEV		"usb_serial0"
#define USER_CONSOLE_DEV	"usb_serial0"

#if 0
/* Cec */
#define CEC_PIN         GPIO_PIN(PC,4)

/* Sony A1 */
// previous use of PB0 changed, because of burned gpio pin
#define A1_PIN          GPIO_PIN(PC,5)

// OBD1 GM ALDL
#define OBD160_PIN          GPIO_PIN(PC,5)

/* Timer 1 Ch1 & Ch 2, AF1 */
#define TIM1_CH1_PIN	GPIO_PIN(PE,9)
#define TIM1_CH2_PIN	GPIO_PIN(PE,11)

/* Timer 8 ch1, AF3 */
#define TIM8_CH1_PIN	GPIO_PIN(PC,6)
#endif
