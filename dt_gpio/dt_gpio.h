/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "dt_common_types.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DT_GPIO_H_
#define DT_GPIO_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Macros --------------------------------------------------------------------*/
#define DT_GPIO_TABLE\
	X_GPIO(gpio_LED1, GPIOC, 13, OUT_10MHz, OUT_PP, LOW) \
	X_GPIO(gpio_BUTTON1, GPIOC, 14, INPUT, IN_PULL, UP) \
	
/* Types ---------------------------------------------------------------------*/
typedef struct{
	GPIO_TypeDef* GPIOx;		// Port name
	uint16_t GPIO_Pin;			// Pin number
	uint8_t mode;						// Pin mode
	uint8_t def_state;			// Default state (copy to ODR)
} dt_gpio_t;

typedef enum{
	#define X_GPIO(a,b,c,d,e,f) a,
	DT_GPIO_TABLE
	#undef X_GPIO
	DT_GPIO_PIN_COUNT
} dt_gpio_pin_t;

typedef enum{
	LOW = 0,
	HIGH = 1,
	DOWN = 0,
	UP = 1,
	DT_GPIO_STATE_COUNT = 2
} dt_gpio_state_t;

typedef enum{
	INPUT		= 0x00,
	OUT_10MHz	= 0x01,
	OUT_2MHz	= 0x02,
	OUT_50MHz	= 0x03,
	DT_GPIO_MODE_COUNT
} dt_gpio_mode_t;

typedef enum{
	OUT_PP		= 0x00,
	OUT_OD		= 0x04,
	OUT_APP		= 0x08,
	OUT_AOD		= 0x0C,

	IN_ADC		= 0x00,
	IN_HIZ		= 0x04,
	IN_PULL		= 0x08,
	DT_GPIO_PULL_COUNT
} dt_gpio_pull_t;

/* Macros --------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void dt_gpio_init(void);																						// init all pins from GPIO_TABLE
dt_status_t dt_gpio_init_pin(dt_gpio_pin_t pin, dt_gpio_mode_t mode, dt_gpio_pull_t pull, dt_gpio_state_t state);
																																		// init one pin from GPIO_TABLE. Initialization values may differ from GPIO_TABLE
dt_status_t dt_gpio_set(dt_gpio_pin_t pin, dt_gpio_state_t state);	// set one pin from GPIO_TABLE
dt_status_t dt_gpio_toggle(dt_gpio_pin_t pin);											// toggle one pin from GPIO_TABLE
uint8_t			dt_gpio_read(dt_gpio_pin_t pin);												// read one pin from GPIO_TABLE

#ifdef __cplusplus
}
#endif
#endif /* DT_GPIO_H_ */
