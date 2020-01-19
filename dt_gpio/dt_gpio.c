#include "dt_gpio.h"

const dt_gpio_t GPIOs[DT_GPIO_PIN_COUNT] =
{
	#define X_GPIO(a,b,c,d,e,f) {b,c,d|e,f},	
	DT_GPIO_TABLE
	#undef X_GPIO
};

void dt_gpio_enable_clock_pin(void)
{
	// Manual enable clock on ports
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	//RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	//RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
	//RCC->APB2ENR |= RCC_APB2ENR_IOPFEN;
	//RCC->APB2ENR |= RCC_APB2ENR_IOPGEN;
	
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}

void dt_gpio_config_pin(dt_gpio_pin_t pin, uint8_t mode, uint8_t state)
{
	// Check high or low register
	if(GPIOs[pin].GPIO_Pin < 8){
		// Clear bits		
		GPIOs[pin].GPIOx->CRL &= ~(0x0F << (GPIOs[pin].GPIO_Pin * 4));			
		// Write bitmask
		GPIOs[pin].GPIOx->CRL |= mode<<(GPIOs[pin].GPIO_Pin * 4);						
	}
	// Similar for high register
	else{
		GPIOs[pin].GPIOx->CRH &= ~(0x0F << ((GPIOs[pin].GPIO_Pin - 8)* 4)); 
		GPIOs[pin].GPIOx->CRH |= mode<<((GPIOs[pin].GPIO_Pin - 8)* 4);
	}
	// Write ODR (set default state)
	GPIOs[pin].GPIOx->ODR &= ~(1<<GPIOs[pin].GPIO_Pin);										
	GPIOs[pin].GPIOx->ODR |= state<<GPIOs[pin].GPIO_Pin;
}

void dt_gpio_init(void)
{
	dt_gpio_enable_clock_pin();
	// Config every pin in cycle
	for (uint8_t pin = 0; pin < DT_GPIO_PIN_COUNT; pin++){
		dt_gpio_config_pin((dt_gpio_pin_t)pin, GPIOs[pin].mode, GPIOs[pin].def_state);
	}
}

dt_status_t dt_gpio_init_pin(dt_gpio_pin_t pin, dt_gpio_mode_t mode, dt_gpio_pull_t pull, dt_gpio_state_t state)
{
	if (pin < DT_GPIO_PIN_COUNT || mode < DT_GPIO_MODE_COUNT || pull < DT_GPIO_PULL_COUNT || state < DT_GPIO_STATE_COUNT){
		dt_gpio_enable_clock_pin();
		// Config one pin
		dt_gpio_config_pin(pin, (uint8_t) mode | pull, (uint8_t) state);
		return DT_STATUS_OK;
	}
	else{
		return DT_STATUS_INVALID_PARAMETERS;
	}
}

dt_status_t dt_gpio_set(dt_gpio_pin_t pin, dt_gpio_state_t state)
{
	if (pin < DT_GPIO_PIN_COUNT || state < DT_GPIO_STATE_COUNT){
		if ((GPIOs[pin].mode & 0x03) != 0){
			if (state){	 
				GPIOs[pin].GPIOx->BSRR = 1 << GPIOs[pin].GPIO_Pin;
			}
			else{
				GPIOs[pin].GPIOx->BRR = 1 << GPIOs[pin].GPIO_Pin;
			}
			return DT_STATUS_OK;
		}
	}
	return DT_STATUS_INVALID_PARAMETERS;
}

dt_status_t dt_gpio_toggle(dt_gpio_pin_t pin)
{
	if (pin < DT_GPIO_PIN_COUNT){
		if ((GPIOs[pin].mode & 0x03) != 0){
			GPIOs[pin].GPIOx->ODR ^= 1 << GPIOs[pin].GPIO_Pin;
			return DT_STATUS_OK;
		}
	}
	return DT_STATUS_INVALID_PARAMETERS;
}

uint8_t dt_gpio_read(dt_gpio_pin_t pin)
{
	if (pin < DT_GPIO_PIN_COUNT){
		if ((GPIOs[pin].mode & 0x03) == 0){
			return ((GPIOs[pin].GPIOx->IDR) & (1<<GPIOs[pin].GPIO_Pin));
		}
	}
	return DT_STATUS_INVALID_PARAMETERS;
}
