#include <string.h>
#include "stm32f10x.h"
#include "dt_gpio.h"
#include "dt_sys_tick.h"
#include "dt_uart.h"

#define	LED1_DELAY	1000	//ms
#define	TEST_UART		DT_UART1

int main(void)
{
	const char* message = "Hello!!!\r\n";
	
	// init all GPIO
	//dt_gpio_init();
	
	// init selected GPIO
	if (dt_gpio_init_pin(gpio_LED1,OUT_10MHz,OUT_PP,LOW) != DT_STATUS_OK){
		return -1;
	}
	
	if (SysTick_Config(TimerTick) != 0){
		return -1;
	}
	
	dt_uart_init(TEST_UART);
	
	dt_uart_write(TEST_UART, (uint8_t*)message, strlen(message));
	
	// set zero delay for the first check delay
	uint16_t led1_delay = (uint16_t)time_ms;
   
  while(1){
		#if MIN_TIME_BITSIZE < SYS_TICK_BITSIZE
		if (dt_check_time(&led1_delay,sizeof(led1_delay))){
		
		#elif MIN_TIME_BITSIZE == SYS_TICK_BITSIZE
		if (dt_check_time(led1_delay)){
		#endif
						
			uint8_t buff[10] = {0};
			uint16_t rx_count = dt_uart_get_rx_count(TEST_UART);

			if (rx_count != 0){
				if (dt_uart_read(TEST_UART, buff, rx_count) != rx_count){
					return -1;
				}
				
				while (dt_uart_busy(TEST_UART)) {}
					
				if (dt_uart_write(TEST_UART, buff, rx_count) != DT_STATUS_OK){
					return -1;
				}
			}
			
			led1_delay += LED1_DELAY;
			if (dt_gpio_toggle(gpio_LED1) != DT_STATUS_OK){
				return -1;
			}
		}
  }
}
