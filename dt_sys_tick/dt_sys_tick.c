#include "dt_sys_tick.h"
#include "stm32f10x.h"

volatile sys_tick_t time_ms;

void SysTick_Handler(void)
{
  time_ms++;
}

#if MIN_TIME_BITSIZE < SYS_TICK_BITSIZE
bool dt_check_time(void* time, uint8_t time_size)
{
	if (time_size == 1){
		if ((uint8_t)(time_ms - *(uint8_t*)time + 0xF0) >= 0xF0){
			return true;
		}
	}
	else if (time_size == 2){
		if ((uint16_t)(time_ms - *(uint16_t*)time + 0xFF00) >= 0xFF00){
			return true;
		}		
	}
	#if SYS_TICK_BITSIZE == 32
	else if (time_size == 4){
		if ((time_ms - *(uint32_t*)time + 0xFFFF0000) >= 0xFFFF0000){
			return true;
		}	
	}
	#endif	
	return false;
}

#elif MIN_TIME_BITSIZE == SYS_TICK_BITSIZE
bool dt_check_time(sys_tick_t time){
	#if SYS_TICK_BITSIZE == 16
	if ((uint16_t)(time_ms - time + 0xFF00) >= 0xFF00){
		return true;
	}
	#elif SYS_TICK_BITSIZE == 32
	if ((time_ms - time + 0xFFFF0000) >= 0xFFFF0000){
		return true;
	}
	#else
		#error CHECK SYS_TICK_BITSIZE & MIN_TIME_BITSIZE
	#endif
	return false;
}

#else
	#error CHECK SYS_TICK_BITSIZE & MIN_TIME_BITSIZE
#endif
