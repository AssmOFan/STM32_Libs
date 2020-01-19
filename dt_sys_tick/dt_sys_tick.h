/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DT_SYS_TICK_
#define DT_SYS_TICK_

#ifdef __cplusplus
 extern "C" {
#endif

/* Macros --------------------------------------------------------------------*/
#define SYS_TICK_BITSIZE	16										// use only uint16_t or uint32_t
#define MIN_TIME_BITSIZE	SYS_TICK_BITSIZE/1		// must be <= SYS_TICK_BITSIZE
// use uint8_t (max_time == 240 ms), uint16_t (max_time == 65280 ms), uint32_t (max_time == 4 294 901 760 ms)

#define CONCAT_(A, B, C) A ## B ## C
#define CONCAT(A, B, C) CONCAT_(A, B, C)
#define sys_tick_t CONCAT(uint, SYS_TICK_BITSIZE, _t)
#define TimerTick	SystemCoreClock/1000-1
/* Types ---------------------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/
extern volatile sys_tick_t time_ms;

/* Functions -----------------------------------------------------------------*/
// for config SysTick, use CMSIS function: SysTick_Config(TimerTick);

#if (SYS_TICK_BITSIZE == 16 || SYS_TICK_BITSIZE == 32) && (MIN_TIME_BITSIZE == 8 || MIN_TIME_BITSIZE == 16 || MIN_TIME_BITSIZE == 32)
	#if MIN_TIME_BITSIZE < SYS_TICK_BITSIZE
	bool dt_check_time(void* time, uint8_t time_size);		// sizeof(time) <= sizeof(sys_tick_t)
	#elif MIN_TIME_BITSIZE == SYS_TICK_BITSIZE
	bool dt_check_time(sys_tick_t time);									// sizeof(time) == sizeof(sys_tick_t)
	#endif
#else
#error CHECK SYS_TICK_BITSIZE & MIN_TIME_BITSIZE
#endif

#ifdef __cplusplus
}
#endif
#endif /* DT_SYS_TICK_ */
