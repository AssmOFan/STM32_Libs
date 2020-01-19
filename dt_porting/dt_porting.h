#ifndef __DT_PORTING_H
#define __DT_PORTING_H

#if defined(WIN32)
#include <windows.h>

#define DT_PORTING_ENTER_CRITICAL() 
#define DT_PORTING_EXIT_CRITICAL() 

#define DT_PORTING_WAIT(x)     Sleep(x)
#define DT_PORTING_GET_TICK()  0

#elif defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
#include "stm32f10x.h"

#define DT_PORTING_ENTER_CRITICAL() __disable_irq()
#define DT_PORTING_EXIT_CRITICAL() __enable_irq()

#elif defined(STM32L432xx)
#include "stm32l4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define DT_PORTING_ENTER_CRITICAL() vPortEnterCritical()
#define DT_PORTING_EXIT_CRITICAL() vPortExitCritical()

#define DT_PORTING_GET_TIME()  osKernelSysTick()
#define DT_PORTING_WAIT(x) osDelay(x)

#else
#error "Platform is unsupported"
#endif

#endif // __DT_PORTING_H
