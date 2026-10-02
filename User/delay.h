#ifndef __CORE_DELAY_H
#define __CORE_DELAY_H

#include "stm32f4xx.h"

/* 获取内核时钟频率 */
#define GET_CPU_ClkFreq()       HAL_RCC_GetSysClockFreq()
#define CPU_MHZ									168
#define SysClockFreq            (CPU_MHZ*1000000)

#define CPU_TS_INIT_IN_DELAY_FUNCTION   0  

uint32_t CPU_TS_TmrRd(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
uint32_t HAL_GetTick(void);

void CPU_TS_Tmr_Delay_US(uint32_t us);
#define HAL_Delay(ms)     					CPU_TS_Tmr_Delay_US(ms*1000)
#define CPU_TS_Tmr_Delay_S(s)       CPU_TS_Tmr_Delay_MS(s*1000)
		
#define delay_ms(ms)	CPU_TS_Tmr_Delay_US(1000*ms)
#define delay_us(us)	CPU_TS_Tmr_Delay_US(us)

#endif
