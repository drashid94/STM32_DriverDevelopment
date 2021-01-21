#ifndef RCC_DRIVER_C
#define RCC_DRIVER_C

#include "stm32f446xx.h"

uint32_t RCC_Get_APBClck1_Freq();

uint32_t RCC_Get_APBClck2_Freq();

uint32_t RCC_GetPLLOutputClock(void);

#endif