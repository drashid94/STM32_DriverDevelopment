#ifndef RCC_DRIVER_C
#define RCC_DRIVER_C

#include "stm32f446xx.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

uint32_t RCC_Get_APBClck1_Freq();

uint32_t RCC_Get_APBClck2_Freq();

#endif