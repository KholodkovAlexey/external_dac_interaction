#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx.h"

#define PLLM 8
#define PLLN 336
#define PLLP 2

#define AHB_CLK (((HSE_VALUE / PLLM) * PLLN) / PLLP)
#define APB1_CLK (AHB_CLK >> 2)
#define APB2_CLK (AHB_CLK >> 1)

#define PLLI2SN 344
#define PLLI2SR 2
#define I2S_CLK (((HSE_VALUE / PLLM) * PLLI2SN) / PLLI2SR)

void Set_PLL_HSE_Clock(void);
void Set_PLL_I2S_HSE_Clock(void);

#endif /* CLOCK_H_ */
