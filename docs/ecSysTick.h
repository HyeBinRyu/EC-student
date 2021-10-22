#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#ifndef __ECSYSTICK_H
#define __ECSYSTICK_H

#define FALL		0
#define RISE		1

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void SysTick_init(void); 
void delay_ms(uint32_t num);
void SysTick_reset(void);

#ifdef __cplusplus
} 
#endif /* __cplusplus */

#endif