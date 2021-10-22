#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#ifndef __ECEXTI_H
#define __ECEXTI_H

#define FALL		0
#define RISE		1

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void EXTI_init(GPIO_TypeDef *port, int pin, int trig_type, int priority);
void EXTI_enable(uint32_t pin); // mask in IMR
void EXTI_disable(uint32_t pin); // umask in IMR
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);

 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
