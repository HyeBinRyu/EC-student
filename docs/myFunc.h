#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __MYFUNC_H
#define __MYFUNC_H

#define PIN_A    0x05
#define PIN_B    0x06
#define PIN_C    0x07
#define PIN_D    0x06
#define PIN_E    0x07
#define PIN_F    0x09
#define PIN_G    0x08
#define PIN_DP   0x10

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void LED_toggle(GPIO_TypeDef* Port, int PIN);
void sevensegment_init(void);
void sevensegment_decode(uint8_t num);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
