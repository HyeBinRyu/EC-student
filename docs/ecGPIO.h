// Distributed for LAB: GPIO

#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H
// mode
#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03
// otype
#define PUSHPULL 0
#define DRAIN    1
// pupdr
#define NOPULLUPDOWN  0x00
#define PULLUP        0x01  
#define PULLDOWN      0x02   
#define RESERVED      0x03
// ospeed
#define LOWSPEED     0x00
#define MEDIUMSPEED   0x01  
#define FASTSPEED     0x02   
#define HIGHSPEED     0x03

#define HIGH 1
#define LOW  0

#define BUTTON_PIN 13



#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
// This is what we have to do in this time..	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pudr(GPIO_TypeDef* Port, int pin, int pudr);
void EXTI15_10_IRQHandler(void);
void sevensegment_init(void);
void sevensegment_decode(uint8_t num);	

 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
