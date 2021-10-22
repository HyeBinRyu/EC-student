/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM
* @brief   Embedded Controller:  Tutorial ___
*					 - _________________________________
*
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "myFunc.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

//int flag = 0;

// Initialization
void setup(void){
	RCC_HSI_init();
	
	GPIO_init(GPIOA, LED_PIN, OUTPUT);
	//GPIO_otype(GPIOA, LED_PIN, PUSHPULL);
	//GPIO_pudr(GPIOA, LED_PIN, NOPULLUPDOWN);
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pudr(GPIOC, BUTTON_PIN, PULLDOWN);
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL ,0);
}

void EXTI15_10_IRQHandler(void){
	if(is_pending_EXTI(BUTTON_PIN)){
		LED_toggle(GPIOA, LED_PIN);
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}

int main(void){
	// Initialization
	setup();
	// Infinite loop
	while(1){}
}
