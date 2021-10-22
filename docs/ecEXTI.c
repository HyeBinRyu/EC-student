#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"

void EXTI_init(GPIO_TypeDef *port, int pin, int trig_type, int priority){
	// SYSCFG peripheral clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN ;
	int share     = pin/4;
	int remainder = pin%4;
	int value = 0;
	// Button: PC_13 -> EXTICR3(EXTI13)
	SYSCFG->EXTICR[share] &= ~(15 <<((pin%4)*4)); //~15<<4
	if     (port == GPIOA) value = 0; //13pin is at EXTICR4
	else if(port == GPIOB) value = 1;
	else if(port == GPIOC) value = 2;
	else if(port == GPIOD) value = 3;
	else if(port == GPIOE) value = 4;
	else if(port == GPIOH) value = 7;
	SYSCFG->EXTICR[share] |= value <<((remainder)*4); 
	// Falling trigger enable (Button: pull-up)
	if(trig_type == FALL)      EXTI->FTSR |= 1<<pin;
	else if(trig_type == RISE) EXTI->RTSR |= 1<<pin;
	
	EXTI_enable(pin);
	
	// Interrupt IRQn, Priority
	uint32_t EXTI_IRQx;
	if (pin<5)	EXTI_IRQx = pin + 6;
	else if (pin < 10) EXTI_IRQx = 23;
	else if (pin < 16) EXTI_IRQx = 40;
	
	NVIC_SetPriority(EXTI_IRQx, priority);  		// Set EXTI priority as 0	
	NVIC_EnableIRQ(EXTI_IRQx); 			// Enable EXTI 


}

void EXTI_enable(uint32_t pin){
	EXTI->IMR |= 1<<pin ;	
} // mask in IMR

void EXTI_disable(uint32_t pin){
	EXTI->IMR &= ~(1<<pin) ;
} // umask in IMR

uint32_t is_pending_EXTI(uint32_t pin){
	if((EXTI->PR & (0x1UL << pin)) == (0x1UL <<pin)) return 1;
	else return 0;
}

void clear_pending_EXTI(uint32_t pin){
	EXTI->PR |= 0x1UL << pin;
}

