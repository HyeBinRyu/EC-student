// 21800238 Hyebin Ryu
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "myFunc.h"

uint32_t count= 0UL;
// Initialization
void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	sevensegment_init();
}

int main(void){
	// Initialization -------------------------------------------
	setup();
	// Infinite Loop  -------------------------------------------
	while(1){
		sevensegment_decode(count);
		delay_ms(1000UL);
		count++;
		if(count >10UL) count = 0UL;
		SysTick_reset();
	}
}