#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "myFunc.h"

int SegPin[8] = {PIN_A, PIN_B, PIN_C, PIN_D, PIN_E, PIN_F, PIN_G, PIN_DP}; 
GPIO_TypeDef* SegGPIO[8] = {GPIOA, GPIOA, GPIOA, GPIOB, GPIOC, GPIOA, GPIOA, GPIOB};

int SegTable[10][8] = { 
	{0, 0, 0, 0, 0, 0, 1,0},
	{1, 0, 0, 1, 1, 1, 1,0},
	{0, 0, 1, 0, 0, 1, 0,0},
	{0, 0, 0, 0, 1, 1, 0,0},
	{1, 0, 0, 1, 1, 0, 0,0},
	{0, 1, 0, 0, 1, 0, 0,0},
	{0, 1, 0, 0, 0, 0, 0,0},
	{0, 0, 0, 1, 1, 1, 1,0},
	{0, 0, 0, 0, 0, 0, 0,0},
	{0, 0, 0, 0, 1, 0, 0,0}
};

void LED_toggle(GPIO_TypeDef* Port, int PIN){
	Port->ODR ^= 1 << PIN;
}

void sevensegment_init(void){
    GPIO_init(GPIOA, PIN_A, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_otype(GPIOA, PIN_A, PUSHPULL); // #define PIN_C    0x05
    GPIO_pudr(GPIOA, PIN_A, NOPULLUPDOWN); 
    //GPIO_ospeed(GPIOA, PIN_A, FASTSPEED);
    
    GPIO_init(GPIOA, PIN_B, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_otype(GPIOA, PIN_B, PUSHPULL); // #define PIN_D    0x06
    GPIO_pudr(GPIOA, PIN_B, NOPULLUPDOWN); 
    //GPIO_ospeed(GPIOA, PIN_B, FASTSPEED);
   
    GPIO_init(GPIOA, PIN_C, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_otype(GPIOA, PIN_C, PUSHPULL); // #define PIN_E    0x07
    GPIO_pudr(GPIOA, PIN_C, NOPULLUPDOWN); 
    //GPIO_ospeed(GPIOA, PIN_C, FASTSPEED);
   
    GPIO_init(GPIOB, PIN_D, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_otype(GPIOB, PIN_D, PUSHPULL); // #define PIN_A    0x09
    GPIO_pudr(GPIOB, PIN_D, NOPULLUPDOWN); 
    //GPIO_ospeed(GPIOB, PIN_D, FASTSPEED);
    
    GPIO_init(GPIOC, PIN_E, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_otype(GPIOC, PIN_E, PUSHPULL); // #define PIN_B    0x08
    GPIO_pudr(GPIOC, PIN_E, NOPULLUPDOWN); 
    //GPIO_ospeed(GPIOC, PIN_E, FASTSPEED);
    
    GPIO_init(GPIOA, PIN_F, OUTPUT);    // calls RCC_GPIOB_enable()
    GPIO_otype(GPIOA, PIN_F, PUSHPULL); // #define PIN_G    0x06
    GPIO_pudr(GPIOA, PIN_F, NOPULLUPDOWN); 
    //GPIO_ospeed(GPIOA, PIN_F, FASTSPEED);
    
    GPIO_init(GPIOA, PIN_G, OUTPUT);    // calls RCC_GPIOB_enable()
    GPIO_otype(GPIOA, PIN_G, PUSHPULL); // #define PIN_DP      0x10
    GPIO_pudr(GPIOA, PIN_G, NOPULLUPDOWN); 
    //GPIO_ospeed(GPIOA, PIN_G, FASTSPEED);
    
    GPIO_init(GPIOB, PIN_DP, OUTPUT);    // calls RCC_GPIOC_enable()
    GPIO_otype(GPIOB, PIN_DP, PUSHPULL); // #define PIN_F    0x07
    GPIO_pudr(GPIOB, PIN_DP, NOPULLUPDOWN); 
    //GPIO_ospeed(GPIOB, PIN_DP, FASTSPEED);
}

void sevensegment_decode(uint8_t num){
    for(int i = 0; i < 9; i++){
					 GPIO_write(SegGPIO[i], SegPin[i], SegTable[num][i]);
	  }
}  
