# Embedded Controller - Student



---

### Embedded Controller by Young-Keun Kim, Handong Global University 

Author:  Hyebin Ryu

Date:  Updated 2021-10-08



---

# 

This is a private github repository for Embedded Controller Class that contains my firmware library files.



## Documentation

[See here for API documentation ](./docs/EC_HAL_Documentation.md)

[[Click here]([HyeBinRyu/EC-student (github.com)](https://github.com/HyeBinRyu/EC-student))]

## Contents

This repository contains

* \include : header and definition files of EC lib

* \docs: documentation 

* \src: assignment source files for main()

* \tutorial: exercise and tutorial files used in class

* _add more descriptions here_ 

  

_// Add your notes here_



## Environment

Program Language: C/C++

IDE/Compiler : Keil uVision 5

OS: WIn10

MCU:  STM32F411RE, Nucleo-64



## Installation

_// Add your notes here_



## Resource

### GPIO Digital In/Out

#### Header File

`#include "ecGPIO.h"`

```c
#include "stm32f411xe.h"

#ifndef __EC_GPIO_H
#define __EC_GPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13


void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pudr);

#endif
```

 

#### GPIO_init() 

Initializes GPIO pins with default setting and Enables GPIO Clock. Mode: In/Out/AF/Analog

```C
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:** Port Number, GPIOA~GPIOH

* **pin**: pin number (int) 0~15

* **mode**: INPUT (0), OUTPUT (1), AF(02), ANALOG (03)

  `ecGPIO.c`

  ```C
  void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
     
  	if (Port == GPIOA)
  		RCC_GPIOA_enable();
  	if (Port == GPIOB)
  		RCC_GPIOB_enable();
  	if (Port == GPIOC)
  		RCC_GPIOC_enable();
  	if (Port == GPIOD)
  		RCC_GPIOD_enable();
  	if (Port == GPIOE)
  		RCC_GPIOE_enable();
  	if (Port == GPIOH)
  		RCC_GPIOH_enable();
  
  	GPIO_mode(Port, pin, mode);
  	// The rest are default values
  }
  ```

  

#### GPIO_write()

Configures GPIO pin modes: In/Out/AF/Analog

```C
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **mode**: INPUT (0), OUTPUT (1), AF(02), ANALOG (03)

`ecGPIO.c`

```c
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
	Port->ODR &= ~(1UL << pin);		
    Port->ODR |= (Output << pin);
}
```



#### GPIO_read()

Configures GPIO pin modes: In/Out/AF/Analog

```C
int  GPIO_read(GPIO_TypeDef *Port, int pin);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15

`ecGPIO.c`

```c
int  GPIO_read(GPIO_TypeDef *Port, int pin){
		int btVal = (Port->IDR) & (1UL << pin);
	
		return btVal;
}
```

#### 

#### GPIO_mode()

Configures GPIO pin modes: In/Out/AF/Analog

```C
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **mode**: INPUT (0), OUTPUT (1), AF(02), ANALOG (03)

`ecGPIO.c`

```c
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |=  (mode<<(2*pin));    
}
```

#### 

#### GPIO_ospeed()

Configures GPIO pin modes: In/Out/AF/Analog

```C
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **speed**: LOWSPEED (0), MEDIUMSPEED(1), FASTSPEED(02), HIGHSPEED(03)

`ecGPIO.c`

```c
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed){
		Port->OSPEEDR &= ~(3UL<<(2*pin));
		Port->OSPEEDR |=  (speed<<(2*pin));
}
```

#### 

#### GPIO_otype()

Configures GPIO pin modes: In/Out/AF/Analog

```C
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **type**: PUSHPULL(0), DRAIN(1)

`ecGPIO.c`

```c
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type){
		Port->OTYPER &= ~(1UL<<pin);
		Port->OTYPER |= (type<<pin);
}
```

#### 

#### GPIO_pupdr()   

Configures GPIO pin modes: In/Out/AF/Analog

```
void GPIO_pudr(GPIO_TypeDef* Port, int pin, int pudr);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15 
- **pupr**: NOPULLUPDOWN(0), PULLUP(1), PULLDOWN(2), RESERVED(3)

`ecGPIO.c`

```c
void GPIO_pudr(GPIO_TypeDef* Port, int pin, int pudr){
		Port->PUPDR &= ~(3UL<<(2*pin));
		Port->PUPDR |=  (pudr<<(2*pin));
}
```

#### <LAB_GPIO_7segment>

`LAB_GPIO_7segment.cpp`

```cpp
int main(void) { 
   // Initialiization -------------------------------------
   setup();
   bool tempVar = true;
   // Inifinite Loop --------------------------------------
   while(1){   
      uint8_t cnt = 9;      
    // Inifinite Loop ------------------------------------- 
      while(1){      
            if(GPIO_read(GPIOC, BUTTON_PIN) && tempVar){
                cnt++;
                if (cnt > 8) cnt = 0; 
                tempVar=false;
            }
            else if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
               for(int i = 0; i < 500000;i++){} 
               tempVar=true;
            }                   
            sevensegment_decode(cnt % 10); 
       }
   }
}

void setup(void)
{
    RCC_HSI_init();      
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);  
	sevensegment_init();
}
```

##### void sevensegment_init(void)

```cpp
void sevensegment_init(void){
    GPIO_init(GPIOA, PIN_A, OUTPUT);   
    GPIO_otype(GPIOA, PIN_A, PUSHPULL); 
    GPIO_pudr(GPIOA, PIN_A, NOPULLUPDOWN); 
    GPIO_ospeed(GPIOA, PIN_A, FASTSPEED);
    
    GPIO_init(GPIOA, PIN_B, OUTPUT);   
    GPIO_otype(GPIOA, PIN_B, PUSHPULL);
    GPIO_pudr(GPIOA, PIN_B, NOPULLUPDOWN); 
    GPIO_ospeed(GPIOA, PIN_B, FASTSPEED);
   
    GPIO_init(GPIOA, PIN_C, OUTPUT);   
    GPIO_otype(GPIOA, PIN_C, PUSHPULL);
    GPIO_pudr(GPIOA, PIN_C, NOPULLUPDOWN); 
    GPIO_ospeed(GPIOA, PIN_C, FASTSPEED);
   
    GPIO_init(GPIOB, PIN_D, OUTPUT);   
    GPIO_otype(GPIOB, PIN_D, PUSHPULL); 
    GPIO_pudr(GPIOB, PIN_D, NOPULLUPDOWN); 
    GPIO_ospeed(GPIOB, PIN_D, FASTSPEED);
    
    GPIO_init(GPIOC, PIN_E, OUTPUT);    
    GPIO_otype(GPIOC, PIN_E, PUSHPULL); 
    GPIO_pudr(GPIOC, PIN_E, NOPULLUPDOWN); 
    GPIO_ospeed(GPIOC, PIN_E, FASTSPEED);
    
    GPIO_init(GPIOA, PIN_F, OUTPUT);   
    GPIO_otype(GPIOA, PIN_F, PUSHPULL); 
    GPIO_pudr(GPIOA, PIN_F, NOPULLUPDOWN); 
    GPIO_ospeed(GPIOA, PIN_F, FASTSPEED);
    
    GPIO_init(GPIOA, PIN_G, OUTPUT);    
    GPIO_otype(GPIOA, PIN_G, PUSHPULL); 
    GPIO_pudr(GPIOA, PIN_G, NOPULLUPDOWN); 
    GPIO_ospeed(GPIOA, PIN_G, FASTSPEED);
    
    GPIO_init(GPIOB, PIN_DP, OUTPUT);   
    GPIO_otype(GPIOB, PIN_DP, PUSHPULL);
    GPIO_pudr(GPIOB, PIN_DP, NOPULLUPDOWN); 
    GPIO_ospeed(GPIOB, PIN_DP, FASTSPEED);
}
```

##### void sevensegment_decode(uint8_t num)

```

```



```cpp
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

void sevensegment_decode(uint8_t num){
    for(int i = 0; i < 9; i++){
	  GPIO_write(SegGPIO[i], SegPin[i], SegTable[num][i]);
	}
}  
```

## EXTI & SysTick

### LED Toggle with EXTI Button 	

```c
void setup(void){
	RCC_HSI_init();
	GPIO_init(GPIOA, LED_PIN, OUTPUT);
	//GPIO_otype(GPIOA, LED_PIN, PUSHPULL);
	//GPIO_pudr(GPIOA, LED_PIN, NOPULLUPDOWN);
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pudr(GPIOC, BUTTON_PIN, PULLDOWN);

	EXTI_init(GPIOC, BUTTON_PIN, FALL ,0);
}
```


```c
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
```

`ecEXTI.c`

```c
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


```

`ecEXTI.h`

```c
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
```



### 7 Segment Display with EXTI Button

```c
void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	sevensegment_init();
}

int main(void){
	// Initialization 
	setup();
	// Infinite Loop  
	while(1){
		sevensegment_decode(count);
		delay_ms(1000UL);
		count++;
		if(count >10UL) count = 0UL;
		//SysTick_reset();
	}
}
```

```c
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


```

`ecSysTick.c`

```c
#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecSysTick.h"
void SysTick_init(){
	// SysTick Initialiization ------------------------------------------------------				
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;				// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= 1<<2;  

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = (0.001 * MCU_CLK_PLL) - 1; 				// ticks period i want 1ms

	// Clear SysTick Current Value 
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= 1<<1;
		
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= 1;
		
	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}

void delay_ms(uint32_t num){
	uint32_t msTicks = 0;
	while(1){
		uint32_t curTicks = msTicks;
		while((msTicks - curTicks) < num);
		msTicks = 0;
	}
	
void SysTick_reset(void){
	if(GPIO_read(GPIOA, BUTTON_PIN) == 0) count = 0UL;
}
```

`ecSysTick.h`

```c
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
```

###### myFunc.c

```c
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

```

myFunc.h

```c
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

```

