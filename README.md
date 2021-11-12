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

[Manual] https://www.st.com/resource/en/reference_manual/dm00119316-stm32f411xc-e-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf

[Programming Manual: SysTick, NVIC 여깄음] https://docs.google.com/viewerng/viewer?url=https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf

[교수님 Gitbook] https://ykkim.gitbook.io/ec/course/tutorial/tutorial-creating-application-api

[내 Github] https://github.com/HyeBinRyu/EC-student

<Bitwise 연산자>

Clear &= ~(1<<) //0으로 

Set  |=    // 원하는 숫자 shifting & set

Read pin0 = Reg&**(1<<3)** //읽고 싶은 레지스터로 가서 1이랑 &연산

 

\1. GPIO

슈미트 트리거: High Low 를 결정짓는 threshold 값이 voltage가 rising일 때와 falling일 때를 다르게 하여 noise에 대해 강인성을 준다.

Pullup&pulldown: 3가지 종류의 인풋인 High, low, High-impedance에 대해 High, Low 응답을 보장한다. Pullup의 경우 HiZ 인풋은 High로 읽어들이고 Pulldown은 HiZ는 low로 읽어들인다.

Pushpull&OpenDrain: 

-Pushpull은 한 쌍의 complementary transistor로 이루어져있다. GPIO output이 1이면 source current가 외부회로로 나가고 GPIO output이 0이면 외부회로로부터 Drain current가 들어온다.

-Open Drain은 한 쌍의 same type transistor로 이루어져있다. GPIO output이 1이면 외부회로에 HiZ을 갖고 GPIO output이 0이면 외부 회로에 Drain Current를 갖는다. 

\2. EXTI

Polling VS Interrupt: polling은 하드웨어의 변화를 지속적으로 읽어 들이며 이벤트의 수행 여부를 주기적으로 검사하여 해당 신호를 받았을때 이벤트를 실행하는 방식이다. Interrupt는 하드웨어의 변화를 감지하여 외부로부터의 입력을 CPU가 알아채는 방법이다. Polling의 구현이 쉽다는 점이고 단점은 시스템의 리소스를 많이 소비한다는 점이다. 반면 intterrupt는 시스템 부하가 적다는 장점이 있는 반면 구현이 복잡하다는 단점이 있다. 

Interrupt 순서: Main함수 한줄한줄 실행하다가 interrupt 발생하면 main함수 멈추고 

EXTIㅁ_ㅁ_IRQHandler함수를 실행한다. 그 함수 안에는 반드시 clear_pending_EXTI으로 끝마쳐야한다. 그러면 다시 main함수를 이어서 수행한다.

ISR: interrupt을 위한 predefined function

Exceptoin Vector Table(ISR Vector Table): ISR 주소를 모아놓은 Table이다.

IRQ: Asynchronous Exception을 요구하는 것

NVIC: 중첩된 인터럽트의 우선순위를 정하거나 ON/OFF를 통해 제어한다.

IRQn: -16~-1은 Exception(NVIC 함수 파라미터에 16더해서 넣어준다), 0~240은 Interrupt

***EXTI Register Configuration

// Digital Input Setting

Enable GPIO peripheral clock(**RCC_AHB1ENR**) GPIO_init부르면 하게 돼있음 

Configure digital input pin(push button) using GPIO registers

// EXTI setting EXTI_init

Enable SYSCFG peripheral clock (**RCC_APB2ENR**) syscfg는 2에 있다

Connect the corresponding external line to GPIO (**SYSCFG_EXTICR**) syscfg다~~! 0~3은 share=0 EXTICR[share], 4~7은 1에 port값 나머지*4 이동시켜 넣은거임

Configure the trigger edge (**EXTI_FTSR/RTSR**)

Enable EXTI (**EXTI_IMR**) EXTI_enable()

// NVIC setting

Configure the priority of EXTI interrupt request (**NVIC_SetPriority**) 

Enable EXTI interrupt request (**NVIC_EnableIRQ**)

// EXTI use

EXTI_IRQHandler(){

​      If(is_pending) ~~ clear_pending

}

\3. SysTick

SysTick interrupt Time Period = (SysTick_Load + 1) * ClockPeriod // 만약 PLL이면 ClockPeriod는 84Mhz임, 만약 down counter이고 SysTick_Load가 6이면 84Mhz가 7개 모이면서 내려올 때마다 Tick이 일어나는 거임.

SysTick_disable(); // enable 시키기 전에 해야할 일들이 있음 그래서 diable해놔야됨

SysTick->Load에 Load값을 할당함으로 SysTick 주기를 결정해주면 되는데 내 코드에서는 1초마다 켜질 수 있게 했다 따로 변수로 안하고, 따져볼 때 단위조심하고 sec로 다 하든지 그렇게

SysTick->VAL = 0; // enable하기전에 깨끗하게

SysTick->CTRL 첫번째꺼는 Counter Enable/Disable

*** SysTick Register Configuration

// RCC system clock

// System Tick Configuration

Disable SysTick Timer: **SysTick->CTRL = 0**

Choose clock signal: System clock or ref. clock(STCLK): **SysTick->CTRL CLKSOURCE=0 or 1**

Choose to use Tick interrupt: **SysTick->CTRL TICKINT = 0 or 1**

Write reload counting value **SysTick->LOAD RELOAD = (value - 1)** 

Start SysTick Timer: **SysTick->CTRL ENABLE = 1**

 

\4. Timer

Timer Interrupt

(1) Update Interrupt: CNT has overflow or underflow

(2) Compare & Capture Interrupt: CNT matches CCR

How can we tell which event has occurred-> check the flag in Status Register

(1) UIF(update interrupt flag) Overflow or underflow event

(2) CCIF(Capture & compare flag) CNT == CCR event

èvoid TIMx_IRQHandler(void){

If(is_CCR(TIMx), if(is_UIF(TIMx))) 일 때의 실행할 거 쓰기

}

Advanced-control Timer는 (16-bit)TIM1, TIM8

Gerneral-purpose Timer는 (32-bit)TIM2, TIM5, (16-bit)TIM3, TIM4, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14

Basic Timer는 (16-bit)TIM6, TIM7 타이머를 사용합니다.  

고급타이머는 Up카운트, Down카운트, Center-aligned 카운트 모두 가능합니다.

범용타이머는 TIM2~TIM5 타이머까지 UP 카운트, Down 카운트, Center-aligned 카운트 모두 가능하나, TIM10~TIM14 타이머는 UP카운트만 가능합니다.

***Timer Register Configuration 

// System clock setting

RCC setting

// Timer setting

Enable Timer Peripheral clock (**RCC_APB1ENR**)

Set Timer Clock prescaler value (**TIMx_PSC->PSC[15:0]**)

Set auto reload value (**TIMx_ARR->ARR**)

Set counting Direction (**TIMx_CR1->DIR**)

Enable timer DMA, interrupt (**TIMx_DIER->UIE**) or (**TIMx_DIER->CCyE**)

Enable counter (**TIMx_CR1->CEN**)   

//NVIC setting

Enable TIMx interrupt NVIC_EnableIRQ(TIMx_IRQn)

Set interrupt priority NVIC_Setpriority(TIMx_IRQn,m)

\+ //Compare output setting

Set output mode (**TIMx_CCMR->OCnM**)

Set CompareCapture Polarity (**TIMx_CCRn->CCR**)

Select Output Polarity (**TIMx_CCER->CCyP**)

Enable CompareCaptureOutput (**TIMx_CCER->CCyE**)

\5. PWM

Duty Cycle = 켜져있는 “시간”/주기 <=1 : upcounting은 CCR/(ARR+1), Center alligned는 

1-CCR/ARR

Count Period : (1+ARR)/f_ckcount, 2ARR/f_ckcount

Event Period : (1+ARR)(1+PSC)/f_ckcount

f_ckcount는 PSC로 결정한다. Ex) 84MHz system clock인데 10kHz PWM이랑 50% dutycycle을 원한다면 카운터 주파수를 f_ckcount 84000000/(PSC+1)=1000000 -> **PSC=83**으로 해서 1MHz로 잡아두고(그러니까 PSC 값에 따라 방법은 사실 여러가지다)

PWM 주파수 f_PWM = f_ckcount/(1+ARR) = 1000000/(1+ARR)=1000 -> **ARR = 999**

PWM 듀티는 0.5=CCR/(ARR+1) -> **CCR=500**

## Stepper Motor

`LAB_Steppermotor.c`

```c
/**
******************************************************************************
* @author  Ryu HyeBin
* @Mod     2021-8-12 by YKKIM     
* @brief   Embedded Controller
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecEXTI.h"
#include "ecSysTick.h"
#include "ecStepper.h"

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	Stepper_step(10000, 0, FULL);  // (Step : 1024 if rpm=1, Direction : 0 or 1, Mode : FULL or HALF)
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}

// Initialiization 
void setup(void)
{	
	
	RCC_PLL_init();                                 // System Clock = 84MHz
	SysTick_init();                                 // Systick init
	
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);             // External Interrupt Setting
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);           // GPIOC pin13 initialization

	Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); // Stepper GPIO pin initialization
	Stepper_setSpeed(2);                          //  set stepper motor speed
}

void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)) {
		Stepper_stop();
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}
```

`ecStepper.c`

```c
#include "stm32f4xx.h"
#include "ecStepper.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  uint32_t next[4];
} State_full_t;

State_full_t FSM_full[4] = {  
 {0b1100,{S1,S3}},
 {0b0110,{S2,S0}},
 {0b0011,{S3,S1}},
 {0b1011,{S0,S2}}
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  uint32_t next[8];
} State_half_t;

State_half_t FSM_half[8] = { 
 {0b1000,{S1,S7}},
 {0b1100,{S2,S0}},
 {0b0100,{S3,S1}},
 {0b0110,{S4,S2}},
 {0b0010,{S5,S3}},
 {0b0011,{S6,S4}},
 {0b0001,{S7,S5}},
 {0b1001,{S0,S6}}
};


void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
//  GPIO Digital Out Initiation
	 myStepper.port1 = port1;
   myStepper.pin1  = pin1;
	 myStepper.port2 = port2;
   myStepper.pin2  = pin2;
	 myStepper.port3 = port3;
   myStepper.pin3  = pin3;
	 myStepper.port4 = port4;
   myStepper.pin4  = pin4;
	
//  GPIO Digital Out Initiation
	GPIO_init(port1, pin1, OUTPUT);
	GPIO_init(port2, pin2, OUTPUT);
	GPIO_init(port3, pin3, OUTPUT);
	GPIO_init(port4, pin4, OUTPUT);  
	GPIO_pudr(port1, pin1, NOPULLUPDOWN);
	GPIO_pudr(port2, pin2, NOPULLUPDOWN);
	GPIO_pudr(port3, pin3, NOPULLUPDOWN);
	GPIO_pudr(port4, pin4, NOPULLUPDOWN);
	GPIO_otype(port1, pin1, PUSHPULL);
	GPIO_otype(port2, pin2, PUSHPULL);
	GPIO_otype(port3, pin3, PUSHPULL);
	GPIO_otype(port4, pin4, PUSHPULL);
	GPIO_ospeed(port1, pin1, FASTSPEED);
	GPIO_ospeed(port2, pin2, FASTSPEED);
	GPIO_ospeed(port3, pin3, FASTSPEED);
	GPIO_ospeed(port4, pin4, FASTSPEED);
}

void Stepper_pinOut (uint32_t state, int mode){
	
	      if (mode ==FULL){         // FULL mode
         GPIO_write(myStepper.port1, myStepper.pin1, (FSM_full[state].out & 0x08)>>3);
         GPIO_write(myStepper.port2, myStepper.pin2,   (FSM_full[state].out & 0x04)>>2);
         GPIO_write(myStepper.port3, myStepper.pin3,   (FSM_full[state].out & 0x02)>>1);
         GPIO_write(myStepper.port4, myStepper.pin4,   (FSM_full[state].out & 0x01)>>0);
        }    
				else if (mode ==HALF){    // HALF mode
         GPIO_write(myStepper.port1, myStepper.pin1, (FSM_half[state].out & 0x08)>>3);
         GPIO_write(myStepper.port2, myStepper.pin2,   (FSM_half[state].out & 0x04)>>2);
         GPIO_write(myStepper.port3, myStepper.pin3,   (FSM_half[state].out & 0x02)>>1);
         GPIO_write(myStepper.port4, myStepper.pin4,   (FSM_half[state].out & 0x01)>>0);      
        }
}


void Stepper_setSpeed (long whatSpeed){      // rpm	
	step_delay = 	60000/step_per_rev/whatSpeed/32;   // Convert rpm to milli sec // 360doneundae 64step pilyo
}


void Stepper_step(int steps, int direction,int mode){
	 int step_number = 0;
	 myStepper._step_num = steps;
	 int state_number = 0;
	 int max_step = 3;
	 if (mode == HALF) max_step = 7; 
	 
	
	 for(myStepper._step_num=steps;myStepper._step_num>0;myStepper._step_num--){ // run for step size
				delay_ms(step_delay);                         // delay (step_delay); 
				
		    if(direction == 1) step_number++;                  // + direction step number++
				if(direction == 0) step_number--;                  // - direction step number--
				
				if(direction ==1 && step_number == max_step) step_number = 0; //  step_number must be 0 to max_step
		    else if(direction ==0 && step_number == -1) step_number = max_step; 
		 
				state_number=step_number;
	
				Stepper_pinOut(state_number, mode);
   }
}


void Stepper_stop (void){ 
     
  myStepper._step_num = 0;    
	// All pins(Port1~4, Pin1~4) set as DigitalOut '0'
	GPIO_write(myStepper.port1, myStepper.pin1,LOW);
	GPIO_write(myStepper.port2, myStepper.pin2,LOW);
	GPIO_write(myStepper.port3, myStepper.pin3,LOW);
	GPIO_write(myStepper.port4, myStepper.pin4,LOW);
}

```

`ecStepper.h`

```c
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
			
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
   GPIO_TypeDef *port1;
   int pin1;
	 GPIO_TypeDef *port2;
   int pin2;
	 GPIO_TypeDef *port3;
   int pin3;
	 GPIO_TypeDef *port4;
   int pin4;
	 int _step_num;
} Stepper_t;

	 
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
void Stepper_setSpeed (long whatSpeed);
void Stepper_step(int steps, int direction, int mode); 
void Stepper_run (int direction, int mode); 
void Stepper_stop (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```

### Timer InputCapture

`LAB_TIMER_Inputcap_UltraSonic.c`

```c
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
#include "math.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecUART_student.h"
#include "ecSysTIck.h"

uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float timeSt = 0;
float timeEnd= 0;

void setup(void);
// *** 초음파 센서로부터 거리출력 ***
int main(void){
	
	setup();
	
	while(1){
	  distance = (float) timeInterval/58*100; // Ultrasonic speed[m/s] * echo pulse duration[us]
		printf("%f[mm]\r\n",distance);
		delay_ms(500);
	}
}

// *** PWM trig 생성,  ***
void setup(){

	RCC_PLL_init(); 
	SysTick_init(); % 시스틱을 켜줘야 TIM_IRQHandler가 실행된다.
	UART2_init();
  
// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;						// PWM1 for trig
	PWM_init(&trig, GPIOA, 6);	    // PWM init as PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig,50000);    	// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig,10);   	// PWM pulse width of 10us
	
// Input Capture configuration -----------------------------------------------------------------------	
	IC_t echo;						// Input Capture for echo
	ICAP_init(&echo, GPIOB, 10);    // ICAP init as PB10 as input caputre
 	ICAP_counter_us(&echo, 10);   	// ICAP counter step time as 10us
	ICAP_setup(&echo, 3, RISE);   	// TIM2_CH3 as IC3 , rising edge detect
	ICAP_setup(&echo, 4, FALL);     // TIM2_CH3 as IC4 , falling edge detect

// Enable TIMx interrupt-----------------------------------------------------------------------	
	TIM_INT_enable(TIM2);  			// TIM2 Interrupt Enable

}
```

```c
// ***  ***
void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){                     // Update interrupt
		ovf_cnt++;  					  // overflow count
		clear_UIF(TIM2);  				  // clear update interrupt flag
	}
	if(is_CCIF(TIM2,3)){ 				  // TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		timeSt = TIM2->CCR3;			  // Capture TimeStart from CC3 : 값 읽어오기!
		clear_CCIF(TIM2,3);               // clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2,4)){ 			  // TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		timeEnd = TIM2->CCR4;			  // Capture TimeEnd from CC4 : 값 읽어오기!
    	timeInterval = ((timeEnd - timeSt)+(TIM2->ARR+1)*ovf_cnt*10); 		// Total time of echo pulse
		ovf_cnt = 0;                      // overflow reset
		clear_CCIF(TIM2,4);				  // clear capture/compare interrupt flag
	}
}
```

`ecIC.c`

```c
#include "ecTIM.h"
#include "ecGPIO.h"
#include "math.h"

/* Input Capture  */
// *** 캡처할 두 개의 변수 선언 ***
void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin){
// 0. Match Input Capture Port and Pin for TIMx
	ICx->port = port;
	ICx->pin  = pin;
	ICAP_pinmap(ICx);	  							// Port, Pin --(mapping)--> TIMx, Channel
	
	TIM_TypeDef *TIMx = ICx->timer;
	int TIn = ICx->ch; 		
	int ICn = TIn;
	ICx->ICnum=ICn;									// (default) TIx=ICx

// GPIO configuration ---------------------------------------------------------------------	
// 1. Initialize GPIO port and pin as AF
	GPIO_init(port, pin, AF);  						// GPIO init as AF=2
	GPIO_ospeed(port, pin, 3);  					// speed VHIGH=3	

// 2. Configure GPIO AFR by Pin num.
	uint32_t val;
	 if(TIMx==TIM1 ||TIMx==TIM2) val  = 0x0001;
	 else if(TIMx==TIM3 ||TIMx==TIM4 ||TIMx==TIM5) val = 0x0002;
	 else if(TIMx==TIM9 ||TIMx==TIM10 ||TIMx==TIM11) val = 0x0003;

	 if(pin<=7) port->AFR[0] = val<<(4*(pin%8));
	 else if(pin<=15) port->AFR[1] = val<<(4*(pin%8));  

// TIMER configuration ---------------------------------------------------------------------			
// 1. Initialize Timer 
	TIM_init(TIMx, 1);
// 2. Initialize Timer Interrpt 
	TIM_INT_init(TIMx, 1);        				// TIMx Interrupt initialize 
	// *** Timer 설정에서 PSC, ARR 이거 설정하는 방법 다시 알아보기 ***
// 3. Modify ARR Maxium for 1MHz
	TIMx->PSC = 84-1;						  	// Timer counter clock: 1MHz(1us)  for PLL
	TIMx->ARR = 0xFFFF;							// Set auto reload register to maximum (count up to 65535)
// 4. Disable Counter during configuration
	TIMx->CR1 &= ~TIM_CR1_CEN;  				// Disable Counter during configuration


	
// Input Capture configuration ---------------------------------------------------------------------			// *** 채널 별로 조건따라 설정해주면 될 듯 ***
// 1. Select Timer channel(TIx) for Input Capture channel(ICx)
	// Default Setting
	TIMx->CCMR1 |= 	TIM_CCMR1_CC1S_0;      	//01<<0   CC1S    TI1=IC1
	TIMx->CCMR1 |= 	TIM_CCMR1_CC2S_0;  		//01<<8   CC2s    TI2=IC2
	TIMx->CCMR2 |= 	TIM_CCMR2_CC3S_0;       //01<<0   CC3s    TI3=IC3
	TIMx->CCMR2 |= 	TIM_CCMR2_CC4S_0;  		//01<<8   CC4s    TI4=IC4


// 2. Filter Duration (use default)

// 3. IC Prescaler (use default)
// default is okay.
	
    // *** 엣지마다 반응할 수 있게 이네이블 ***
// 4. Activation Edge: CCyNP/CCyP		
	TIMx->CCER &= ~(5<<1);					// CCy(Rising) for ICn
//TIMX -> CCER |= b1010 is also the answer.
//RISE: TIMX -> CCER &= ~b1010<<4*(ICn-1) is also the answer.
//FALL: TIMX -> CCER |= b0010<<4*(ICn-1) is also the answer.
//BOTH: TIMX -> CCER |= b1010<<4*(ICn-1) is also the answer.
	// *** 아웃풋 이네이블 ***
// 5.	Enable CCy Capture, Capture/Compare interrupt
	TIMx->CCER |= 1 << 4*(ICn-1);			// CCn(ICn) Capture Enable	
	// *** 캡처 인터럽트 이네이블 ***
// 6.	Enable Interrupt of CC(CCyIE), Update (UIE)
	TIMx->DIER |= 0x1UL << ICn;				// Capture/Compare Interrupt Enable	for ICn
	TIMx->DIER |= TIM_DIER_UIE;				// Update Interrupt enable	

// 7.	Enable Counter 
	TIMx->CR1	 |= TIM_CR1_CEN;			// Counter enable	
}

// Configure Selecting TIx-ICy and Edge Type
void ICAP_setup(IC_t *ICx, int ICn, int edge_type){
	TIM_TypeDef *TIMx = ICx->timer;	        // TIMx
	int 	    CHn   = ICx->ch;	// Timer Channel CHn
	ICx->ICnum=ICn;
	// *** 디스에이블 이따가 다시 이네이블 시킬거야~ ***
// Disable  CC. Disable CCInterrupt for ICn. 
	TIMx->CCER &= ~(1 << 4*(ICn-1));		// Capture Disable
	TIMx->DIER &= ~(0x1UL << ICn);			// CCn Interrupt Disable	
	
	
// Configure  IC number(user selected) with given IC pin(TIMx_CHn)
	switch(ICn){
			case 1:
					TIMx->CCMR1 &= ~TIM_CCMR1_CC1S;						//reset   CC1S
					if (ICn==CHn) TIMx->CCMR1 |= TIM_CCMR1_CC1S_0;      //01<<0   CC1S    Tx_Ch1=IC1
					else TIMx->CCMR1 |= TIM_CCMR1_CC1S_1;      			//10<<0   CC1S    Tx_Ch2=IC1
					break;
			case 2:
					TIMx->CCMR1 &= ~TIM_CCMR1_CC2S;						//reset   CC2S
					if (ICn==CHn) TIMx->CCMR1  |= TIM_CCMR1_CC2S_0;     //01<<0   CC2S    Tx_Ch2=IC2
					else TIMx->CCMR1  |= TIM_CCMR1_CC2S_1; 	    		//10<<0   CC2S    Tx_Ch1=IC2
					break;
			case 3:
					TIMx->CCMR2 &= ~TIM_CCMR2_CC3S;						//reset   CC3S
					if (ICn==CHn) TIMx->CCMR2  |= TIM_CCMR2_CC3S_0 ;	//01<<0   CC3S    Tx_Ch3=IC3
					else TIMx->CCMR2  |= TIM_CCMR2_CC3S_1;		     	//10<<0   CC3S    Tx_Ch4=IC3
					break;
			case 4:
					TIMx->CCMR2 &= ~TIM_CCMR2_CC4S;						//reset   CC4S
					if (ICn==CHn) TIMx->CCMR2  |= TIM_CCMR2_CC4S_0;	    //01<<0   CC4S    Tx_Ch4=IC4
					else TIMx->CCMR2  |= TIM_CCMR2_CC4S_1;	     		//10<<0   CC4S    Tx_Ch3=IC4
					break;
			default: break;
		}

	//*** 클리어 해주고 설정, 근데 설정에 따라 캡처가능한 상황이 달라지는 거 같은데 공부가 필요할 듯 ***
// Configure Activation Edge direction
	TIMx->CCER  &= ~(5 << (4*ICn+1));	  								// Clear CCnNP/CCnP bits for ICn
	switch(edge_type){
		case RISE: TIMx -> CCER &= ~0b1010<<4*(ICn-1); break;
		case FALL: TIMx -> CCER |= 0b0010<<4*(ICn-1); break;
    	case BOTH: TIMx -> CCER |= 0b1010<<4*(ICn-1); break;
	}
	
// Enable CC. Enable CC Interrupt. 
	TIMx->CCER |= 1 << (4*(ICn - 1)); 									// Capture Enable
	TIMx->DIER |= 1 << ICn; 											// CCn Interrupt enabled	
}

 
// Time span for one counter step
void ICAP_counter_us(IC_t *ICx, int usec){	
	TIM_TypeDef *TIMx = ICx->timer;	
	TIMx->PSC = 84*usec-1;						  // Timer counter clock: 1us * usec
	TIMx->ARR = 0xFFFF;			 				  // Set auto reload register to maximum (count up to 65535)
}

uint32_t is_pending_TIM(TIM_TypeDef *TIMx){
	return ((TIMx->SR & TIM_SR_UIF) == 1);	

}

void clear_pending_TIM(TIM_TypeDef *TIMx){	
	TIM2->SR &= ~TIM_SR_UIF;
}

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum){
	return (TIMx->SR & (1<<ccNum))!= 0; 			//KKok 1 e anilasu false ga aninji hwakinhaneunguya		
}

void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum){
	TIMx->SR &= ~(1<<ccNum);	
}

//DO NOT MODIFY THIS
void ICAP_pinmap(IC_t *timer_pin){
   GPIO_TypeDef *port = timer_pin->port;
   int pin = timer_pin->pin;
   
   if(port == GPIOA) {
      switch(pin){
         case 0 : timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         case 1 : timer_pin->timer = TIM2; timer_pin->ch = 2; break;
         case 5 : timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         case 6 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         //case 7: timer_pin->timer = TIM1; timer_pin->ch = 1N; break;
         case 8 : timer_pin->timer = TIM1; timer_pin->ch = 1; break;
         case 9 : timer_pin->timer = TIM1; timer_pin->ch = 2; break;
         case 10: timer_pin->timer = TIM1; timer_pin->ch = 3; break;
         case 15: timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: timer_pin->timer = TIM1; timer_pin->ch = 2N; break;
         //case 1: timer_pin->timer = TIM1; timer_pin->ch = 3N; break;
         case 3 : timer_pin->timer = TIM2; timer_pin->ch = 2; break;
         case 4 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         case 5 : timer_pin->timer = TIM3; timer_pin->ch = 2; break;
         case 6 : timer_pin->timer = TIM4; timer_pin->ch = 1; break;
         case 7 : timer_pin->timer = TIM4; timer_pin->ch = 2; break;
         case 8 : timer_pin->timer = TIM4; timer_pin->ch = 3; break;
         case 9 : timer_pin->timer = TIM4; timer_pin->ch = 3; break;
         case 10: timer_pin->timer = TIM2; timer_pin->ch = 3; break;
         
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         case 7 : timer_pin->timer = TIM3; timer_pin->ch = 2; break;
         case 8 : timer_pin->timer = TIM3; timer_pin->ch = 3; break;
         case 9 : timer_pin->timer = TIM3; timer_pin->ch = 4; break;
         
         default: break;
      }
   }
}
```

ADC

- Prescaler : ADC->CCR &= ~(3<<16);	//0000: PCLK2 divided by 2	(42MHz) 얘가 분모로 들어가서 sampling time 을 구하게 됨.

### ADC IR sensor (Regular & HW trigger)

`LAB_ADC_IRSensor`

```c
/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  Tutorial ___
*					 - _________________________________
* 
******************************************************************************
*/

#include "myFunc.h"
//IR parameter//
float result_v =0;
int IR1 =0, IR2 = 0; 	// 데이터 받는 변수, 이번에는 두 개를 받아서 2개다.
int flag = 0;			// 어떤 데이터인지 분간하게 하는 플레그
void setup(void);		
void ADC_IRQHandler(void);	

seq[2] = {8,9}; // chogihwa below code will apply this variable automatically // GPIO핀이 반환하는 채널
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();

	ADC_sequence(2, seq); // ADC_Sequence(length, *seq)
	ADC_start();
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		printf("IR1 = %d\r\n",IR1);
		if(IR1 >= 3000) printf("GO LEFT\r\n");
		else if(IR2 >= 3000) printf("GO RIGHT\r\n");
		printf("IR2 = %d\r\n",IR2);
		printf("\r\n");
		delay_ms(1000);	// 1초마다 출력
	}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                         // System Clock = 84MHz
	SysTick_init();							// 이걸 써야 External trigger가 작동함..!, RCC다음에 ADC전에 선언!!
	UART2_init();
	ADC_init(GPIOB,0,TRGO);
	ADC_init(GPIOB,1,TRGO); // ->ADC_TRGO is linked to this function, 타이퍼랑 polarity는 써줘야됨!
	
}

void ADC_IRQHandler(void){
	if(is_ADC_OVR()) clear_ADC_OVR();
	if(is_ADC_EOC(ADC1)){       //after finishing sequence
		flag++;
		if(flag%2 == 1) IR1 = ADC_read();
		else if(flag%2 == 0) IR2 = ADC_read();
	// flag => whether it is IR1 or IR2
  }
}

```

`ecADC.c`

```c
#include "stm32f411xe.h"
#include "ecSysTick.h"
#include "ecADC.h"
#include "ecGPIO.h"
#include "ecTIM.h"
#include <stdint.h>
uint32_t result;

//*********************************************Setup에 관한 함수!*********************************************
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode){  //mode 0 : SW, 1 : TRGO
// 0. Match Port and Pin for ADC channel	
	int CHn = ADC_pinmap(port, pin);			// ADC Channel <->Port/Pin mapping

// GPIO configuration ---------------------------------------------------------------------	
// 1. Initialize GPIO port and pin as ANALOG, no pull up / pull down
	GPIO_init(port, pin, ANALOG);  				// ANALOG = 3
	GPIO_pudr(port, pin, NOPULLUPDOWN);  		// EC_NONE = 0

// ADC configuration	---------------------------------------------------------------------			
// 1. Total time of conversion setting
	// Enable ADC pheripheral clock
	RCC->APB2ENR  |= (1<<8); 		// Enable the clock of RCC_APB2ENR_ADC1EN
	
	// Configure ADC clock pre-scaler
	ADC->CCR &= ~(3<<16);					// 0000: PCLK2 divided by 2	(42MHz)
	
	// Configure ADC resolution 
	ADC1->CR1 &= ~(3<<24);     		// 00: 12-bit resolution (15cycle+)
	
	// Configure channel sampling time of conversion.	
	// Software is allowed to write these bits only when ADSTART=0 and JADSTART=0	!!
	// ADC clock cycles @42MHz = 2us
	
	if(CHn < 10){
		//ADC1->SMPR2  &= ~(7U << 3*(CHn));
		ADC1->SMPR2  |= 4U << 3*(CHn);					
	} // sampling time conversion : 84  			
	else{
		//ADC1->SMPR1	 &= ~(15U << 3*(CHn - 10));
		ADC1->SMPR1  |= 4U << 3*(CHn - 10);
	}				 
	
// 2. Regular / Injection Group 
	//Regular: SQRx, Injection: JSQx

// 3. Repetition: Single or Continuous conversion
	ADC1->CR2 |= ADC_CR2_CONT;//1<<1;      			// Enable Continuous conversion mode	
	
// 4. Single Channel or Scan mode
	//  - Single Channel: scan mode, right alignment
	ADC1->CR1 |= 1<<8;						// 1: Scan mode enable  여기는 표에 없음..!
	ADC1->CR2 &= ~(1<<11);   				// 0: Right alignment	여기는 표에 없음..!
	// Configure the sequence length
	ADC1->SQR1 &= ~(15<<20); 				// 0000: 1 conversion in the regular channel conversion sequence
	
	// Configure the channel sequence 
	ADC1->SQR3 &= ~(0x1F << 0);				 	// SQ1 clear bits
	ADC1->SQR3 |= (CHn & (0x1F << 0)); 	// Choose the channel to convert firstly
	
// 5. Interrupt Enable
	// Enable EOC(conversion) interrupt. 
	ADC1->CR1 &= ~(1<<5);          	// Interrupt reset
	ADC1->CR1 |= 1<<5;           // Interrupt enable
	
	// Enable ADC_IRQn 
	NVIC_SetPriority(ADC_IRQn,2); 			// Set Priority to 2
	NVIC_EnableIRQ(ADC_IRQn);      	// Enable interrupt form ACD1 peripheral	



/* -------------------------------------------------------------------------------------*/
//					HW TRIGGER MODE
/* -------------------------------------------------------------------------------------*/
	
	// TRGO Initialize : TIM3, 1msec, RISE edge
	if(trigmode==TRGO) ADC_TRGO(TIM3, 1, RISE);				
	
}

void ADC_TRGO(TIM_TypeDef* TIMx, int msec, int edge){
	// set timer
	int timer = 0;
	if(TIMx==TIM2) timer=2;
	else if(TIMx==TIM3) timer=3;	
	
	// Single conversion mode (disable continuous conversion)
	ADC1->CR2 &= ~(1<<1);     			// Discontinuous conversion mode
	ADC1->CR2 |= 1<<10;  					// Enable EOCS
	

	// HW Trigger configuration -------------------------------------------------------------
	
// 1. TIMx Trigger Output Config
	// Enable TIMx Clock
	TIM_init(TIMx, msec);
	TIMx->CR1 &= ~(1<<0); 					//counter disable
	
	// Set PSC, ARR
  TIM_period_ms(TIMx, msec);
	
  // Master Mode Selection MMS[2:0]: Trigger output (TRGO)
  TIMx->CR2 &= ~(7<<4);				// reset MMS
  TIMx->CR2 |= 4<<4;   				//100: Compare - OC1REF signal is used as trigger output (TRGO)
   
	// Output Compare Mode //PWM saengsung
  TIMx->CCMR1 &= ~(7<<4);       			// OC1M : output compare 1 Mode , clear
  TIMx->CCMR1 |= 6<<4;          			// OC1M = 110 for compare 1 Mode ch1 , set
	
  // OC1 signal 
  TIMx->CCER |= 1<<0;          // CC1E Capture enabled
  TIMx->CCR1  = (TIMx->ARR)/2; // duty ratio 50%
   
  // Enable TIMx 
  TIMx->CR1 |= 1<<0; 					//counter enable

// 2. ADC HW Trigger Config.
	// Select Trigger Source  	 		
	ADC1->CR2 &= ~ADC_CR2_EXTSEL; 			// reset EXTSEL
	ADC1->CR2 |= (timer*2+2)<<24; 			// TIMx TRGO event (ADC : TIM2, TIM3 TRGO)
	
	//Select Trigger Polarity
	ADC1->CR2 &= ~ADC_CR2_EXTEN;				// reset EXTEN, default
	if(edge==RISE) ADC1->CR2 |= ADC_CR2_EXTEN_0;				// trigger detection rising edge
	else if(edge==FALL) ADC1->CR2 |= ADC_CR2_EXTEN_1;		// trigger detection falling edge
	else if(edge==BOTH) ADC1->CR2 |= ADC_CR2_EXTEN_Msk;	// trigger detection both edge

}
// trigger chogihwa and setting end!
//**********************************************************************************************************
void ADC_continue(int contmode){
	if(contmode==CONT){
		// Repetition: Continuous conversion
		ADC1->CR2 |= 1<<1;      	// Enable Continuous conversion mode	
		ADC1->CR1 &= ~ADC_CR1_SCAN;				// 0: Scan mode disable 
	}
	else 										//if(contmode==SINGLE)
		{
		// Repetition: Single conversion
		ADC1->CR2 &= ~ADC_CR2_CONT;      		// Disable Continuous conversion mode	
		ADC1->CR1 |= ADC_CR1_SCAN;				// 1: Scan mode enable
	}
} 

void ADC_sequence(int length, int *seq){
	
	ADC1->SQR1 &= ~(0xF<<20); 						// reset length of conversions in the regular channel 	
	ADC1->SQR1 |= (length-1)<<20; 				// conversions in the regular channel conversion sequence
	
	for(int i = 0; i<length; i++){ // 채널들의 읽을 순서를 지정해주는 과정
		if (i<6){
			ADC1->SQR3 &= ~(0x1F<<i*5);				// SQn clear bits
			ADC1->SQR3 |= seq[i]<<i*5;				// Choose the channel to convert sequence
		}
		else if (i <12){
			ADC1->SQR2 &= ~(0x1F<<(i-7)*5);				// SQn clear bits
			ADC1->SQR2 |= seq[i]<<(i-7)*5;				// Choose the channel to convert sequence
		}
		else{
			ADC1->SQR1 &= ~(0x1F<<(i-12)*5);	// SQn clear bits
			ADC1->SQR1 |= seq[i]<<(i-12)*5;		// Choose the channel to convert sequence
		}
	}
}
//***************************************이제 시작하고 나서 불러오는 함수들!************************************
void ADC_start(void){  // ADC 1hangaelasu
	// Enable ADON, SW Trigger-------------------------------------------------------------------------------
	ADC1->CR2 |=  ADC_CR2_ADON; //1<<0; // ADON
	ADC1->CR2 |=  ADC_CR2_SWSTART; //1<<30; // SWSTART = 1
}


uint32_t ADC_read(){
	return ADC1->DR; // DATA Read
}

uint32_t is_ADC_EOC(ADC_TypeDef *ADCx){ //ADC->DR가서 읽을 때가 됐다는 뜻, DR가서 읽으면 알아서 clear됨
	if((ADCx->SR & ADC_SR_EOC) == ADC_SR_EOC){
		return 1;
	}else return 0;
}

uint32_t is_ADC_OVR(void){
	return (ADC1->SR & (ADC_SR_OVR)==ADC_SR_OVR);
}

void clear_ADC_OVR(void){
	ADC1->SR &= ~(1<<5);
}

uint32_t ADC_pinmap(GPIO_TypeDef *Port, int Pin){ // ADC_init()에서 호출됨! pin에 해당하는 채널 반환
	if(Port == GPIOA){
		if(Pin == 0) 			return 0;
		else if(Pin == 1) return 1;
		else if(Pin == 4) return 4;
		else if(Pin == 5) return 5;
		else if(Pin == 6) return 6;
		else if(Pin == 7) return 7;
		else 							while(1);
	}
	else if(Port == GPIOB){
		if(Pin == 0) 			return 8;
		else if(Pin == 1)	return 9;
		else 							while(1);
	}
	else if(Port == GPIOC){
		if(Pin == 0)			return 10;
		else if(Pin == 1)	return 11;
		else if(Pin == 2)	return 12;
		else if(Pin == 3)	return 13;
		else if(Pin == 4)	return 14;
		else if(Pin == 5)	return 15;
		else							while(1);
	}
}

```

