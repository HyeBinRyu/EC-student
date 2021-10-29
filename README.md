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
