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
			if(Output == LOW) Port->ODR |= (1UL << pin);
			else Port->ODR &= ~(1UL << pin);
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
		if(type == 1) Port->OTYPER |= (1UL<<pin);
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

#### 





