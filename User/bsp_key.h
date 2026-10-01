#ifndef __KEY_H
#define	__KEY_H

#include "stm32f4xx.h"
#include "main.h"

#define KEY1_PIN                  GPIO_PIN_0                 
#define KEY1_GPIO_PORT            GPIOA

#define KEY2_PIN                  GPIO_PIN_2                 
#define KEY2_GPIO_PORT            GPIOG

#define KEY3_PIN                  GPIO_PIN_13                 
#define KEY3_GPIO_PORT            GPIOC

#define KEY4_PIN                  GPIO_PIN_3                
#define KEY4_GPIO_PORT            GPIOG

#define KEY5_PIN                  GPIO_PIN_4                 
#define KEY5_GPIO_PORT            GPIOG

#define KEY_ON	1
#define KEY_OFF	0

uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
void Key_process(void);
#endif /* __LED_H */

