/*
 * Delay.h
 *
 *  Created on: 7 feb. 2017
 *      Author: scma
 */
#include <stm32f4xx_hal.h>

#ifndef DELAY_H_
#define DELAY_H_
extern TIM_HandleTypeDef htim11;
extern volatile uint32_t sekTick;
void delay_init(void);
void delay_us(uint32_t us);

#endif /* DELAY_H_ */
