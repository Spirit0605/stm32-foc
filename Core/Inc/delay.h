#ifndef __DELAY_H
#define __DELAY_H

#include "tim.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

void delay_us(uint16_t us);
void delay_ms(uint16_t ms);

#endif