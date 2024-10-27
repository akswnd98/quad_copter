/*
 * systick.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include <tick.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "main.h"

int tick_cnt = 0;
