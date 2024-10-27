/*
 * timer.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include "main.h"
#include "timer.h"

void start_esc_update_tim () {
  HAL_TIM_Base_Start_IT(&htim4);
}

void start_sensor_update_tim () {
  HAL_TIM_Base_Start_IT(&htim5);
}

void start_debug_values_tx_tim () {
  HAL_TIM_Base_Start_IT(&htim13);
}

uint8_t euler_state_update_plan = 0;
uint8_t esc_update_plan = 0;
uint8_t debug_values_tx_plan = 0;
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
  if (htim->Instance == htim4.Instance) {
    esc_update_plan = 1;
  } else if (htim->Instance == htim5.Instance) {
    euler_state_update_plan = 1;
  } else if (htim->Instance == htim13.Instance) {
    debug_values_tx_plan = 1;
  }
}
