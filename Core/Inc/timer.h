/*
 * timer.h
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

void start_esc_update_tim ();
void start_sensor_update_tim ();
void start_debug_values_tx_tim ();

extern uint8_t euler_state_update_plan;
extern uint8_t esc_update_plan;
extern uint8_t debug_values_tx_plan;

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim);

#endif /* INC_TIMER_H_ */
