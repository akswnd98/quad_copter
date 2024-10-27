/*
 * control_receive.h
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#ifndef INC_CONTROL_RECEIVE_H_
#define INC_CONTROL_RECEIVE_H_

#define MAX_CHANNEL_NUM 14
#define NUM_OF_CONTROLS 6
#define PHI_COMMAND_IDX 1
#define THETA_COMMAND_IDX 0
#define PSI_DOT_COMMAND_IDX 3
#define BASE_THRUST_COMMAND_IDX 2

extern int16_t control[20];
extern float command[4];
extern uint8_t shutdown;

void process_control_receive (uint8_t data);

#endif /* INC_CONTROL_RECEIVE_H_ */
