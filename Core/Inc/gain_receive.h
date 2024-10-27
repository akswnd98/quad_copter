/*
 * gain_receive.h
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#ifndef INC_GAIN_RECEIVE_H_
#define INC_GAIN_RECEIVE_H_

#define NUM_OF_GAINS 10

#define P_PHI_IDX 0
#define D_PHI_IDX 1
#define P_THETA_IDX 2
#define D_THETA_IDX 3
#define P_PHI_DOT_IDX 4
#define D_PHI_DOT_IDX 5
#define P_THETA_DOT_IDX 6
#define D_THETA_DOT_IDX 7
#define P_PSI_DOT_IDX 8
#define D_PSI_DOT_IDX 9

extern uint16_t gain[20];

void process_gain_receive (uint8_t data);

#endif /* INC_GAIN_RECEIVE_H_ */
