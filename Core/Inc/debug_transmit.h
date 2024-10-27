/*
 * debug.h
 *
 *  Created on: May 2, 2023
 *      Author: akswnd98
 */

#ifndef INC_DEBUG_TRANSMIT_H_
#define INC_DEBUG_TRANSMIT_H_

#include "main.h"

#define NUM_OF_DEBUG_VALUES 15

#define PHI_IDX 0
#define THETA_IDX 1
#define PHI_DOT_IDX 2
#define THETA_DOT_IDX 3
#define PSI_DOT_IDX 4
#define ESC_1_IDX 5
#define ESC_2_IDX 6
#define ESC_3_IDX 7
#define ESC_4_IDX 8
#define GYRO_X_IDX 9
#define GYRO_Y_IDX 10
#define GYRO_Z_IDX 11
#define ACC_X_IDX 12
#define ACC_Y_IDX 13
#define ACC_Z_IDX 14

void process_debug_values_tx ();
uint16_t convert_float_to_uint16 (float value, float scale);
void fill_debug_values (uint8_t *buf, int idx, uint16_t value);

#endif /* INC_DEBUG_TRANSMIT_H_ */
