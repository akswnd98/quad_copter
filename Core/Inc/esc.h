/*
 * esc.h
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

void start_esc ();
void terminate_esc ();
void update_throttle (int throttle);
void skip_arming_mode ();
void do_calibration ();
extern float base_throttle;
extern float min_throttle;
extern float max_throttle;

void update_output ();
extern float esc_vector[4];
void update_esc_vector ();

#endif /* INC_ESC_H_ */
