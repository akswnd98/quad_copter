/*
 * esc.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include "esc.h"
#include "main.h"
#include <stdio.h>
#include "dynamics.h"
#include "control_receive.h"

void start_esc () {
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // pwm 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // pwm 2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // pwm 3
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // pwm 4
}

void terminate_esc () {
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
}

void update_throttle (int throttle) {
  htim3.Instance->CCR3 = throttle;
  htim3.Instance->CCR4 = throttle;
  htim2.Instance->CCR4 = throttle;
  htim2.Instance->CCR3 = throttle;
}

void skip_arming_mode () {
  update_throttle(300);
  HAL_Delay(100);
  update_throttle(0);
  HAL_Delay(3000);
  update_throttle(5250);
  HAL_Delay(3000);
}

void do_calibration () {
  printf("calibration start\n");
  update_throttle(10400);
  HAL_Delay(10000);
  update_throttle(5250);
  HAL_Delay(10000);
  printf("calibration end\n");
}

float base_throttle = 5250;
float min_throttle = 5300;
float max_throttle = 10400;

void update_output () {
  update_esc_vector();
  htim3.Instance->CCR3 = (int)MAX(MIN(base_throttle + esc_vector[0], max_throttle), min_throttle);
  htim3.Instance->CCR4 = (int)MAX(MIN(base_throttle + esc_vector[1], max_throttle), min_throttle);
  htim2.Instance->CCR4 = (int)MAX(MIN(base_throttle + esc_vector[2], max_throttle), min_throttle);
  htim2.Instance->CCR3 = (int)MAX(MIN(base_throttle + esc_vector[3], max_throttle), min_throttle);
}

float esc_vector[4];
void update_esc_vector () {
  float eta_dot_dot_ref[3];
  float phi_dot_ref = get_dot_ref_by_pd(
    get_error(command[PHI_COMMAND_IDX] * 0.0005, phi),
    P_phi,
    phi_dot,
    D_phi
  );
  float phi_dot_dot_ref = get_dot_ref_by_pd(
    get_error(phi_dot_ref, phi_dot),
    P_phi_dot,
    phi_dot_dot,
    D_phi_dot
  );
  float theta_dot_ref = get_dot_ref_by_pd(
    get_error(command[THETA_COMMAND_IDX] * 0.0005, theta),
    P_theta,
    theta_dot,
    D_theta
  );
  float theta_dot_dot_ref = get_dot_ref_by_pd(
    get_error(theta_dot_ref, theta_dot),
    P_theta_dot,
    theta_dot_dot,
    D_theta_dot
  );
  float psi_dot_dot_ref = get_dot_ref_by_pd(
    get_error((command[PSI_DOT_COMMAND_IDX]) * 0.005, psi_dot),
    P_psi_dot,
    psi_dot_dot,
    D_psi_dot
  );
  get_eta_dot_dot_ref(phi_dot_dot_ref, theta_dot_dot_ref, psi_dot_dot_ref, eta_dot_dot_ref);
  float force_vector[4];
  float C[3][3];
  get_C(phi, theta, C);
  get_force_vector(eta_dot_dot_ref, T_inv, I, C, force_vector);
  get_esc_vector(force_vector, esc_vector);
}
