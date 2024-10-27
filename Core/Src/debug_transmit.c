/*
 * debug.c
 *
 *  Created on: May 2, 2023
 *      Author: akswnd98
 */

#include "debug_transmit.h"
#include "main.h"
#include "dynamics.h"
#include "uart_utils.h"
#include "timer.h"
#include "esc.h"
#include "sensor.h"

void process_debug_values_tx () {
  uint8_t buf[100];
  buf[0] = 0x80;
  buf[1] = 0x81;

  fill_debug_values(buf, PHI_IDX, convert_float_to_uint16(phi, 2000));
  fill_debug_values(buf, THETA_IDX, convert_float_to_uint16(theta, 2000));
  fill_debug_values(buf, PHI_DOT_IDX, convert_float_to_uint16(phi_dot, 2000));
  fill_debug_values(buf, THETA_DOT_IDX, convert_float_to_uint16(theta_dot, 2000));
  fill_debug_values(buf, PSI_DOT_IDX, convert_float_to_uint16(psi_dot, 2000));
  fill_debug_values(buf, ESC_1_IDX, convert_float_to_uint16(base_throttle + esc_vector[0], 1));
  fill_debug_values(buf, ESC_2_IDX, convert_float_to_uint16(base_throttle + esc_vector[1], 1));
  fill_debug_values(buf, ESC_3_IDX, convert_float_to_uint16(base_throttle + esc_vector[2], 1));
  fill_debug_values(buf, ESC_4_IDX, convert_float_to_uint16(base_throttle + esc_vector[3], 1));
  fill_debug_values(buf, GYRO_X_IDX, convert_float_to_uint16(raw_gyro_x_sensor, 1));
  fill_debug_values(buf, GYRO_Y_IDX, convert_float_to_uint16(raw_gyro_y_sensor, 1));
  fill_debug_values(buf, GYRO_Z_IDX, convert_float_to_uint16(raw_gyro_z_sensor, 1));
  fill_debug_values(buf, ACC_X_IDX, convert_float_to_uint16(raw_acc_x_sensor, 1));
  fill_debug_values(buf, ACC_Y_IDX, convert_float_to_uint16(raw_acc_y_sensor, 1));
  fill_debug_values(buf, ACC_Z_IDX, convert_float_to_uint16(raw_acc_z_sensor, 1));
  uint16_t crc = 0xffff;
  for (int i = 0; i < 2 + NUM_OF_DEBUG_VALUES * 2; i++) {
    crc -= buf[i];
  }
  buf[2 + NUM_OF_DEBUG_VALUES * 2] = crc & 0x00ff;
  buf[2 + NUM_OF_DEBUG_VALUES * 2 + 1] = (uint8_t)((crc >> 8) & 0x00ff);
  uart_transmit_it(1, buf, 2 + NUM_OF_DEBUG_VALUES * 2 + 2);
}

uint16_t convert_float_to_uint16 (float value, float scale) {
  int ret = (int)(value * scale);
  ret = MIN(MAX(ret, -15000), 15000);
  ret += 15000;
  return (uint16_t)ret;
}

void fill_debug_values (uint8_t *buf, int idx, uint16_t value) {
  buf[2 + idx * 2] = (value & 0x00ff);
  buf[2 + idx * 2 + 1] = (value >> 8) & 0x00ff;
}
