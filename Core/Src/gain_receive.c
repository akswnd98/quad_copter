/*
 * gain_receive.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include "main.h"
#include "gain_receive.h"
#include "dynamics.h"
#include <stdio.h>

uint8_t gain_rx_buf[100];
int gain_payload_cnt = 0;
uint16_t gain[20] = {0, };
uint16_t gain_crc = 0;

void process_gain_receive (uint8_t data) {
  if (gain_payload_cnt == 0 && data == 0x20) {
    gain_rx_buf[gain_payload_cnt] = data;
    gain_payload_cnt++;
  } else if (gain_payload_cnt == 1 && data == 0x40) {
    gain_rx_buf[gain_payload_cnt] = data;
    gain_payload_cnt++;
  } else if (gain_payload_cnt >= 2 && gain_payload_cnt < 2 + 2 * NUM_OF_GAINS) {
    gain_rx_buf[gain_payload_cnt] = data;
    gain_payload_cnt++;
  } else if (gain_payload_cnt == 2 + 2 * NUM_OF_GAINS) {
    gain_crc &= 0xff00;
    gain_crc |= data;
    gain_payload_cnt++;
  } else if (gain_payload_cnt == 2 + 2 * NUM_OF_GAINS + 1) {
    uint16_t check_sum = 0;
    for (int i = 0; i < (NUM_OF_GAINS + 1) * 2; i++) {
      check_sum += gain_rx_buf[i];
    }
    gain_crc &= 0x00ff;
    gain_crc |= ((uint16_t)data << 8);
    if (0xffff - check_sum == gain_crc) {
      for (int i = 0, j = 2; i < NUM_OF_GAINS; i++, j += 2) {
        gain[i] = *(uint16_t *)(gain_rx_buf + j) & 0xfff;
      }
      P_phi = MIN(300, gain[P_PHI_IDX]);
      D_phi = MIN(100, gain[D_PHI_IDX]);
      P_theta = MIN(300, gain[P_THETA_IDX]);
      D_theta = MIN(100, gain[D_THETA_IDX]);
      P_phi_dot = MIN(100, gain[P_PHI_DOT_IDX]);
      D_phi_dot = MIN(50, gain[D_PHI_DOT_IDX]);
      P_theta_dot = MIN(100, gain[P_THETA_DOT_IDX]);
      D_theta_dot = MIN(50, gain[D_THETA_DOT_IDX]);
      P_psi_dot = MIN(1000, gain[P_PSI_DOT_IDX]);
      D_psi_dot = MIN(100, gain[D_PSI_DOT_IDX]);
    }
    gain_payload_cnt = 0;
  } else {
    gain_payload_cnt = 0;
  }
}
