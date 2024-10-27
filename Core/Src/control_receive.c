/*
 * controll_receive.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include "main.h"
#include "control_receive.h"

float command[4];
int control_payload_cnt = 0;
uint8_t control_rx_buf[100];
int16_t control[20] = {0, };
uint16_t control_crc = 0;
uint8_t shutdown = 0;

void process_control_receive (uint8_t data) {
  if (control_payload_cnt == 0 && data == 0x20) {
    control_rx_buf[control_payload_cnt] = data;
    control_payload_cnt++;
  } else if (control_payload_cnt == 1 && data == 0x40) {
    control_rx_buf[control_payload_cnt] = data;
    control_payload_cnt++;
  } else if (control_payload_cnt >= 2 && control_payload_cnt < 2 + 2 * MAX_CHANNEL_NUM) {
    control_rx_buf[control_payload_cnt] = data;
    control_payload_cnt++;
  } else if (control_payload_cnt == 2 + 2 * MAX_CHANNEL_NUM) {
    control_crc &= 0xff00;
    control_crc |= data;
    control_payload_cnt++;
  } else if (control_payload_cnt == 2 + 2 * MAX_CHANNEL_NUM + 1) {
    control_crc &= 0x00ff;
    control_crc |= (uint16_t)data << 8;
    uint16_t check_sum = 0;
    for (int i = 0; i < (MAX_CHANNEL_NUM + 1) * 2; i++) {
      check_sum += control_rx_buf[i];
    }
    if (0xffff - check_sum == control_crc) {
      for (int i = 0, j = 2; i < MAX_CHANNEL_NUM; i++, j += 2) {
        control[i] = *(uint16_t *)(control_rx_buf + j) & 0xfff;
      }
      control[0] -= 1500;
      control[1] -= 1500;
      control[2] -= 1000;
      control[2] = MAX(control[2], 0);
      control[3] -= 1500;
      control[3] = -control[3];
      for (int i = 0; i < 4; i++) {
        command[i] = control[i];
      }
      if (control[4] == 2000) {
        shutdown = 1;
      }
    }
    control_payload_cnt = 0;
  } else {
    control_payload_cnt = 0;
  }
}
