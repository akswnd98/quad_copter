/*
 * uart_utils.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include "main.h"
#include <string.h>
#include "uart_utils.h"
#include "control_receive.h"
#include "gain_receive.h"

int active_uart_nums[ACTIVE_UART_SIZE] = {1, 3};
USART_TypeDef *uart_typedefs[4] = {0, USART1, 0, USART3};

uint8_t uart_lock[4] = {0, };
uint8_t cur_transmit_cnt[4] = {0, };
uint8_t cur_transmit_len[4] = {0, };
uint8_t uart_transmit_buf[4][1000];

void init_uart () {
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_RXNE(USART3);
}

void uart_transmit_it (int uart_num, uint8_t *buf, int len) {
  if (!uart_lock[uart_num]) {
    uart_lock[uart_num] = 1;
    memcpy(uart_transmit_buf[uart_num], buf, len);
    cur_transmit_cnt[uart_num] = 0;
    cur_transmit_len[uart_num] = len;
    LL_USART_EnableIT_TXE(uart_typedefs[uart_num]);
  }
}

void handle_uart_tx_it (int uart_num) {
  if (LL_USART_IsActiveFlag_TXE(uart_typedefs[uart_num])) {
    LL_USART_TransmitData8(uart_typedefs[uart_num], uart_transmit_buf[uart_num][cur_transmit_cnt[uart_num]]);
    if (++cur_transmit_cnt[uart_num] >= cur_transmit_len[uart_num]) {
      LL_USART_DisableIT_TXE(uart_typedefs[uart_num]);
      uart_lock[uart_num] = 0;
    }
  }
}

void handle_uart1_rx_it () {
  if (LL_USART_IsActiveFlag_RXNE(USART1)) {
    uint8_t rx_data = LL_USART_ReceiveData8(USART1);
    process_gain_receive(rx_data);
  }
}

void handle_uart3_rx_it () {
  if (LL_USART_IsActiveFlag_RXNE(USART3)) {
    uint8_t rx_data = LL_USART_ReceiveData8(USART3);
    process_control_receive(rx_data);
  }
}
