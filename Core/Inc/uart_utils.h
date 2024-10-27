/*
 * uart_utils.h
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#ifndef INC_UART_UTILS_H_
#define INC_UART_UTILS_H_

#define ACTIVE_UART_SIZE 2

extern int active_uart_nums[ACTIVE_UART_SIZE];
extern USART_TypeDef *uart_typedefs[4];

extern uint8_t uart_lock[4];
extern uint8_t remaining_transmit_cnt[4];
extern uint8_t uart_transmit_buf[4][1000];

void init_uart ();
void uart_transmit_it (int uart_num, uint8_t *buf, int len);
void handle_uart_tx_it (int uart_num);
void handle_uart1_rx_it ();
void handle_uart3_rx_it ();

#endif /* INC_UART_UTILS_H_ */
