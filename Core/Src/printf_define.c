/*
 * printf_define.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

int _write (int fd, char *buf, int len) {
  uart_transmit_it(1, buf, len);
  return len;
}
