/*
 * sensor.c
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include "main.h"
#include <stdio.h>
#include "sensor.h"
#include "math.h"
#include "dynamics.h"
#include "stm32f4xx_hal.h"

#define _USE_MATH_DEFINES

void read_mpu (uint8_t addr, uint8_t *data, uint8_t len) {
  uint8_t tmp = 0x80 | addr;
  while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &tmp, 1, 100);
  HAL_SPI_Receive(&hspi1, data, len, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void write_mpu (uint8_t addr, uint8_t *data) {
  while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
  HAL_SPI_Transmit(&hspi1, data, 1, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void init_mpu () {
  int delay_time = 100;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(delay_time);

  uint8_t tx_val = 0x00;
  uint8_t rx_val;

  read_mpu(0x75, &rx_val, 1);

  tx_val = 0x80;
  write_mpu(0x6B, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x01;
  write_mpu(0x6B, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x00;
  write_mpu(0x19, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x05;
  write_mpu(0x1a, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x03;
  write_mpu(0x1d, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x10; // +- 1000dps
  write_mpu(0x6A, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x08; // +- 4g
  write_mpu(0x1B, &tx_val);
  HAL_Delay(delay_time);

  write_mpu(0x1C, &tx_val);
  HAL_Delay(delay_time);
}

void calibrate_mpu () {
  int delay_time = 100;

  write_mpu(0x13, ((uint8_t *)&gyro_y_offset) + 1);
  HAL_Delay(delay_time);

  write_mpu(0x14, (uint8_t *)&gyro_y_offset);
  HAL_Delay(delay_time);

  write_mpu(0x15, ((uint8_t *)&gyro_x_offset) + 1);
  HAL_Delay(delay_time);

  write_mpu(0x16, (uint8_t *)&gyro_x_offset);
  HAL_Delay(delay_time);

  write_mpu(0x17, ((uint8_t *)&gyro_z_offset) + 1);
  HAL_Delay(delay_time);

  write_mpu(0x18, (uint8_t *)&gyro_z_offset);
  HAL_Delay(delay_time);

  /* write_mpu(0x77, ((uint8_t *)&acc_y_offset) + 1);
  HAL_Delay(delay_time);

  write_mpu(0x78, (uint8_t *)&acc_y_offset);
  HAL_Delay(delay_time);

  write_mpu(0x79, ((uint8_t *)&acc_x_offset) + 1);
  HAL_Delay(delay_time);

  write_mpu(0x7a, (uint8_t *)&acc_x_offset);
  HAL_Delay(delay_time);

  write_mpu(0x7b, ((uint8_t *)&acc_z_offset) + 1);
  HAL_Delay(delay_time);

  write_mpu(0x7c, (uint8_t *)&acc_z_offset);
  HAL_Delay(delay_time); */
}

uint8_t mpu_buf[14];
int16_t raw_acc_x_sensor = 0;
int16_t raw_acc_y_sensor = 0;
int16_t raw_acc_z_sensor = 0;
int16_t raw_gyro_x_sensor = 0;
int16_t raw_gyro_y_sensor = 0;
int16_t raw_gyro_z_sensor = 0;
float acc_x_sensor = 0;
float acc_y_sensor = 0;
float acc_z_sensor = 0;
float gyro_x_sensor = 0;
float gyro_y_sensor = 0;
float gyro_z_sensor = 0;

uint8_t gyro_offset[6];
int16_t gyro_x_offset = -24;
int16_t gyro_y_offset = 21;
int16_t gyro_z_offset = -14;

uint8_t acc_offset[6];
int16_t acc_x_offset = 0;
int16_t acc_y_offset = 0;
int16_t acc_z_offset = 0;

void read_mpu_offset () {
  read_mpu(0x13, gyro_offset, 6);
  gyro_x_offset = (int16_t)(((uint16_t)gyro_offset[2] << 8) | (uint16_t)gyro_offset[3]);
  gyro_y_offset = (int16_t)(((uint16_t)gyro_offset[0] << 8) | (uint16_t)gyro_offset[1]);
  gyro_z_offset = -(int16_t)(((uint16_t)gyro_offset[4] << 8) | (uint16_t)gyro_offset[5]);

  read_mpu(0x77, acc_offset, 6);
  acc_x_offset = -(int16_t)(((uint16_t)acc_offset[2] << 8) | (uint16_t)acc_offset[3]);
  acc_y_offset = -(int16_t)(((uint16_t)acc_offset[0] << 8) | (uint16_t)acc_offset[1]);
  acc_z_offset = (int16_t)(((uint16_t)acc_offset[4] << 8) | (uint16_t)acc_offset[5]);
}

void read_mpu_sensor () {
  read_mpu(0x3B, mpu_buf, 14);

  raw_acc_x_sensor = -(int16_t)(((uint16_t)mpu_buf[2] << 8) | (uint16_t)mpu_buf[3]);
  acc_x_sensor = (float)raw_acc_x_sensor;
  acc_x_sensor = acc_x_sensor / 32767.0 * 4.0 * g;

  raw_acc_y_sensor = -(int16_t)(((uint16_t)mpu_buf[0] << 8) | (uint16_t)mpu_buf[1]);
  acc_y_sensor = (float)raw_acc_y_sensor;
  acc_y_sensor = acc_y_sensor / 32767.0 * 4.0 * g;

  raw_acc_z_sensor = (int16_t)(((uint16_t)mpu_buf[4] << 8) | (uint16_t)mpu_buf[5]);
  acc_z_sensor = (float)raw_acc_z_sensor;
  acc_z_sensor = acc_z_sensor / 32767.0 * 4.0 * g;

  raw_gyro_x_sensor = (int16_t)(((uint16_t)mpu_buf[10] << 8) | (uint16_t)mpu_buf[11]);
  gyro_x_sensor = (float)raw_gyro_x_sensor;
  gyro_x_sensor = gyro_x_sensor / 32767.0 * 1000.0 * M_PI / 180.0;

  raw_gyro_y_sensor = (int16_t)(((uint16_t)mpu_buf[8] << 8) | (uint16_t)mpu_buf[9]);
  gyro_y_sensor = (float)raw_gyro_y_sensor;
  gyro_y_sensor = gyro_y_sensor / 32767.0 * 1000.0 * M_PI / 180.0;

  raw_gyro_z_sensor = -(int16_t)(((uint16_t)mpu_buf[12] << 8) | (uint16_t)mpu_buf[13]);
  gyro_z_sensor = (float)raw_gyro_z_sensor;
  gyro_z_sensor = gyro_z_sensor / 32767.0 * 1000.0 * M_PI / 180.0;
}
