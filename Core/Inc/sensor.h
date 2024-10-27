/*
 * sensor.h
 *
 *  Created on: Apr 28, 2023
 *      Author: akswnd98
 */

#include "main.h"

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

void read_mpu (uint8_t addr, uint8_t *data, uint8_t len);
void write_mpu (uint8_t addr, uint8_t *data);
void init_mpu ();
void calibrate_mpu ();

extern uint8_t mpu_buf[14];
extern int16_t raw_acc_x_sensor;
extern int16_t raw_acc_y_sensor;
extern int16_t raw_acc_z_sensor;
extern int16_t raw_gyro_x_sensor;
extern int16_t raw_gyro_y_sensor;
extern int16_t raw_gyro_z_sensor;
extern float acc_x_sensor;
extern float acc_y_sensor;
extern float acc_z_sensor;
extern float gyro_x_sensor;
extern float gyro_y_sensor;
extern float gyro_z_sensor;

extern uint8_t gyro_offset[6];
extern int16_t gyro_x_offset;
extern int16_t gyro_y_offset;
extern int16_t gyro_z_offset;

extern uint8_t acc_offset[6];
extern int16_t acc_x_offset;
extern int16_t acc_y_offset;
extern int16_t acc_z_offset;

void read_mpu_offset ();
void read_mpu_sensor ();

#endif /* INC_SENSOR_H_ */
