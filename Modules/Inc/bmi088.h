#ifndef BMI088_H
#define BMI088_H

#include "main.h"

extern uint8_t gyro_zero_calib_flag;

extern float bmi088_gyro[3], bmi088_gyro_deg[3], bmi088_accel[3], bmi088_temperature;
extern float bmi088_gyro_zero[3], bmi088_gyro_deg_zero[3], bmi088_gyro_calib[3], bmi088_gyro_deg_calib[3];

void BMI088_init(void);
void BMI088_calib_gyro_zero(float gyro[3], float gyro_deg[3]);
void BMI088_read_gyro(void);
void BMI088_read_accel(void);
void BMI088_read_temperature(void);

#endif
