#ifndef INS_H
#define INS_H

#include "main.h"


extern float INS_angle_deg[3];
extern float dial_rotate_angle,INS_yaw_angle_dial;

extern void AHRS_update(float gyro[3], float accel[3], float mag[3]);
extern void INS_dial_update(void);

#endif
