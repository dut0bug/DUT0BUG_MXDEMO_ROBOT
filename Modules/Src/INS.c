#include "INS.h"
#include "MahonyAHRS.h"
#include <math.h>


float INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
float INS_angle_deg[3] = {0.0f, 0.0f, 0.0f};

float dial_rotate_angle =0.0f; //逆时针转刻度盘 0 90 180 270
float INS_yaw_angle_dial;

//
void AHRS_update(float gyro[3], float accel[3], float mag[3])
{
	//使用地磁时调用MahonyAHRSupdate
	//MahonyAHRSupdate(INS_quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
	
	//不使用地磁时调用MahonyAHRSupdateIMU
	MahonyAHRSupdateIMU(INS_quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
	
	//
	INS_angle[0] = atan2f(2.0f*(INS_quat[0]*INS_quat[3]+INS_quat[1]*INS_quat[2]), 2.0f*(INS_quat[0]*INS_quat[0]+INS_quat[1]*INS_quat[1])-1.0f);
  INS_angle[1] = asinf(-2.0f*(INS_quat[1]*INS_quat[3]-INS_quat[0]*INS_quat[2]));
  INS_angle[2] = atan2f(2.0f*(INS_quat[0]*INS_quat[1]+INS_quat[2]*INS_quat[3]),2.0f*(INS_quat[0]*INS_quat[0]+INS_quat[3]*INS_quat[3])-1.0f);
		
	//
	INS_angle_deg[0] = INS_angle[0] * 180.0f / 3.141592653589f;
	INS_angle_deg[1] = INS_angle[1] * 180.0f / 3.141592653589f;
	INS_angle_deg[2] = INS_angle[2] * 180.0f / 3.141592653589f;

	//
	INS_dial_update();
}

//
void INS_dial_update(void)
{
	//
	INS_yaw_angle_dial = INS_angle_deg[0] - dial_rotate_angle;
	if(INS_yaw_angle_dial < -180.0f) {INS_yaw_angle_dial = INS_yaw_angle_dial + 360.0f;}
	else if(INS_yaw_angle_dial > 180.0f) {INS_yaw_angle_dial = INS_yaw_angle_dial - 360.0f;}
}
