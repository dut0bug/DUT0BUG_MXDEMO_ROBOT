#ifndef PID_H
#define PID_H

#include "main.h"

/*---------------------VARIABLES---------------------*/
typedef struct _PID
{
	int32_t Ref; //�ο�ֵ(Ŀ��ֵ)
	int32_t FeedBack; //����ֵ

	int32_t Error; //���
	int32_t DError;
	int32_t DDError;
	int32_t PreError;
	int32_t PreDError;
	
	uint16_t Kp; //pid����
	uint16_t Ki;
	uint16_t Kd;
	
	int32_t MaxOutValue; //����޷�
	int32_t MinOutValue;
	
	int32_t Out; //���ֵ

} PID;

/*---------------------DECLARES----------------------*/
extern PID pid_chassis[4], pid_gimbal[2], pid_ammobooster[3], pid_moveball, pid_yaw, pid_pitch, pid_rotation, pid_gyrotemp;

extern void InitPID(void);
extern void PIDSetPara(PID *pid, uint16_t kp, uint16_t ki, uint16_t kd);

extern int32_t PIDControl(PID *pid, int32_t fdb);

#endif
