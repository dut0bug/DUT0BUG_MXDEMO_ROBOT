#include "PID.h"

PID pid_chassis[4]; //底盘电机控速PID(4个M3508)
PID pid_gimbal[2]; //云台电机控速PID(2个GM6020)
PID pid_ammobooster[3]; //发射机构电机控速PID(2个M3508,1个M2006)

PID pid_yaw, pid_pitch; //云台姿态控位PID(yaw,pitch)
PID pid_rotation; //底盘旋转控位PID
PID pid_moveball; //发射拨弹控位PID

PID pid_gyrotemp; //陀螺控温PID

//
void InitPID(void)
{
	uint8_t i;
	
	//底盘电机控速PID
	for(i=0;i<4;i++)
	{
		pid_chassis[i].Ref = 0;
		pid_chassis[i].FeedBack = 0;
		pid_chassis[i].Error = 0;
		pid_chassis[i].DError = 0;
		pid_chassis[i].DDError = 0;
		pid_chassis[i].PreError = 0;
		pid_chassis[i].PreDError = 0;
		pid_chassis[i].Kp = 1200;
		pid_chassis[i].Ki = 700;
		pid_chassis[i].Kd = 10;
		pid_chassis[i].MaxOutValue = 5000; //16384
		pid_chassis[i].MinOutValue = -5000;
		pid_chassis[i].Out = 0;
	}
	
	//云台电机控速PID(yaw,pitch)
	for(i=0;i<2;i++)
	{
		pid_gimbal[i].Ref = 0;
		pid_gimbal[i].FeedBack = 0;
		pid_gimbal[i].Error = 0;
		pid_gimbal[i].DError = 0;
		pid_gimbal[i].DDError = 0;
		pid_gimbal[i].PreError = 0;
		pid_gimbal[i].PreDError = 0;
		pid_gimbal[i].Kp = 2000;
		pid_gimbal[i].Ki = 300;
		pid_gimbal[i].Kd = 10;
		pid_gimbal[i].MaxOutValue = 30000;
		pid_gimbal[i].MinOutValue = -30000;
		pid_gimbal[i].Out = 0;
	}
	
	//摩擦轮电机控速PID
	for(i=0;i<2;i++)
	{
		pid_ammobooster[i].Ref = 0;
		pid_ammobooster[i].FeedBack = 0;
		pid_ammobooster[i].Error = 0;
		pid_ammobooster[i].DError = 0;
		pid_ammobooster[i].DDError = 0;
		pid_ammobooster[i].PreError = 0;
		pid_ammobooster[i].PreDError = 0;
		pid_ammobooster[i].Kp = 2000;
		pid_ammobooster[i].Ki = 1500;
		pid_ammobooster[i].Kd = 30;
		pid_ammobooster[i].MaxOutValue = 16384;
		pid_ammobooster[i].MinOutValue = -16384;
		pid_ammobooster[i].Out = 0;
	}
	
	//拨弹电机控速PID
	pid_ammobooster[2].Ref = 0;
	pid_ammobooster[2].FeedBack = 0;
	pid_ammobooster[2].Error = 0;
	pid_ammobooster[2].DError = 0;
	pid_ammobooster[2].DDError = 0;
	pid_ammobooster[2].PreError = 0;
	pid_ammobooster[2].PreDError = 0;
	pid_ammobooster[2].Kp = 800;
	pid_ammobooster[2].Ki = 300;
	pid_ammobooster[2].Kd = 10;
	pid_ammobooster[2].MaxOutValue = 10000;
	pid_ammobooster[2].MinOutValue = -10000;
	pid_ammobooster[2].Out = 0;
	
	//云台姿态控位PID(yaw,pitch)
	pid_yaw.Ref = 0;
	pid_yaw.FeedBack = 0;
	pid_yaw.Error = 0;
	pid_yaw.DError = 0;
	pid_yaw.DDError = 0;
	pid_yaw.PreError = 0;
	pid_yaw.PreDError = 0;
	pid_yaw.Kp = 600;
	pid_yaw.Ki = 10;
	pid_yaw.Kd = 5;
	pid_yaw.MaxOutValue = 6000;
	pid_yaw.MinOutValue = -6000;
	pid_yaw.Out = 0;
	
	pid_pitch.Ref = 0;
	pid_pitch.FeedBack = 0;
	pid_pitch.Error = 0;
	pid_pitch.DError = 0;
	pid_pitch.DDError = 0;
	pid_pitch.PreError = 0;
	pid_pitch.PreDError = 0;
	pid_pitch.Kp = 600;
	pid_pitch.Ki = 10;
	pid_pitch.Kd = 5;
	pid_pitch.MaxOutValue = 6000;
	pid_pitch.MinOutValue = -6000;
	pid_pitch.Out = 0;
	
	//底盘旋转位置PID
	pid_rotation.Ref = 0;
	pid_rotation.FeedBack = 0;
	pid_rotation.Error = 0;
	pid_rotation.DError = 0;
	pid_rotation.DDError = 0;
	pid_rotation.PreError = 0;
	pid_rotation.PreDError = 0;
	pid_rotation.Kp = 600;
	pid_rotation.Ki = 30;
	pid_rotation.Kd = 2;
	pid_rotation.MaxOutValue = 3600;
	pid_rotation.MinOutValue = -3600;
	pid_rotation.Out = 0;
	
	//发射拨弹控位PID
	pid_moveball.Ref = 0;
	pid_moveball.FeedBack = 0;
	pid_moveball.Error = 0;
	pid_moveball.DError = 0;
	pid_moveball.DDError = 0;
	pid_moveball.PreError = 0;
	pid_moveball.PreDError = 0;
	pid_moveball.Kp = 10;
	pid_moveball.Ki = 5;
	pid_moveball.Kd = 0;
	pid_moveball.MaxOutValue = 660;
	pid_moveball.MinOutValue = -660;
	pid_moveball.Out = 0;
	
	//陀螺控温PID
	pid_gyrotemp.Ref = 4500;
	pid_gyrotemp.FeedBack = 0;
	pid_gyrotemp.Error = 0;
	pid_gyrotemp.DError = 0;
	pid_gyrotemp.DDError = 0;
	pid_gyrotemp.PreError = 0;
	pid_gyrotemp.PreDError = 0;
	pid_gyrotemp.Kp = 800;
	pid_gyrotemp.Ki = 500;
	pid_gyrotemp.Kd = 0;
	pid_gyrotemp.MaxOutValue = 1000;
	pid_gyrotemp.MinOutValue = 0;
	pid_gyrotemp.Out = 0;
}

//
void PIDSetPara(PID *pid, uint16_t kp, uint16_t ki, uint16_t kd)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

//
int32_t PIDControl(PID *pid, int32_t fdb)
{
	//
	pid->FeedBack = fdb;

	//
	pid->Error = pid->Ref - pid->FeedBack;
	pid->DError = pid->Error - pid->PreError;
	pid->DDError = pid->DError - pid->PreDError;
	
	pid->PreError = pid->Error;
	pid->PreDError = pid->DError;

	//
	pid->Out += (pid->Kp * pid->DError + pid->Ki * pid->Error / 10 + pid->Kd * pid->DDError * 100);

	//
	if(pid->Out > (pid->MaxOutValue*100))
	{
		 pid->Out = (pid->MaxOutValue*100);	
	}
	if(pid->Out < (pid->MinOutValue*100))
	{
		 pid->Out = (pid->MinOutValue*100);
	}

	//
	return pid->Out/100;
}
