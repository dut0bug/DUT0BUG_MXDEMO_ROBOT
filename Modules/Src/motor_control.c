#include "motor_control.h"
#include "remote_control.h"
#include "INS.h"
#include "bmi088.h"
#include "pid.h"
#include "can.h"
#include "tim.h"

#define YAW6020_FORWARD_POSITION	4096	//机器人固定参数

//
Motor_Status_t motor_status_chassis[4] = {0};
Motor_Status_t motor_status_ammobooster[3] = {0};
Motor_Status_t motor_status_gimbal[2] = {0};
Motor_Status_Ex_t motor_status_ex_trigger = {0};

Robot_Motion_t robot_motion;


//
void UpdateMotorStatus(Motor_Status_t *ptr, uint8_t *data)
{
	//
	ptr->angle = (uint16_t)((data[0]<<8)|data[1]);
	ptr->speed_rpm = (int16_t)((data[2]<<8)|data[3]);
	ptr->current = (int16_t)((data[4]<<8)|data[5]);
	ptr->temperate = data[6];
}

//
void UpdateMotorStatusEx(Motor_Status_Ex_t *ptr_ex, Motor_Status_t *ptr)
{
	//
	ptr_ex->delt_angle = ptr->angle - ptr_ex->last_angle;
	
	if(ptr_ex->delt_angle<-4096){ptr_ex->round_cnt++;}
	else if(ptr_ex->delt_angle>4096){ptr_ex->round_cnt--;}
	
	ptr_ex->total_angle = ptr_ex->round_cnt*8192 + ptr->angle - ptr_ex->offset_angle;
	
	ptr_ex->last_angle = ptr->angle;
}

//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	uint8_t i;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if(hcan==&hcan1)
	{
		switch (rx_header.StdId)
		{
			case CAN_3508_MOTOR1_ID: 
			case CAN_3508_MOTOR2_ID:
			case CAN_3508_MOTOR3_ID:
			case CAN_3508_MOTOR4_ID:
			{
				i = rx_header.StdId - CAN_3508_MOTOR1_ID;
				UpdateMotorStatus(&motor_status_chassis[i], rx_data);
				break;
			}
			case CAN_3508_SHOOT1_ID:  
			case CAN_3508_SHOOT2_ID:
			{
				i = rx_header.StdId - CAN_3508_SHOOT1_ID;
				UpdateMotorStatus(&motor_status_ammobooster[i], rx_data);
				break;
			}
			case CAN_2006_TRIGGER_ID:
			{
				UpdateMotorStatus(&motor_status_ammobooster[2], rx_data);
				UpdateMotorStatusEx(&motor_status_ex_trigger, &motor_status_ammobooster[2]);
				break;
			}
			default: break;
		}
	}
	else if(hcan==&hcan2)
	{
		switch (rx_header.StdId)
		{
			case CAN_6020_YAW_ID: 
			case CAN_6020_PIT_ID:
			{
				i = rx_header.StdId - CAN_6020_YAW_ID;
				UpdateMotorStatus(&motor_status_gimbal[i], rx_data);
				//UpdateMotorStatus(&motor_status_gimbal[i/2], rx_data);			
				break;
			}
			default: break;
		}
	}
}

//
void CAN_cmd_motor(CAN_HandleTypeDef *hcan, uint32_t identifier, int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];
	uint32_t tx_mail_box;
	
	tx_header.StdId = identifier;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	
	tx_data[0] = (data1 >> 8);
	tx_data[1] = data1;
	tx_data[2] = (data2 >> 8);
	tx_data[3] = data2;
	tx_data[4] = (data3 >> 8);
	tx_data[5] = data3;
	tx_data[6] = (data4 >> 8);
	tx_data[7] = data4;
	
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mail_box);
}

//
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	CAN_cmd_motor(&hcan1, CAN_CHASSIS_CTRL_ID, motor1, motor2, motor3, motor4);
}

//
void CAN_cmd_ammobooster(int16_t shoot1, int16_t shoot2, int16_t trigger)
{
	CAN_cmd_motor(&hcan1, CAN_AMMOBOOSTER_CTRL_ID, shoot1, shoot2, trigger, 0);
}

//
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
	CAN_cmd_motor(&hcan2, CAN_GIMBAL_CTRL_ID, yaw, pitch, 0, 0);
}

//
void set_chassis_motor_speed(int16_t wheel_speed[4])
{
	uint8_t i;
	int32_t output_chassis[4];
	
	for(i=0;i<4;i++)
	{
		pid_chassis[i].Ref = wheel_speed[i];
		output_chassis[i] = PIDControl(&pid_chassis[i], motor_status_chassis[i].speed_rpm);
	}
				
	CAN_cmd_chassis(output_chassis[0], output_chassis[1], output_chassis[2], output_chassis[3]);
}

//
void set_gimbal_motor_speed(int16_t yaw_speed, int16_t pitch_speed)
{
	float yaw_axis_speed,pitch_axis_speed;
	int32_t output_yaw, output_pitch;
	
	pid_gimbal[0].Ref = yaw_speed;
	yaw_axis_speed = 10*sqrtf(bmi088_gyro_deg_calib[2]*bmi088_gyro_deg_calib[2]+bmi088_gyro_deg_calib[0]*bmi088_gyro_deg_calib[0]);
	if(bmi088_gyro_deg_calib[2]<0){yaw_axis_speed=-yaw_axis_speed;}
	output_yaw = PIDControl(&pid_gimbal[0], yaw_axis_speed);
	
	pid_gimbal[1].Ref = pitch_speed;
	pitch_axis_speed = 10*bmi088_gyro_deg_calib[1];
	output_pitch = PIDControl(&pid_gimbal[1], pitch_axis_speed);
				
	CAN_cmd_gimbal(output_yaw, output_pitch);
}

//
void set_ammobooster_speed(int16_t shoot_speed, int16_t trigger_speed)
{
	int32_t output_shoot[2],output_trigger;
	
	pid_ammobooster[0].Ref = shoot_speed;
	output_shoot[0] = PIDControl(&pid_ammobooster[0], motor_status_ammobooster[0].speed_rpm);
	
	pid_ammobooster[1].Ref = -shoot_speed;
	output_shoot[1] = PIDControl(&pid_ammobooster[1], motor_status_ammobooster[1].speed_rpm);

	pid_ammobooster[2].Ref = trigger_speed;
	output_trigger = PIDControl(&pid_ammobooster[2], motor_status_ammobooster[2].speed_rpm);
				
	CAN_cmd_ammobooster(output_shoot[0], output_shoot[1], output_trigger);
}

//
void set_gimbal_motor_angle(int16_t yaw_angle, int16_t pitch_angle)
{
	pid_yaw.Ref = yaw_angle;
	robot_motion.yaw_speed = PIDControl(&pid_yaw, INS_yaw_angle_dial*10);
	
	pid_pitch.Ref = pitch_angle;
	robot_motion.pitch_speed = PIDControl(&pid_pitch, INS_angle_deg[1]*10);
}

//
void set_ammobooster_trigger_one(void)
{
	//
	if(robot_ctrl.trigger_movingflag==1)
	{
		robot_motion.trigger_speed = PIDControl(&pid_moveball, motor_status_ex_trigger.total_angle);
		
		if(ABS(pid_moveball.Error)<100)
		{
			robot_ctrl.trigger_movingflag=0;
			robot_motion.trigger_speed=0;
		}
	}
}

//
void robot_motion_resolving(Robot_Motion_t *motion, ROBOT_ctrl_t *ctrl)
{
	int16_t vw_set;
	int16_t vx_resolve,vy_resolve;
	float angle;
	
	//陀螺仪姿态控制
	set_gimbal_motor_angle(ctrl->yaw_angle_set,ctrl->pitch_angle_set);
	//motion->yaw_speed = ctrl->yaw_angle_set;
	//motion->pitch_speed = ctrl->pitch_angle_set;
	
	//底盘运动控制
	if(ctrl->movement_mode==CHASSIS_FOLLOW_GIMBAL)
	{
		pid_rotation.Ref = YAW6020_FORWARD_POSITION;
		vw_set = -1*PIDControl(&pid_rotation, motor_status_gimbal[0].angle); //当前yaw6020的角度值
		//vw_set=0;
	}
	else if(ctrl->movement_mode==CHASSIS_GYRO_ROTATION)
	{
		vw_set = 2000;
		
		if(ABS(motor_status_gimbal[0].angle-YAW6020_FORWARD_POSITION)<500) {ctrl->gyro_rotate_endflag=1;}
		else {ctrl->gyro_rotate_endflag=0;}
	}
	
	angle = (4096-motor_status_gimbal[0].angle)*6.2831853f/8192;
	vx_resolve = ctrl->vx_set*cosf(angle) + ctrl->vy_set*sinf(angle);
	vy_resolve = ctrl->vy_set*cosf(angle) - ctrl->vx_set*sinf(angle);
	
	motion->chassis_speed[0] = -(vx_resolve + vy_resolve + vw_set);
	motion->chassis_speed[1] = vx_resolve - vy_resolve - vw_set;
	motion->chassis_speed[2] = vx_resolve + vy_resolve - vw_set;
	motion->chassis_speed[3] = -(vx_resolve - vy_resolve + vw_set);
	
	//
	//motion->chassis_speed[0] = -(ctrl->vx_set + ctrl->vy_set + vw_set);
	//motion->chassis_speed[1] = ctrl->vx_set - ctrl->vy_set - vw_set;
	//motion->chassis_speed[2] = ctrl->vx_set + ctrl->vy_set - vw_set;
	//motion->chassis_speed[3] = -(ctrl->vx_set - ctrl->vy_set + vw_set);
	
	
	//摩擦轮和红点激光器控制
	if(ctrl->shoot_open_flag==1)
	{
		HAL_GPIO_WritePin(LASER_GPIO_Port,LASER_Pin,GPIO_PIN_SET); //open
		motion->shoot_speed=4500;
	}
	else
	{
		HAL_GPIO_WritePin(LASER_GPIO_Port,LASER_Pin,GPIO_PIN_RESET); //close
		motion->shoot_speed=0;
	}
	
	//拨弹动作控制
	if(ctrl->trigger_one_flag==1)
	{
		motor_status_ex_trigger.offset_angle = motor_status_ammobooster[2].angle;
		motor_status_ex_trigger.round_cnt=0;
		UpdateMotorStatusEx(&motor_status_ex_trigger, &motor_status_ammobooster[2]);
		pid_moveball.Ref = 32768;

		ctrl->trigger_movingflag=1;
		ctrl->trigger_one_flag=0;
	}
	else if((ctrl->trigger_continuos_flag==1)&&(ctrl->trigger_movingflag==0))
	{
		ctrl->trigger_one_flag=1;
	}
	
	set_ammobooster_trigger_one();
	//motion->trigger_speed = ctrl->trigger_angle_set;
	
	//弹舱盖动作控制
	if(ctrl->ballroom_open_flag==1)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1800); //open
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,600); //close
	}
	
}

