#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "remote_control.h"

//CAN send and receive ID
typedef enum
{
		CAN_CHASSIS_CTRL_ID = 0x200,
		CAN_AMMOBOOSTER_CTRL_ID = 0x1FF,
		CAN_GIMBAL_CTRL_ID = 0x1FF, //0x2FF
	
    CAN_3508_MOTOR1_ID = 0x201, //chassis can1
    CAN_3508_MOTOR2_ID = 0x202,
    CAN_3508_MOTOR3_ID = 0x203,
    CAN_3508_MOTOR4_ID = 0x204,

		CAN_3508_SHOOT1_ID = 0x205, //ammo-booster can1
		CAN_3508_SHOOT2_ID = 0x206,
		CAN_2006_TRIGGER_ID = 0x207,
	
    CAN_6020_YAW_ID = 0x205, //gimbal can2 0x209
    CAN_6020_PIT_ID = 0x206 // 0x20B
	
} can_msg_id;

//rm motor data
typedef struct
{
	uint16_t angle;
	int16_t speed_rpm;
	int16_t current;
	uint8_t temperate;
	
} Motor_Status_t;

typedef struct
{
	uint16_t offset_angle;
	uint16_t last_angle;
	int16_t delt_angle;
	int16_t round_cnt;
	int32_t total_angle;
	
} Motor_Status_Ex_t;

//robot motion data
typedef struct
{
	int16_t chassis_speed[4];
	int16_t yaw_speed, pitch_speed;
	int16_t shoot_speed;
	int16_t trigger_speed;
	
} Robot_Motion_t;

//
extern Motor_Status_t motor_status_chassis[4];
extern Motor_Status_t motor_status_ammobooster[3];
extern Motor_Status_t motor_status_gimbal[2];
extern Motor_Status_Ex_t motor_status_ex_trigger;

extern Robot_Motion_t robot_motion;

//
void CAN_cmd_motor(CAN_HandleTypeDef *hcan, uint32_t identifier, int16_t data1, int16_t data2, int16_t data3, int16_t data4);

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_ammobooster(int16_t shoot1, int16_t shoot2, int16_t trigger);
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch);

void set_chassis_motor_speed(int16_t wheel_speed[4]);
void set_gimbal_motor_speed(int16_t yaw_speed, int16_t pitch_speed);
void set_ammobooster_speed(int16_t shoot_speed, int16_t trigger_speed);

void set_gimbal_motor_angle(int16_t yaw_angle, int16_t pitch_angle);
void set_ammobooster_trigger_one(void);

void robot_motion_resolving(Robot_Motion_t *robot_motion, ROBOT_ctrl_t *robot_ctrl);

#endif
