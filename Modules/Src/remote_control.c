#include "remote_control.h"
#include "INS.h"

RC_ctrl_t rc_ctrl = {0};
ROBOT_ctrl_t robot_ctrl = {0};

uint8_t rc_update_flag = 0;

//
void dbus_to_rc(uint8_t *dbus_buf, RC_ctrl_t *rc_ctrl)
{
	if (dbus_buf == NULL || rc_ctrl == NULL) { return; }

	rc_ctrl->ch1 = (dbus_buf[0] | (dbus_buf[1] << 8)) & 0x07ff;
	rc_ctrl->ch2 = ((dbus_buf[1] >> 3) | (dbus_buf[2] << 5)) & 0x07ff;
	rc_ctrl->ch3 = ((dbus_buf[2] >> 6) | (dbus_buf[3] << 2) | (dbus_buf[4] << 10)) & 0x07ff;
	rc_ctrl->ch4 = ((dbus_buf[4] >> 1) | (dbus_buf[5] << 7)) & 0x07ff;
	rc_ctrl->sw_right = ((dbus_buf[5] >> 4) & 0x0003);
	rc_ctrl->sw_left = ((dbus_buf[5] >> 4) & 0x000C) >> 2;
	rc_ctrl->mouse.vx = dbus_buf[6] | (dbus_buf[7] << 8);
	rc_ctrl->mouse.vy = dbus_buf[8] | (dbus_buf[9] << 8);
	rc_ctrl->mouse.vz = dbus_buf[10] | (dbus_buf[11] << 8);
	rc_ctrl->mouse.press_l = dbus_buf[12];
	rc_ctrl->mouse.press_r = dbus_buf[13];
	rc_ctrl->keyboard.key_code = dbus_buf[14] | (dbus_buf[15] << 8);
	rc_ctrl->wheel = dbus_buf[16] | (dbus_buf[17] << 8);

	rc_ctrl->ch1 -= 1024;
	rc_ctrl->ch2 -= 1024;
	rc_ctrl->ch3 -= 1024;
	rc_ctrl->ch4 -= 1024;
	rc_ctrl->wheel -= 1024;
}

//
void rc_to_robot(RC_ctrl_t *rc_ctrl, ROBOT_ctrl_t *robot_ctrl)
{
	static uint8_t key_gyro_rotation_flag = 0;
	static uint8_t key_shoot_open_flag = 0;
	
	//
	if(rc_ctrl->sw_right==RC_SW_DOWN) //右拨档下，全部RM电机失能模式
	{
		robot_ctrl->movement_mode = MOTOR_DISABLE;
	}
	else //电机闭环使能
	{	
		//vx
		if(rc_ctrl->keyboard.key_bit.W) {robot_ctrl->vx_set += 30; }
		else if(rc_ctrl->keyboard.key_bit.S) {robot_ctrl->vx_set -= 30; }
		else {robot_ctrl->vx_set = rc_ctrl->ch4*5;}
		
		//vx limit
		if(robot_ctrl->vx_set>3000){robot_ctrl->vx_set=3000;}
		else if(robot_ctrl->vx_set<-3000){robot_ctrl->vx_set=-3000;}
		
		//vy
		if(rc_ctrl->keyboard.key_bit.A){robot_ctrl->vy_set += 30; }
		else if(rc_ctrl->keyboard.key_bit.D){robot_ctrl->vy_set -= 30; }
		else{robot_ctrl->vy_set = -rc_ctrl->ch3*5;}
		
		//vy limit
		if(robot_ctrl->vy_set>3000){robot_ctrl->vy_set=3000;}
		else if(robot_ctrl->vy_set<-3000){robot_ctrl->vy_set=-3000;}
		
		//yaw
		if(rc_ctrl->mouse.vx!=0) {robot_ctrl->yaw_angle_set -= rc_ctrl->mouse.vx/2;}
		else {robot_ctrl->yaw_angle_set -= rc_ctrl->ch1/20;}
			
		//yaw limit
		if(robot_ctrl->yaw_angle_set>=900)
		{
			robot_ctrl->yaw_angle_set = robot_ctrl->yaw_angle_set-900;
			dial_rotate_angle+=90.0f; if(dial_rotate_angle==360.0f){dial_rotate_angle=0.0f;}
			INS_dial_update();
		}
		else if(robot_ctrl->yaw_angle_set<=-900)
		{
			robot_ctrl->yaw_angle_set = robot_ctrl->yaw_angle_set+900;
			dial_rotate_angle-=90.0f; if(dial_rotate_angle==-90.0f){dial_rotate_angle=270.0f;}
			INS_dial_update();
		}
		
		//pitch
		if(rc_ctrl->mouse.vy!=0) {robot_ctrl->pitch_angle_set -= rc_ctrl->mouse.vy/2;}
		else {robot_ctrl->pitch_angle_set += rc_ctrl->ch2/50;}
		
		//pitch limit
		if(robot_ctrl->pitch_angle_set>300){robot_ctrl->pitch_angle_set=300;}
		else if(robot_ctrl->pitch_angle_set<-150){robot_ctrl->pitch_angle_set=-150;}
		
		//
		if(rc_ctrl->keyboard.key_bit.F){key_gyro_rotation_flag=1;} //短按F，开陀螺旋转
		else if(rc_ctrl->keyboard.key_bit.G){key_gyro_rotation_flag=0;} //短按G，关陀螺旋转
			
		//
		if((key_gyro_rotation_flag==1)||(rc_ctrl->sw_right==RC_SW_UP)) //右拨档上，底盘陀螺旋转模式
		{
			robot_ctrl->movement_mode = CHASSIS_GYRO_ROTATION;
			
		}
		else if((key_gyro_rotation_flag==0)||(rc_ctrl->sw_right==RC_SW_MID)) //右拨档中，底盘跟随云台模式
		{
			if((robot_ctrl->movement_mode==MOTOR_DISABLE) || ((robot_ctrl->movement_mode==CHASSIS_GYRO_ROTATION)&&(robot_ctrl->gyro_rotate_endflag==1)))
			{
				robot_ctrl->movement_mode = CHASSIS_FOLLOW_GIMBAL;
			}
		}
		
		//
		if(rc_ctrl->keyboard.key_bit.Q){key_shoot_open_flag=1;} //短按Q，开摩擦轮和红点激光
		else if(rc_ctrl->keyboard.key_bit.E){key_shoot_open_flag=0;} //短按E，关摩擦轮和红点激光
		
		//
		if((key_shoot_open_flag==1)||(rc_ctrl->sw_left==RC_SW_UP)) //左拨档上，开摩擦轮电机和红点激光
		{
			robot_ctrl->shoot_open_flag = 1;
			
			if((robot_ctrl->trigger_lock_flag==0)&&((rc_ctrl->wheel>550)||(rc_ctrl->mouse.press_l)))
			{
				robot_ctrl->trigger_lock_flag = 1;
				robot_ctrl->trigger_one_flag = 1;
			}
			else if((robot_ctrl->trigger_lock_flag==0)&&((rc_ctrl->wheel<-550)||(rc_ctrl->mouse.press_r)))
			{
				robot_ctrl->trigger_lock_flag = 1;
				robot_ctrl->trigger_continuos_flag = 1;
			}
			else if((robot_ctrl->trigger_movingflag==0)&&(rc_ctrl->wheel==0))
			{
				robot_ctrl->trigger_lock_flag = 0;
				robot_ctrl->trigger_continuos_flag = 0;
			}
			
			//robot_ctrl->trigger_angle_set = (rc_ctrl->wheel>=0)?(0):(-rc_ctrl->wheel);
			//robot_ctrl->trigger_angle_set = rc_ctrl->wheel;
		}
		else if((key_shoot_open_flag==0)||(rc_ctrl->sw_left==RC_SW_MID)) //左拨档中，关闭摩擦轮电机和红点激光
		{
			robot_ctrl->shoot_open_flag = 0;
		}
		
		//
		if((rc_ctrl->keyboard.key_bit.R)||(rc_ctrl->sw_left==RC_SW_DOWN)) //常按R/左拨档下，开弹舱盖
		{
			robot_ctrl->ballroom_open_flag = 1;
		}
		else
		{
			robot_ctrl->ballroom_open_flag = 0;
		}
		
	}

}
