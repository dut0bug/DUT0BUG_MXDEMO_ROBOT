#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/* ----------------------- ROBOT Mode Definition---------------------------- */
#define MOTOR_DISABLE										((uint16_t)1)
#define CHASSIS_FOLLOW_GIMBAL						((uint16_t)2)
#define CHASSIS_GYRO_ROTATION						((uint16_t)3)


/* ----------------------- Data Struct ------------------------------------- */
typedef struct 
{
  // joystick channel value
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  // three_step switch value
  uint8_t sw_left; //sw1
  uint8_t sw_right; //sw2
  // mouse move_speed and button_press value
  struct
  {
    int16_t vx;
    int16_t vy;
    int16_t vz;

    uint8_t press_l;
    uint8_t press_r;
  } mouse;
  // keyboard key value
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } key_bit;
  } keyboard;
  //left_top courner wheel value
  int16_t wheel;
} RC_ctrl_t;

typedef struct
{
	uint8_t movement_mode;
	
	int16_t vx_set,vy_set;
	int16_t yaw_angle_set,pitch_angle_set;
	int16_t trigger_angle_set;
	
	uint8_t ballroom_open_flag;
	uint8_t shoot_open_flag;
	
	uint8_t trigger_lock_flag;
	uint8_t trigger_one_flag;
	uint8_t trigger_continuos_flag;
	
	uint8_t gyro_rotate_endflag;
	uint8_t trigger_movingflag;
	
} ROBOT_ctrl_t;


extern RC_ctrl_t rc_ctrl;
extern ROBOT_ctrl_t robot_ctrl;

extern uint8_t rc_update_flag;

void dbus_to_rc(uint8_t *dbus_buf, RC_ctrl_t *rc_ctrl);
void rc_to_robot(RC_ctrl_t *rc_ctrl, ROBOT_ctrl_t *robot_ctrl);


#endif
