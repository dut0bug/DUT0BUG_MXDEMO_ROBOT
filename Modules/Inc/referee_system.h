#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

#include "main.h"
#include "usart.h"

#define REFEREE_FIFO_BUF_LEN	4
#define REFEREE_FIFO_BUF_NUM REFEREE_BUF_NUM+16

//0x0001,1Hz,比赛状态数据
typedef __packed struct
{ 
	uint8_t game_type : 4; 
	uint8_t game_progress : 4; 
	uint16_t stage_remain_time; 
} ext_game_status_t;

//0x0002,比赛结束后发送,比赛结果数据
typedef __packed struct 
{ 
	uint8_t winner; 
} ext_game_result_t;

//0x0003,1Hz,比赛机器人血量数据
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP; 
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP; 
	uint16_t red_base_HP; 
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t blue_5_robot_HP; 
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP; 
	uint16_t blue_base_HP; 
} ext_game_robot_HP_t;

//0x0004,飞镖发射时发送,飞镖发射状态数据
typedef __packed struct 
{ 
	uint8_t dart_belong; 
	uint16_t stage_remaining_time; 
} ext_dart_status_t;

//0x0005,1Hz,人工智能挑战赛加成与惩罚区状态
typedef __packed struct 
{ 
	uint8_t F1_zone_status:1;
	uint8_t F1_zone_buff_debuff_status:3;
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3;
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3;
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3;
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3;
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;
} ext_ICRA_buff_debuff_zone_status_t;

//0x0101,1Hz,场地事件数据
typedef __packed struct 
{ 
	uint32_t event_type; 
} ext_event_data_t;

//0x0102,动作发生后发送,场地补给站动作标识数据
typedef __packed struct 
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id; 
	uint8_t supply_projectile_step; 
	uint8_t supply_projectile_num; 
} ext_supply_projectile_action_t;

//0x0104,警告发生后发送,裁判警告信息
typedef __packed struct 
{ 
	uint8_t level; 
	uint8_t foul_robot_id; 
} ext_referee_warning_t;

//0x0105,1Hz,飞镖发射口倒计时
typedef __packed struct 
{ 
	uint8_t dart_remaining_time; 
} ext_dart_remaining_time_t;

//0x0201,10Hz,机器人状态数据
typedef __packed struct 
{ 
	uint8_t robot_id; 
	uint8_t robot_level; 
	uint16_t remain_HP; 
	uint16_t max_HP; 
	uint16_t shooter_heat0_cooling_rate; 
	uint16_t shooter_heat0_cooling_limit; 
	uint16_t shooter_heat1_cooling_rate; 
	uint16_t shooter_heat1_cooling_limit; 
	uint8_t shooter_heat0_speed_limit; 
	uint8_t shooter_heat1_speed_limit; 
	uint8_t max_chassis_power; 
	uint8_t mains_power_gimbal_output : 1; 
	uint8_t mains_power_chassis_output : 1; 
	uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_status_t;

//0x0202,50Hz,实时功率热量数据
typedef __packed struct 
{ 
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer; 
	uint16_t shooter_heat0; 
	uint16_t shooter_heat1; 
	uint16_t mobile_shooter_heat2; 
} ext_power_heat_data_t;

//0x0203,10Hz,机器人位置数据
typedef __packed struct 
{ 
	float x; 
	float y; 
	float z; 
	float yaw; 
} ext_game_robot_pos_t;

//0x0204,1Hz,机器人增益数据
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t;

//0x0205,10Hz,空中机器人能量状态数据,仅空中机器人
typedef __packed struct 
{ 
	uint16_t energy_point; 
	uint8_t attack_time; 
} ext_aerial_robot_energy_t;

//0x0206,伤害发生后发送,伤害状态数据
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t;

//0x0207,子弹发射后发送,实时射击数据
typedef __packed struct 
{ 
	uint8_t bullet_type; 
	uint8_t bullet_freq; 
	float bullet_speed; 
} ext_shoot_data_t;

//0x0208,1Hz,子弹剩余发射数,仅空中机器人、哨兵和ICRA机器人
typedef __packed struct 
{ 
	uint16_t bullet_remaining_num; 
} ext_bullet_remaining_t;

//0x0209,1Hz,机器人RFID状态数据
typedef __packed struct 
{ 
	uint32_t rfid_status;
} ext_rfid_status_t;

//0x020A,10Hz,飞镖机器人客户端指令数据
typedef __packed struct
{
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint8_t first_dart_speed;
	uint8_t second_dart_speed;
	uint8_t third_dart_speed;
	uint8_t fourth_dart_speed;
	uint16_t last_dart_launch_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

//0x0301,发送方机器人触发，机器人间交互数据
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

//
typedef __packed struct
{
	uint8_t operate_tpye;
	uint8_t layer;
} ext_client_custom_graphic_delete_t;

//图形数据
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data_struct_t;

//
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

//
typedef __packed struct
{ 
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

//
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

//
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//绘制字符
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;


//机器人总状态数据
typedef __packed struct
{
	ext_game_status_t game_status; //0x0001
	ext_game_result_t game_result; //0x0002
	ext_game_robot_HP_t game_robot_HP; //0x0003
	ext_dart_status_t dart_status; //0x0004
	ext_ICRA_buff_debuff_zone_status_t ICRA_buff_debuff_zone_status; //0x0005
	
	ext_event_data_t event_data; //0x0101
	ext_supply_projectile_action_t supply_projectile_action; //0x0102
	ext_referee_warning_t referee_warning; //0x0104
	ext_dart_remaining_time_t dart_remaining_time; //0x0105
	
	ext_game_robot_status_t game_robot_status; //0x0201
	ext_power_heat_data_t power_heat_data; //0x0202
	ext_game_robot_pos_t game_robot_pos; //0x0203
	ext_buff_t buff; //0x0204
	ext_aerial_robot_energy_t aerial_robot_energy; //0x0205
	ext_robot_hurt_t robot_hurt; //0x0206
	ext_shoot_data_t shoot_data; //0x0207
	ext_bullet_remaining_t bullet_remaining; //0x0208
	ext_rfid_status_t rfid_status; //0x0209
	ext_dart_client_cmd_t dart_client_cmd; //0x020A
	
} RobotStatus_t;

extern RobotStatus_t robot_status;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LEN][REFEREE_FIFO_BUF_NUM];

void PushToRefereeFIFOBuf(uint8_t *pdata, uint16_t size);
void ParseRefereeSystemData(void);


#endif
