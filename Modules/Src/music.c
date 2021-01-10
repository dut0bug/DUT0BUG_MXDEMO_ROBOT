#include "music.h"
#include "tim.h"

//
uint8_t buzzer_state = PLAYING_STOP;

//����
//1 - do - 262Hz
//2 - re - 294Hz
//3 - mi - 330Hz
//4 - fa - 349Hz
//5 - so - 392Hz
//6 - la - 440Hz
//7 - si - 494Hz
//����
//1 - do - 523Hz
//2 - re - 587Hz
//3 - mi - 659Hz
//4 - fa - 698Hz
//5 - so - 784Hz
//6 - la - 880Hz
//7 - si - 988Hz
//����
//1 - do - 1046Hz
//2 - re - 1175Hz
//3 - mi - 1318Hz
//4 - fa - 1397Hz
//5 - so - 1568Hz
//6 - la - 1760Hz
//7 - si - 1976Hz

uint16_t music_steps[3][7] = {{262,294,330,349,392,440,494},
															{523,587,659,698,784,880,988},
															{1046,1175,1318,1397,1568,1760,1976}};

//С����
/*uint16_t song_littlestar[42] = {0x1411,0x1411,0x1415,0x1415,0x1416,0x1416,0x1815,
																0x1414,0x1414,0x1413,0x1413,0x1412,0x1412,0x1811,
																0x1415,0x1415,0x1414,0x1414,0x1413,0x1413,0x1812,
																0x1415,0x1415,0x1414,0x1414,0x1413,0x1413,0x1812,
																0x1411,0x1411,0x1415,0x1415,0x1416,0x1416,0x1815,
																0x1414,0x1414,0x1413,0x1413,0x1412,0x1412,0x1811};*/

//���տ���
/*uint16_t song_happybirthday[25] = {0x1205,0x1205,0x1406,0x1405,0x1411,0x1807,
																	 0x1205,0x1205,0x1406,0x1405,0x1412,0x1811,
																	 0x1205,0x1205,0x1415,0x1413,0x1411,0x0407,0x1406,
																	 0x1614,0x1214,0x1413,0x1411,0x1412,0x1811};*/

//������	
/*uint16_t song_eastred[41] = {0x1415,0x0215,0x1216,0x1812,0x1411,0x0211,0x1206,0x1812,
														0x1415,0x1415,0x0216,0x1221,0x1216,0x1215,0x1411,0x0211,0x1206,0x1812,
														0x1415,0x1412,0x1411,0x0207,0x1206,0x1405,0x1415,0x1412,0x1213,0x1212,0x1411,0x0211,0x1206,
														0x1212,0x1213,0x1212,0x1211,0x0212,0x1211,0x0207,0x1206,0x0405,0x1805};*/
															
//���״�ʦ������������															
/*uint16_t song_robomasteryou[148] = {0x1213,0x1213,0x1112,0x0113,0x1213,0x1212,0x1213,0x0215,
																		0x1215,0x1213,0x1213,0x1112,0x0113,0x1213,0x1212,0x1213,0x0215,
																		0x1215,0x0211,0x1411,0x8001,
																		0x1213,0x1213,0x1112,0x0113,0x1213,0x1212,0x1211,0x0212,
																		0x1212,0x1213,0x1213,0x1112,0x0113,0x1213,0x1212,0x1213,0x0215,
																		0x1215,0x1213,0x1213,0x1112,0x0113,0x1213,0x1212,0x1213,0x0221,
																		0x1221,0x0211,0x1411,0x8001,
																		0x1213,0x1213,0x1112,0x0113,0x1213,0x1212,0x1207,0x0106,0x1105,
																		0x0405,0x1105,0x1105,0x1105,0x1105,0x1415,0x1313,0x1112,
																		0x1212,0x0211,0x1211,0x1211,0x1211,0x1212,0x1213,0x0206,
																		0x1406,0x8001,0x1206,0x1213,0x1212,0x1211,0x0212,
																		0x1812,0x1413,0x1414,
																		0x0415,0x1215,0x1213,0x1215,0x1113,0x0115,0x1215,0x0217,
																		0x1217,0x0221,0x1221,0x1211,0x1212,0x1213,0x0215,
																		0x1215,0x0216,0x1216,0x1115,0x0116,0x1216,0x1115,0x0115,0x1115,0x0215,
																		0x1212,0x0212,0x1412,0x1413,0x1414,
																		0x0415,0x1215,0x1213,0x1215,0x1113,0x0115,0x1115,0x0217,
																		0x1217,0x0221,0x1421,0x1211,0x1212,0x1213,0x0217,
																		0x1217,0x0216,0x1416,0x1216,0x1115,0x0116,0x1116,0x0221,
																		0x1221,0x0222,0x1422,0x8001,0x1215,0x1221,0x1117,0x0121,0x1821};*/
																		
//���״�ʦ������������(����)
/*uint16_t song_robomasteryou2[66] = {0x1413,0x1414,0x0415,0x1215,0x1213,0x1215,0x1113,0x0115,0x1215,0x0217,
																		0x1217,0x0221,0x1421,0x1211,0x1212,0x1213,0x0215,
																		0x1215,0x0216,0x1216,0x1115,0x0116,0x1216,0x1115,0x0115,0x1215,0x0215,
																		0x1212,0x0212,0x1412,0x4001,0x1413,0x1414,
																		0x0415,0x1215,0x1213,0x1215,0x1113,0x0115,0x1215,0x0217,
																		0x1217,0x0221,0x1421,0x4001,0x1211,0x1212,0x1213,0x0217,
																		0x1217,0x0216,0x1416,0x1216,0x1115,0x0116,0x1216,0x0221,
																		0x1221,0x0222,0x1422,0x8001,0x1215,0x1221,0x1117,0x0121,0x1821};*/

//��֮��																		
uint16_t song_robomasterlickdog[13] = {0x1413,0x1414,0x0415,0x1215,0x1213,0x1215,0x1113,0x0115,0x1215,0x0217,0x1217,0x0221,0x1421};

//
uint8_t sound_warning[6] = {0x4B,0x40,0x4B,0x40,0x4B,0x40}; //B__B__B__
uint8_t sound_error[6] = {0x2B,0x10,0x2B,0x10,0x2B,0x10}; //B_B_B_

uint8_t sound_gyrocalibrating[6] = {0x2D,0x40,0x2D,0x40,0x2D,0x40}; //D__D__D__
uint8_t sound_autoaiming[6] = {0x2D,0x10,0x2D,0x10,0x2D,0x10}; //D_D_D_

//
static uint16_t bzply_n = 0;
static uint8_t bzply_count = 1;


//�رշ�����
void SetBuzzerOff(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}

//���÷�����Ƶ��
void SetBuzzerFrequence(uint16_t freq)
{
	//buzzer --> tim4.channel3
	//��Ƶ��Ϊ1000000Hz
	uint16_t period = 1000000/freq -1;
	
	__HAL_TIM_SET_AUTORELOAD(&htim4, period);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period/2);
}

//���÷�����״̬
void SetBuzzerState(uint8_t state)
{
	if(buzzer_state==PLAYING_INIT_MUSIC){return;}
	
	buzzer_state = state;
	if(state==PLAYING_STOP) {SetBuzzerOff();}
	
	bzply_n = 0;
	bzply_count = 1;
}

//��������
void PlayingSound(uint8_t *sound, uint16_t len)
{
	uint8_t delay, freq;
	
	delay = ((sound[bzply_n]&0xF0)>>4)*2;
	freq = (sound[bzply_n]&0x0F);
	
	if(bzply_count<delay)
	{
		bzply_count++;
		if(freq==0x00) {SetBuzzerOff();}
		else if(freq==0x0B) {SetBuzzerFrequence(640);}
		else if(freq==0x0D) {SetBuzzerFrequence(256);}
	}
	else
	{
		bzply_count=1;
		bzply_n++; if(bzply_n>=len){bzply_n=0;}
	}
}

//���Ÿ���
void PlayingSong(uint16_t *song, uint16_t len)
{
	uint8_t off_delay, on_delay, level, step;
	
	off_delay = ((song[bzply_n]&0xF000)>>12);
	on_delay = ((song[bzply_n]&0x0F00)>>8)*6;
	level = ((song[bzply_n]&0x00F0)>>4);
	step = (song[bzply_n]&0x000F)-1;
	
	if(bzply_count<on_delay)
	{
		bzply_count++;
		SetBuzzerFrequence(music_steps[level][step]);
	}
	else if(bzply_count<(on_delay+off_delay))
	{
		bzply_count++;
		SetBuzzerOff();
	}
	else
	{
		bzply_count=1;
		bzply_n++; if(bzply_n>=len){bzply_n=0;buzzer_state=PLAYING_STOP;SetBuzzerOff();}
	}
}
