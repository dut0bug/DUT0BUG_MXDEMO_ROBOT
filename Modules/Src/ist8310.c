#include "ist8310.h"
#include "gpio.h"
#include "i2c.h"


float ist8310_mag[3];

void IST8310_init(void)
{
	uint8_t res;
	
	//∏¥Œª
	HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port,IST8310_RSTN_Pin,GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port,IST8310_RSTN_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	
	//∂¡»°ID
	res = I2C_Read_Single_Reg(IST8310_ADDRESS, 0x00); //device id
	if(res!=0x10){HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);}
	
	//≈‰÷√ºƒ¥Ê∆˜
	I2C_Write_Single_Reg(IST8310_ADDRESS, 0x0B, 0x08); //0000 1000 DRDY pin active low
	I2C_Write_Single_Reg(IST8310_ADDRESS, 0x41, 0x09); //0000 1001 average by 2 times 
	I2C_Write_Single_Reg(IST8310_ADDRESS, 0x42, 0xC0); //1100 0000 normal pulse duration
	I2C_Write_Single_Reg(IST8310_ADDRESS, 0x0A, 0x0B); //0000 1011 continuous measurement mode 6ms
	
	HAL_Delay(10);
}

void IST8310_read_mag(void)
{
	uint8_t buf[6];
	int16_t tmp;
	
	//
	I2C_Read_Mulit_Reg(IST8310_ADDRESS, 0x03, buf, 6); //XYZ axis measure data
	
	tmp=(int16_t)((buf[1]<<8)|buf[0]); ist8310_mag[0]=tmp*0.3f; //uT
	tmp=(int16_t)((buf[3]<<8)|buf[2]); ist8310_mag[1]=tmp*0.3f;
	tmp=(int16_t)((buf[5]<<8)|buf[4]); ist8310_mag[2]=tmp*0.3f;
}
