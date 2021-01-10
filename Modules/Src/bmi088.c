#include "bmi088.h"
#include "spi.h"
#include "flash.h"

#define GYRO_FLASH_ADDRESS  ADDR_FLASH_SECTOR_10

uint8_t gyro_zero_calib_flag=0;

float bmi088_gyro[3], bmi088_gyro_deg[3], bmi088_accel[3], bmi088_temperature;
float bmi088_gyro_zero[3], bmi088_gyro_deg_zero[3], bmi088_gyro_calib[3], bmi088_gyro_deg_calib[3];

//
uint8_t BMI088_read_write_byte(uint8_t tx_data)
{
	uint8_t rx_data;
	HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 1000);
	return rx_data;
}

//
void BMI088_ACCEL_NS_L(void)
{
	HAL_GPIO_WritePin(BMI088_ACCEL_CS1_GPIO_Port, BMI088_ACCEL_CS1_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
	HAL_GPIO_WritePin(BMI088_ACCEL_CS1_GPIO_Port, BMI088_ACCEL_CS1_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
	HAL_GPIO_WritePin(BMI088_GYRO_CS1_GPIO_Port, BMI088_GYRO_CS1_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
	HAL_GPIO_WritePin(BMI088_GYRO_CS1_GPIO_Port, BMI088_GYRO_CS1_Pin, GPIO_PIN_SET);
}

//
void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data)
{                                            
	BMI088_ACCEL_NS_L();                     
	BMI088_read_write_byte(reg);
	BMI088_read_write_byte(data);
	BMI088_ACCEL_NS_H();                     
}

uint8_t BMI088_accel_read_single_reg(uint8_t reg) 
{ 
	uint8_t tmp;
	
	BMI088_ACCEL_NS_L();                    
	BMI088_read_write_byte(reg|0x80);   
	BMI088_read_write_byte(0x55);           
	tmp=BMI088_read_write_byte(0x55);  
	BMI088_ACCEL_NS_H();
	
	return tmp;
}

void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) 
{                                              
	BMI088_ACCEL_NS_L();                       
	BMI088_read_write_byte(reg|0x80);
  BMI088_read_write_byte(0x55); 	
	while(len!=0)
	{
		*buf=BMI088_read_write_byte(0x55);
		buf++;
		len--;
	}     
	BMI088_ACCEL_NS_H();                       
}

//
void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data) 
{                                           
	BMI088_GYRO_NS_L();                     
	BMI088_read_write_byte(reg);
	BMI088_read_write_byte(data);
	BMI088_GYRO_NS_H();                     
}

uint8_t BMI088_gyro_read_single_reg(uint8_t reg)  
{                                           
	uint8_t tmp;
	
	BMI088_GYRO_NS_L();                     
	BMI088_read_write_byte(reg|0x80);
	tmp=BMI088_read_write_byte(0x55);
	BMI088_GYRO_NS_H();
	
	return tmp;
}

void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len)   
{                                               
	BMI088_GYRO_NS_L();                        
	BMI088_read_write_byte(reg|0x80);
	while(len!=0)
	{
		*buf=BMI088_read_write_byte(0x55);
		buf++;
		len--;
	}
	BMI088_GYRO_NS_H();                         
}

//
void BMI088_init(void)
{
	//uint8_t res;
	
	//
	//res=BMI088_accel_read_single_reg(0x00); //chip id  0x1E
	BMI088_accel_write_single_reg(0x7E, 0xB6); //soft_rest
	HAL_Delay(100);
	
	BMI088_accel_write_single_reg(0x40, 0xAB); //ACC_CONF 1010 1010 800Hz,230/200Hz
	BMI088_accel_write_single_reg(0x41, 0x00); //ACC_RANGE 00-3g;01-6g;02-12g;03-24g
	BMI088_accel_write_single_reg(0x53, 0x08); //INT1_IO_CONF 0000 1000 INT1 as output pp acitiv_low
	BMI088_accel_write_single_reg(0x58, 0x04); //INT_MAP_DATA 0000 0100 INT1 drdy interrupt
	BMI088_accel_write_single_reg(0x7C ,0x00); HAL_Delay(10); //power config-acitve mode
	BMI088_accel_write_single_reg(0x7D, 0x04); HAL_Delay(10); //power control-acc enable
	
	//
	//res=BMI088_gyro_read_single_reg(0x00); //chip id  0x0F
	BMI088_gyro_write_single_reg(0x14, 0xB6); //soft_rest
	HAL_Delay(100);
	
	BMI088_gyro_write_single_reg(0x0F, 0x00); //GYRO_RANGE 00-2000бу/s;01-1000бу/s;02-500бу/s;03-250бу/s;04-125бу/s
	BMI088_gyro_write_single_reg(0x10 ,0x82); //GYRO_ODR,BANDWIDTH 00-2000Hz,532Hz;01-2000Hz,230Hz;02-1000Hz,116Hz;03-400Hz,47Hz;04-200Hz,23Hz;05-100Hz,12Hz;06-200Hz,64Hz;07-100Hz,32Hz
	BMI088_gyro_write_single_reg(0x11, 0x00); //GYRO_LPM1 normal mode
	BMI088_gyro_write_single_reg(0x15, 0x80); //GYRO_INT_CTRL 1000 0000 enable drdy
	BMI088_gyro_write_single_reg(0x16, 0x00); //INT_IO_CONF 0000 0000 pp active_low
	BMI088_gyro_write_single_reg(0x18, 0x01); //INT_IO_MAP 0000 0001 drdy to INT3
	
	//read flash
	flash_read_address(GYRO_FLASH_ADDRESS+0, (uint32_t *)bmi088_gyro_zero, 3);
	flash_read_address(GYRO_FLASH_ADDRESS+12, (uint32_t *)bmi088_gyro_deg_zero, 3);
}

//
void BMI088_calib_gyro_zero(float gyro[3], float gyro_deg[3])
{
	uint16_t n;
	float sum[3]={0.0f,0.0f,0.0f};
	float sum_deg[3]={0.0f,0.0f,0.0f};
	
	for(n=0;n<1000;n++)
	{
		sum[0] += gyro[0];
		sum[1] += gyro[1];
		sum[2] += gyro[2];
		
		sum_deg[0] += gyro_deg[0];
		sum_deg[1] += gyro_deg[1];
		sum_deg[2] += gyro_deg[2];
		
		HAL_Delay(10);
	}
	
	bmi088_gyro_zero[0] = sum[0]/1000.0f;
	bmi088_gyro_zero[1] = sum[1]/1000.0f;
	bmi088_gyro_zero[2] = sum[2]/1000.0f;
	
	bmi088_gyro_deg_zero[0] = sum_deg[0]/1000.0f;
	bmi088_gyro_deg_zero[1] = sum_deg[1]/1000.0f;
	bmi088_gyro_deg_zero[2] = sum_deg[2]/1000.0f;
	
	//write flash
	flash_erase_address(GYRO_FLASH_ADDRESS, 1);
	flash_write_address(GYRO_FLASH_ADDRESS+0, (uint32_t *)bmi088_gyro_zero, 3);
	flash_write_address(GYRO_FLASH_ADDRESS+12, (uint32_t *)bmi088_gyro_deg_zero, 3);
}

//
void BMI088_read_gyro(void)
{
	uint8_t buf[6];
	int16_t bmi088_raw_tmp;

	//
	BMI088_gyro_read_muli_reg(0x02, buf, 6);

	bmi088_raw_tmp = (int16_t)((buf[1]<<8)|buf[0]);
	bmi088_gyro[0] = bmi088_raw_tmp * 0.00106526443603169529841533860381f; //((tmp/32768)*(2000*pi/180))rad/s
	bmi088_gyro_deg[0] = bmi088_raw_tmp * 0.06103515625f; //((tmp/32768)*2000)deg/s
	
	bmi088_raw_tmp = (int16_t)((buf[3]<<8)|buf[2]);
	bmi088_gyro[1] = bmi088_raw_tmp * 0.00106526443603169529841533860381f;
	bmi088_gyro_deg[1] = bmi088_raw_tmp * 0.06103515625f;
	
	bmi088_raw_tmp = (int16_t)((buf[5]<<8)|buf[4]);
	bmi088_gyro[2] = bmi088_raw_tmp * 0.00106526443603169529841533860381f;
	bmi088_gyro_deg[2] = bmi088_raw_tmp * 0.06103515625f;
	
	//
	bmi088_gyro_calib[0] = bmi088_gyro[0]-bmi088_gyro_zero[0];
	bmi088_gyro_calib[1] = bmi088_gyro[1]-bmi088_gyro_zero[1];
	bmi088_gyro_calib[2] = bmi088_gyro[2]-bmi088_gyro_zero[2];
	
	bmi088_gyro_deg_calib[0] = bmi088_gyro_deg[0]-bmi088_gyro_deg_zero[0];
	bmi088_gyro_deg_calib[1] = bmi088_gyro_deg[1]-bmi088_gyro_deg_zero[1];
	bmi088_gyro_deg_calib[2] = bmi088_gyro_deg[2]-bmi088_gyro_deg_zero[2];
}

//
void BMI088_read_accel(void)
{
	uint8_t buf[6];
	int16_t bmi088_raw_tmp;
	
	//
	BMI088_accel_read_muli_reg(0x12, buf, 6);

	bmi088_raw_tmp = (int16_t)((buf[1]<<8)|buf[0]);
	bmi088_accel[0] = bmi088_raw_tmp * 0.00089782562255859375f; //(tmp/32768)*3*9.80665 m/s2
	bmi088_raw_tmp = (int16_t)((buf[3]<<8)|buf[2]);
	bmi088_accel[1] = bmi088_raw_tmp * 0.00089782562255859375f;
	bmi088_raw_tmp = (int16_t)((buf[5]<<8)|buf[4]);
	bmi088_accel[2] = bmi088_raw_tmp * 0.00089782562255859375f;
}

//
void BMI088_read_temperature(void)
{
	uint8_t buf[2];
	int16_t bmi088_raw_tmp;

	//
	BMI088_accel_read_muli_reg(0x22, buf, 2);

	bmi088_raw_tmp = (int16_t)((buf[0]<<3)|(buf[1]>>5));
	if(bmi088_raw_tmp > 1023) {bmi088_raw_tmp -= 2048;}
	bmi088_temperature = bmi088_raw_tmp * 0.125f + 23.0f;
}
