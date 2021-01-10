#include "flash.h"

//获取flash的sector号
uint32_t flash_ger_sector(uint32_t address)
{
	if(address<ADDR_FLASH_SECTOR_1) {return FLASH_SECTOR_0;}
	else if(address<ADDR_FLASH_SECTOR_2){return FLASH_SECTOR_1;}
	else if(address<ADDR_FLASH_SECTOR_3) {return FLASH_SECTOR_2;}
	else if(address<ADDR_FLASH_SECTOR_4) {return FLASH_SECTOR_3;}
	else if(address<ADDR_FLASH_SECTOR_5) {return FLASH_SECTOR_4;}
	else if(address<ADDR_FLASH_SECTOR_6) {return FLASH_SECTOR_5;}
	else if(address<ADDR_FLASH_SECTOR_7) {return FLASH_SECTOR_6;}
	else if(address<ADDR_FLASH_SECTOR_8) {return FLASH_SECTOR_7;}
	else if(address<ADDR_FLASH_SECTOR_9) {return FLASH_SECTOR_8;}
	else if(address<ADDR_FLASH_SECTOR_10) {return FLASH_SECTOR_9;}
	else if(address<ADDR_FLASH_SECTOR_11) {return FLASH_SECTOR_10;}
	else {return FLASH_SECTOR_11;}
}

//擦除flash
void flash_erase_address(uint32_t address, uint16_t len)
{
	FLASH_EraseInitTypeDef flash_erase;
	uint32_t error;

	flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	flash_erase.Sector = flash_ger_sector(address);
	flash_erase.NbSectors = len;
	flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&flash_erase, &error);
	HAL_FLASH_Lock();
}

//往flash写数据
void flash_write_address(uint32_t address, uint32_t *buf, uint16_t len)
{
	uint16_t i;
	
  HAL_FLASH_Unlock();
	
	for(i=0;i<len;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+4*i, buf[i]);
	}
	
	HAL_FLASH_Lock();
}

//从flash读数据
void flash_read_address(uint32_t address, uint32_t *buf, uint16_t len)
{
	memcpy(buf, (void*)address, len *4);
}
