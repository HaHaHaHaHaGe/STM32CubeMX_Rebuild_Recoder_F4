#include "stmflash.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//STM32内部FLASH读写 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

 
 
//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(u32*)faddr; 
}  

//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(u32 *pBuffer,u32 NumToWrite)	
{ 
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef recv;
	u32 serror;
	u32 WriteAddr = ADDR_FLASH_SECTOR_7;
	FLASH_EraseInitTypeDef f;
  f.TypeErase = TYPEERASE_SECTORS;
  f.VoltageRange = VOLTAGE_RANGE_3;
	f.Sector = FLASH_SECTOR_7;
	f.NbSectors = 1;
	
	while(HAL_FLASHEx_Erase(&f,&serror))
	{
		HAL_Delay(100);
	}

	while(NumToWrite--)
	{
		HAL_FLASH_Program(TYPEPROGRAM_WORD,WriteAddr,*pBuffer++);
		WriteAddr += 4;
	}
	

	HAL_FLASH_Lock();
} 

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	u32 ReadAddr = ADDR_FLASH_SECTOR_7;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}














