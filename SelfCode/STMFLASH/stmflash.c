#include "stmflash.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//STM32�ڲ�FLASH��д ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

 
 
//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(u32*)faddr; 
}  

//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
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

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	u32 ReadAddr = ADDR_FLASH_SECTOR_7;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}














