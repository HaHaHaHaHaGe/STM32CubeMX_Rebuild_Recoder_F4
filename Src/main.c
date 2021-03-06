/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_write_wav.h"
#include "../inc/ringbuffer.h"
#include "malloc.h"	   
#include <speex/speex.h>
#include "oled.h"
#include "stmflash.h"
#include "24l01.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "wav.h"
#include "gt20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_Blink  0x01
#define COMMAND_Begin 0x02
#define COMMAND_End 0x04
#define COMMAND_Init 0x08
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct FLASH_SAVE
{
	unsigned char WIFI_NAME[20];
	unsigned char WIFI_PASS[20];
	unsigned char SERVER_IP[20];
	unsigned char SERVER_PORT[20];
	unsigned char BIND_NAME[20];
	unsigned char DEVICE_ID[28];
	unsigned char GROUP;
	int check;
}FLASH_SAVE;


FLASH_SAVE FLASH_DATA;
uint8_t UART_BUFFER[1024];
u8 String_Windows_Time[64] = {0};
uint8_t isPowerOn = 0;
unsigned char Real_Time_Year=0,Real_Time_Month=0,Real_Time_Day=0,Real_Time_Hour=0,Real_Time_Minute=0,Real_Time_Second=0;
unsigned int Real_Time_Millise=0;
unsigned char Real_Time_Year_STOP=0,Real_Time_Month_STOP=0,Real_Time_Day_STOP=0,Real_Time_Hour_STOP=0,Real_Time_Minute_STOP=0,Real_Time_Second_STOP=0;
unsigned int Real_Time_Millise_STOP=0;
char SoftID;
char SessID[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char BordID[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
char recv_wifi[20] = {0};
char recv_pass[20] = {0};
char recv_ip[20] = {0};
char recv_port[20] = {0};
unsigned char Binding_Bool = 1;
char *recv_A = "WIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";
char *recv_B = "CONNECT\r\n";
char *recv_C = "ALREADY CONNECTED";
//u8 initfilename[20] = {0};

int RecvComLoc1 = 0;
int RecvComLoc2 = 0;
int RecvComLoc3 = 0;
int RecvComLoc4 = 0;
u8 Buffer[64];
extern unsigned char AM_Factor;
extern ringbuffer buffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
DIR dir;
FILINFO inf;
FRESULT sult;
HAL_SD_CardInfoTypeDef sdinf;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static void USART2_UART_Init(int baud)
{
	HAL_UART_DeInit(&huart2);
  huart2.Instance = USART2;
  huart2.Init.BaudRate = baud;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

int fputc(int ch, FILE *f)
{
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}

void UARTSendData(u8 *data,u16 len)
{
	u16 l = 0;
	for(l = 0;l<len;l++)
	{
		while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
		USART2->DR = data[l];
	}
}


u8 PressKey()
{
	u8 count = 0;
	while(HAL_GPIO_ReadPin(KEY_FLAG_GPIO_Port,KEY_FLAG_Pin));
	while(!HAL_GPIO_ReadPin(KEY_FLAG_GPIO_Port,KEY_FLAG_Pin))
	{
		HAL_Delay(100);
		count++;
	}
	if(count > 10)
		return 1;
	return 0;
}

int StrEqual(unsigned char *target,unsigned char *str,int targetlen, int strlen)
{
	unsigned char *p = target;
	int i = 0;
	int j = 0;
	for(i = 0;i<targetlen;i++)
	{
		if(p[i] == str[j])
			j++;
		else
			j=0;
		if(j==strlen)
			return i;
	}
	return -1;
}

void ClearBuffer(unsigned char *target,int targetlen)
{
	int i = 0;
	for(i = 0;i<targetlen;i++)
	target[i] = 0;
}


u8 hex_to_char( u8 ch)
{
	u8 value = 0;
	if(ch >= 0 && ch <= 9)
	{
		value = ch + 0x30;
	}
	else if(ch >=10 && ch <=15)
	{
	// ????
		value = ch + 0x37;
	}
	 
	//printf("%s hex : 0x%02x\n", __func__, value);
	return value;
}

void check_firstrun()
{

	uint32_t rand_i = 0;
	STMFLASH_Read((u32*)&FLASH_DATA,sizeof(FLASH_SAVE) / 4);
	
	if(FLASH_DATA.check != 0x12345678)
	{
		OLED_Init( );	 
		OLED_Clear( );
		OLED_ShowString(0,0,(unsigned char*)"Initialization..",8);
		OLED_ShowString(0,2,(unsigned char*)"Do Not Turn off",8);
		
		srand(HAL_GetUIDw0());
		rand_i += rand();
		srand(HAL_GetUIDw1());
		rand_i += rand();
		srand(HAL_GetUIDw2());
		rand_i += rand();
		srand(rand_i);
		
		FLASH_DATA.check = 0x12345678;
		FLASH_DATA.DEVICE_ID[0] = hex_to_char((u8)(HAL_GetUIDw0() & 0x0000000f));
		FLASH_DATA.DEVICE_ID[1] = hex_to_char((u8)((HAL_GetUIDw0() >> 4) & 0x0000000f));
		FLASH_DATA.DEVICE_ID[2] = hex_to_char((u8)((HAL_GetUIDw0() >> 16) & 0x0000000f));
		FLASH_DATA.DEVICE_ID[3] = hex_to_char((u8)((HAL_GetUIDw0() >> 20) & 0x0000000f));
		FLASH_DATA.DEVICE_ID[4] = hex_to_char((u8)(HAL_GetUIDw1() & 0x0000000f));
		FLASH_DATA.DEVICE_ID[5] = hex_to_char((u8)((HAL_GetUIDw1() >> 4) & 0x0000000f));
		FLASH_DATA.DEVICE_ID[6] = (HAL_GetUIDw1() >> 16) & 0x000000ff;
		FLASH_DATA.DEVICE_ID[7] = (HAL_GetUIDw1() >> 24) & 0x000000ff;
		FLASH_DATA.DEVICE_ID[8] = (HAL_GetUIDw2()) & 0x000000ff;
		FLASH_DATA.DEVICE_ID[9] = (HAL_GetUIDw2() >> 8) & 0x000000ff;
		FLASH_DATA.DEVICE_ID[10] = (HAL_GetUIDw2() >> 16) & 0x000000ff;
		FLASH_DATA.DEVICE_ID[11] = (HAL_GetUIDw2() >> 24) & 0x000000ff;
		FLASH_DATA.DEVICE_ID[12] = 0;
		FLASH_DATA.GROUP = 'A';
		FLASH_DATA.BIND_NAME[0] = 0;
		
		HAL_Delay(200);
		STMFLASH_Write((u32*)&FLASH_DATA,sizeof(FLASH_SAVE) / 4);
		
		
		USART2_UART_Init(115200);
		HAL_UART_Receive_IT(&huart2,UART_BUFFER,sizeof(UART_BUFFER));
		HAL_Delay(2000);	
		HAL_UART_Transmit(&huart2,(unsigned char*)"AT+CWMODE=1\r\n",13,100);
		HAL_Delay(2000);	
		HAL_UART_Transmit(&huart2,(unsigned char*)"AT+UART=921600,8,1,0,0\r\n",24,100);
		OLED_Clear( );
		initial_buffer(&buffer,YES,20*1024);
		HAL_TIM_Base_Start(&htim3);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)Buffer,32);
		
		int j = 0;
		while(1)
		{
			j = 0;
			for(char i = 0;i<32;i++)
				j += abs(((short*)Buffer)[i] - 2048);
			j/=32;
			sprintf(String_Windows_Time,"%d",j);
			
			OLED_Clear( );
			OLED_ShowString(0,0,(unsigned char*)String_Windows_Time,8);
			HAL_Delay(50);	
		}
		//printf("AT+UART=921600,8,1,0,0\r\n");
//		HAL_Delay(2000);	
//		HAL_GPIO_WritePin(PWR_CTL_GPIO_Port,PWR_CTL_Pin,0);
//		OLED_Clear( );
//		OLED_ShowString(0,0,(unsigned char*)"Please turn off",8);
//		while(1);
	}
}

extern FIL speex_file;
extern u32 speexdata_len;
extern UINT br;
u32 PackegStart = 0;
u32 PackegEnd = 0;				
u8 ErrorRecData[2048];

int wifi_link_check_int = 0;
void wifi_link_check()
{
	int start = StrEqual(UART_BUFFER,"link is not valid",sizeof(UART_BUFFER),strlen("link is not valid"));
	if(start != -1 || wifi_link_check_int != 0)
	{
		
		if(wifi_link_check_int % 3000 == 0)
		{
			printf("AT+CIPSTART=\"TCP\",\"%s\",%s\r\n",FLASH_DATA.SERVER_IP,FLASH_DATA.SERVER_PORT);
			ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
		}
		if(StrEqual(UART_BUFFER,(unsigned char*)recv_B,sizeof(UART_BUFFER),strlen(recv_B)) != -1)
		{
			ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
			wifi_link_check_int = 0;
			return;
		}
		
		wifi_link_check_int++;
		
	}
	else
		wifi_link_check_int = 0;
}

unsigned char wifi_link_server()
{
		
		if(wifi_link_check_int % 3000 == 0)
		{
			printf("AT+CIPSTART=\"TCP\",\"%s\",%s\r\n",FLASH_DATA.SERVER_IP,FLASH_DATA.SERVER_PORT);
			ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
		}
		if(StrEqual(UART_BUFFER,(unsigned char*)recv_C,sizeof(UART_BUFFER),strlen(recv_C)) != -1)
		{
			ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
			wifi_link_check_int = 0;
			return 1;
		}
		if(StrEqual(UART_BUFFER,(unsigned char*)recv_B,sizeof(UART_BUFFER),strlen(recv_B)) != -1)
		{
			ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
			wifi_link_check_int = 0;
			return 1;
		}
		
		wifi_link_check_int++;
		return 0;
}
	//录音 I2S_DMA接收中断服务函数.在中断里面写入数据
int SendspeexData_and_FixWifiData(unsigned char* data,unsigned int writesize,unsigned int begin_position) 
{    
	u16 bw;
	u8 res;
	//u32 writesize;
	u32 i = 0;
	u32 datasum = 0;
	int start;
	u32 ErrorS,ErrorE,start_len,start_sum;
	//if(rec_sta==0X80)//录音模式
	{  
		
		start = StrEqual(UART_BUFFER,"Fix Data Request\r\n",sizeof(UART_BUFFER),strlen("Fix Data Request\r\n"));
		if(start != -1 && wifi_link_check_int == 0)
		{
			start_len = UART_BUFFER[start + 1] << 24;
			start_len |= UART_BUFFER[start + 2] << 16;
			start_len |= UART_BUFFER[start + 3] << 8;
			start_len |= UART_BUFFER[start + 4];
			
			start_sum = UART_BUFFER[start + 5] << 24;
			start_sum |= UART_BUFFER[start + 6] << 16;
			start_sum |= UART_BUFFER[start + 7] << 8;
			start_sum |= UART_BUFFER[start + 8];
			
			datasum = 0;
			start_len &= 0x0000ffff;
			for(i = 0; i < (u8)start_len + 12; i++)
				datasum += UART_BUFFER[start + 9 + i];
			for(i = 0; i < strlen("Fix Data Request\r\n"); i++)
				datasum += "Fix Data Request\r\n"[i];
			
			if(datasum == start_sum)
			{
				ErrorS = UART_BUFFER[start + 21] << 24;
				ErrorS |= UART_BUFFER[start + 22] << 16;
				ErrorS |= UART_BUFFER[start + 23] << 8;
				ErrorS |= UART_BUFFER[start + 24];
				
				ErrorE = UART_BUFFER[start + 25] << 24;
				ErrorE |= UART_BUFFER[start + 26] << 16;
				ErrorE |= UART_BUFFER[start + 27] << 8;
				ErrorE |= UART_BUFFER[start + 28];
				
				if(ErrorE > ErrorS && ErrorE < speexdata_len && ErrorS < speexdata_len && ErrorE > 0 && (ErrorS > 0 || ErrorS == 0))
				{
					f_lseek(&speex_file,ErrorS);
					if((ErrorE - ErrorS) > 1000)
					{
						f_read(&speex_file,ErrorRecData,1000,&br);
						i = 1000;
						PackegEnd = ErrorS + 1000;
					}
					else
					{
						f_read(&speex_file,ErrorRecData,(ErrorE - ErrorS),&br);
						i = (ErrorE - ErrorS);
						PackegEnd = ErrorE;
					}
					
					
					
					PackegStart = ErrorS;
					
					ErrorS = i;
					
					
					PackegEnd = PackegEnd - PackegStart + 12;
					datasum = 0;
					while(i--)
						datasum+=ErrorRecData[i];
					for(i = 0;i<12;i++)
						datasum+=BordID[i];
					for(i = 0;i<strlen("Recoder Repair Data\r\n");i++)
						datasum+="Recoder Repair Data\r\n"[i];
					datasum += ((u8*)&PackegStart)[3];
					datasum += ((u8*)&PackegStart)[2];
					datasum += ((u8*)&PackegStart)[1];
					datasum += ((u8*)&PackegStart)[0];
					datasum += Real_Time_Year_STOP + Real_Time_Month_STOP + Real_Time_Day_STOP + Real_Time_Hour_STOP + Real_Time_Minute_STOP + Real_Time_Second_STOP;
					datasum += ((u8*)&Real_Time_Millise_STOP)[1];
					datasum += ((u8*)&Real_Time_Millise_STOP)[0];
					
					printf("AT+CIPSEND=%d\r\n",53 + ErrorS);
					HAL_Delay(10);
					printf("Recoder Repair Data\r\n");
					UARTSendData(&((u8*)&PackegEnd)[3],1);
					UARTSendData(&((u8*)&PackegEnd)[2],1);
					UARTSendData(&((u8*)&PackegEnd)[1],1);
					UARTSendData(&((u8*)&PackegEnd)[0],1);
					
					UARTSendData(&((u8*)&datasum)[3],1);
					UARTSendData(&((u8*)&datasum)[2],1);
					UARTSendData(&((u8*)&datasum)[1],1);
					UARTSendData(&((u8*)&datasum)[0],1);
					
					UARTSendData(BordID,12);
					 
					
					UARTSendData(&((u8*)&PackegStart)[3],1);
					UARTSendData(&((u8*)&PackegStart)[2],1);
					UARTSendData(&((u8*)&PackegStart)[1],1);
					UARTSendData(&((u8*)&PackegStart)[0],1);
					
					

					
					UARTSendData(&Real_Time_Year_STOP,1);
					UARTSendData(&Real_Time_Month_STOP,1);
					UARTSendData(&Real_Time_Day_STOP,1);
					UARTSendData(&Real_Time_Hour_STOP,1);
					UARTSendData(&Real_Time_Minute_STOP,1);
					UARTSendData(&Real_Time_Second_STOP,1);
					UARTSendData(&((u8*)&Real_Time_Millise_STOP)[1],1);
					UARTSendData(&((u8*)&Real_Time_Millise_STOP)[0],1);
					//
//					for(i = 0;i<1024;i++)
//						ErrorRecData[i] = 11;
//					i = 1024;
//					ErrorS = i;
//					while(i--)
//						datasum+=ErrorRecData[i];
					//
					

					UARTSendData(ErrorRecData,ErrorS);

					
					datasum = 0;
					ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
					f_lseek(&speex_file,speexdata_len);
					
					i = 0;
					while(StrEqual(UART_BUFFER,"SEND OK",sizeof(UART_BUFFER),strlen("SEND OK")) == -1 && i < 25)
					{
						HAL_Delay(5);
						i++;
					}
					
					return 0;
				}
			}
		}
		
		
		
		
		if(wifi_link_check_int == 0 && writesize > 0)
		{
			//writesize = secondcache_write_loc - secondcache_read_loc;
			//res=f_write(f_rec,&secondcache[secondcache_read_loc],writesize,(UINT*)&bw);//写入文件
			
			i = writesize;
			datasum = 0;
			PackegStart = begin_position;
			PackegEnd = writesize + 12;
			while(i--)
				datasum+=data[i];
			for(i = 0;i<12;i++)
				datasum+=BordID[i];
			for(i = 0;i<strlen("Recoder Source Data\r\n");i++)
				datasum+="Recoder Source Data\r\n"[i];
			datasum += ((u8*)&PackegStart)[3];
			datasum += ((u8*)&PackegStart)[2];
			datasum += ((u8*)&PackegStart)[1];
			datasum += ((u8*)&PackegStart)[0];
			datasum += Real_Time_Year_STOP + Real_Time_Month_STOP + Real_Time_Day_STOP + Real_Time_Hour_STOP + Real_Time_Minute_STOP + Real_Time_Second_STOP;
			datasum += ((u8*)&Real_Time_Millise_STOP)[1];
			datasum += ((u8*)&Real_Time_Millise_STOP)[0];
			
			printf("AT+CIPSEND=%d\r\n",53 + writesize);
					HAL_Delay(10);
			printf("Recoder Source Data\r\n");
			UARTSendData(&((u8*)&PackegEnd)[3],1);
			UARTSendData(&((u8*)&PackegEnd)[2],1);
			UARTSendData(&((u8*)&PackegEnd)[1],1);
			UARTSendData(&((u8*)&PackegEnd)[0],1);
			
			UARTSendData(&((u8*)&datasum)[3],1);
			UARTSendData(&((u8*)&datasum)[2],1);
			UARTSendData(&((u8*)&datasum)[1],1);
			UARTSendData(&((u8*)&datasum)[0],1);
			
			UARTSendData(BordID,12);
			
			UARTSendData(&((u8*)&PackegStart)[3],1);
			UARTSendData(&((u8*)&PackegStart)[2],1);
			UARTSendData(&((u8*)&PackegStart)[1],1);
			UARTSendData(&((u8*)&PackegStart)[0],1);
			
			
					UARTSendData(&Real_Time_Year_STOP,1);
					UARTSendData(&Real_Time_Month_STOP,1);
					UARTSendData(&Real_Time_Day_STOP,1);
					UARTSendData(&Real_Time_Hour_STOP,1);
					UARTSendData(&Real_Time_Minute_STOP,1);
					UARTSendData(&Real_Time_Second_STOP,1);
					UARTSendData(&((u8*)&Real_Time_Millise_STOP)[1],1);
					UARTSendData(&((u8*)&Real_Time_Millise_STOP)[0],1);
			
			UARTSendData(data,writesize);
			
			ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
			i = 0;
			while(StrEqual(UART_BUFFER,"SEND OK",sizeof(UART_BUFFER),strlen("SEND OK")) == -1 && i < 25)
			{
				HAL_Delay(5);
				i++;
			}
			return 1;
		}

	} 
	return 2;

}				

void SendDeviceIDLE()
{
	int i,sendlen;
			printf("AT+CIPSEND=%d\r\n",37);
					HAL_Delay(10);
			printf("Device is Idle\r\n");
			sendlen = 1;
			UARTSendData(&((u8*)&sendlen)[3],1);
			UARTSendData(&((u8*)&sendlen)[2],1);
			UARTSendData(&((u8*)&sendlen)[1],1);
			UARTSendData(&((u8*)&sendlen)[0],1);
			sendlen = 0;
			for(i = 0;i<strlen("Device is Idle\r\n");i++)
				sendlen+="Device is Idle\r\n"[i];
			for(i = 0;i<12;i++)
				sendlen+=BordID[i];
			sendlen+=5;
			UARTSendData(&((u8*)&sendlen)[3],1);
			UARTSendData(&((u8*)&sendlen)[2],1);
			UARTSendData(&((u8*)&sendlen)[1],1);
			UARTSendData(&((u8*)&sendlen)[0],1);
			UARTSendData(BordID,12);
			sendlen = 5;
			UARTSendData(&((u8*)&sendlen)[0],1);
			
			i = 0;
					while(StrEqual(UART_BUFFER,"SEND OK",sizeof(UART_BUFFER),strlen("SEND OK")) == -1 && i < 80)
					{
						HAL_Delay(5);
						i++;
					}
}





void DataCheck()
{
	unsigned char delay_loop = 0;
	int count = 0;
	u32 i = 0;
	u32 datasum = 0;
	int start,end;
	u32 ErrorS,ErrorE,RecvComLoc3,start_len,start_sum;
	ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
	HAL_GPIO_WritePin(GPIOC,LED1_Pin,0);
	HAL_GPIO_WritePin(GPIOC,LED2_Pin,1);
	do
	{
		delay_loop++;
		count++;
		wifi_link_check();
		
		if(delay_loop == 15 && wifi_link_check_int == 0)
		{
			delay_loop = 0;
			printf("AT+CIPSEND=%d\r\n",37);
					HAL_Delay(10);
			printf("Device is Idle\r\n");
			RecvComLoc3 = 1;
			UARTSendData(&((u8*)&RecvComLoc3)[3],1);
			UARTSendData(&((u8*)&RecvComLoc3)[2],1);
			UARTSendData(&((u8*)&RecvComLoc3)[1],1);
			UARTSendData(&((u8*)&RecvComLoc3)[0],1);
			RecvComLoc3 = 0;
			for(i = 0;i<strlen("Device is Idle\r\n");i++)
				RecvComLoc3+="Device is Idle\r\n"[i];
			for(i = 0;i<12;i++)
				RecvComLoc3+=BordID[i];
			RecvComLoc3+=5;
			UARTSendData(&((u8*)&RecvComLoc3)[3],1);
			UARTSendData(&((u8*)&RecvComLoc3)[2],1);
			UARTSendData(&((u8*)&RecvComLoc3)[1],1);
			UARTSendData(&((u8*)&RecvComLoc3)[0],1);
			UARTSendData(BordID,12);
			RecvComLoc3 = 5;
			UARTSendData(&((u8*)&RecvComLoc3)[0],1);
		}
		
		
		
		
		
		
		
		
		HAL_GPIO_TogglePin(GPIOC,LED1_Pin);
		HAL_GPIO_TogglePin(GPIOC,LED2_Pin);
		HAL_Delay(50);
		
		
		
		
		
		start = StrEqual(UART_BUFFER,"Fix Data Request\r\n",sizeof(UART_BUFFER),strlen("Fix Data Request\r\n"));
		if(start != -1 && wifi_link_check_int == 0)
		{
			start_len = UART_BUFFER[start + 1] << 24;
			start_len |= UART_BUFFER[start + 2] << 16;
			start_len |= UART_BUFFER[start + 3] << 8;
			start_len |= UART_BUFFER[start + 4];
			
			start_sum = UART_BUFFER[start + 5] << 24;
			start_sum |= UART_BUFFER[start + 6] << 16;
			start_sum |= UART_BUFFER[start + 7] << 8;
			start_sum |= UART_BUFFER[start + 8];
			
			datasum = 0;
			start_len &= 0x0000ffff;
			for(i = 0; i < (u8)start_len + 12; i++)
				datasum += UART_BUFFER[start + 9 + i];
			for(i = 0; i < strlen("Fix Data Request\r\n"); i++)
				datasum += "Fix Data Request\r\n"[i];
			
			if(datasum == start_sum)
			{
				delay_loop = 14;
				ErrorS = UART_BUFFER[start + 21] << 24;
				ErrorS |= UART_BUFFER[start + 22] << 16;
				ErrorS |= UART_BUFFER[start + 23] << 8;
				ErrorS |= UART_BUFFER[start + 24];
				
				ErrorE = UART_BUFFER[start + 25] << 24;
				ErrorE |= UART_BUFFER[start + 26] << 16;
				ErrorE |= UART_BUFFER[start + 27] << 8;
				ErrorE |= UART_BUFFER[start + 28];
				if(ErrorE > ErrorS && ErrorE < speexdata_len && ErrorS < speexdata_len && ErrorE > 0 && (ErrorS > 0 || ErrorS == 0))
				{
					f_lseek(&speex_file,ErrorS);
					if((ErrorE - ErrorS) > 1020)
					{
						f_read(&speex_file,ErrorRecData,1020,&br);
						i = 1020;
						PackegEnd = ErrorS + 1020;
					}
					else
					{
						f_read(&speex_file,ErrorRecData,(ErrorE - ErrorS),&br);
						i = (ErrorE - ErrorS);
						PackegEnd = ErrorE;
					}
					
					
					count = 0;
					PackegStart = ErrorS;
					
					ErrorS = i;
					PackegEnd = ErrorS + 12;
					datasum = 0;
					while(i--)
						datasum+=ErrorRecData[i];
					
					
					
					for(i = 0;i<12;i++)
						datasum+=BordID[i];
					for(i = 0;i<strlen("Recoder Repair Data\r\n");i++)
						datasum+="Recoder Repair Data\r\n"[i];
					datasum += ((u8*)&PackegStart)[3];
					datasum += ((u8*)&PackegStart)[2];
					datasum += ((u8*)&PackegStart)[1];
					datasum += ((u8*)&PackegStart)[0];
					datasum += Real_Time_Year_STOP + Real_Time_Month_STOP + Real_Time_Day_STOP + Real_Time_Hour_STOP + Real_Time_Minute_STOP + Real_Time_Second_STOP;
					datasum += ((u8*)&Real_Time_Millise_STOP)[1];
					datasum += ((u8*)&Real_Time_Millise_STOP)[0];
					
					printf("AT+CIPSEND=%d\r\n",53 + ErrorS);
					HAL_Delay(10);
					printf("Recoder Repair Data\r\n");
					UARTSendData(&((u8*)&PackegEnd)[3],1);
					UARTSendData(&((u8*)&PackegEnd)[2],1);
					UARTSendData(&((u8*)&PackegEnd)[1],1);
					UARTSendData(&((u8*)&PackegEnd)[0],1);
					
					UARTSendData(&((u8*)&datasum)[3],1);
					UARTSendData(&((u8*)&datasum)[2],1);
					UARTSendData(&((u8*)&datasum)[1],1);
					UARTSendData(&((u8*)&datasum)[0],1);
					
					UARTSendData(BordID,12);
					
					UARTSendData(&((u8*)&PackegStart)[3],1);
					UARTSendData(&((u8*)&PackegStart)[2],1);
					UARTSendData(&((u8*)&PackegStart)[1],1);
					UARTSendData(&((u8*)&PackegStart)[0],1);
					
					

					
					UARTSendData(&Real_Time_Year_STOP,1);
					UARTSendData(&Real_Time_Month_STOP,1);
					UARTSendData(&Real_Time_Day_STOP,1);
					UARTSendData(&Real_Time_Hour_STOP,1);
					UARTSendData(&Real_Time_Minute_STOP,1);
					UARTSendData(&Real_Time_Second_STOP,1);
					UARTSendData(&((u8*)&Real_Time_Millise_STOP)[1],1);
					UARTSendData(&((u8*)&Real_Time_Millise_STOP)[0],1);
					

					UARTSendData(ErrorRecData,ErrorS);
					
					datasum = 0;
					ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
					f_lseek(&speex_file,speexdata_len);
					//return;
				}
			}
		}
		
		if(count == 100)
			return;
	}while(StrEqual(UART_BUFFER,"Device is Idle\r\n",sizeof(UART_BUFFER),strlen("Device is Idle\r\n")) == -1);
}







u8 rec_sta=0;		//录音状态
char pname[64];
unsigned char res;
u8 wav_recorder(u8 key,u32 fs)
{   
			switch(key)
			{		
				case 1://KEY2_PRES:	//STOP&SAVE
					if(rec_sta&0X80)//有录音
					{
						rec_sta=0;	//关闭录音
						end_recoder();
						//DataCheck();
						stop_recoder();
					}
					else
					{
						return 0;
					}
					rec_sta=0;
					return 0xff;

				case 2://KEY0_PRES:	//REC/PAUSE
					if(rec_sta&0X01)//原来是暂停,继续录音
					{
						rec_sta&=0XFE;//取消暂停
						return 1;
					}else if(rec_sta&0X80)//已经在录音了,暂停
					{
						rec_sta|=0X01;	//暂停
						return 1;
					}else				//还没开始录音 
					{


						recoder_new_pathname(pname);			//得到新的名字
						
						res = initial_recoder(pname,fs);
						
						
						
						if(res != FR_OK)			//文件创建失败
						{
							rec_sta=0;	//创建文件失败,不能录音
							//rval=0XFE;	//提示是否存在SD卡
							return 0;
						}else 
						{
							//res=f_write(f_rec,(const void*)wavhead,sizeof(__WaveHeader),&bw);//写入头数据

							return 0xab;
						} 
 					}
				case 3:
					tick_recoder();
					return rec_sta;
				case 4:
					rec_sta|=0X80;	//开始录音	
					start_recoder();
				default:
					return rec_sta;
			} 

}


u8 gad_sta=0;		//录音状态
u8 gad_recorder(u8 key,u32 time,u32 fs)
{   
			switch(key)
			{		
				case 1://KEY2_Pgad_res:	//STOP&SAVE
					if(gad_sta&0X80)//有录音
					{
						gad_sta=0;	//关闭录音
						
						//f_close(f_gad);

					}
					else
					{
						return 0;
					}
					gad_sta=0;
//					secondcache_write_Loc = 0;  
//					secondcache_read_Loc = 0; 
//					gad_plane = 0;
//					gad_time = 0;
//					gad_times = 0;
//				 	//LED1=1;	 						//关闭DS1
//			    
//					myfree(SRAMIN,data_string);	//释放内存
//					myfree(SRAMIN,f_gad);		//释放内存
//					myfree(SRAMIN,gad_pname);		//释放内存  
//					myfree(SRAMIN,data_stringbuffer);		//释放内存  
					return 0xff;

				case 2://KEY0_Pgad_res:	//REC/PAUSE
					if(gad_sta&0X01)//原来是暂停,继续录音
					{
						gad_sta&=0XFE;//取消暂停
						return 1;
					}else if(gad_sta&0X80)//已经在录音了,暂停
					{
						gad_sta|=0X01;	//暂停
						return 1;
					}else				//还没开始录音 
					{
//						while(f_opendir(&gaddir,"0:/RECORDER"))//打开录音文件夹
//						{	 			  
//							delay_ms(200);				  
//							f_mkdir("0:/RECORDER");				//创建该目录   
//						}   
//						data_string=mymalloc(SRAMIN,30);//I2S录音内存1申请
//						data_stringbuffer=mymalloc(SRAMIN,30 * 200);//I2S录音内存1申请
//							f_gad=(FIL *)mymalloc(SRAMIN,sizeof(FIL));		//开辟FIL字节的内存区域  
//						gad_pname=mymalloc(SRAMIN,30);						//申请30个字节内存,类似"0:RECORDER/REC00001.wav" 
//						if(!data_string||!f_gad||!gad_pname||!data_stringbuffer)
//							return 0xff;

//						gad_plane = time; 
//						gad_times = time;

//						Timeparticle = (512.0 / (double)fs);
//						
//						gad_pname[0]=0;					//gad_pname没有任何文件名 
//						
//						
//						


//						gad_new_pathname(gad_pname);			//得到新的名字
//	 					gad_res=f_open(f_gad,(const TCHAR*)gad_pname, FA_CREATE_ALWAYS | FA_WRITE); 
						if(0)			//文件创建失败
						{
							gad_sta=0;	//创建文件失败,不能录音

							return 0;
						}else 
						{

							return 0xab;
						} 
 					}
				case 3:
//						if(GAD_Rx_Flag)
//						{
//							GAD_Rx_Flag =0;
//							gad_rx_callback();
//						}
						//MPU_Get_Accelerometer(&gad_aacx,&gad_aacy,&gad_aacz);	//得到加速度传感器数据
						//MPU_Get_Gyroscope(&gad_gyrox,&gad_gyroy,&gad_gyroz);	//得到陀螺仪数据


						//sprintf((char*)data_string,"Time:%10.3f,AX:%05d,AY:%05d,AZ:%05d,GX:%05d,GY:%05d,GZ:%05d\r\n",(gad_times * 0.128), gad_aacx, gad_aacy, gad_aacz, gad_gyrox, gad_gyroy, gad_gyroz);
						//sprintf((char*)data_string,"Data Time:%10.3f , Pitch:%8.2f , Roll:%8.2f , Yaw:%8.2f , AacX:%05d , AacY:%05d , AacZ:%05d , GyroX:%05d , GyroY:%05d , GyroZ:%05d\r\n",(gad_times * 0.032), gad_pitch, gad_roll, gad_yaw, gad_aacx, gad_aacy, gad_aacz, gad_gyrox, gad_gyroy, gad_gyroz);
					
					return gad_sta;
					
				case 4:
					gad_sta|=0X80;	//开始录音	 
				default:
					return gad_sta;
			} 

}
void wifi_update_bindname()
{
	unsigned char i = 0;
	RecvComLoc2 = strlen((char*)FLASH_DATA.BIND_NAME);

	if(wifi_link_check_int == 0)
	{
		printf("AT+CIPSEND=%d\r\n",40 + RecvComLoc2);
		HAL_Delay(50);
		printf("Do Bindind Cheking\r\n");
		RecvComLoc3 = 0;
		UARTSendData(&((u8*)&RecvComLoc2)[3],1);
		UARTSendData(&((u8*)&RecvComLoc2)[2],1);
		UARTSendData(&((u8*)&RecvComLoc2)[1],1);
		UARTSendData(&((u8*)&RecvComLoc2)[0],1);
		for(i = 0;i<strlen("Do Bindind Cheking\r\n");i++)
			RecvComLoc3+="Do Bindind Cheking\r\n"[i];
		for(i = 0;i<12;i++)
			RecvComLoc3+=BordID[i];
		for(i = 0;i< RecvComLoc2;i++)
			RecvComLoc3+=FLASH_DATA.BIND_NAME[i];
		UARTSendData(&((u8*)&RecvComLoc3)[3],1);
		UARTSendData(&((u8*)&RecvComLoc3)[2],1);
		UARTSendData(&((u8*)&RecvComLoc3)[1],1);
		UARTSendData(&((u8*)&RecvComLoc3)[0],1);
		UARTSendData((unsigned char*)BordID,12);
		UARTSendData(FLASH_DATA.BIND_NAME,RecvComLoc2);
	}
}
void RF_Command_Check()
{
	u8 i,sum;
	//if(NRF24L01_RxPacket(Buffer)!=0)
	//		return;
	RecvComLoc1 = StrEqual(Buffer,(unsigned char*)"SERCH:Begin_",32,strlen("SERCH:Begin_"));
	if(RecvComLoc1 != -1)
	{
			sum = 0;
			for(i = RecvComLoc1 - 11;i < RecvComLoc1 + 2;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc1 + 2])
			{
				if(Buffer[RecvComLoc1 + 1] == FLASH_DATA.GROUP)
				{
					ClearBuffer(Buffer,sizeof(Buffer));
					return;
				}
				
				
				
				OLED_Clear( );
				//OLED_ShowString(0,2,(unsigned char*)"Request from:",8);
				//OLED_ShowString(0,4,(u8*)&Buffer[RecvComLoc1 + 1],1);
				//OLED_ShowChar(0,4,Buffer[RecvComLoc1 + 1],8);
				//OLED_ShowString(0,6,(unsigned char*)"Connect?",8);
				if(PressKey() == 0)
				{
					//sum = Send24L01Data("AB","00000000");
					SoftID = Buffer[RecvComLoc1 + 1];
					FLASH_DATA.GROUP = SoftID;
					OLED_Clear( );
					HAL_Delay(10);
					//OLED_ShowString(0,0,(unsigned char*)"Connection",8);
					//OLED_ShowString(0,2,(unsigned char*)"Succeeded!",8);
					//OLED_ShowString(0,4,(u8*)&FLASH_DATA.GROUP,1);
					HAL_Delay(2000);
					OLED_Clear( );
					HAL_Delay(10);
					//OLED_ShowString(0,0,(unsigned char*)"Waiting WIFI",8);
					STMFLASH_Write((u32*)&FLASH_DATA,sizeof(FLASH_SAVE) / 4);
					
					
					
				}
				OLED_Clear( );
				OLED_ShowString(0,0,"Wait Recording",8);
				OLED_ShowChar(120,0,FLASH_DATA.GROUP,8);
				OLED_ShowString(0,2,"ID:",8);
				OLED_ShowString(24,2,FLASH_DATA.DEVICE_ID,8);
				OLED_ShowString(0,4,"Name:",8);
					if(FLASH_DATA.BIND_NAME[0] != 0xff)
						display_GB2312_string(4,56,(unsigned char*)FLASH_DATA.BIND_NAME);
						//OLED_ShowString(56,4,(unsigned char*)FLASH_DATA.BIND_NAME,8);
			}
			ClearBuffer(Buffer,sizeof(Buffer));
			return;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	RecvComLoc1 = StrEqual(Buffer,(unsigned char*)"$WIFI",32,strlen("$WIFI"));
		RecvComLoc2 = StrEqual(Buffer,(unsigned char*)"$PASS",32,strlen("$PASS"));
		RecvComLoc3 = StrEqual(Buffer,(unsigned char*)"$IPAD",32,strlen("$IPAD"));
		RecvComLoc4 = StrEqual(Buffer,(unsigned char*)"$PORT",32,strlen("$PORT"));
		if(RecvComLoc1 != -1)
		{
			RecvComLoc2 = StrEqual(Buffer,(unsigned char*)"\n",32,strlen("\n"));
			if(RecvComLoc2 < RecvComLoc1)
			{
				ClearBuffer(Buffer,sizeof(Buffer));
				return;
			}
			
			sum = 0;
			for(i = RecvComLoc1 - 5;i < RecvComLoc2 + 1;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc2 + 1])
			{
				if(FLASH_DATA.GROUP != Buffer[RecvComLoc1 - 5])
				{
					ClearBuffer(Buffer,sizeof(Buffer));
					return;
				}
				sum = 0;
				for(i = 0;i<RecvComLoc2 - RecvComLoc1 - 1;i++)
				{
					if(FLASH_DATA.WIFI_NAME[i] != Buffer[i + RecvComLoc1 + 1])
						sum = 1;
					recv_wifi[i] = Buffer[i + RecvComLoc1 + 1];
					FLASH_DATA.WIFI_NAME[i] = recv_wifi[i];
				}
				FLASH_DATA.WIFI_NAME[i] = 0;
			}
			ClearBuffer(Buffer,sizeof(Buffer));
			if(sum == 0)
			{
				recv_wifi[0] = 0;
				recv_pass[0] = 0;
				recv_ip[0] = 0;
				recv_port[0] = 0;
				return;
			}
		}
		else if (RecvComLoc2 != -1)
		{
			RecvComLoc1 = StrEqual(Buffer,(unsigned char*)"\n",32,strlen("\n"));
			if(RecvComLoc1 < RecvComLoc2)
			{
				ClearBuffer(Buffer,sizeof(Buffer));
				return;
			}
			
			sum = 0;
			for(i = RecvComLoc2 - 5;i < RecvComLoc1 + 1;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc1 + 1])
			{
				if(FLASH_DATA.GROUP != Buffer[RecvComLoc2 - 5])
				{
					ClearBuffer(Buffer,sizeof(Buffer));
					return;
				}
				for(i = 0;i<RecvComLoc1 - RecvComLoc2 - 1;i++)
				{
					recv_pass[i] = Buffer[i + RecvComLoc2 + 1];
					FLASH_DATA.WIFI_PASS[i] = recv_pass[i];
				}
				FLASH_DATA.WIFI_PASS[i] = 0;
			}
			ClearBuffer(Buffer,sizeof(Buffer));
		}
		else if (RecvComLoc3 != -1)
		{
			RecvComLoc2 = StrEqual(Buffer,(unsigned char*)"\n",32,strlen("\n"));
			if(RecvComLoc2 < RecvComLoc3)
			{
				ClearBuffer(Buffer,sizeof(Buffer));
				return;
			}
			sum = 0;
			for(i = RecvComLoc3 - 5;i < RecvComLoc2 + 1;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc2 + 1])
			{
				if(FLASH_DATA.GROUP != Buffer[RecvComLoc3 - 5])
				{
					ClearBuffer(Buffer,sizeof(Buffer));
					return;
				}
				for(i = 0;i<RecvComLoc2 - RecvComLoc3 - 1;i++)
				{
					recv_ip[i] = Buffer[i + RecvComLoc3 + 1];
					FLASH_DATA.SERVER_IP[i] = recv_ip[i];
				}
				FLASH_DATA.SERVER_IP[i] = 0;
			}
			ClearBuffer(Buffer,sizeof(Buffer));
		}
		else if (RecvComLoc4 != -1)
		{
			RecvComLoc2 = StrEqual(Buffer,(unsigned char*)"\n",32,strlen("\n"));
			if(RecvComLoc2 < RecvComLoc4)
			{
				ClearBuffer(Buffer,sizeof(Buffer));
				return;
			}
			sum = 0;
			for(i = RecvComLoc4 - 5;i < RecvComLoc2 + 1;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc2 + 1])
			{
				if(FLASH_DATA.GROUP != Buffer[RecvComLoc4 - 5])
				{
					ClearBuffer(Buffer,sizeof(Buffer));
					return;
				}
				for(i = 0;i<RecvComLoc2 - RecvComLoc4 - 1;i++)
				{
					recv_port[i] = Buffer[i + RecvComLoc4 + 1];
					FLASH_DATA.SERVER_PORT[i] = recv_port[i];
				}
				FLASH_DATA.SERVER_PORT[i] = 0;
			}
			ClearBuffer(Buffer,sizeof(Buffer));
		}
		else
		{
			
		}
		if(recv_wifi[0] != 0 && recv_pass[0] != 0 && recv_ip[0] != 0 && recv_port[0] != 0)
		{
			recv_wifi[0] = 0;
			recv_pass[0] = 0;
			recv_ip[0] = 0;
			recv_port[0] = 0;
				OLED_Clear( );
				HAL_Delay(10);
				display_GB2312_string(0,0,"连接网络中");
	OLED_ShowString(0,2,(u8*)FLASH_DATA.WIFI_NAME,8);
	OLED_ShowString(0,3,(u8*)FLASH_DATA.WIFI_PASS,8);
				
				///////////////////////////////////////////////////
				//ESP8266 Init
				printf("AT+CWMODE=1\r\n");
				HAL_Delay(1000);
				i = 0;
				do
				{
					//printf("AT+CWJAP=\"CU_kh2m\",\"m9svhdhq\"\r\n");
					printf("AT+CWJAP=\"%s\",\"%s\"\r\n",FLASH_DATA.WIFI_NAME,FLASH_DATA.WIFI_PASS);
					ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
					HAL_GPIO_WritePin(GPIOC,LED2_Pin,1);
					HAL_Delay(5000);
					HAL_GPIO_WritePin(GPIOC,LED2_Pin,0);
					HAL_Delay(5000);
					printf("%s",UART_BUFFER);
					i++;
					if(i == 3)
					{
						
						OLED_Clear( );
						HAL_Delay(10);
						display_GB2312_string(0,0,"连接网络失败");
						HAL_Delay(1000);
						ClearBuffer(Buffer,sizeof(Buffer));
						return;
					}
				}while(StrEqual(UART_BUFFER,(unsigned char*)recv_A,sizeof(UART_BUFFER),strlen(recv_A)) == -1);
				
				ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
				
				
				
				OLED_Clear( );
				HAL_Delay(10);
				display_GB2312_string(0,0,"连接服务器中");
				OLED_ShowString(0,2,(u8*)FLASH_DATA.SERVER_IP,8);
				OLED_ShowString(0,3,(u8*)FLASH_DATA.SERVER_PORT,8);
				
				i = 0;
				do
				{
					printf("AT+CIPSTART=\"TCP\",\"%s\",%s\r\n",FLASH_DATA.SERVER_IP,FLASH_DATA.SERVER_PORT);
					ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
					HAL_GPIO_WritePin(GPIOC,LED1_Pin,1);
					HAL_Delay(2500);
					HAL_GPIO_WritePin(GPIOC,LED1_Pin,0);
					HAL_Delay(2500);
					i++;
					if(i == 6)
					{
						OLED_Clear( );
						HAL_Delay(10);
						display_GB2312_string(0,0,"连接服务器失败");
						HAL_Delay(1000);
						ClearBuffer(Buffer,sizeof(Buffer));
						return;
					}
				}while(StrEqual(UART_BUFFER,(unsigned char*)recv_B,sizeof(UART_BUFFER),strlen(recv_B)) == -1);
			STMFLASH_Write((u32*)&FLASH_DATA,sizeof(FLASH_SAVE) / 4);
			OLED_Clear( );
			display_GB2312_string(0,0,"等待录音");
			OLED_ShowChar(120,0,FLASH_DATA.GROUP,8);
			if(FLASH_DATA.BIND_NAME[0] != 0xff)
				display_GB2312_string(2,0,(unsigned char*)FLASH_DATA.BIND_NAME);
				//OLED_ShowString(56,4,(unsigned char*)FLASH_DATA.BIND_NAME,8);
			ClearBuffer(Buffer,sizeof(Buffer));
			return;
		}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				RecvComLoc3 = StrEqual(UART_BUFFER,(unsigned char*)"Request Binding\r\n",sizeof(UART_BUFFER),strlen("Request Binding\r\n"));
		if(RecvComLoc3 != -1)
		{
			
			RecvComLoc2 = UART_BUFFER[RecvComLoc3 + 1] << 24;
			RecvComLoc2 |= UART_BUFFER[RecvComLoc3 + 2] << 16;
			RecvComLoc2 |= UART_BUFFER[RecvComLoc3 + 3] << 8;
			RecvComLoc2 |= UART_BUFFER[RecvComLoc3 + 4];
			
			RecvComLoc1 = UART_BUFFER[RecvComLoc3 + 5] << 24;
			RecvComLoc1 |= UART_BUFFER[RecvComLoc3 + 6] << 16;
			RecvComLoc1 |= UART_BUFFER[RecvComLoc3 + 7] << 8;
			RecvComLoc1 |= UART_BUFFER[RecvComLoc3 + 8];
			
			
			RecvComLoc4 = 0;
			for(i = 0; i < (u8)RecvComLoc2 + 12; i++)
				RecvComLoc4 += UART_BUFFER[RecvComLoc3 + 9 + i];
			for(i = 0; i < strlen("Request Binding\r\n"); i++)
				RecvComLoc4 += "Request Binding\r\n"[i];
			
			
			
			if(RecvComLoc4 == RecvComLoc1)
			{
//				for(i = 0;i<8;i++)
//				{
//					initfilename[i] = BordID[i];
//				}
				for(i = 0;i< RecvComLoc2;i++)
				{
					//initfilename[i + 8] = UART_BUFFER[RecvComLoc3 + 17 + i];
					FLASH_DATA.BIND_NAME[i] = UART_BUFFER[RecvComLoc3 + 21 + i];
				}
				FLASH_DATA.BIND_NAME[i] = 0;
				wifi_update_bindname();
				STMFLASH_Write((u32*)&FLASH_DATA,sizeof(FLASH_SAVE) / 4);
				
				OLED_Clear( );
				display_GB2312_string(0,0,"等待录音");
				OLED_ShowChar(120,0,FLASH_DATA.GROUP,8);
				if(FLASH_DATA.BIND_NAME[0] != 0xff)
					display_GB2312_string(2,0,(unsigned char*)FLASH_DATA.BIND_NAME);
					//OLED_ShowString(56,4,(unsigned char*)FLASH_DATA.BIND_NAME,8);
			}
			ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
		}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		RecvComLoc3 = StrEqual(UART_BUFFER,(unsigned char*)"Synchronous Device Info\r\n",sizeof(UART_BUFFER),strlen("Synchronous Device Info\r\n"));
		if(RecvComLoc3 != -1 && wifi_link_check_int == 0)
		{
			printf("AT+CIPSEND=%d\r\n",45);
			HAL_Delay(50);
			printf("Synchronous Device Info\r\n");
			RecvComLoc3 = 0;
			UARTSendData((u8*)&RecvComLoc3,4);
			for(i = 0;i<strlen("Synchronous Device Info\r\n");i++)
				RecvComLoc3+="Synchronous Device Info\r\n"[i];
			for(i = 0;i<12;i++)
				RecvComLoc3+=BordID[i];
			
			
			UARTSendData(&((u8*)&RecvComLoc3)[3],1);
			UARTSendData(&((u8*)&RecvComLoc3)[2],1);
			UARTSendData(&((u8*)&RecvComLoc3)[1],1);
			UARTSendData(&((u8*)&RecvComLoc3)[0],1);
			UARTSendData((unsigned char*)BordID,12);
			ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
			return;
		}
}

void PVD_Init(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	PWR_PVDTypeDef PWR_PVDStruct;
	
	PWR_PVDStruct.PVDLevel = PWR_PVDLEVEL_7; // 2.9V
	PWR_PVDStruct.Mode = PWR_PVD_MODE_IT_RISING_FALLING; //降至阈值电压时触发

	HAL_PWR_ConfigPVD(&PWR_PVDStruct);
	
	HAL_NVIC_SetPriority(PVD_IRQn,4,0);
  HAL_NVIC_EnableIRQ(PVD_IRQn);
	HAL_PWR_EnablePVD();
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	UINT bw2;
	u8 presskeyvalue = 1;
	u32 timecnt = 0;
	u32 LVD_Count = 0;
	u8 connectstate = 0;
	u8 i,sum = 0;
	u8 *str1;
	u8 WavFlag = 0xff;
	u8 GadFlag = 0xff;
	u32 FS = 0,GADFS = 0;
	u8 state = 0;
	FIL config;
	FRESULT conres;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	my_mem_init(SRAMIN);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	MX_GPIO_Init();
	isPowerOn = 0;
	HAL_GPIO_WritePin(PWR_CTL_GPIO_Port,PWR_CTL_Pin,1);
	while(!HAL_GPIO_ReadPin(PWR_FLAG_GPIO_Port,PWR_FLAG_Pin))
	{
		HAL_Delay(50);
	}
	isPowerOn = 1;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SDIO_SD_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(WIFI_RST_GPIO_Port,WIFI_RST_Pin,1);
	PVD_Init();
	my_mem_init(0);
	Speex_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	
	check_firstrun();
	
	for (i = 0; i < 12; i++)
		BordID[i] = FLASH_DATA.DEVICE_ID[i];
	
	USART2_UART_Init(921600);
	HAL_UART_Receive_IT(&huart2,UART_BUFFER,sizeof(UART_BUFFER));
	//HAL_UART_Transmit(&huart2,"AT+CIPSTATUS\r\n",14,100);
	GT20_Init();
	OLED_Init( );	 
	OLED_Clear( );


	
		if(HAL_GPIO_ReadPin(LVD_GPIO_Port,LVD_Pin) == 0)
		{
			display_GB2312_string(0,0,"电量低");
			HAL_Delay(2000);
			HAL_GPIO_WritePin(PWR_CTL_GPIO_Port,PWR_CTL_Pin,0);
			while(1);
		}
	
	//display_GB2312_string(2,56,"啊这");
	//while(1);
	display_GB2312_string(0,0,"开机中");
	//OLED_ShowString(0,0,(unsigned char*)"Check NRF24L01..",8);
	NRF24L01_Init();
	while(NRF24L01_Check())
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		HAL_Delay(200);
	}
	HAL_Delay(100);
	NRF24L01_RX_Mode();
	HAL_Delay(100);
	
	
	//OLED_ShowString(0,2,(unsigned char*)"Check MPU6050..",8);
//	MPU_Init();
//	HAL_Delay(200);
//	while(mpu_dmp_init())
//	{
//		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
// 		HAL_Delay(200);
//	}
	
	
	
	//OLED_ShowString(0,4,(unsigned char*)"Check SDCard..",8);
	MX_FATFS_Init();
	if(retSD != FR_OK)
	{
		OLED_Clear( );
		HAL_Delay(500);
		display_GB2312_string(0,0,"存储卡错误！");
		while(1);
	}
  MX_USB_DEVICE_Init();
	
	while(f_opendir(&dir,"0:/RECORDER"))//打开录音文件夹
	{	 			  
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		HAL_Delay(200);				  
		f_mkdir("0:/RECORDER");				//创建该目录   
	}

	printf("AT+CWMODE_DEF=1\r\n");
	//OLED_ShowString(0,6,(unsigned char*)"Check Over!",8);
	HAL_Delay(2000);
	OLED_Clear( );
	HAL_Delay(500);

	
	
	
	if(FLASH_DATA.WIFI_NAME[0] != 0xff)
	{
			OLED_Clear( );
			HAL_Delay(10);

			{
				for(i = 0; i< 20;i++)
					recv_wifi[i] = FLASH_DATA.WIFI_NAME[i];
				for(i = 0; i< 20;i++)
					recv_pass[i] = FLASH_DATA.WIFI_PASS[i];
				for(i = 0; i< 20;i++)
					recv_ip[i] = FLASH_DATA.SERVER_IP[i];
				for(i = 0; i< 20;i++)
					recv_port[i] = FLASH_DATA.SERVER_PORT[i];
				goto connect_wifi;
			}
//			else
//			{
//				OLED_Clear( );
//				HAL_Delay(10);
//				OLED_ShowString(0,0,(unsigned char*)"Waiting WIFI",8);
//			}
	}
	
	
	
	
	
	
	
	
	
	
	//////////////////////////////////////////////////

	
	//////////////////////////////////////////////////
	//wifi
	wifi:
	recv_wifi[0] = 0;
	recv_pass[0] = 0;
	recv_ip[0] = 0;
	recv_port[0] = 0;
	OLED_Clear( );
	HAL_Delay(10);
	display_GB2312_string(0,0,"等待连接网络中");
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
	while(1)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		HAL_Delay(25);
		NRF24L01_RX_Mode();
		HAL_Delay(25);
		
		if(NRF24L01_RxPacket(Buffer)!=0)
			continue;
		
			RecvComLoc1 = StrEqual(Buffer,(unsigned char*)"SERCH:Begin_",32,strlen("SERCH:Begin_"));
	if(RecvComLoc1 != -1)
	{
			sum = 0;
			for(i = RecvComLoc1 - 11;i < RecvComLoc1 + 2;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc1 + 2])
			{
				if(Buffer[RecvComLoc1 + 1] == FLASH_DATA.GROUP)
				{
					continue;
				}
				
				
				
				
				//OLED_ShowString(0,2,(unsigned char*)"Request from:",8);
				//OLED_ShowString(0,4,(u8*)&Buffer[RecvComLoc1 + 1],1);
				//OLED_ShowChar(0,4,Buffer[RecvComLoc1 + 1],8);
				//OLED_ShowString(0,6,(unsigned char*)"Connect?",8);
				if(PressKey() == 0)
				{
					//sum = Send24L01Data("AB","00000000");
					SoftID = Buffer[RecvComLoc1 + 1];
					FLASH_DATA.GROUP = SoftID;
					OLED_Clear( );
					HAL_Delay(10);
					//OLED_ShowString(0,0,(unsigned char*)"Connection",8);
					//OLED_ShowString(0,2,(unsigned char*)"Succeeded!",8);
					//OLED_ShowString(0,4,(u8*)&FLASH_DATA.GROUP,1);
					HAL_Delay(2000);
					OLED_Clear( );
					HAL_Delay(10);
					//OLED_ShowString(0,0,(unsigned char*)"Waiting WIFI",8);
					STMFLASH_Write((u32*)&FLASH_DATA,sizeof(FLASH_SAVE) / 4);
					OLED_Clear( );
					HAL_Delay(10);
					display_GB2312_string(0,0,"等待连接网络中");
					//OLED_ShowString(0,0,(unsigned char*)"Waiting WIFI",8);
					HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
					HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
				}
				continue;
				
				
			}
			
	}
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		RecvComLoc1 = StrEqual(Buffer,(unsigned char*)"$WIFI",32,strlen("$WIFI"));
		RecvComLoc2 = StrEqual(Buffer,(unsigned char*)"$PASS",32,strlen("$PASS"));
		RecvComLoc3 = StrEqual(Buffer,(unsigned char*)"$IPAD",32,strlen("$IPAD"));
		RecvComLoc4 = StrEqual(Buffer,(unsigned char*)"$PORT",32,strlen("$PORT"));
		if(RecvComLoc1 != -1)
		{
			RecvComLoc2 = StrEqual(Buffer,(unsigned char*)"\n",32,strlen("\n"));
			if(RecvComLoc2 < RecvComLoc1)
				continue;
			
			sum = 0;
			for(i = RecvComLoc1 - 5;i < RecvComLoc2 + 1;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc2 + 1])
			{
				if(FLASH_DATA.GROUP != Buffer[RecvComLoc1 - 5])
					continue;
				for(i = 0;i<RecvComLoc2 - RecvComLoc1 - 1;i++)
				{
					recv_wifi[i] = Buffer[i + RecvComLoc1 + 1];
					FLASH_DATA.WIFI_NAME[i] = recv_wifi[i];
				}
				FLASH_DATA.WIFI_NAME[i] = 0;
			}
		}
		else if (RecvComLoc2 != -1)
		{
			RecvComLoc1 = StrEqual(Buffer,(unsigned char*)"\n",32,strlen("\n"));
			if(RecvComLoc1 < RecvComLoc2)
				continue;
			
			sum = 0;
			for(i = RecvComLoc2 - 5;i < RecvComLoc1 + 1;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc1 + 1])
			{
				if(FLASH_DATA.GROUP != Buffer[RecvComLoc2 - 5])
					continue;
				for(i = 0;i<RecvComLoc1 - RecvComLoc2 - 1;i++)
				{
					recv_pass[i] = Buffer[i + RecvComLoc2 + 1];
					FLASH_DATA.WIFI_PASS[i] = recv_pass[i];
				}
				FLASH_DATA.WIFI_PASS[i] = 0;
			}
		}
		else if (RecvComLoc3 != -1)
		{
			RecvComLoc2 = StrEqual(Buffer,(unsigned char*)"\n",32,strlen("\n"));
			if(RecvComLoc2 < RecvComLoc3)
				continue;
			
			sum = 0;
			for(i = RecvComLoc3 - 5;i < RecvComLoc2 + 1;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc2 + 1])
			{
				if(FLASH_DATA.GROUP != Buffer[RecvComLoc3 - 5])
					continue;
				for(i = 0;i<RecvComLoc2 - RecvComLoc3 - 1;i++)
				{
					recv_ip[i] = Buffer[i + RecvComLoc3 + 1];
					FLASH_DATA.SERVER_IP[i] = recv_ip[i];
				}
				FLASH_DATA.SERVER_IP[i] = 0;
			}
		}
		else if (RecvComLoc4 != -1)
		{
			RecvComLoc2 = StrEqual(Buffer,(unsigned char*)"\n",32,strlen("\n"));
			if(RecvComLoc2 < RecvComLoc4)
				continue;
			
			sum = 0;
			for(i = RecvComLoc4 - 5;i < RecvComLoc2 + 1;i++)
				sum+=Buffer[i];
			if(sum == Buffer[RecvComLoc2 + 1])
			{
				if(FLASH_DATA.GROUP != Buffer[RecvComLoc4 - 5])
					continue;
				for(i = 0;i<RecvComLoc2 - RecvComLoc4 - 1;i++)
				{
					recv_port[i] = Buffer[i + RecvComLoc4 + 1];
					FLASH_DATA.SERVER_PORT[i] = recv_port[i];
				}
				FLASH_DATA.SERVER_PORT[i] = 0;
			}
		}
		else
		{
			
		}
		if(recv_wifi[0] != 0 && recv_pass[0] != 0 && recv_ip[0] != 0 && recv_port[0] != 0)
		{
			ClearBuffer(Buffer,sizeof(Buffer));
			break;
		}
	}
	
	
	/////////////////////////////////////////////////
	
	
	
	//////////////////////////////////////////////////
	connect_wifi:
	
	OLED_Clear( );
	HAL_Delay(10);
	display_GB2312_string(0,0,"连接网络中");
	OLED_ShowString(0,2,(u8*)FLASH_DATA.WIFI_NAME,8);
	OLED_ShowString(0,3,(u8*)FLASH_DATA.WIFI_PASS,8);
	///////////////////////////////////////////////////
	//ESP8266 Init
//	printf("AT+CWMODE=1\r\n");
//	HAL_Delay(500);
//	printf("AT+SLEEP=1\r\n");
//	HAL_Delay(500);
//	printf("AT+GSLP=3600000\r\n");
//	HAL_Delay(1000);
	i = 0;
	do
	{
		//printf("AT+CWJAP=\"CU_kh2m\",\"m9svhdhq\"\r\n");
		printf("AT+CWJAP=\"%s\",\"%s\"\r\n",FLASH_DATA.WIFI_NAME,FLASH_DATA.WIFI_PASS);
		ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
		HAL_Delay(5000);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
		HAL_Delay(5000);
		i++;
		if(i == 3)
		 goto wifi;
		printf("%s",UART_BUFFER);
	}while(StrEqual(UART_BUFFER,(unsigned char*)recv_A,sizeof(UART_BUFFER),strlen(recv_A)) == -1);
	
	ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
	
	
	
	OLED_Clear( );
	HAL_Delay(10);
	display_GB2312_string(0,0,"连接服务器中");
	OLED_ShowString(0,2,(u8*)FLASH_DATA.SERVER_IP,8);
	OLED_ShowString(0,3,(u8*)FLASH_DATA.SERVER_PORT,8);
	i = 0;
	do
	{
		printf("AT+CIPSTART=\"TCP\",\"%s\",%s\r\n",FLASH_DATA.SERVER_IP,FLASH_DATA.SERVER_PORT);
		ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
		HAL_Delay(2500);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED1_Pin,0);
		HAL_Delay(2500);
		i++;
		if(i == 6)
		 goto wifi;
	}while(StrEqual(UART_BUFFER,(unsigned char*)recv_B,sizeof(UART_BUFFER),strlen(recv_B)) == -1);
	
	
	
	recv_wifi[0] = 0;
	recv_pass[0] = 0;
	recv_ip[0] = 0;
	recv_port[0] = 0;
	
	

	
	
	main_start:

STMFLASH_Write((u32*)&FLASH_DATA,sizeof(FLASH_SAVE) / 4);
	ClearBuffer(UART_BUFFER,sizeof(UART_BUFFER));
	HAL_Delay(200);
	
	
	OLED_Clear( );
	HAL_Delay(10);
	display_GB2312_string(0,0,"等待录音");
	OLED_ShowChar(120,0,FLASH_DATA.GROUP,8);
	if(FLASH_DATA.BIND_NAME[0] != 0xff)
		display_GB2312_string(2,0,(unsigned char*)FLASH_DATA.BIND_NAME);
		//OLED_ShowString(56,4,(unsigned char*)FLASH_DATA.BIND_NAME,8);
	
	
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
	
	printf("AT+CWMODE=1\r\n");
	HAL_Delay(200);
	printf("AT+SLEEP=1\r\n");
	HAL_Delay(200);
//	printf("AT+CWMODE=1\r\n");
//	HAL_Delay(200);
//	printf("AT+SLEEP=1\r\n");
//	HAL_Delay(200);
//	printf("AT+GSLP=3600000\r\n");
//	HAL_Delay(500);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//wifi_link_check();   //20200705
		if(HAL_GPIO_ReadPin(LVD_GPIO_Port,LVD_Pin) == 0)
		{
			LVD_Count++;
		}
		else
			LVD_Count = 0;
		if(LVD_Count >= 400*10)
		{
//			OLED_Clear( );
			LVD_Count = 0;
			display_GB2312_string(2,80,"电量低");
//			HAL_Delay(5000);
//			HAL_GPIO_WritePin(PWR_CTL_GPIO_Port,PWR_CTL_Pin,0);
//			while(1);
		}
		if(NRF24L01_RxPacket(Buffer)==0)
		{
			str1 = (u8*)strstr((char *)Buffer,(char *)"CALIB");
			if(str1)
			{
				sum = 0;
				for(i = 0;i<14;i++)
				{
					sum += *(str1 + i);
				}
				if(sum == *(str1 + 14))
				{
					sum = 0;
					if(*(str1 + 5) != FLASH_DATA.GROUP)
						sum = 1;
					if(sum == 0)
					{
						HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
						HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
						Real_Time_Year = *(str1 + 6);
						Real_Time_Month = *(str1 + 7);
						Real_Time_Day = *(str1 + 8);
						Real_Time_Hour = *(str1 + 9);
						Real_Time_Minute = *(str1 + 10);
						Real_Time_Second = *(str1 + 11);
						Real_Time_Millise = 0;
						Real_Time_Millise = *(str1 + 12) << 8;
						Real_Time_Millise |= *(str1 + 13);
					}
				}
			}
			
			
			str1 = (u8*)strstr((char *)Buffer,(char *)"BEGIN:");
			if(str1)
			{
				sum = 0;
				if(*(str1 - 1) != FLASH_DATA.GROUP)
					sum = 1;
			}
			if(str1 && sum == 0)
			{
				if(*(str1 + 6)=='B' && *(str1 + 7)=='l' && *(str1 + 8)=='i' && *(str1 + 9)=='n' && *(str1 + 10)=='k')
				{
					state = (~state & COMMAND_Blink) | (state & ~COMMAND_Blink);
					HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
					HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
				}
				if(*(str1 + 6)=='I' && *(str1 + 7)=='n' && *(str1 + 8)=='i' && *(str1 + 9)=='t')
				{
					FS = atoi((char *)(str1 + 10));
					//printf("Begin Create Wave File!File Name=%s%s&&&End",SessID,initfilename);
					if(*(str1 + 15) == '-')
					{
						GADFS = atoi((char *)(str1 + 16));
						if((GADFS / 100) == 0)
						{
							AM_Factor = atoi((char *)(str1 + 18));
						}
					}
					else if(*(str1 + 14) == '-')
					{
						GADFS = atoi((char *)(str1 + 15));
						if((GADFS / 100) == 0)
						{
							AM_Factor = atoi((char *)(str1 + 17));
						}
					}
					else
						continue;
					
					
					
					
					if(FS > 36000 || FS < 8000 || GADFS == 0)
						continue;
					if(GadFlag == 0xff && WavFlag == 0xff)
					{
						state &= ~COMMAND_Blink;
						state |= COMMAND_Init;
						HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
						HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
						
						OLED_Clear( );
						HAL_Delay(10);
						display_GB2312_string(0,0,"正在录音");
						OLED_ShowChar(120,0,FLASH_DATA.GROUP,8);
						if(FLASH_DATA.BIND_NAME[0] != 0xff)
							display_GB2312_string(2,0,(unsigned char*)FLASH_DATA.BIND_NAME);
					}
				}
				if(*(str1 + 6)=='B' && *(str1 + 7)=='e' && *(str1 + 8)=='g' && *(str1 + 9)=='i' && *(str1 + 10)=='n')
				{
					if(GadFlag == 0xab && WavFlag == 0xab)
					{
						printf("AT+GSLP=3600000\r\n");
						state &= ~COMMAND_Blink;
						state |= COMMAND_Begin;
						HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
						HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
					}
				}
				if(*(str1 + 6)=='E' && *(str1 + 7)=='n' && *(str1 + 8)=='d')
				{
					if(GadFlag == 0x80 && WavFlag == 0x80)
					{
						HAL_GPIO_WritePin(WIFI_RST_GPIO_Port,WIFI_RST_Pin,0);
						HAL_Delay(200);
						HAL_GPIO_WritePin(WIFI_RST_GPIO_Port,WIFI_RST_Pin,1);
						state &= ~COMMAND_Blink;
						state |= COMMAND_End;
						HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
						HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
					}
				}
				if(*(str1 + 6)=='S' && *(str1 + 7)=='E')
				{
					for(char i = 0; i < 12; i++)
						if(hex_to_char((*(str1 + 8 + i / 2) >> 4 * (((i % 2) * -1) + 1)) & 0x0f) != FLASH_DATA.DEVICE_ID[0])
							break;
					if(GadFlag == 0x80 && WavFlag == 0x80)
					{
						HAL_GPIO_WritePin(WIFI_RST_GPIO_Port,WIFI_RST_Pin,0);
						HAL_Delay(200);
						HAL_GPIO_WritePin(WIFI_RST_GPIO_Port,WIFI_RST_Pin,1);
						state &= ~COMMAND_Blink;
						state |= COMMAND_End;
						HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
						HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
					}
				}
				*str1 = 0x30;
			}
		}
		
		if(GadFlag==0x80 && WavFlag==0x80)
		{
			gad_recorder(3,0,0);   //获得数据 
			wav_recorder(3,0);
		}
		
		if(state & COMMAND_Init)
		{
			state &= ~COMMAND_Init;
			if(GadFlag == 0xff && WavFlag == 0xff)
			{
				MX_USB_DEVICE_DeInit();
				GadFlag = gad_recorder(2,GADFS,FS);//初始化，采样频率
				WavFlag = wav_recorder(2,FS);//初始化
			}
		}
				
		if(state & COMMAND_Begin)
		{
			state &= ~COMMAND_Begin;
			if(GadFlag == 0xab && WavFlag == 0xab)
			{
				Real_Time_Year_STOP = Real_Time_Year;
				Real_Time_Month_STOP = Real_Time_Month;
				Real_Time_Day_STOP = Real_Time_Day;
				Real_Time_Hour_STOP = Real_Time_Hour;
				Real_Time_Minute_STOP = Real_Time_Minute;
				Real_Time_Second_STOP = Real_Time_Second;
				Real_Time_Millise_STOP = Real_Time_Millise;

				GadFlag = gad_recorder(4,0,0); //开始
				WavFlag = wav_recorder(4,0);//开始
			}
		}
			
		if(state & COMMAND_End)
		{
			state &= ~COMMAND_End;
			if(GadFlag == 0x80 && WavFlag == 0x80)
			{
				GadFlag = gad_recorder(1,0,0);       //停止
				WavFlag = wav_recorder(1,0);       //停止
				OLED_Clear( );
				HAL_Delay(10);
						display_GB2312_string(0,0,"等待录音");
						OLED_ShowChar(120,0,FLASH_DATA.GROUP,8);
						if(FLASH_DATA.BIND_NAME[0] != 0xff)
							display_GB2312_string(2,0,(unsigned char*)FLASH_DATA.BIND_NAME);
				MX_USB_DEVICE_Init();
			}
		}


		if((state&COMMAND_Blink) && (GadFlag == 0xff) && (WavFlag == 0xff))
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			HAL_Delay(250);
			//printf("State"); //告诉服务器目前待机
		}
		if(WavFlag == 0xab && GadFlag == 0xab)    //可以录音提示
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			HAL_Delay(100);
			//printf("State"); //告诉服务器目前待机
			//printf("Begin Create Wave File!File Name=%s%s&&&End",SessID,initfilename);
			if(wifi_link_check_int == 0)
			{
			printf("AT+CIPSEND=%d\r\n",44);
			HAL_Delay(10);
			printf("Ready To Start Recoder\r\n");
			RecvComLoc3 = 0;
			UARTSendData((u8*)&RecvComLoc3,4);
			for(i = 0;i<strlen("Ready To Start Recoder\r\n");i++)
				RecvComLoc3+="Ready To Start Recoder\r\n"[i];
			for(i = 0;i<12;i++)
				RecvComLoc3+=BordID[i];
			
			
			UARTSendData(&((u8*)&RecvComLoc3)[3],1);
			UARTSendData(&((u8*)&RecvComLoc3)[2],1);
			UARTSendData(&((u8*)&RecvComLoc3)[1],1);
			UARTSendData(&((u8*)&RecvComLoc3)[0],1);
			UARTSendData(BordID,12);
			}
		}
		else if(WavFlag == 0x80 && GadFlag == 0x80)    //正在录音提示
		{
			timecnt++;
			if((timecnt%100)==0)
			{
					HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);

					//printf("State is Idle!"); //告诉服务器目前待机
			}
		}
		else
		{
			RF_Command_Check();
			wifi_link_check();
			timecnt++;
			if((timecnt%1000)==0) //告诉服务器目前待机
			{
				
				
//					sprintf(String_Windows_Time,"%0.2d",Real_Time_Month);
//					OLED_ShowString(0,6,String_Windows_Time,8);
//				OLED_ShowString(16,6,"-",8);
//				
//				sprintf(String_Windows_Time,"%0.2d",Real_Time_Day);
//					OLED_ShowString(24,6,String_Windows_Time,8);
//				OLED_ShowString(40,6," ",8);
//				
//				sprintf(String_Windows_Time,"%0.2d",Real_Time_Hour);
//					OLED_ShowString(48,6,String_Windows_Time,8);
//				
//				OLED_ShowString(64,6,":",8);
//				
//				sprintf(String_Windows_Time,"%0.2d",Real_Time_Minute);
//					OLED_ShowString(72,6,String_Windows_Time,8);
//				OLED_ShowString(88,6,":",8);
//				
//				sprintf(String_Windows_Time,"%0.2d",Real_Time_Second);
//					OLED_ShowString(96,6,String_Windows_Time,8);
				
				if(wifi_link_check_int == 0)
				{
					RecvComLoc2 = strlen((char*)FLASH_DATA.BIND_NAME);
				printf("AT+CIPSEND=%d\r\n",37 + RecvComLoc2);
					HAL_Delay(10);
					printf("Device is Idle\r\n");
					RecvComLoc3 = 1 + RecvComLoc2;
					UARTSendData(&((u8*)&RecvComLoc3)[3],1);
					UARTSendData(&((u8*)&RecvComLoc3)[2],1);
					UARTSendData(&((u8*)&RecvComLoc3)[1],1);
					UARTSendData(&((u8*)&RecvComLoc3)[0],1);
					RecvComLoc3 = 0;
					for(i = 0;i<strlen("Device is Idle\r\n");i++)
						RecvComLoc3+="Device is Idle\r\n"[i];
					for(i = 0;i<12;i++)
						RecvComLoc3+=BordID[i];
					for(i = 0;i<RecvComLoc2;i++)
						RecvComLoc3+=FLASH_DATA.BIND_NAME[i];
					
					RecvComLoc3+=1;
					UARTSendData(&((u8*)&RecvComLoc3)[3],1);
					UARTSendData(&((u8*)&RecvComLoc3)[2],1);
					UARTSendData(&((u8*)&RecvComLoc3)[1],1);
					UARTSendData(&((u8*)&RecvComLoc3)[0],1);
					UARTSendData(BordID,12);
					RecvComLoc3 = 1;
					UARTSendData(&((u8*)&RecvComLoc3)[0],1);
					UARTSendData(FLASH_DATA.BIND_NAME,RecvComLoc2);
				}
				
			}

		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 11999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CHG_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GT20_CSN_GPIO_Port, GT20_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PWR_CTL_Pin|I2C_SDA_Pin|I2C_SCL_Pin|WIFI_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF_CE_Pin|NRF_CSN_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_WK_GPIO_Port, WIFI_WK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CHG_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = CHG_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_FLAG_Pin */
  GPIO_InitStruct.Pin = KEY_FLAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_FLAG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GT20_CSN_Pin */
  GPIO_InitStruct.Pin = GT20_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GT20_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_CTL_Pin I2C_SDA_Pin I2C_SCL_Pin WIFI_RST_Pin */
  GPIO_InitStruct.Pin = PWR_CTL_Pin|I2C_SDA_Pin|I2C_SCL_Pin|WIFI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_FLAG_Pin */
  GPIO_InitStruct.Pin = PWR_FLAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PWR_FLAG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_IRQ_Pin LVD_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin|LVD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_WK_Pin */
  GPIO_InitStruct.Pin = WIFI_WK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_WK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Ovveride the _speex_fatal function of the speex library
  * @param  None
  * @retval : None
  */
void _speex_fatal(const char *str, const char *file, int line)
{
  while(1)
  {
  };
}
/**
  * @brief  Ovveride the _speex_putc function of the speex library
  * @param  None
  * @retval : None
  */
void _speex_putc(int ch, void *file)
{
  while(1)
  {
  };
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
