#ifndef __GT20_H
#define __GT20_H	 		  
#include "main.h" 

#define u8 unsigned char
#define uchar unsigned char

#define GT20_CE(x)  HAL_GPIO_WritePin(GT20_CSN_GPIO_Port,GT20_CSN_Pin,x)
void GT20_Init(void);
void display_GB2312_string(unsigned char y,unsigned char x,unsigned char *text);
#endif
