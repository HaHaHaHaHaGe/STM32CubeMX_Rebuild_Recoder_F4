///////////////////////////////////////////////////////////////////
//创建时间：2018-11-16
//修改时间：2018-11-30
//创建人员：HaHaHaHaHaGe
//修改人员：HaHaHaHaHaGe
//主要功能：basic_function中定义了与本协议栈无关的工具类函数
//			如，内存的管理等
//		
//日志：
//
//2018-11-16:
//创建此文件，增加
//void* basic_malloc(u32 size)
//void basic_free(void* block)
//函数，用于内存的分配与释放
//
//2018-11-21:
//增加函数void basic_memcpy(u8* dst,u8* src,u32 size)
//
//2018-11-30:
//增加basic_swap函数，用于处理不同大小端的数据
//////////////////////////////////////////////////////////////////
#ifndef __BASIC_FUNCTION_H__
#define __BASIC_FUNCTION_H__

#include "../basic_header.h"

/*
分配内存
入口参数:
size:需要申请的空间大小，单位字节
*/
void* basic_malloc(u32 size);


/*
释放内存
入口参数:
block:释放空间的指针
*/
void basic_free(void* block);




/*
内存拷贝
入口参数：
dst：被拷贝的位置
src：需要拷贝的位置
size：需要拷贝的大小（字节）
*/
void basic_memcpy(u8* dst, u8* src, u32 size);



/*
根据大小端的不同修改数据
入口参数：
x：源数据
返回值：
调整后的数据
*/
u32 basic_swap(u32 x);
#endif // !__BASIC_FUNCTION_H__
