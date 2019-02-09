///////////////////////////////////////////////////////////////////
//创建时间：2018-11-23
//修改时间：2018-11-27
//创建人员：HaHaHaHaHaGe
//修改人员：HaHaHaHaHaGe
//主要功能：测试所有代码
//			
//		
//日志：
//
//2018-11-23:
//创建此文件，编写ringbuffer的测试代码ringbuffer_test.c
//______write_buffer_data____get_unread_ptr_____
//增加____SHOW__DEBUG____宏定义
//
//2018-11-27
//对______write_buffer_data____get_unread_ptr_____增加打印原始缓存
//数据功能，并
//增加测试函数
//______write_buffer_len____get_unread_ptr_____
//增加测试函数
//______write_buffer_len____get_unread_data_____
//修改来自stbrumme的crc32功能及测试代码
//并更名为______crc32_16bytes______
//////////////////////////////////////////////////////////////////


#ifndef __TEST_MAIN_H__
#define __TEST_MAIN_H__

//#define ____SHOW__DEBUG____

#include "../Factory/basic_header.h"
#include "../BasicDataStream/inc/ringbuffer.h"
#include "../Factory/inc/basic_crc32.h"
#include "../Factory/inc/basic_function.h"
#include "../BasicDataStream/inc/packeg_taken.h"
/*
入口参数:
i: 测试次数
返回值：成功/失败
*/
u8 ______write_buffer_data____get_unread_ptr_____(u32 loop);



/*
入口参数:
i: 测试次数
返回值：成功/失败
*/
u8 ______write_buffer_len____get_unread_ptr_____(u32 loop);






/*
入口参数:
i: 测试次数
返回值：成功/失败
*/
u8 ______write_buffer_len____get_unread_data_____(u32 loop);




/*
入口参数:
NumBytes: 生成测试数据的大小
返回值：CRC校验结果
*/
u32 ______crc32_16bytes______(u32 NumBytes);



u32 ______search_packeg______(u32 loop);
#endif