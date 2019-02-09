///////////////////////////////////////////////////////////////////
//创建时间：2011-11-13
//修改时间：2018-11-30
//创建人员：stbrumme
//修改人员：HaHaHaHaHaGe
//主要功能：fork后进行修改，主要用于数据包的crc校验
//		
//日志：
//
//2018-11-27:
//从stbrumme的git上fork的代码进行删减修改后的代码
//源github：https://github.com/stbrumme/crc32
//创始人web：https://create.stephan-brumme.com/crc32/
//保留crc32_16bytes函数，其余删除
//增加Little_Endian宏定义，用于切换大小端
//
//2018-11-30
//移除swap函数到basic_function中，并更名为basic_swap
//移除Little_Endian宏定义到basic_function中
//////////////////////////////////////////////////////////////////
#ifndef __BASIC_CRC32_H__
#define __BASIC_CRC32_H__

#include "../basic_header.h"
#include "../inc/basic_function.h"





/*
crc校验
入口参数：
data:数据指针
length:数据长度
previousCrc32:之前的crc32校验结果（用于分布式校验）
返回值：
crc校验结果
*/
u32 crc32_16bytes(u32* data, u32 length, u32 previousCrc32);

#endif // !__BASIC_CRC32_H__