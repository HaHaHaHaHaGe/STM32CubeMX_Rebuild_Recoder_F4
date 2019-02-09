///////////////////////////////////////////////////////////////////
//创建时间：2018-11-16
//修改时间：2018-11-16
//创建人员：HaHaHaHaHaGe
//修改人员：HaHaHaHaHaGe
//主要功能：basic_type中重定义了数据类型,并用明显的方式区别出来
//			如u32代表32位的无符号数据类型,方便在移植到其他平台时修改
//		
//日志：
//
//2018-11-16:
//创建此文件，增加10种类型，无符号类64、32、16、8位
//有符号类64、32、16、8位，浮点型64、32位
//////////////////////////////////////////////////////////////////


#ifndef __BASIC_TYPE_H__
#define __BASIC_TYPE_H__

typedef unsigned long long u64;
typedef signed long long s64;

typedef unsigned int u32;
typedef signed int s32;

typedef unsigned short int u16;
typedef signed short int s16;

typedef unsigned char u8;
typedef signed char s8;

typedef double f64;
typedef float f32;

#endif // !__BASIC_TYPE_H__
