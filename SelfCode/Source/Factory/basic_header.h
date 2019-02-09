///////////////////////////////////////////////////////////////////
//创建时间：2018-11-16
//修改时间：2018-11-30
//创建人员：HaHaHaHaHaGe
//修改人员：HaHaHaHaHaGe
//主要功能：包含常用的头文件
//		
//日志：
//
//2018-11-16:
//创建此文件，增加3种头文件
//basic_function 基础函数
//basic_macro 基础宏定义
//basic_type 基础类型重定义
//
//2018-11-23
//增加4种头文件，删去basic_function
//<malloc.h> <string.h> <stdlib.h> <stdio.h>
//
//2018-11-30
//增加宏定义Little_Endian用于适应不同大小端系统
//////////////////////////////////////////////////////////////////
#ifndef __BASIC_HEADER_H__
#define __BASIC_HEADER_H__


#include "basic_macro.h"
#include "basic_type.h"


#include <malloc.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


#define Little_Endian


#endif // !__BASIC_HEADER_H__
