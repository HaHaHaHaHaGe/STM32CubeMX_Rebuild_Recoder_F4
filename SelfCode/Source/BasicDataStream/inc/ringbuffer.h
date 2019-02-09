///////////////////////////////////////////////////////////////////
//创建时间：2018-11-16
//修改时间：2018-12-1
//创建人员：HaHaHaHaHaGe
//修改人员：HaHaHaHaHaGe
//主要功能：ringbuffer中主要实现数据缓冲功能。环形的读取可以有效
//降低数据被覆盖的可能性
//		
//日志：
//
//2018-11-16:
//创建此文件，增加
//void write_buffer(u8* data, u32 datalen);
//u8 deinitial_buffer(void);
//u8 initial_buffer(u8* ptr, u32 size);
//用于写缓冲区、释放缓冲区、初始化缓冲区
//
//2018-11-20
//将void write_buffer(u8* data, u32 datalen);
//更改为void write_buffer_data(u8* data, u32 datalen)
//并修复在写入的数据刚好等于整个缓存大小、并且读指针为起始值时
//无法正确判断并设置当前状态为“超前态”
//增加读取函数
//u8* get_unread_data(u32 *len);
//用于一口气读出所有的未读数据
//
//2018-11-22
//将u8* get_unread_data(u32 *len,u8 preview);
//增加入口参数preview
//增加函数
//void get_unread_ptr(u8** ptr_1, u8** ptr_2, u32* len_1, u32* len_2, u8 preview);
//用于直接返回未读数据的指针（效率最高，但是会为两部分）
//修复get_unread_data函数存在的bug（未考虑超前态）
//
//2018-11-27
//修复get_unread_data函数没有对输入的*len进行赋值造成野指针的bug
//修改全部函数，并增加结构体ringbuffer，使用此结构可初始化不同的实例
//修改initial_buffer增加参数u8 self_mem，用于标志是否自行分配内存
//增加void update_readlocation(ringbuffer *ptr)函数
//用于手动更新read指针
//
//2018-12-1
//删除部分影响效率的判断空指针代码（会增加跑飞风险）
//////////////////////////////////////////////////////////////////
#ifndef  __RINGBUFFER_H__
#define __RINGBUFFER_H__
#include "../../Factory/basic_header.h"
#include "../../Factory/inc/basic_function.h"




/*
用于存储ringbuffer中的关键全局变量
使用此结构可构建不同的ringbuffer
*/
typedef struct ringbuffer
{
	u8* start_ringbuffer_ptr;															//缓冲区起始位置
	u8* end_ringbuffer_ptr;																//缓冲区截止位置
	u8* write_location_ptr;																//写数据位置
	u8* read_location_ptr;																//读数据位置

	u8* return_data_buffer;																//用于返回的数据组

	u32 ringbuffer_size;																//缓冲区大小
	u8 malloc_flag;																		//改缓冲区是否是自行创建的

	u8 leading_flag;																	//当前写数据是否领先读数据一个周期

}ringbuffer;








/*
写数据到缓冲区函数
入口参数：
ptr:操作对象
data:写入的数据指针
datalen:写入的数据长度
*/
void write_buffer_data(ringbuffer *ptr, u8* data, u32 datalen);


/*
释放缓冲区函数
注：如果在创建时指定了已经开辟好的内存，
那么该函数只会清理此模块内部的变量，
不会更改外部的数据
入口参数：
ptr:操作对象
*/
u8 deinitial_buffer(ringbuffer *ptr);




/*
创建缓冲区函数
注：可在创建时指定已经开辟好的内存，
入口参数：
ptr:操作对象
self_mem: 是否由本函数自行分配空间，如果不需要，
应输入NO并在操作对象的start ptr中分配好相应的空间
size: 需要开辟空间的大小
*/
u8 initial_buffer(ringbuffer *ptr, u8 self_mem, u32 size);


/*
外部写入数据后使用此函数更新内部参数
入口参数：
ptr:操作对象
datalen:外部写入的数据长度
*/
void write_buffer_len(ringbuffer *ptr, u32 datalen);


/*
获取全部未读数据
入口参数：
ptr:操作对象
len: 读出数据的长度（字节）
preview：若为YES则此次读取不会修改ringbuff内指针状态
*/
u8* get_unread_data(ringbuffer *ptr, u32 *len,u8 preview);





/*
获取全部未读数据的指针（分两部分）
入口参数：
ptr:操作对象
ptr_1: 保存第一个指针
ptr_2: 保存第二个指针
len_1: 第一个指针内容的大小（字节）
len_2: 第二个指针内容的大小（字节）
preview：若为YES则此次读取不会修改ringbuff内指针状态
*/
void get_unread_ptr(ringbuffer *ptr, u8** ptr_1, u8** ptr_2, u32* len_1, u32* len_2, u8 preview);











/*
手动更新read指针
注:一般在get_unread_ptr后根据自身情况使用
入口参数：
ptr:操作对象
*/
void update_readlocation(ringbuffer *ptr);
#endif //  __RINGBUFFER_H_
