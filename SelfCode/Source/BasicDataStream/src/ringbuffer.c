#include "../inc/ringbuffer.h"



u8 initial_buffer(ringbuffer *ptr,u8 self_mem,u32 size)
{
	if (self_mem == NO)															//判断是否是提供好的空间
	{
		ptr->malloc_flag = NO;															//记录是不是自行分配的空间
	}
	else
	{
		ptr->start_ringbuffer_ptr = basic_malloc(size);									//内存分配与成功与否检测							
		if (ptr->start_ringbuffer_ptr == 0)
			return FAIL;
		ptr->malloc_flag = YES;															//记录是自行分配的空间
	}
	ptr->return_data_buffer = basic_malloc(size);										//分配用于返回读取的空间
	ptr->end_ringbuffer_ptr = ptr->start_ringbuffer_ptr + size;
	ptr->write_location_ptr = ptr->start_ringbuffer_ptr;										//初始化相关指针
	ptr->read_location_ptr = ptr->start_ringbuffer_ptr;
	ptr->ringbuffer_size = size;															//保存分配的空间大小
	ptr->leading_flag = NO;
	return SUCCESS;
}



u8 deinitial_buffer(ringbuffer *ptr)
{
	if (ptr->malloc_flag == YES)															//判断是否是自行分配的空间
		basic_free(ptr->start_ringbuffer_ptr);											//释放空间
	ptr->ringbuffer_size = 0;
	basic_free(ptr->return_data_buffer);
	ptr->start_ringbuffer_ptr = NULL_PTR;												//防止野指针
	ptr->end_ringbuffer_ptr = NULL_PTR;
	ptr->write_location_ptr = NULL_PTR;
	ptr->read_location_ptr = NULL_PTR;
	ptr->return_data_buffer = NULL_PTR;
	return SUCCESS;
}




void write_buffer_len(ringbuffer *ptr,u32 datalen)
{
	u32 write_len;																	//写数据大小											
	if (ptr->write_location_ptr + datalen < ptr->end_ringbuffer_ptr)							//当写入数据量在 write 与 end 之间时
	{
		ptr->write_location_ptr += datalen % ptr->ringbuffer_size;
		if (ptr->leading_flag == YES)
			ptr->read_location_ptr = ptr->write_location_ptr;
	}
	else if ((datalen - 1) / ptr->ringbuffer_size > 0)									//当写入数据大于整个缓冲区时
	{
		ptr->write_location_ptr = ptr->start_ringbuffer_ptr + (ptr->read_location_ptr - ptr->start_ringbuffer_ptr);
		ptr->leading_flag = YES;
	}
	else																			//当写入数据量在缓冲区大小内 但 不在write 与 end 之间时
	{
		write_len = datalen - (ptr->end_ringbuffer_ptr - ptr->write_location_ptr);
		ptr->write_location_ptr = ptr->start_ringbuffer_ptr;
		if (write_len + ptr->start_ringbuffer_ptr >= ptr->read_location_ptr)
		{
			ptr->leading_flag = YES;
			if (write_len == ptr->ringbuffer_size)
				ptr->read_location_ptr = ptr->start_ringbuffer_ptr;
			else
				ptr->read_location_ptr = ptr->write_location_ptr + write_len;
		}
		ptr->write_location_ptr += write_len;
		if (ptr->leading_flag == YES)
			ptr->read_location_ptr = ptr->write_location_ptr;
	}
}





void write_buffer_data(ringbuffer *ptr, u8* data, u32 datalen)
{
	u32 write_len;																	//写数据大小
	u8* read_ptr;																	//读数据指针
	u32 _len;																		//临时用于保存上次读数据的大小
	if (ptr->write_location_ptr + datalen < ptr->end_ringbuffer_ptr)							//当写入数据量在 write 与 end 之间时
	{
		write_len = datalen;
		read_ptr = data;
		while (write_len--)
			*(ptr->write_location_ptr++) = *(read_ptr++);
		if (ptr->write_location_ptr == ptr->end_ringbuffer_ptr)
			ptr->write_location_ptr = ptr->start_ringbuffer_ptr;
		if (ptr->leading_flag == YES)
			ptr->read_location_ptr = ptr->write_location_ptr;
	}
	else if ((datalen - 1) / ptr->ringbuffer_size > 0)									//当写入数据大于整个缓冲区时
	{
		write_len = ptr->end_ringbuffer_ptr - ptr->read_location_ptr;
		read_ptr = data + datalen - ptr->ringbuffer_size;
		ptr->write_location_ptr = ptr->read_location_ptr;
		while (write_len--)
			*(ptr->write_location_ptr++) = *(read_ptr++);
		ptr->write_location_ptr = ptr->start_ringbuffer_ptr;
		write_len = ptr->read_location_ptr - ptr->start_ringbuffer_ptr;
		while (write_len--)
			*(ptr->write_location_ptr++) = *(read_ptr++);
		ptr->leading_flag = YES;
	}
	else																			//当写入数据量在缓冲区大小内 但 不在write 与 end 之间时
	{
		write_len = ptr->end_ringbuffer_ptr - ptr->write_location_ptr;
		_len = write_len;
		read_ptr = data;
		while (write_len--)
			*(ptr->write_location_ptr++) = *(read_ptr++);
		ptr->write_location_ptr = ptr->start_ringbuffer_ptr;
		write_len = datalen - _len;



		if (write_len + ptr->start_ringbuffer_ptr >= ptr->read_location_ptr)
		{
			ptr->leading_flag = YES;
			if (write_len == ptr->ringbuffer_size)
				ptr->read_location_ptr = ptr->start_ringbuffer_ptr;
			else
				ptr->read_location_ptr = ptr->write_location_ptr + write_len;
		}
		while (write_len--)
			*(ptr->write_location_ptr++) = *(read_ptr++);
		if (ptr->leading_flag == YES)
			ptr->read_location_ptr = ptr->write_location_ptr;
	}

}



u8* get_unread_data(ringbuffer *ptr, u32 *len, u8 preview)										//获取所有未读数据
{
	if (ptr->read_location_ptr < ptr->write_location_ptr)									//判断read是否超前write
	{
		basic_memcpy(ptr->return_data_buffer, ptr->read_location_ptr, ptr->write_location_ptr - ptr->read_location_ptr);
		*len = ptr->write_location_ptr - ptr->read_location_ptr;
	}
	else if (ptr->read_location_ptr == ptr->write_location_ptr && ptr->leading_flag == NO)		//没有未读数据
	{
		*len = 0;
		return NULL_PTR;
	}
	else																		//分段存在的数据（read  > write）
	{
		basic_memcpy(ptr->return_data_buffer, ptr->read_location_ptr, ptr->end_ringbuffer_ptr - ptr->read_location_ptr);
		basic_memcpy(ptr->return_data_buffer + (ptr->end_ringbuffer_ptr - ptr->read_location_ptr), ptr->start_ringbuffer_ptr, ptr->write_location_ptr - ptr->start_ringbuffer_ptr);
		*len = ptr->end_ringbuffer_ptr - ptr->read_location_ptr + ptr->write_location_ptr - ptr->start_ringbuffer_ptr;
	}
	if (preview == NO)															//是否是预览模式，预览模式下不修改读指针
	{
		ptr->read_location_ptr = ptr->write_location_ptr;
		ptr->leading_flag = NO;
	}
	return ptr->return_data_buffer;
}


void get_unread_ptr(ringbuffer *ptr, u8** ptr_1,u8** ptr_2,u32* len_1,u32* len_2, u8 preview)	//获取未读数据所在的位置
{
	if (ptr->read_location_ptr < ptr->write_location_ptr)									//判断read是否超前write
	{
		*ptr_1 = ptr->read_location_ptr;
		*len_1 = ptr->write_location_ptr - ptr->read_location_ptr;
		*ptr_2 = NULL_PTR;
		*len_2 = 0;
	}
	else if(ptr->read_location_ptr == ptr->write_location_ptr && ptr->leading_flag == NO)		//没有未读数据
	{
		*ptr_1 = NULL_PTR;
		*len_1 = 0;
		*ptr_2 = NULL_PTR;
		*len_2 = 0;
	}
	else																		//分段存在的数据（read  > write）
	{
		*ptr_1 = ptr->read_location_ptr;
		*len_1 = ptr->end_ringbuffer_ptr - ptr->read_location_ptr;
		*ptr_2 = ptr->start_ringbuffer_ptr;
		*len_2 = ptr->write_location_ptr - ptr->start_ringbuffer_ptr;
	}
	if (preview == NO)															//是否是预览模式，预览模式下不修改读指针
	{
		ptr->read_location_ptr = ptr->write_location_ptr;
		ptr->leading_flag = NO;
	}
}



void update_readlocation(ringbuffer *ptr)
{
	ptr->read_location_ptr = ptr->write_location_ptr;
	ptr->leading_flag = NO;
}
