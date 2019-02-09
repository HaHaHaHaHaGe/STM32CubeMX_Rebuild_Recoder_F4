#include "../inc/basic_function.h"

void* basic_malloc(u32 size)
{
	return mymalloc(SRAMIN,size);
}

void basic_free(void* block)
{
	myfree(SRAMIN,block);
}

void basic_memcpy(u8* dst,u8* src,u32 size)
{
	//memcpy(dst,src,size);
	while(size--)
		dst[size] = src[size];
}

u32 basic_swap(u32 x)
{
	return (x >> 24) |
		((x >> 8) & 0x0000FF00) |
		((x << 8) & 0x00FF0000) |
		(x << 24);
}
