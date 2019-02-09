#include "test_main.h"

u8 ______write_buffer_data____get_unread_ptr_____(u32 loop)
{
	u32 i,j,k;
	u8* s1, *s2;
	u32 l,m = 0,n = 0;
	u8* str = "1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
	u8 check;
	ringbuffer ring;
#ifdef ____SHOW__DEBUG____
	printf("%d", strlen(str));
#endif // ____SHOW__DEBUG____
	initial_buffer(&ring, YES, strlen(str));
	while (loop--)
	{
#ifdef ____SHOW__DEBUG____
		printf("缓存总大小：%d \r\n", strlen(str));
		printf("Buffer数据：\r\n");
		for( k =0; k< strlen(str);k++)
			printf("%c", start_ringbuffer_ptr[k]);
		printf("\r\n写入数据：");
#endif // ____SHOW__DEBUG____
		l = rand() % (strlen(str) + 1);
#ifdef ____SHOW__DEBUG____
		for (k = 0; k < l; k++)
			printf("%c", str[k]);
		printf("\r\n");
#endif // ____SHOW__DEBUG____
		write_buffer_data(&ring, str, l);
		get_unread_ptr(&ring, &s1,&s2,&i,&j,NO);
#ifdef ____SHOW__DEBUG____
		printf("读出数据：");
#endif // ____SHOW__DEBUG____
		check = YES;
		for (k = 0; k < i; k++) {
#ifdef ____SHOW__DEBUG____
			printf("%c", s1[k]);
#endif // ____SHOW__DEBUG____
			if (str[k] != s1[k])
				check = NO;
		}
		for (k = 0; k < j; k++) {
#ifdef ____SHOW__DEBUG____
			printf("%c", s2[k]);
#endif // ____SHOW__DEBUG____
			if (str[i+k] != s2[k])
				check = NO;
		}
		if (check == NO)
			m++;
		n++;
#ifdef ____SHOW__DEBUG____
		printf("\r\n");
		printf("读写不同出现次数：%d",m);
		printf("测试总次数：%d", n);
		system("cls");
#endif // ____SHOW__DEBUG____
	}
	if (m == 0)
		return YES;
	return NO;
}


u8 ______write_buffer_len____get_unread_ptr_____(u32 loop)
{
	u32 i, j, k, o;
	u8* s1, *s2;
	u32 l, m = 0, n = 0;
	u8* str = "1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
	u8 check;
	ringbuffer ring;
#ifdef ____SHOW__DEBUG____
	printf("%d", strlen(str));
#endif // ____SHOW__DEBUG____
	initial_buffer(&ring, YES, strlen(str));
	
	while (loop--)
	{
#ifdef ____SHOW__DEBUG____
		printf("缓存总大小：%d \r\n", strlen(str));
		printf("Buffer数据：\r\n");
		for (k = 0; k < strlen(str); k++)
			printf("%c", start_ringbuffer_ptr[k]);
		printf("\r\n写入数据：");
#endif // ____SHOW__DEBUG____
		l = rand() % (strlen(str) + 1);
#ifdef ____SHOW__DEBUG____
		for (k = 0; k < l; k++)
			printf("%c", str[k]);
		printf("\r\n");
#endif // ____SHOW__DEBUG____
		o = 999999;
		for (k = 0; k < l; k++)
		{
			if (&ring.write_location_ptr[k] == ring.end_ringbuffer_ptr)
			{
				o = 0;
			}
			if(o == 999999)
				ring.write_location_ptr[k] = str[k];
			else
				ring.start_ringbuffer_ptr[o++] = str[k];
		}
		write_buffer_len(&ring, l);
		get_unread_ptr(&ring, &s1, &s2, &i, &j, NO);
#ifdef ____SHOW__DEBUG____
		printf("读出数据：");
#endif // ____SHOW__DEBUG____
		check = YES;
		for (k = 0; k < i; k++) {
#ifdef ____SHOW__DEBUG____
			printf("%c", s1[k]);
#endif // ____SHOW__DEBUG____
			if (str[k] != s1[k])
				check = NO;
		}
		for (k = 0; k < j; k++) {
#ifdef ____SHOW__DEBUG____
			printf("%c", s2[k]);
#endif // ____SHOW__DEBUG____
			if (str[i + k] != s2[k])
				check = NO;
		}
		if (check == NO)
			m++;
		n++;
#ifdef ____SHOW__DEBUG____
		printf("\r\n");
		printf("读写不同出现次数：%d", m);
		printf("测试总次数：%d", n);
		system("cls");
#endif // ____SHOW__DEBUG____
	}
	if (m == 0)
		return YES;
	return NO;
}



u8 ______write_buffer_len____get_unread_data_____(u32 loop)
{
	u32 i, k, o;
	u8* s1;
	u32 l, m = 0, n = 0;
	u8* str = "1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
	u8 check;
	ringbuffer ring;
#ifdef ____SHOW__DEBUG____
	printf("%d", strlen(str));
#endif // ____SHOW__DEBUG____
	initial_buffer(&ring, YES, strlen(str));

	while (loop--)
	{
#ifdef ____SHOW__DEBUG____
		printf("缓存总大小：%d \r\n", strlen(str));
		printf("Buffer数据：\r\n");
		for (k = 0; k < strlen(str); k++)
			printf("%c", start_ringbuffer_ptr[k]);
		printf("\r\n写入数据：");
#endif // ____SHOW__DEBUG____
		l = rand() % (strlen(str) + 1);
#ifdef ____SHOW__DEBUG____
		for (k = 0; k < l; k++)
			printf("%c", str[k]);
		printf("\r\n");
#endif // ____SHOW__DEBUG____
		o = 999999;
		for (k = 0; k < l; k++)
		{
			if (&ring.write_location_ptr[k] == ring.end_ringbuffer_ptr)
			{
				o = 0;
			}
			if (o == 999999)
				ring.write_location_ptr[k] = str[k];
			else
				ring.start_ringbuffer_ptr[o++] = str[k];
		}
		write_buffer_len(&ring, l);
		s1 = get_unread_data(&ring, &i, NO);
#ifdef ____SHOW__DEBUG____
		printf("读出数据：");
#endif // ____SHOW__DEBUG____
		check = YES;
		for (k = 0; k < i; k++) {
#ifdef ____SHOW__DEBUG____
			printf("%c", s1[k]);
#endif // ____SHOW__DEBUG____
			if (str[k] != s1[k])
				check = NO;
		}
		if (check == NO)
			m++;
		n++;
#ifdef ____SHOW__DEBUG____
		printf("\r\n");
		printf("读写不同出现次数：%d", m);
		printf("测试总次数：%d", n);
		system("cls");
#endif // ____SHOW__DEBUG____
	}
	if (m == 0)
		return YES;
	return NO;
}



