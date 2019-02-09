#include "test_main.h"

int main(void)
{
	u8 s;
	u32 crc;

	______search_packeg______(1);
	while (1)
	{

	}




	s = ______write_buffer_len____get_unread_data_____(1000000);
	if (s == YES)
		printf("______write_buffer_data____get_unread_ptr_____:					YES\r\n");
	else
		printf("______write_buffer_data____get_unread_ptr_____:					NO\r\n");


	s = ______write_buffer_len____get_unread_ptr_____(1000000);
	if (s == YES)
		printf("______write_buffer_len____get_unread_ptr_____:					YES\r\n");
	else
		printf("______write_buffer_len____get_unread_ptr_____:					NO\r\n");


	s = ______write_buffer_data____get_unread_ptr_____(1000000);
	if (s == YES)
		printf("______write_buffer_data____get_unread_ptr_____:					YES\r\n");
	else
		printf("______write_buffer_data____get_unread_ptr_____:					NO\r\n");


	crc = ______crc32_16bytes______(1024*1024);
	printf("______crc32_16bytes______:							0x%X\r\n", crc);
	while (1);
}