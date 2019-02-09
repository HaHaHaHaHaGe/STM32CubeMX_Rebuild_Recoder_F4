#include "test_main.h"

u32 ______search_packeg______(u32 loop)
{
	u8 *src_1 = basic_malloc(48);
	u8 *src_2 = basic_malloc(48);

	u8 *rdata_1, *rdata_2;
	u32 len_1, len_2;
	u32 i;

	u8 success = NO;
	
	src_1[0] = 0xff;
	src_1[1] = 0xff;
	src_1[2] = 0xff;
	src_1[3] = 0xff;

	src_1[4] = 0x00;
	src_1[5] = 0x00;
	src_1[6] = 0x00;
	src_1[7] = 0x08;

	src_1[8] = 0x7a;
	src_1[9] = 0x6d;
	src_1[10] = 0x1d;
	src_1[11] = 0x1d;

	src_1[12] = 0x12;
	src_1[13] = 0x23;
	src_1[14] = 0x34;
	src_1[15] = 0x45;
	src_1[16] = 0x56;
	src_1[17] = 0x67;
	src_1[18] = 0x78;
	src_1[19] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	",rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");

	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
////////////////////////////////////////////////////////////////////////////////////////////////




	src_1[32] = 0xff;
	src_1[33] = 0xff;
	src_1[34] = 0xff;
	src_1[35] = 0xff;

	src_1[36] = 0x00;
	src_1[37] = 0x00;
	src_1[38] = 0x00;
	src_1[39] = 0x08;

	src_1[40] = 0x7a;
	src_1[41] = 0x6d;
	src_1[42] = 0x1d;
	src_1[43] = 0x1d;

	src_1[44] = 0x12;
	src_1[45] = 0x23;
	src_1[46] = 0x34;
	src_1[47] = 0x45;
	src_2[0] = 0x56;
	src_2[1] = 0x67;
	src_2[2] = 0x78;
	src_2[3] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[36] = 0xff;
	src_1[37] = 0xff;
	src_1[38] = 0xff;
	src_1[39] = 0xff;

	src_1[40] = 0x00;
	src_1[41] = 0x00;
	src_1[42] = 0x00;
	src_1[43] = 0x08;

	src_1[44] = 0x7a;
	src_1[45] = 0x6d;
	src_1[46] = 0x1d;
	src_1[47] = 0x1d;

	src_2[0] = 0x12;
	src_2[1] = 0x23;
	src_2[2] = 0x34;
	src_2[3] = 0x45;
	src_2[4] = 0x56;
	src_2[5] = 0x67;
	src_2[6] = 0x78;
	src_2[7] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[38] = 0xff;
	src_1[39] = 0xff;
	src_1[40] = 0xff;
	src_1[41] = 0xff;

	src_1[42] = 0x00;
	src_1[43] = 0x00;
	src_1[44] = 0x00;
	src_1[45] = 0x08;

	src_1[46] = 0x7a;
	src_1[47] = 0x6d;
	src_2[0] = 0x1d;
	src_2[1] = 0x1d;

	src_2[2] = 0x12;
	src_2[3] = 0x23;
	src_2[4] = 0x34;
	src_2[5] = 0x45;
	src_2[6] = 0x56;
	src_2[7] = 0x67;
	src_2[8] = 0x78;
	src_2[9] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[37] = 0xff;
	src_1[38] = 0xff;
	src_1[39] = 0xff;
	src_1[40] = 0xff;

	src_1[41] = 0x00;
	src_1[42] = 0x00;
	src_1[43] = 0x00;
	src_1[44] = 0x08;

	src_1[45] = 0x7a;
	src_1[46] = 0x6d;
	src_1[47] = 0x1d;
	src_2[0] = 0x1d;

	src_2[1] = 0x12;
	src_2[2] = 0x23;
	src_2[3] = 0x34;
	src_2[4] = 0x45;
	src_2[5] = 0x56;
	src_2[6] = 0x67;
	src_2[7] = 0x78;
	src_2[8] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[39] = 0xff;
	src_1[40] = 0xff;
	src_1[41] = 0xff;
	src_1[42] = 0xff;

	src_1[43] = 0x00;
	src_1[44] = 0x00;
	src_1[45] = 0x00;
	src_1[46] = 0x08;

	src_1[47] = 0x7a;
	src_2[0] = 0x6d;
	src_2[1] = 0x1d;
	src_2[2] = 0x1d;

	src_2[3] = 0x12;
	src_2[4] = 0x23;
	src_2[5] = 0x34;
	src_2[6] = 0x45;
	src_2[7] = 0x56;
	src_2[8] = 0x67;
	src_2[9] = 0x78;
	src_2[10] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[40] = 0xff;
	src_1[41] = 0xff;
	src_1[42] = 0xff;
	src_1[43] = 0xff;

	src_1[44] = 0x00;
	src_1[45] = 0x00;
	src_1[46] = 0x00;
	src_1[47] = 0x08;

	src_2[0] = 0x7a;
	src_2[1] = 0x6d;
	src_2[2] = 0x1d;
	src_2[3] = 0x1d;

	src_2[4] = 0x12;
	src_2[5] = 0x23;
	src_2[6] = 0x34;
	src_2[7] = 0x45;
	src_2[8] = 0x56;
	src_2[9] = 0x67;
	src_2[10] = 0x78;
	src_2[11] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[41] = 0xff;
	src_1[42] = 0xff;
	src_1[43] = 0xff;
	src_1[44] = 0xff;

	src_1[45] = 0x00;
	src_1[46] = 0x00;
	src_1[47] = 0x00;
	src_2[0] = 0x08;

	src_2[1] = 0x7a;
	src_2[2] = 0x6d;
	src_2[3] = 0x1d;
	src_2[4] = 0x1d;

	src_2[5] = 0x12;
	src_2[6] = 0x23;
	src_2[7] = 0x34;
	src_2[8] = 0x45;
	src_2[9] = 0x56;
	src_2[10] = 0x67;
	src_2[11] = 0x78;
	src_2[12] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[42] = 0xff;
	src_1[43] = 0xff;
	src_1[44] = 0xff;
	src_1[45] = 0xff;

	src_1[46] = 0x00;
	src_1[47] = 0x00;
	src_2[0] = 0x00;
	src_2[1] = 0x08;

	src_2[2] = 0x7a;
	src_2[3] = 0x6d;
	src_2[4] = 0x1d;
	src_2[5] = 0x1d;

	src_2[6] = 0x12;
	src_2[7] = 0x23;
	src_2[8] = 0x34;
	src_2[9] = 0x45;
	src_2[10] = 0x56;
	src_2[11] = 0x67;
	src_2[12] = 0x78;
	src_2[13] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[43] = 0xff;
	src_1[44] = 0xff;
	src_1[45] = 0xff;
	src_1[46] = 0xff;

	src_1[47] = 0x00;
	src_2[0] = 0x00;
	src_2[1] = 0x00;
	src_2[2] = 0x08;

	src_2[3] = 0x7a;
	src_2[4] = 0x6d;
	src_2[5] = 0x1d;
	src_2[6] = 0x1d;

	src_2[7] = 0x12;
	src_2[8] = 0x23;
	src_2[9] = 0x34;
	src_2[10] = 0x45;
	src_2[11] = 0x56;
	src_2[12] = 0x67;
	src_2[13] = 0x78;
	src_2[14] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[44] = 0xff;
	src_1[45] = 0xff;
	src_1[46] = 0xff;
	src_1[47] = 0xff;

	src_2[0] = 0x00;
	src_2[1] = 0x00;
	src_2[2] = 0x00;
	src_2[3] = 0x08;

	src_2[4] = 0x7a;
	src_2[5] = 0x6d;
	src_2[6] = 0x1d;
	src_2[7] = 0x1d;

	src_2[8] = 0x12;
	src_2[9] = 0x23;
	src_2[10] = 0x34;
	src_2[11] = 0x45;
	src_2[12] = 0x56;
	src_2[13] = 0x67;
	src_2[14] = 0x78;
	src_2[15] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[45] = 0xff;
	src_1[46] = 0xff;
	src_1[47] = 0xff;
	src_2[0] = 0xff;

	src_2[1] = 0x00;
	src_2[2] = 0x00;
	src_2[3] = 0x00;
	src_2[4] = 0x08;

	src_2[5] = 0x7a;
	src_2[6] = 0x6d;
	src_2[7] = 0x1d;
	src_2[8] = 0x1d;

	src_2[9] = 0x12;
	src_2[10] = 0x23;
	src_2[11] = 0x34;
	src_2[12] = 0x45;
	src_2[13] = 0x56;
	src_2[14] = 0x67;
	src_2[15] = 0x78;
	src_2[16] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[46] = 0xff;
	src_1[47] = 0xff;
	src_2[0] = 0xff;
	src_2[1] = 0xff;

	src_2[2] = 0x00;
	src_2[3] = 0x00;
	src_2[4] = 0x00;
	src_2[5] = 0x08;

	src_2[6] = 0x7a;
	src_2[7] = 0x6d;
	src_2[8] = 0x1d;
	src_2[9] = 0x1d;

	src_2[10] = 0x12;
	src_2[11] = 0x23;
	src_2[12] = 0x34;
	src_2[13] = 0x45;
	src_2[14] = 0x56;
	src_2[15] = 0x67;
	src_2[16] = 0x78;
	src_2[17] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_1[47] = 0xff;
	src_2[0] = 0xff;
	src_2[1] = 0xff;
	src_2[2] = 0xff;

	src_2[3] = 0x00;
	src_2[4] = 0x00;
	src_2[5] = 0x00;
	src_2[6] = 0x08;

	src_2[7] = 0x7a;
	src_2[8] = 0x6d;
	src_2[9] = 0x1d;
	src_2[10] = 0x1d;

	src_2[11] = 0x12;
	src_2[12] = 0x23;
	src_2[13] = 0x34;
	src_2[14] = 0x45;
	src_2[15] = 0x56;
	src_2[16] = 0x67;
	src_2[17] = 0x78;
	src_2[18] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	src_2[0] = 0xff;
	src_2[1] = 0xff;
	src_2[2] = 0xff;
	src_2[3] = 0xff;

	src_2[4] = 0x00;
	src_2[5] = 0x00;
	src_2[6] = 0x00;
	src_2[7] = 0x08;

	src_2[8] = 0x7a;
	src_2[9] = 0x6d;
	src_2[10] = 0x1d;
	src_2[11] = 0x1d;

	src_2[12] = 0x12;
	src_2[13] = 0x23;
	src_2[14] = 0x34;
	src_2[15] = 0x45;
	src_2[16] = 0x56;
	src_2[17] = 0x67;
	src_2[18] = 0x78;
	src_2[19] = 0x89;


	success = search_packeg(src_1, src_2, 48, 48, &rdata_1, &rdata_2, &len_1, &len_2);
	if (success == NO)
		printf("ERROR\r\n");
	for (i = 0; i < len_1; i++)
		printf("0x%0X	", rdata_1[i]);
	for (i = 0; i < len_2; i++)
		printf("0x%0X	", rdata_2[i]);
	printf("\r\n");
	for (i = 0; i < 48; i++)
	{
		src_1[i] = 0;
		src_2[i] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	return 0;
}