#include "../inc/packeg_taken.h"
//原始数据包包头
const u8 packet_header[4] = { 0xFF,0xFF,0xFF,0xFF };


u8 search_packeg(u8*src1, u8*src2, u32 len1, u32 len2, u8**rdata_1,u8**rdata_2,u32*rlen_1,u32*rlen_2)
{
	u8 n = 0;
	u8* src_1 = src1;
	u8* src_2 = src2;
	u32 len_1 = len1;
	u32 len_2 = len2;
	u32 crc32_1 = 0;
	u32 crc32_2;
	u32 len = 0;
	while (len_1--)
		if (*(src_1++) == packet_header[n])
		{
			n++;
			if (n == sizeof(packet_header))
			{
				if (src_1 + 4 > src1 + len1)
				{
#ifdef Little_Endian
					if (src_1 + 1 == src1 + len1 && len2 >= 3)
					{
						((u8*)&len)[3] = *src_1;
						((u8*)&len)[2] = *src2;
						((u8*)&len)[1] = *(src2 + 1);
						((u8*)&len)[0] = *(src2 + 2);
					}
					else if (src_1 + 2 == src1 + len1 && len2 >= 2)
					{
						((u8*)&len)[3] = *src_1;
						((u8*)&len)[2] = *(src_1 + 1);
						((u8*)&len)[1] = *(src2);
						((u8*)&len)[0] = *(src2 + 1);
					}
					else if (src_1 + 3 == src1 + len1 && len2 >= 1)
					{
						((u8*)&len)[3] = *src_1;
						((u8*)&len)[2] = *(src_1 + 1);
						((u8*)&len)[1] = *(src_1 + 2);
						((u8*)&len)[0] = *(src2);
					}
					else
					{
						len = *((u32*)src2);
						len = basic_swap(len);
					}
#else
					if (src_1 + 1 == src1 + len1 && len2 >= 3)
					{
						((u8*)&len)[0] = *src_1;
						((u8*)&len)[1] = *src2;
						((u8*)&len)[2] = *(src2 + 1);
						((u8*)&len)[3] = *(src2 + 2);
					}
					else if (src_1 + 2 == src1 + len1 && len2 >= 2)
					{
						((u8*)&len)[0] = *src_1;
						((u8*)&len)[1] = *(src_1 + 1);
						((u8*)&len)[2] = *(src2);
						((u8*)&len)[3] = *(src2 + 1);
					}
					else
					{
						((u8*)&len)[0] = *src_1;
						((u8*)&len)[1] = *(src_1 + 1);
						((u8*)&len)[2] = *(src_1 + 2);
						((u8*)&len)[3] = *(src2);
					}
#endif
					if ((src_1 + 4 - src1 - len1) + 4 + len > len2)
						return NO;
					if (len > __REDUCTION_DATA_LENGTH__)
						return NO;
					crc32_1 = *(u32*)(src2 + (src_1 + 4 - src1 - len1));
#ifdef Little_Endian
					crc32_1 = basic_swap(crc32_1);
#endif
					crc32_2 = crc32_16bytes((u32*)(src2 + (src_1 + 4 - src1 - len1) + 4), len, 0);
					if (crc32_2 == crc32_1)
					{
						*rlen_1 = len;
						*rdata_1 = (src2 + (src_1 + 4 - src1 - len1) + 4);
						*rlen_2 = NULL_PTR;
						*rdata_2 = 0;
						return YES;
					}
					return NO;
				}
				len = *(u32*)src_1;
#ifdef Little_Endian
				len = basic_swap(len);
#endif
				if (len > __REDUCTION_DATA_LENGTH__)
					return NO;
				if (src_1 + 4 + 4 > src1 + len1)
				{
#ifdef Little_Endian
					if (src_1 + 4 + 1 == src1 + len1 && len2 >= 3)
					{
						((u8*)&crc32_1)[3] = *(src_1 + 4);
						((u8*)&crc32_1)[2] = *src2;
						((u8*)&crc32_1)[1] = *(src2 + 1);
						((u8*)&crc32_1)[0] = *(src2 + 2);
					}
					else if (src_1 + 4 + 2 == src1 + len1 && len2 >= 2)
					{
						((u8*)&crc32_1)[3] = *(src_1 + 4);
						((u8*)&crc32_1)[2] = *(src_1 + 4 + 1);
						((u8*)&crc32_1)[1] = *(src2);
						((u8*)&crc32_1)[0] = *(src2 + 1);
					}
					else if (src_1 + 4 + 3 == src1 + len1 && len2 >= 1)
					{
						((u8*)&crc32_1)[3] = *(src_1 + 4);
						((u8*)&crc32_1)[2] = *(src_1 + 4 + 1);
						((u8*)&crc32_1)[1] = *(src_1 + 4 + 2);
						((u8*)&crc32_1)[0] = *(src2);
					}
					else
					{
						crc32_1 = *((u32*)src2);
						crc32_1 = basic_swap(crc32_1);
					}
#else
					if (src_1 + 4 + 1 == src1 + len1 && len2 >= 3)
					{
						((u8*)&crc32_1)[0] = *(src_1 + 4);
						((u8*)&crc32_1)[1] = *src2;
						((u8*)&crc32_1)[2] = *(src2 + 1);
						((u8*)&crc32_1)[3] = *(src2 + 2);
					}
					else if (src_1 + 4 + 2 == src1 + len1 && len2 >= 2)
					{
						((u8*)&crc32_1)[0] = *(src_1 + 4);
						((u8*)&crc32_1)[1] = *(src_1 + 4 + 1);
						((u8*)&crc32_1)[2] = *(src2);
						((u8*)&crc32_1)[3] = *(src2 + 1);
					}
					else
					{
						((u8*)&crc32_1)[0] = *(src_1 + 4);
						((u8*)&crc32_1)[1] = *(src_1 + 4 + 1);
						((u8*)&crc32_1)[2] = *(src_1 + 4 + 2);
						((u8*)&crc32_1)[3] = *(src2);
					}
#endif
					if ((src_1 + 4 + 4 - src1 - len1) + len > len2)
						return NO;
					crc32_2 = crc32_16bytes((u32*)(src2 + (src_1 + 4 + 4 - src1 - len1)), len, 0);
					if (crc32_2 == crc32_1)
					{
						*rlen_1 = len;
						*rdata_1 = (src2 + (src_1 + 4 - src1 - len1) + 4);
						*rlen_2 = NULL_PTR;
						*rdata_2 = 0;
						return YES;
					}
					return NO;
				}
				crc32_1 = *(u32*)(src_1 + 4);
#ifdef Little_Endian
				crc32_1 = basic_swap(crc32_1);
#endif

				if ((src_1 + 4 + 4 + len) > src1 + len1)
					if(len - (src1 + len1 - (src_1 + 4 + 4)) > len2)
						return NO;
					else
					{
						crc32_2 = crc32_16bytes((u32*)(src_1 + 4 + 4), (src1 + len1 - (src_1 + 4 + 4)), 0);
						crc32_2 = crc32_16bytes((u32*)src2, len - (src1 + len1 - (src_1 + 4 + 4)), crc32_2);
						if (crc32_2 == crc32_1)
						{
							*rlen_1 = (src1 + len1 - (src_1 + 4 + 4));
							*rdata_1 = (src_1 + 4 + 4);
							*rlen_2 = len - (src1 + len1 - (src_1 + 4 + 4));
							*rdata_2 = src2;
							return YES;
						}
						return NO;
					}
				crc32_2 = crc32_16bytes((u32*)(src_1 + 4 + 4), len, 0);
				if (crc32_2 == crc32_1)
				{
					*rlen_1 = len;
					*rdata_1 = (src_1 + 4 + 4);
					*rlen_2 = NULL_PTR;
					*rdata_2 = 0;
					return YES;
				}
				return NO;
			}
		}
	
		
		
		
	if (len2 >= 3 + 8)
		if ((*(src1 + len1 - 1) == packet_header[0]) && (*(src2) == packet_header[1]) && (*(src2 + 1) == packet_header[2]) && (*(src2 + 2) == packet_header[3]))
			src_2 = src2 + 3;

	if (len2 >= 2 + 8)
		if ((*(src1 + len1 - 1) == packet_header[1]) && (*(src1 + len1 - 2) == packet_header[0]) && (*(src2) == packet_header[2]) && (*(src2 + 1) == packet_header[3]))
			src_2 = src2 + 2;

	if (len2 >= 1 + 8)
		if ((*(src1 + len1 - 1) == packet_header[2]) && (*(src1 + len1 - 2) == packet_header[1]) && (*(src1 + len1 - 3) == packet_header[0]) && (*(src2) == packet_header[3]))
			src_2 = src2 + 1;
	if (src_2 != src2)
	{
		if ((src_2 - src2) + 8 > len2)
			return NO;
		len = *(u32*)src_2;
		crc32_1 = *(u32*)(src_2 + 4);
#ifdef Little_Endian
		len = basic_swap(len);
		crc32_1 = basic_swap(crc32_1);
#endif
		if ((src_2 - src2) + 8 + len > len2)
			return NO;
		crc32_2 = crc32_16bytes((u32*)(src_2 + 8), len, 0);
		if (crc32_2 == crc32_1)
		{
			*rlen_1 = len;
			*rdata_1 = src_2 + 8;
			*rlen_2 = NULL_PTR;
			*rdata_2 = 0;
			return YES;
		}
		return NO;
	}

	n = 0;
	while (len_2--)
		if (*(src_2++) == packet_header[n])
		{
			n++;
			if (n == sizeof(packet_header))
			{
				if (src_2 + 8 > src2 + len2)
					return NO;
				len = *(u32*)src_2;
				crc32_1 = *(u32*)(src_2 + 4);
#ifdef Little_Endian
				len = basic_swap(len);
				crc32_1 = basic_swap(crc32_1);
#endif
				if(len > __REDUCTION_DATA_LENGTH__)
					return NO;
				if (src_2 + 8 + len > src2 + len2)
					return NO;
				crc32_2 = crc32_16bytes((u32*)(src_2 + 8), len, 0);
				if (crc32_2 == crc32_1)
				{
					*rlen_1 = len;
					*rdata_1 = src_2 + 8;
					*rlen_2 = NULL_PTR;
					*rdata_2 = 0;
					return YES;
				}
				return NO;
			}
		}
	return NO;
}

u8 unpacking(u8* src1,u8* src2,u32 len1,u32 len2)
{
	u8 Success;
	u8 *data_1, *data_2;
	u32 len_1, len_2;

	if (src1 == NULL_PTR)
		return FAIL;
	Success = search_packeg(src1, src2, len1, len2, &data_1, &data_2, &len_1, &len_2);
	
	if (Success == YES)
	{

	}
	return NO;
}


