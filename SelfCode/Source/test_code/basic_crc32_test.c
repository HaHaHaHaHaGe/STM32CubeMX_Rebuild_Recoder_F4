#include "test_main.h"


u32 ______crc32_16bytes______(u32 NumBytes)
{
	u32 crc;
	u8* data;
	u32 i;
	u32 randomNumber = 0x27121978;
	 // initialize
	data = basic_malloc(NumBytes);
	for (i = 0; i < NumBytes; i++)
	{
		data[i] = randomNumber & 0xFF;
		// simple LCG, see http://en.wikipedia.org/wiki/Linear_congruential_generator
		randomNumber = 1664525 * randomNumber + 1013904223;
	}

	crc = crc32_16bytes((u32*)data, NumBytes,0);

  return crc;
}
