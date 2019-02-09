#include "fatfs_write_wav.h"
#include "../inc/ringbuffer.h"
#include <speex/speex.h>

#define ENCODED_FRAME_SIZE      20
#define IN_SOURE_FRAME_SIZE			160


__WaveHeader wav_head;
FIL wav_file;
FIL speex_file;
UINT br,bw;
ringbuffer buffer;
u8 dmabuffer[2048];
u8*data_1,*data_2;
u32 data1_len,data2_len;



char out_bytes[ENCODED_FRAME_SIZE];


extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern SpeexBits bits;
extern void *enc_state;
unsigned char close_wav_file()
{
	retSD = f_lseek(&wav_file,0);
	if(retSD != FR_OK)
		return retSD;
	retSD = f_write(&wav_file, (const void*)&wav_head, sizeof(__WaveHeader), &bw);
	if(retSD != FR_OK)
		return retSD;
	retSD = f_close(&wav_file);
	return retSD;
	
}


unsigned char create_wav_file(char*filename,unsigned int fs)
{
	retSD = f_open(&wav_file, filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	if(retSD != FR_OK)
		return retSD;
	recoder_wav_init(&wav_head,fs);
	retSD = f_write(&wav_file, (const void*)&wav_head, sizeof(__WaveHeader), &bw);
	return retSD;
}


unsigned char write_wav_file(unsigned char*wavdata,unsigned int len)
{
	retSD = f_write(&wav_file, wavdata, len, &bw);
	if(retSD != FR_OK)
		return retSD;
	recoder_wav_update(&wav_head,len);
	return retSD;
}



unsigned char create_speex_file(char*filename)
{
	return f_open(&speex_file, filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
}
unsigned char write_speex_file(unsigned char*wavdata,unsigned int len)
{
	return f_write(&speex_file, wavdata, len, &bw);
}
unsigned char close_speex_file()
{
	return f_close(&speex_file);
}




void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	write_buffer_data(&buffer,dmabuffer,1024);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	write_buffer_data(&buffer,&dmabuffer[1024],1024);
}


char init_file[20];
char* file_ptr = init_file;
unsigned char initial_recoder(char*filename,unsigned int fs)
{
	file_ptr = init_file;
	while(*filename)
		*(file_ptr++) = *(filename++);
	*(file_ptr++) = '.';
	*(file_ptr++) = 'w';
	*(file_ptr++) = 'a';
	*(file_ptr++) = 'v';
	*(file_ptr) = 0;
	htim3.Instance->ARR = (96000000/fs) - 1;
	initial_buffer(&buffer,YES,25*1024);
	retSD = create_wav_file(init_file,fs);
	if(retSD != FR_OK)
		return retSD;
	file_ptr -= 3;
	*(file_ptr++) = 's';
	*(file_ptr++) = 'p';
	*(file_ptr++) = 'e';
	*(file_ptr++) = 'e';
	*(file_ptr++) = 'x';
	*(file_ptr) = 0;
	return create_speex_file(init_file);
}

void start_recoder()
{
	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)dmabuffer,1024);
}


u8 stop_recoder()
{
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop(&htim3);
	close_wav_file();
	return close_speex_file();
}


unsigned char rdata[1024*20];
unsigned int rdata_len = 0;
unsigned int wdata_len = 0;

unsigned char recoder_outdata[1024*10];
unsigned int recoder_out_data_loc = 0;

unsigned int rd_i;


unsigned char Encoder_Flag = 0;

void tick_recoder()
{
		__disable_irq();
		get_unread_ptr(&buffer,&data_1,&data_2,&data1_len,&data2_len,NO);
		__enable_irq();
		if(data1_len != 0)
		{
			for(rd_i = 0;rd_i<data1_len;rd_i++)
				rdata[wdata_len++] = data_1[rd_i];
//			/* Flush all the bits in the struct so we can encode a new frame */
//			speex_bits_reset(&bits);
//			/* Encode the frame */
//			speex_encode_int(enc_state, (spx_int16_t*)data_1, &bits);
//			/* Copy the bits to an array of char that can be decoded */
//			speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
			write_wav_file(data_1,data1_len);
		}
		if(data2_len != 0)
		{
			for(rd_i = 0;rd_i<data2_len;rd_i++)
				rdata[wdata_len++] = data_2[rd_i];
//			/* Flush all the bits in the struct so we can encode a new frame */
//			speex_bits_reset(&bits);
//			/* Encode the frame */
//			speex_encode_int(enc_state, (spx_int16_t*)data_2, &bits);
//			/* Copy the bits to an array of char that can be decoded */
//			speex_bits_write(&bits, (char *)out_bytes, ENCODED_FRAME_SIZE);
			write_wav_file(data_2,data2_len);
		}
		Encoder_Flag = 0;
		//speex_bits_reset(&bits);
		while((wdata_len - rdata_len) / (IN_SOURE_FRAME_SIZE*2) > 0)
		{
			Encoder_Flag = 1;
			speex_bits_reset(&bits);
			/* Encode the frame */
			speex_encode_int(enc_state, (spx_int16_t*)&rdata[rdata_len], &bits);
			/* Copy the bits to an array of char that can be decoded */
			speex_bits_write(&bits, (char *)&recoder_outdata[recoder_out_data_loc], ENCODED_FRAME_SIZE);
			rdata_len+=IN_SOURE_FRAME_SIZE*2;
			recoder_out_data_loc+=ENCODED_FRAME_SIZE;
		}
		if(Encoder_Flag == 1)
		{
			
			//recoder_out_data_loc = speex_bits_write(&bits, (char *)recoder_outdata, ENCODED_FRAME_SIZE*1024*100);
			
			for(rd_i = 0; rd_i < (wdata_len - rdata_len); rd_i++)
				rdata[rd_i] = rdata[rdata_len + rd_i];

			wdata_len = (wdata_len - rdata_len);
			rdata_len = 0;
			
			write_speex_file(recoder_outdata,recoder_out_data_loc);
			recoder_out_data_loc = 0;
		}
}








