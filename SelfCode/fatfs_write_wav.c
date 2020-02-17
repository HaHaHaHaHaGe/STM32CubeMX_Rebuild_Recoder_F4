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
u32 speexdata_len = 0;
char filepath[64];
char filepath2[64];
char out_bytes[ENCODED_FRAME_SIZE];


extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;



SpeexBits bits;/* Holds bits so they can be read and written by the Speex routines */
void *enc_state, *dec_state;/* Holds the states of the encoder & the decoder */
int quality = 4, complexity=1, vbr=0, enh=1;/* SPEEX PARAMETERS, MUST REMAINED UNCHANGED */
int frame_size;
void Speex_Init(void)
{
  /* Speex encoding initializations */ 
  speex_bits_init(&bits);
  enc_state = speex_encoder_init(&speex_nb_mode);
  speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &vbr);
  speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY,&quality);
  speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
	
	speex_encoder_ctl(enc_state,SPEEX_GET_SAMPLING_RATE,&frame_size);

  /* speex decoding intilalization */
  dec_state = speex_decoder_init(&speex_nb_mode);
  speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &enh);
}



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
	strcpy(filepath,filename);
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
//	f_lseek(&wav_file,0);
//	retSD = f_write(&wav_file, (const void*)&wav_head, sizeof(__WaveHeader), &bw);
//	f_lseek(&wav_file,wav_head.riff.ChunkSize);
	return retSD;
}



unsigned char create_speex_file(char*filename)
{
	strcpy(filepath2,filename);
	return f_open(&speex_file, filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
}
unsigned char write_speex_file(unsigned char*wavdata,unsigned int len)
{
	retSD = f_write(&speex_file, wavdata, len, &bw);
	f_sync(&speex_file);
	return retSD;
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
	speexdata_len = 0;
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
	*(file_ptr++) = 'w';
	*(file_ptr++) = 'z';
	*(file_ptr++) = 'r';
	*(file_ptr) = 0;
	return create_speex_file(init_file);
}




u8 stop_recoder()
{
	u8 wav_res,speex_res;
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop(&htim3);
	wav_res = close_wav_file();
	speex_res = close_speex_file();
	deinitial_buffer(&buffer);
	return wav_res || speex_res;
}


unsigned char rdata[1024*20];
unsigned int rdata_len = 0;
unsigned int wdata_len = 0;

unsigned char recoder_outdata[1024*10];
unsigned int recoder_out_data_loc = 0;

unsigned int rd_i;


unsigned char Encoder_Flag = 0;

unsigned char AM_Factor = 0;


void start_recoder()
{
	speexdata_len = 0;
	rdata_len = 0;
	wdata_len = 0;
	recoder_out_data_loc = 0;
	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)dmabuffer,1024);
}



void tick_recoder()
{
		u16 wav16bits;
		__disable_irq();
		get_unread_ptr(&buffer,&data_1,&data_2,&data1_len,&data2_len,NO);
		__enable_irq();
		if(data1_len != 0)
		{
			for(rd_i = 0;rd_i<data1_len;rd_i+=2)
			{
				wav16bits = data_1[rd_i] | (data_1[rd_i + 1] << 8);
				*(u16*)&data_1[rd_i] = (wav16bits - 1894) << AM_Factor; //4
				rdata[wdata_len++] = data_1[rd_i];
				rdata[wdata_len++] = data_1[rd_i + 1];
			}
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
			for(rd_i = 0;rd_i<data2_len;rd_i+=2)
			{
				wav16bits = data_2[rd_i] | (data_2[rd_i + 1] << 8);
				*(u16*)&data_2[rd_i] = (wav16bits - 1984) << AM_Factor;
				rdata[wdata_len++] = data_2[rd_i];
				rdata[wdata_len++] = data_2[rd_i + 1];
			}
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
			SendspeexData_and_FixWifiData(recoder_outdata,recoder_out_data_loc,speexdata_len);
			speexdata_len += recoder_out_data_loc;
			write_speex_file(recoder_outdata,recoder_out_data_loc);
			recoder_out_data_loc = 0;
			
		}
}



//通过时间获取文件名
//仅限在SD卡保存,不支持FLASH DISK保存
//组合成:形如"0:RECORDER/REC20120321210633.wav"的文件名
void recoder_new_pathname(char *pname)
{	 
	u8 res;					 
	u16 index=0;
	while(index<0XFFFF)
	{
		sprintf((char*)pname,"0:RECORDER/REC%05d.wav",index);
		res=f_open(&wav_file,(const TCHAR*)pname,FA_READ);//尝试打开这个文件
		f_close(&wav_file);
		if(res==FR_NO_FILE)
		{
			sprintf((char*)pname,"0:RECORDER/REC%05d",index);
			break;		//该文件名不存在=正是我们需要的.
		}
		
		index++;
	}
} 



