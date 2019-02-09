
#include "wav.h"


void recoder_wav_init(__WaveHeader* wavhead , unsigned int fs) //初始化WAV头			   
{
	wavhead->riff.ChunkID=0X46464952;	//"RIFF"
	wavhead->riff.ChunkSize=0;			//还未确定,最后需要计算
	wavhead->riff.Format=0X45564157; 	//"WAVE"
	wavhead->fmt.ChunkID=0X20746D66; 	//"fmt "
	wavhead->fmt.ChunkSize=16; 			//大小为16个字节
	wavhead->fmt.AudioFormat=0X01; 		//0X01,表示PCM;0X01,表示IMA ADPCM
 	wavhead->fmt.NumOfChannels=1;		//双声道
 	wavhead->fmt.SampleRate=fs;		//16Khz采样率 采样速率
 	wavhead->fmt.ByteRate=wavhead->fmt.SampleRate*2;//字节速率=采样率*通道数*(ADC位数/8)
 	wavhead->fmt.BlockAlign=2;			//块大小=通道数*(ADC位数/8)
 	wavhead->fmt.BitsPerSample=16;		//16位PCM
   	wavhead->data.ChunkID=0X61746164;	//"data"
 	wavhead->data.ChunkSize=0;			//数据大小,还需要计算  
}

void recoder_wav_update(__WaveHeader* wavhead , unsigned int datalen)
{
	wavhead->data.ChunkSize+=datalen;		//数据大小
	wavhead->riff.ChunkSize=wavhead->data.ChunkSize+36;		//整个文件的大小-8;
}
