




//RIFF块
typedef __packed struct
{
    unsigned int ChunkID;		   	//chunk id;这里固定为"RIFF",即0X46464952
    unsigned int ChunkSize ;		   	//集合大小;文件总大小-8
    unsigned int Format;	   			//格式;WAVE,即0X45564157
}ChunkRIFF ;
//fmt块
typedef __packed struct
{
    unsigned int ChunkID;		   	//chunk id;这里固定为"fmt ",即0X20746D66
    unsigned int ChunkSize ;		   	//子集合大小(不包括ID和Size);这里为:20.
    unsigned short AudioFormat;	  	//音频格式;0X01,表示线性PCM;0X11表示IMA ADPCM
	unsigned short NumOfChannels;		//通道数量;1,表示单声道;2,表示双声道;
	unsigned int SampleRate;			//采样率;0X1F40,表示8Khz
	unsigned int ByteRate;			//字节速率; 
	unsigned short BlockAlign;			//块对齐(字节); 
	unsigned short BitsPerSample;		//单个采样数据大小;4位ADPCM,设置为4
//	unsigned short ByteExtraData;		//附加的数据字节;2个; 线性PCM,没有这个参数
}ChunkFMT;	   
//fact块 
typedef __packed struct 
{
    unsigned int ChunkID;		   	//chunk id;这里固定为"fact",即0X74636166;
    unsigned int ChunkSize ;		   	//子集合大小(不包括ID和Size);这里为:4.
    unsigned int NumOfSamples;	  	//采样的数量; 
}ChunkFACT;
//LIST块 
typedef __packed struct 
{
    unsigned int ChunkID;		   	//chunk id;这里固定为"LIST",即0X74636166;
    unsigned int ChunkSize ;		   	//子集合大小(不包括ID和Size);这里为:4. 
}ChunkLIST;

//data块 
typedef __packed struct 
{
    unsigned int ChunkID;		   	//chunk id;这里固定为"data",即0X5453494C
    unsigned int ChunkSize ;		   	//子集合大小(不包括ID和Size) 
}ChunkDATA;

//wav头
typedef __packed struct
{ 
	ChunkRIFF riff;	//riff块
	ChunkFMT fmt;  	//fmt块
//	ChunkFACT fact;	//fact块 线性PCM,没有这个结构体	 
	ChunkDATA data;	//data块		 
}__WaveHeader; 

//wav 播放控制结构体
typedef __packed struct
{ 
    unsigned short audioformat;			//音频格式;0X01,表示线性PCM;0X11表示IMA ADPCM
	unsigned short nchannels;				//通道数量;1,表示单声道;2,表示双声道; 
	unsigned short blockalign;				//块对齐(字节);  
	unsigned int datasize;				//WAV数据大小 

    unsigned int totsec ;				//整首歌时长,单位:秒
    unsigned int cursec ;				//当前播放时长
	
    unsigned int bitrate;	   			//比特率(位速)
	unsigned int samplerate;				//采样率 
	unsigned short bps;					//位数,比如16bit,24bit,32bit
	
	unsigned int datastart;				//数据帧开始的位置(在文件里面的偏移)
}__wavctrl; 
void recoder_wav_init(__WaveHeader* wavhead , unsigned int fs) ;
void recoder_wav_update(__WaveHeader* wavhead , unsigned int datalen);


