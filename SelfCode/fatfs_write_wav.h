#include "wav.h"
#include "fatfs.h"


unsigned char close_wav_file(void);
unsigned char create_wav_file(char*filename,unsigned int fs);
unsigned char write_wav_file(unsigned char*wavdata,unsigned int len);
unsigned char initial_recoder(char*filename,unsigned int fs);
void start_recoder(void);
unsigned char stop_recoder(void);
void tick_recoder(void);
void Speex_Init(void);
unsigned char wav_recorder(unsigned char key,unsigned int fs);
unsigned char gad_recorder(unsigned char key,unsigned int time,unsigned int fs);
void recoder_new_pathname(char *pname);
void end_recoder();
