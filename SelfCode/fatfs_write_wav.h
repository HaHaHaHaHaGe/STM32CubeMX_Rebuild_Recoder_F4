#include "wav.h"
#include "fatfs.h"

unsigned char close_wav_file(void);
unsigned char create_wav_file(char*filename,unsigned int fs);
unsigned char write_wav_file(unsigned char*wavdata,unsigned int len);
unsigned char initial_recoder(char*filename,unsigned int fs);
void start_recoder(void);
unsigned char stop_recoder(void);
void tick_recoder(void);
