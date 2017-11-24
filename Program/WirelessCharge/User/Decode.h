#ifndef __DECODE_H
#define __DECODE_H


#include "Zhuhc.h"

void Decode_Init(void);
void Decode(void);

extern unsigned char rightData[30], rightLength;
extern unsigned char rightDone, startData;

#endif

