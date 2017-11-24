#ifndef __ADC_H
#define __ADC_H

#include "Zhuhc.h"

#define VIN_9V_AD_VALUE     1840
#define VIN_4V5_AD_VALUE    1178

void ADC_Init(void);
void Get_AllAdc(void);

extern unsigned short AD_I_OUT;
extern unsigned short AD_I_SENSE;
extern unsigned short AD_V_IN;
extern unsigned short AD_TEMP;
extern unsigned short AD_S_OVP;

extern unsigned short AD_I_OUT_Average;
extern unsigned short AD_S_OVP_Average;
extern unsigned short AD_V_IN_Average;

#endif


