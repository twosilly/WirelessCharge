#ifndef __ZHUHC_H
#define __ZHUHC_H

#include "gvm32f030.h"
#include "LED.h"
#include "BSP.h"
#include "UART.h"
#include "SysTick.h"
#include "Control.h"
#include "MagneticField.h"
#include "Qi.h"
#include "Decode.h"
#include "ADC.h"
#include "PID.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define SEND_PORT      UART0

void CX_SendImage(void *imgaddr, uint32_t imgsize);
void CX_SendCCD(void *ccdaddr, uint32_t ccdsize);
void CX_SendWare(void *wareaddr, uint32_t waresize);

#endif

