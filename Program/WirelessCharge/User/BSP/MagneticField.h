#ifndef __MAGNETIC_FIELD_H
#define __MAGNETIC_FIELD_H

#include "Zhuhc.h"

#define MagneticField_Enable()    GPIOB->SET = GPIO_PIN_5
#define MagneticField_Disable()   GPIOB->CLR = GPIO_PIN_5

void MagneticField_Init(void);

#endif

