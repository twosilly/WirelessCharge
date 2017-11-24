#include "QC.h"

//QC快充协议

/*
 * 1.在D+上加载0.325V的电压，维持1.25S。
 * 2.设置D+上的电压为3.3V，D-上 的电压为0.6V，充电器输出9v电压。
 */


void QC_Init(void)
{
  IOCFG->PB0 = IOCFG_DEFAULT | IOCFG_PB0_FUNC_PB0;
  IOCFG->PB1 = IOCFG_DEFAULT | IOCFG_PB1_FUNC_PB1;
  GPIOB->DIR = GPIOB->DIR | GPIO_PIN_0 | GPIO_PIN_1;
  GPIOB->SET |= GPIO_PIN_0;
  GPIOB->CLR |= GPIO_PIN_1;
  
  Power_Up();
}


