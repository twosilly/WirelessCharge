#include "MagneticField.h"

void PWM_Cmd(FunctionalState NewState)
{
  if(NewState != DISABLE)
  {
    /* Enable PWM Output */
    PWM->CTRL |= (0x3F000000 | PWM_CTRL_PWMEN);
    PWM->ICCTRL = PWM_ICCTRL_PAD_EN;
    PWM->OUT = 0x3F;
    PWM->CNTRINI = 0;
    
    PWM->CTRL |= PWM_CTRL_LDOK;
  }
  else
  {
    /* Disable PWM_Output */
    PWM->CTRL &= (~(0x3F000000 | PWM_CTRL_PWMEN));
    PWM->ICCTRL = 0;
    PWM->OUT = 0;
    
    PWM->CTRL |= PWM_CTRL_LDOK;
  }
}

void MagneticField_Init(void)
{
  //互补PWM输出、使能GPIO初始化
  IOCFG->PB5 = IOCFG_DEFAULT;
  GPIOB->DIR = GPIOA->DIR | GPIO_PIN_5;
  MagneticField_Disable();
  
  SYSCON->PRESETCTRL &= ~SYSCON_PRESETCTRL_PWM_RST_N;
  SYSCON->PRESETCTRL |= SYSCON_PRESETCTRL_PWM_RST_N;
  
  PWM->CTRL = (PWM->CTRL & (~PWM_CTRL_PRSC)) | (uint32_t)(0x3 << 5);  //8分频
  PWM->CNFG = PWM_CNFG_EDG;
  PWM->CNFG &= ~(0x01 << 1);  //互补输出
  PWM->CMOD = 10000;
  PWM->CTRL |= PWM_CTRL_LDOK;
  PWM->VAL0 = 5000;
  
  PWM->DTIM0 = 200; //设置死区
  PWM->DTIM1 = 0;
  PWM_Cmd(DISABLE); //关闭输出
}
