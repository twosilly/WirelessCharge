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
  SystemCoreClockUpdate(); 
	SYSCON->SYSAHBCLKCTRL |= (SYSCON_SYSAHBCLKCTRL_PWMEN | SYSCON_SYSAHBCLKCTRL_GPIOAEN);
  IOCFG->PA7 = IOCFG_DEFAULT | IOCFG_PA7_FUNC_PWM_OUT1;
  IOCFG->PA8 = IOCFG_DEFAULT | IOCFG_PA8_FUNC_PWM_OUT0;
  GPIOA->DIR = GPIOA->DIR | GPIO_PIN_7 | GPIO_PIN_8;
  
  //互补PWM输出、使能GPIO初始化
  IOCFG->PB5 = IOCFG_DEFAULT;
  GPIOB->DIR = GPIOB->DIR | GPIO_PIN_5;
  MagneticField_Disable();
  
  SYSCON->PRESETCTRL &= ~SYSCON_PRESETCTRL_PWM_RST_N;
  SYSCON->PRESETCTRL |= SYSCON_PRESETCTRL_PWM_RST_N;
  
  PWM->CTRL = (PWM->CTRL & (~PWM_CTRL_PRSC)) | (uint32_t)(0x0 << 5);  //1分频
  PWM->CNFG = PWM_CNFG_EDG;
  PWM->CNFG &= ~(0x01 << 1);  //互补输出
  PWM->CTRL |= PWM_CTRL_LDOK;
  PWM->CMOD = 143;  //143  71
  PWM->VAL0 = 71;
  
  PWM->DTIM0 = 2;  //设置死区
  PWM->DTIM1 = 4;
  PWM_Cmd(ENABLE); //开启输出
  
  //MagneticField_Enable();  //调试用
  printf("Magnetic Field Init Success!\r\n");
}
