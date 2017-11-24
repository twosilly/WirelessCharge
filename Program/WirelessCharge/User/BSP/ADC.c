#include "ADC.h"

unsigned short AD_I_OUT   = 0;
unsigned short AD_I_SENSE = 0;
unsigned short AD_V_IN    = 0;
unsigned short AD_TEMP    = 0;
unsigned short AD_S_OVP   = 0;

unsigned short AD_I_OUT_Average = 0;

void ADC_Init(void)
{
  uint8_t div;
  
  // 配置AD的管脚
  IOCFG->PA0 = IOCFG_ADM | IOCFG_PA0_FUNC_ADC_IN0;
  IOCFG->PA1 = IOCFG_ADM | IOCFG_PA1_FUNC_ADC_IN1;
  IOCFG->PA3 = IOCFG_ADM | IOCFG_PA3_FUNC_ADC_IN3;
  IOCFG->PA4 = IOCFG_ADM | IOCFG_PA4_FUNC_ADC_IN4;
  IOCFG->PA6 = IOCFG_ADM | IOCFG_PA6_FUNC_ADC_IN6;
  GPIOA->DIR = GPIOA->DIR & (~(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6));
  
	SYSCON->PRESETCTRL &= (~SYSCON_PRESETCTRL_ADC_RST_N);
  SYSCON->PRESETCTRL |= SYSCON_PRESETCTRL_ADC_RST_N;
  //Power down ADC
  SYSCON->PDRUNCFG |= SYSCON_PDRUNCFG_ADC_PD;
  //disable clock of ADC digital
  SYSCON->SYSAHBCLKCTRL &= (~SYSCON_SYSAHBCLKCTRL_ADCEN);
  // Power up ADC
  SYSCON->PDRUNCFG &= (~SYSCON_PDRUNCFG_ADC_PD);
  SYSCON->SYSAHBCLKCTRL |= SYSCON_SYSAHBCLKCTRL_ADCEN;
  ADC->CHSEL = 0x76543210;
  ADC->CR = ((SystemCoreClock / (1000000 * 16))<<8) | ADC_CR_START_SOFTWARE | 0x00000000;
  //insert a delay
  div = 0xFF;
  while(div--);
  /* SCMODE must be 1 */
  ADC->CR |= ADC_CR_SCMODE;
  /* wait ADC ready */
  while(!(ADC->STAT & ADC_STAT_ADCRDY));
  
  ADC->CR &= ~ADC_CR_SCMODE;
  ADC->CR |= 0x5B;      //使能通道
  ADC->CR |= ADC_CR_SCMODE;
  
  ADC->INTEN |= (uint16_t)0x005B; //使能中断
  
  printf("ADC Init Success!\r\n");
}

void Get_AllAdc(void)
{
  ADC->SSCR = 1;
	while((ADC->STAT & 0x5B) != 0x5B)
    ;

  AD_I_OUT   = ADC->DR[0] & ADC_DR_RESULT;    
  AD_I_SENSE = ADC->DR[1] & ADC_DR_RESULT;  
  AD_V_IN    = ADC->DR[3] & ADC_DR_RESULT;  
  AD_TEMP    = ADC->DR[4] & ADC_DR_RESULT;  
  AD_S_OVP   = ADC->DR[6] & ADC_DR_RESULT;  
  //printf("%04X-%04X-%04X-%04X-%04X\n", AD_I_OUT, AD_I_SENSE, AD_V_IN, AD_TEMP, AD_S_OVP);
}   


