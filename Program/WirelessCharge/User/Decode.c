#include "Decode.h"

unsigned short revData[30] = {0}, revLength = 0;
bool revDone = false;
unsigned char rightData[30] = {0}, rightLength = 0;
unsigned char rightDone = 0, startData = 0;

unsigned long I_Sense_Count = 0;

void Decode_Init(void)
{
  IOCFG->PA5 = IOCFG_DEFAULT;// | IOCFG_SMODE;
  GPIOA->DIR = GPIOA->DIR & (~GPIO_PIN_5);
  //SYSCON->IOCONFIGCLKDIV0 = 2500;
  IOCFG->PA1 = IOCFG_DEFAULT;
  GPIOA->DIR = GPIOA->DIR & (~GPIO_PIN_1);
  
  GPIOA->IS  &= ~GPIO_PIN_5;
  GPIOA->IBE |= GPIO_PIN_5;
  
  GPIOA->IS  &= ~GPIO_PIN_1;
  GPIOA->IBE |= GPIO_PIN_1;
  
  NVIC_SetPriority(GPIOA_IRQn, 0);
	NVIC_EnableIRQ(GPIOA_IRQn);	
  GPIOA->IE &= ~GPIO_PIN_5;
  GPIOA->IE &= ~GPIO_PIN_1;
  
  SYSCON->PRESETCTRL &= ~SYSCON_PRESETCTRL_TIM0_RST_N;
  SYSCON->PRESETCTRL |= SYSCON_PRESETCTRL_TIM0_RST_N;
  TIM0->PR = 25 - 1;     //1MHz
  TIM0->MR0 = 5000 - 1;  //5ms
  TIM0->TC = 0;
  TIM0->MCR = (TIM0->MCR & 0xFFFFFFF8) | 0x3;
  NVIC_SetPriority(TIM0_IRQn, 0);
	NVIC_EnableIRQ(TIM0_IRQn);
  TIM0->TCR |= (TIM_TCR_CEN); //Enable
  
  printf("Decode Init Success!\r\n");
}

bool Data_Check(unsigned short data)
{
  unsigned char i;
  unsigned short TempData;
  bool parity = false; 
  TempData = data;
  for(i = 0; i < 9; i++)
  {
    if(TempData & 0x1)
    {
      parity = !parity;
    }
    TempData = TempData >> 1;
  }
  return parity;
}

void Decode(void)
{
  unsigned char i = 0;
  if(revDone == true)
  { 
    //GPIOA->IE &= ~GPIO_PIN_5;
    
    //printf("%02X %02X\r\n", revData[0], revData[1]);
    
    for(i = 0; i < revLength; i++)
    {
      if(Data_Check(revData[i]))
        rightData[i] = (revData[i] & 0xFF);
      else
      {
        revDone = false;
        return;
      }
    }
    rightLength = revLength;
    
    uint8_t checkTemp = 0;
    if(rightLength >= 3)
    {
      for(i = 0; i < (rightLength - 1); i++)
        checkTemp ^= rightData[i];
      if(checkTemp != rightData[rightLength - 1])
      {
        revDone = false;
        return;
      }
      else
      {
        rightLength--;
        if(Get_MessageSize(rightData[0]) == rightLength - 1) //增加长度检验
          rightDone = 1;
        revDone = false;
        
//        if(rightData[0] == 0x51)
//        {
//          for(i = 0; i < rightLength; i++)
//          {
//            printf("%02X ", rightData[i]);
//          }
//          printf("\r\n");
//        }
      }
    }
    
    //GPIOA->IE |= GPIO_PIN_5;
  }
  
//  if(rightDone == true)
//  {
//    for(i = 0; i < rightLength; i++)
//    {
//      printf("%X ", rightData[i]);
//    }
//    printf("\r\n");
//    
//    rightDone = false;
//  }
}

static unsigned char Preamble_Count = 0, Status = 0;
static uint32_t Temp = 0, Temp_Value1 = 0;
static unsigned char TempByte_Count = 0;
static unsigned char startFlag = 0, startFlag1 = 0;

void GPIOA_IRQHandler(void)
{
  if(GPIOA->MIS & GPIO_PIN_1)
  {
    GPIOA->IC = GPIO_PIN_1;
    I_Sense_Count++;
    return;
  }
  
  GPIOA->IC = GPIO_PIN_5;
  
  if(Status == 0) //同步
  {
    if(Preamble_Count == 0)
    {
      TIM0->TC = 0;
      Preamble_Count++;
      Temp_Value1 = 0;
      TempByte_Count = 0;
      startFlag = 0;
      revDone = false;
      Temp_Value1 = 0;
      startFlag1 = 0;
    }
    else
    {
      Temp = TIM0->TC;
      TIM0->TC = 0;
      if(Temp > 150 && Temp < 350)
      {
        Temp_Value1++;
        if(Temp_Value1 == 2)
        {
          Temp_Value1 = 0;
          Preamble_Count++;
        }
      }
      else if(Temp > 350 && Temp < 650)
      {
        Temp_Value1 = 0;
        if(Preamble_Count >= 5)
        {
          Status = 1;
          Preamble_Count = 0;
          TempByte_Count = 0;
          revLength = 0;
          revDone = false;
          startFlag = 0;
          startData = 1;
        }
        else
        {
          Preamble_Count = 0;
        }
      }
      else if(Temp > 650)
      {
        Preamble_Count = 0;
      }
    }
  }
  else //接收
  {
    Temp = TIM0->TC;
    TIM0->TC = 0;
    
    if(Temp > 150 && Temp < 350)
    {
      Temp_Value1++;
      if(Temp_Value1 == 2)
      {
        Temp_Value1 = 0;
        
        if(TempByte_Count == 9)
        {
          revLength++;
          TempByte_Count = 0;
          startFlag = 1;
          return;
        }
        else if(TempByte_Count > 9)
        {
          Status = 0;
          Preamble_Count = 0;
        }
        
        revData[revLength] >>= 1;
        revData[revLength] |= 0x100;
        TempByte_Count++;
      }
    }
    else if(Temp > 350 && Temp < 650)
    {
      Temp_Value1 = 0;

      if(startFlag == 1)
      {
        startFlag = 0;
        startFlag1 = 1;
        return;
      }
      
      revData[revLength] >>= 1;
      TempByte_Count++;
    }
//    else if(Temp > 1000)
//    {
//      Status = 0;
//      Preamble_Count = 0;
//      revDone = true;
//    }
  }
}

void TIM0_IRQHandler(void)
{
	if(TIM0->IR & TIM_IR_MR0INT)
	{
		TIM0->IR = TIM_IR_MR0INT;

    if(startFlag == 1 || startFlag1 == 1)
    {
      revDone = true;
      startFlag = 0;
      startFlag1 = 0;
    }
    Status = 0;
    Preamble_Count = 0;
	}
}
