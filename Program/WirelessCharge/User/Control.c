#include "Control.h"

/*
 * Selection阶段:
 *     在这个阶段，功率发送器通常监测接口表面物体的放置和移除。如果电源发送器发现一个或多个对象，
 *     如果它支持自由定位，它应该尝试定位这些对象。此外，功率发送器可以尝试区分功率接收器和异物，
 *     如钥匙、硬币等。此外，功率发送器应尝试选择用于功率传输的功率接收器。
 */
volatile uint32_t SelectTime = 0;

void SelectControl(void)
{
  if(micros() - SelectTime > 500000)  //500ms
  {
    SelectTime = micros();
  }
  else
  {
  
  }
}

void PingControl(void)
{

}

void ConfigControl(void)
{

}

void PowerTransferControl(void)
{

}

void QiControl(void)
{
  SelectTime = micros();
  while(1)
  {
    if(QiStatus == Select)
    {
      SelectControl();
    }
    else if(QiStatus == Ping)
    { 
      PingControl();
    }
    else if(QiStatus == IdentificationAndConfiguration)
    {
      ConfigControl();
    }
    else if(QiStatus == PowerTransfer)
    {
      PowerTransferControl();
    }
  }
}
