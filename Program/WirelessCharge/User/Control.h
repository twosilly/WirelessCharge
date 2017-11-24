#ifndef __CONTROL_H
#define __CONTROL_H

#include "Zhuhc.h"

#define Time_Ping_Target         (65000)    //us
#define Time_Terminate_Max       (28000)    //us
#define Time_First_Max           (20000)    //us
#define Time_Expire_Max          (90000)    //us
                                 
#define Time_NextPacket_Max      (21000)
#define Time_PacketLength_Max    (170000)
#define Time_Response_Max        (10000)

#define Time_ControlError_Target    (1800000)  //1800ms
#define Time_ControlActive_Target   (20000)    //20ms
#define Time_ControlSettling_Target (5000)     //5ms
#define Time_ReceivedPower_Target   (23000000) //23S

void QiControl(void);

#endif

