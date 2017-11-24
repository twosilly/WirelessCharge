#ifndef __QI_H
#define __QI_H

#include "Zhuhc.h"


#define CodeCycle     500   //500us   2KHz

/*
 * Signal Strength Packet (0x01):
 *     B0: 信号强度值
 *     此字段中的无符号整数值表示主单元和次级线圈之间的耦合程度。
 *     为了使功率发送器能够使用自由定位来决定提供最佳功率传输。
 *     255信号最强
 *
 * End Power Transfer Packet (0x02)：
 *     B0: 终止功率传输码
 *     此字段标识了终止电源传输请求的原因
 *     0x00 --- Unknown
 *     0x01 --- Charge Complete
 *     0x02 --- Internal Fault 
 *     0x03 --- Over Temperature
 *     0x04 --- Over Voltage 
 *     0x05 --- Over Current
 *     0x06 --- Battery Failure
 *     0x07 --- Reserved
 *     0x08 --- No Response
 *     0x09 --- Reserved
 *     0x0A --- Negotiation Failure(Extended Power Profile only)
 *     0x0B --- Restart Power Transfer(Extended Power Profile only)
 *     0x0C to 0xFF:Reserved
 *
 * Control Error Packet (0x03):
 *     B0: 控制误差值
 *     该字段包含的有符号整数值介于-128-127之间，并向功率发送器的工作点控制器提供输入。
 *
 * 8-bit Received Power Packet (0x04):
 *     B0: 接收功率值
 *     此字段中包含的无符号整数表示在配置包中显示的时间窗口中，功率接收器通过其接口表面接收的平均功率。
 *
 * Charge Status Packet (0x05):
 *     B0: 充电状态值
 *     如果移动设备包含可再充电的能量存储设备，则该字段包含的无符号整数表示该能量存储设备的充电电平为充电量的百分比。
 *     为清楚起见，值0表示空储能设备，值100表示完全充电的能量存储装置。如果移动设备不包含充电储能装置或者
 *     功率接收机不能提供充电状态信息，这字段应包含值0xFF，所有其他值都保留，并且不会出现在收费状态包中。
 *
 * Power Control Hold-off Packet (0x06):
 *     B0: 功率控制保持时间
 *     此字段中包含的无符号整数包含接收到控制误差包后毫秒的时间量，功率发送器在调整主线圈电流之前应等待。
 *
 * Configuration Packet (0x51): 
 *     B0: Power Class[7:6] Maximum Power Value[5:0]
 *     B1: Reserved[7:0]
 *     B2: Prop[7] Reserved[6:4] ZERO[3] Count[2:0]
 *     B3: Window Size[7:3] Window Offset[2:0]
 *     B4: Neg[7] Polarity[6] Depth[5:4] Reserved[3:0]
 *
 *     Power Class:此字段应设置为“00”。
 *     Maximum Power Value: 因是否支持FOD扩展而不同
 *     Prop: 如果为0则必须使用Qi协议中定义的PID控制方法，为1则可以自定义控制方法
 *     Reserved: 全为0
 *     Count: 此字段包含一个无符号整数值，该值指示功率接收器在identification & configuration阶段中发送的可选配置包的数目。
 *     Window Size: 此字段中包含的无符号整数表示以4ms为单位的平均接收功率窗口大小，其值应大于1。
 *     Window Offset: 此字段中包含的无符号整数表示在平均接收功率窗口和发送Received Power Packet之间的间隔，单位为4MS。
 *     Neg (FOD extensions only): 如果该位设置为1，则功率发送器应在配置包的末端发送ASK应答，指示功率接收器正在进入协商阶段。
 *     Polarity (FOD extensions only): 0指示功率发送器使用缺省FSK极性。1指示功率发送器使用反向FSK极性。
 *     Depth (FOD extensions only): 无符号整数，选择FSK调制深度。
 *
 * Identification Packet (0x71):
 *     B0: Major Version[7:4] Minor Version[3:0]
 *     B1: MSB  Manufacturer Code
 *     B2:      Manufacturer Code  LSB
 *     B3: MSB  Basic Device Identifier
 *     B4:      Basic Device Identifier
 *     B5:      Basic Device Identifier
 *     B6:      Basic Device Identifier  LSB
 *
 *     Major Version: 主版本号，V1.2则该位为0x1
 *     Minor Version: 次版本号，V1.2则该位为0x2
 *     Manufacturer Code: 此字段中包含的位标识功率接收器制造商，如功率接收器制造商代码、无线功率联盟中指定的那样。
 *     Ext(B3[7]): 如果该位被设置为0，则字符串 Manufacturer Code || Basic Device Identifier识别功率接收器。如果该位被设置为1，
 *                 则字符串 Manufacturer Code || Basic Device Identifier || Extended Device Identifier识别功率接收器。
 *     Basic Device Identifier: 这个字段中包含的位有助于识别功率接收器。
 *
 * Wireless Power ID Packets (0x54 and 0x55):
 *     B0: MSB    WPID
 *     B1:        WPID
 *     B2:        WPID    LSB
 *     B3: MSB    CRC
 *     B4:        CRC     LSB
 *     WPID为唯一标识功率接收器的标识符，48位
 *
 * Extended Identification Packet (0x81):
 *     B0-B7: Extended Device Identifier
 *     Extended Device Identifier: 见Identification Packet
 *
 * 24-bit Received Power Packet—FOD extensions only (0x31):
 *     B0: Reserved[7:3] Mode[2:0]
 *     B1: MSB   Received Power Value
 *     B2:       Received Power Value   LSB
 *     Mode: 提供了关于接收到的功率值的附加信息。
 *           0x0: 正常值；请求响应
 *           0x1: 轻负荷校准值；请求响应
 *           0x2: 连接负载校准值；请求响应
 *           0x4: 正常值；不期望响应
 *     Received Power Value: 这个字段包含的无符号整数表示平均功率。
 *     Reserved: 全为0
 * 
 * General Request Packet—FOD extensions only (0x07):
 *     B0: Request[7:0]
 *     Request: 这个无符号整数值表示请求的信息类型。若要请求特定的包，请将请求字段设置为该包的头部。
 *
 * Specific Request Packet—FOD extensions only (0x20):
 *     B0: Request[7:0]
 *     B1: Request Parameter[7:0]
 *     Request: 这个字段中的无符号整数包含了请求
 *     Request Parameter: 此字段包含请求的参数
 *
 * FOD Status Packet—FOD extensions only (0x22):
 *     B0: Reserved[7:2] Mode[1:0]
 *     B1: Reference Quality Factor Value[7:0]
 *     Reserved: 全为0
 *     Mode: 此字段表示参考品质因数值应用的功率接收器的工作模式。
 *     Reference Quality Factor Value: 此字段中的无符号整数包含功率接收器的参考品质因数。
 *
 * Renegotiate Packet—FOD extensions only (0x09):
 *     B0: Reserved[7:0]
 *     Reserved: 全为0
 */
 


#define Header_SignalStrength         0x01     //Message Size: 1
#define Header_EndPowerTransfer       0x02     //Message Size: 1
#define Header_ControlError           0x03     //Message Size: 1
#define Header_8BitReceivedPower      0x04     //Message Size: 1
#define Header_ChargeStatus           0x05     //Message Size: 1
#define Header_PowerControlHoldOff    0x06     //Message Size: 1
#define Header_GeneralRequest         0x07     //Message Size: 1
#define Header_Renegotiate            0x09     //Message Size: 1
#define Header_Proprietary1           0x18     //Message Size: 1
#define Header_Proprietary2           0x19     //Message Size: 1
#define Header_SpecificRequest        0x20     //Message Size: 2
#define Header_FODStatus              0x22     //Message Size: 2
#define Header_Proprietary3           0x28     //Message Size: 2
#define Header_Proprietary4           0x29     //Message Size: 2
#define Header_24BitReceivedPower     0x31     //Message Size: 3
#define Header_Proprietary5           0x38     //Message Size: 3
#define Header_Proprietary6           0x48     //Message Size: 4
#define Header_Configuration          0x51     //Message Size: 5
#define Header_WPID_Most              0x54     //Message Size: 5
#define Header_WPID_Least             0x55     //Message Size: 5
#define Header_Proprietary7           0x58     //Message Size: 5
#define Header_Proprietary8           0x68     //Message Size: 6
#define Header_Identification         0x71     //Message Size: 7
#define Header_Proprietary9           0x78     //Message Size: 7
#define Header_ExtendedIdentification 0x81     //Message Size: 8
#define Header_Proprietary10          0x84     //Message Size: 8
#define Header_Proprietary11          0xA4     //Message Size: 12
#define Header_Proprietary12          0xC4     //Message Size: 16
#define Header_Proprietary13          0xE2     //Message Size: 20

#define EPT_Unknown                   0x00     //End Power Transfer Code
#define EPT_ChargeComplete            0x01     //End Power Transfer Code
#define EPT_InternalFault             0x02     //End Power Transfer Code
#define EPT_OverTemperature           0x03     //End Power Transfer Code
#define EPT_OverVoltage               0x04     //End Power Transfer Code
#define EPT_OverCurrent               0x05     //End Power Transfer Code
#define EPT_BatteryFailure            0x06     //End Power Transfer Code
#define EPT_NoResponse                0x08     //End Power Transfer Code
#define EPT_NegotiationFailure        0x0A     //End Power Transfer Code
#define EPT_RestartPowerTransfer      0x0B     //End Power Transfer Code

#define RPP24_Normal_Response         0x00
#define RPP24_Light_Response          0x01
#define RPP24_Connected_Response      0x02
#define RPP24_Normal_NoResponse       0x04

#define SRP_EndNegotiation            0x00
#define SRP_GuaranteedPower           0x01
#define SRP_ReceivedPowerPacketType   0x02
#define SRP_FSKParameters             0x03
#define SRP_MaximumPower              0x04

typedef enum
{
  Select                         = 0,
  Ping                           = 1,
  IdentificationAndConfiguration = 2,
  PowerTransfer                  = 3,
} QiStatus_TypeDef;

struct QiConfig_Struct
{
  unsigned char PowerClass;
  unsigned char MaximumPowerValue;
  unsigned char Reserved1;
  unsigned char Prop;
  unsigned char Reserved2;
  unsigned char Count;
  unsigned char WindowSize;
  unsigned char WindowOffset;
  unsigned char Neg;
  unsigned char Polarity;
  unsigned char Depth;
  unsigned char Reserved3;
};

extern QiStatus_TypeDef  QiStatus;
extern struct QiConfig_Struct QiConfig;

uint8_t Get_MessageSize(uint8_t Header);

#endif
