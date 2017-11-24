#include "Qi.h"

/*
 * 单位编码周期：T=500us
 * 编码信号 1：high=250us 并且 low=250us
 * 编码信号 0：high=500us 或者 low=500us
 *
 * 11位异步串行格式传输数据字节。这种格式包括
 * 一个起始位、字节的8个数据位、一个奇偶校验位和一个停止位。
 * 起始位为零。停止位为一。数据位的顺序是LSB优先。奇偶位是奇数。
 * 
 * 功率接收器应使用数据包与功率发射机通信。数据包由4部分组成，即
 * 前导码、报头、消息和校验和。前导码全为1，最低11位，最高25位，
 * 前导码使功率发送器与输入的数据同步，并准确地检测报头的起始位。
 *          Preamble   Header   Message   Checksum
 * 
 * 规则： 功率发送器在开始位之前应该检测出至少4个前导位。
 *        功率发送器未检测到构成部分中的任何字节中的奇偶校验错误。
 *        功率发送器检测到校验和字节后的停止位。
 *        功率发送器确定校验和字节和数据是一致的。
 * 如果电源发送器没有正确接收数据包，则电源发送器应丢弃该包，而不使用其中所包含的任何信息。
 * 在ping阶段，以及在IdentificationAndConfiguration阶段，这通常会导致超时，这将导致功率发送器关闭电源信号。
 * 
 * 报头由一个字节表示分组类型。此外，头端隐式提供包中包含的消息的大小。
 *
 * 消息的第一个字节，字节B0，直接跟随报头。
 *
 * 校验和由一个字节组成，校验和 = 报头⊕消息
 */


QiStatus_TypeDef  QiStatus;
struct QiConfig_Struct QiConfig;

uint8_t Get_MessageSize(uint8_t Header)
{
  if(Header <= 0x1F)
    return 1;                          //Size 1
  else if(Header <= 0x7F)
    return (2 + (Header - 32) / 16);   //Size 2-7
  else if(Header <= 0xDF)
    return (8 + (Header - 128) / 8);   //Size 8-19
  else
    return (20 + (Header - 224) / 4);  //Size 20-27
}
 
 
