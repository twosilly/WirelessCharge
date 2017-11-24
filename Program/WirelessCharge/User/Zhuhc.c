#include "Zhuhc.h"


static void uart_putbuff(UART_TypeDef* UARTx, uint8_t* txBuf, uint16_t txLen)
{
  while(txLen--)
  {
    while(UARTx->SR & UART_SR_TXF);
    UARTx->DR = *txBuf++;
  }
  
  while(UARTx->SR & UART_SR_TXNE);   //等待数据发送完成
}

/*!
 *  @brief      山外多功能调试助手上位机，摄像头显示函数
 *  @param      imgaddr    图像起始地址
 *  @param      imgsize    图像占用空间的大小
 */
void CX_SendImage(void *imgaddr, uint32_t imgsize)
{
#define CMD_IMG     1
  uint8_t cmdf[2] = {CMD_IMG, ~CMD_IMG};    //山外上位机 使用的命令
  uint8_t cmdr[2] = {~CMD_IMG, CMD_IMG};    //山外上位机 使用的命令

  uart_putbuff(SEND_PORT, cmdf, sizeof(cmdf));          //先发送命令
  uart_putbuff(SEND_PORT, (uint8_t *)imgaddr, imgsize); //再发送图像
  uart_putbuff(SEND_PORT, cmdr, sizeof(cmdr));          //先发送命令
}

/*!
 *  @brief      山外多功能调试助手上位机，线性CCD显示函数
 *  @param      ccdaddr    CCD图像起始地址
 *  @param      ccdsize    CCD图像占用空间的大小
 */
void CX_SendCCD(void *ccdaddr, uint32_t ccdsize)
{
#define CMD_CCD     2
  uint8_t cmdf[2] = {CMD_CCD, ~CMD_CCD};    //开头命令
  uint8_t cmdr[2] = {~CMD_CCD, CMD_CCD};    //结尾命令

  uart_putbuff(SEND_PORT, cmdf, sizeof(cmdf));          //先发送命令
  uart_putbuff(SEND_PORT, (uint8_t *)ccdaddr, ccdsize); //再发送图像
  uart_putbuff(SEND_PORT, cmdr, sizeof(cmdr));          //再发送命令
}

/*!
 *  @brief      山外多功能调试助手上位机，虚拟示波器显示函数
 *  @param      wareaddr    波形数组起始地址
 *  @param      waresize    波形数组占用空间的大小
 */
void CX_SendWare(void *wareaddr, uint32_t waresize)
{
#define CMD_WARE     3
  uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
  uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

  uart_putbuff(SEND_PORT, cmdf, sizeof(cmdf));            //先发送前命令
  uart_putbuff(SEND_PORT, (uint8_t *)wareaddr, waresize); //发送数据
  uart_putbuff(SEND_PORT, cmdr, sizeof(cmdr));            //发送后命令
}

