#ifndef __GVM32F030_H
#define __GVM32F030_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Uncomment the line below according to the target GVM32F030 device used in your
   application

  */
#if !defined (GVM32F030x8) && !defined (GVM32F030x6)
  /* #define GVM32F030x8 */
  /* #define GVM32F030x6 */
#endif

#if !defined (GVM32F030x8) && !defined (GVM32F030x6)
  #error "Please select first the target GVM32F030xx device used in your application (in gvm32f030.h file)"
#endif

#if defined(GVM32F030x8) && defined(GVM32F030x6)
  #error "Only one GVM32F030 device selected (in gvm32f030.h file)"
#endif

#if !defined USE_STDPERIPH_DRIVER
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will 
   be based on direct access to peripherals registers 
   */
  /*#define USE_STDPERIPH_DRIVER*/
#endif



/*!< Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ******************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                        */
  SVC_IRQn                    = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                          */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                          */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                      */
  
/******  GVM32F030  specific Interrupt Numbers *************************************/
  WakeUp_IRQn                 = 0,      /*!< GVM32F030 Interrupt                                     */
  PWM_Fault_IRQn              = 1,      /*!< GVM32F030 Interrupt                                     */
  I2C_IRQn                    = 2,      /*!< GVM32F030 Interrupt                                     */
  TIM0_IRQn                   = 3,      /*!< GVM32F030 Interrupt                                     */
  TIM1_IRQn                   = 4,      /*!< GVM32F030 Interrupt                                     */
  TIM2_IRQn                   = 5,      /*!< GVM32F030 Interrupt                                     */
  TIM3_IRQn                   = 6,      /*!< GVM32F030 Interrupt                                     */
  UART0_IRQn                  = 7,      /*!< GVM32F030 Interrupt                                     */
  UART1_IRQn                  = 8,      /*!< GVM32F030 Interrupt                                     */
  ADC_IRQn                    = 9,      /*!< GVM32F030 Interrupt                                     */
  WDT_IRQn                    = 10,     /*!< GVM32F030 Interrupt                                     */
  BOD_IRQn                    = 11,     /*!< GVM32F030 Interrupt                                     */
  GPIOA_IRQn                  = 12,     /*!< GVM32F030 Interrupt                                     */
  GPIOB_IRQn                  = 13,     /*!< GVM32F030 Interrupt                                     */
  GPIOC_IRQn                  = 14,     /*!< GVM32F030 Interrupt                                     */
  RTC_IRQn                    = 15,     /*!< GVM32F030 Interrupt                                     */
  SPI_IRQn                    = 16,     /*!< GVM32F030 Interrupt                                     */
  PWM_Reload_IRQn             = 17,     /*!< GVM32F030 Interrupt                                     */
  
} IRQn_Type;


#define __CM0_REV                 0 /*!< Core Revision r0p0 */
#define __MPU_PRESENT             0 /*!< do not provide MPU */
#define __NVIC_PRIO_BITS          2 /*!< uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0 /*!< Set to 1 if different SysTick Config is used */

#include "core_cm0.h"
#include "system_gvm32f030.h"
#include <stdint.h>

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define CNF_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

#if defined ( __CC_ARM )
#pragma anon_unions
#endif

/** 
  * @brief System Control
  */
typedef struct
{
  __IO uint32_t SYSMEMREMAP;      /*!< System Memory remap register,                                    Address offset: 0x00 */
  __IO uint32_t PRESETCTRL;       /*!< Peripheral reset Control register,                               Address offset: 0x04 */
       uint32_t RESERVED0[7];     /*!< Reserved,                                                                 0x08 - 0x20 */
  __IO uint32_t WDTOSCCTRL;       /*!< Watchdog oscillator control register,                            Address offset: 0x24 */
  __IO uint32_t IRCCTRL;          /*!< Internal RC control register,                                    Address offset: 0x28 */
       uint32_t RESERVED1;        /*!< Reserved,                                                                        0x2C */
  __IO uint32_t SYSRSTSTAT;       /*!< System Reset Status register,                                    Address offset: 0x30 */
       uint32_t RESERVED2[15];    /*!< Reserved,                                                                 0x34 - 0x6C */
  __IO uint32_t MAINCLKSEL;       /*!< Main clock source select register,                               Address offset: 0x70 */
  __IO uint32_t MAINCLKUEN;       /*!< Main clock source update enable register,                        Address offset: 0x74 */
  __IO uint32_t SYSAHBCLKDIV;     /*!< System AHB clock divider register,                               Address offset: 0x78 */
       uint32_t RESERVED3;        /*!< Reserved,                                                                        0x7C */
  __IO uint32_t SYSAHBCLKCTRL;    /*!< System AHB clock control register,                               Address offset: 0x80 */
       uint32_t RESERVED4[4];     /*!< Reserved,                                                                 0x84 - 0x90 */
  __IO uint32_t SPICLKDIV;        /*!< SPI clock divder register,                                       Address offset: 0x94 */
  __IO uint32_t UART0CLKDIV;      /*!< UART0 clock divder register,                                     Address offset: 0x98 */
  __IO uint32_t UART1CLKDIV;      /*!< UART1 clock divder register,                                     Address offset: 0x9C */
       uint32_t RESERVED5[16];    /*!< Reserved,                                                                 0xA0 - 0xDC */
  __IO uint32_t CLKOUTCLKSEL;     /*!< CLKOUT clock source select register,                             Address offset: 0xE0 */
  __IO uint32_t CLKOUTUEN;        /*!< CLKOUT clock source update enable register,                      Address offset: 0xE4 */
  __IO uint32_t CLKOUTDIV;        /*!< CLKOUT clock divder register,                                    Address offset: 0xE8 */
       uint32_t RESERVED6[24];    /*!< Reserved,                                                                0xEC - 0x148 */
  __IO uint32_t IOCONFIGCLKDIV0;  /*!< IOCONFIG Peripheral clock 0 for programmable glitch filter,      Address offset:0x14C */
  __IO uint32_t BODCTRL;          /*!< Brown-out detect control register,                               Address offset:0x150 */
       uint32_t RESERVED7[8];     /*!< Reserved,                                                               0x154 - 0x170 */
  __IO uint32_t INTNMI;           /*!< NMI interrupt source configuration control register,             Address offset:0x174 */
       uint32_t RESERVED8[34];    /*!< Reserved,                                                               0x178 - 0x1FC */
  __IO uint32_t DSWAKECTL;        /*!< Wake up signal edge control register,                            Address offset:0x200 */
  __IO uint32_t DSWAKEEN;         /*!< Wake up signal enable control register,                          Address offset:0x204 */
  __O  uint32_t DSWAKECLR;        /*!< Wake up signal reset register,                                   Address offset:0x208 */
  __I  uint32_t DSWAKE;           /*!< Wake up signal status register,                                  Address offset:0x20C */
       uint32_t RESERVED9[8];     /*!< Reserved,                                                               0x210 - 0x22C */
  __IO uint32_t PDSLEEPCFG;       /*!< Power-down states in Deep-sleep mode register,                   Address offset:0x230 */
  __IO uint32_t PDAWAKECFG;       /*!< Power-down states after wake-up from Deep-sleep mode register,   Address offset:0x234 */
  __IO uint32_t PDRUNCFG;         /*!< Power-down configuration register,                               Address offset:0x238 */
} SYSCON_TypeDef;


/** 
  * @brief Power Management Unit
  */
typedef struct
{
  __IO uint32_t PCON;        /*!< Power control Register,                                       Address offset: 0x00 */
  __IO uint32_t GPREG[4];    /*!< General purpose Registers 0..3,                        Address offset: 0x04 - 0x10 */
  __IO uint32_t SYSCFG;      /*!< System configuration register,                                Address offset: 0x14 */
} PMU_TypeDef;


/** 
  * @brief General Purpose IO
  */
typedef struct
{
  __IO uint32_t MASK;         /*!< GPIO port MASK register,                                 Address offset: 0x00 */
  __I  uint32_t PIN;          /*!< GPIO port i/o state register,                            Address offset: 0x04 */
  __IO uint32_t OUT;          /*!< GPIO port output register,                               Address offset: 0x08 */
  __O  uint32_t SET;          /*!< GPIO port bit set register,                              Address offset: 0x0C */
  __O  uint32_t CLR;          /*!< GPIO port bit clear register,                            Address offset: 0x10 */
  __O  uint32_t NOT;          /*!< GPIO port bit toggle register,                           Address offset: 0x14 */
       uint32_t RESERVED0[2]; /*!< Reserved,                                                         0x18 - 0x1C */
  __IO uint32_t DIR;          /*!< GPIO port configuration register,                        Address offset: 0x20 */
  __IO uint32_t IS;           /*!< GPIO port interrupt configuration register,              Address offset: 0x24 */
  __IO uint32_t IBE;          /*!< GPIO port interrupt tigger mode register,                Address offset: 0x28 */
  __IO uint32_t IEV;          /*!< GPIO port interrupt tigger register,                     Address offset: 0x2C */
  __IO uint32_t IE;           /*!< GPIO port interrupt mask register,                       Address offset: 0x30 */
  __I  uint32_t RIS;          /*!< GPIO port raw interrupt state register,                  Address offset: 0x34 */
  __I  uint32_t MIS;          /*!< GPIO port interrupt state register,                      Address offset: 0x38 */
  __O  uint32_t IC;           /*!< GPIO port interrupt clear register,                      Address offset: 0x3C */
} GPIO_TypeDef;



/** 
  * @brief I/O Configuration
  */
typedef struct
{
  __IO uint32_t PA0;            /*!< PA0  I/O configuration register,                    Address offset: 0x00 */
  __IO uint32_t PA1;            /*!< PA1  I/O configuration register,                    Address offset: 0x04 */
  __IO uint32_t PA2;            /*!< PA2  I/O configuration register,                    Address offset: 0x08 */
  __IO uint32_t PA3;            /*!< PA3  I/O configuration register,                    Address offset: 0x0C */
  __IO uint32_t PA4;            /*!< PA4  I/O configuration register,                    Address offset: 0x10 */
  __IO uint32_t PA5;            /*!< PA5  I/O configuration register,                    Address offset: 0x14 */
  __IO uint32_t PA6;            /*!< PA6  I/O configuration register,                    Address offset: 0x18 */
  __IO uint32_t PA7;            /*!< PA7  I/O configuration register,                    Address offset: 0x1C */
  __IO uint32_t PA8;            /*!< PA8  I/O configuration register,                    Address offset: 0x20 */
  __IO uint32_t PA9;            /*!< PA9  I/O configuration register,                    Address offset: 0x24 */
  __IO uint32_t PA10;           /*!< PA10 I/O configuration register,                    Address offset: 0x28 */
  __IO uint32_t PA11;           /*!< PA11 I/O configuration register,                    Address offset: 0x2C */
  __IO uint32_t PA12;           /*!< PA12 I/O configuration register,                    Address offset: 0x30 */
  __IO uint32_t PA13;           /*!< PA13 I/O configuration register,                    Address offset: 0x34 */
  __IO uint32_t PA14;           /*!< PA14 I/O configuration register,                    Address offset: 0x38 */
  __IO uint32_t PA15;           /*!< PA15 I/O configuration register,                    Address offset: 0x3C */
  __IO uint32_t PB0;            /*!< PB0  I/O configuration register,                    Address offset: 0x40 */
  __IO uint32_t PB1;            /*!< PB1  I/O configuration register,                    Address offset: 0x44 */
  __IO uint32_t PB2;            /*!< PB2  I/O configuration register,                    Address offset: 0x48 */
  __IO uint32_t PB3;            /*!< PB3  I/O configuration register,                    Address offset: 0x4C */
  __IO uint32_t PB4;            /*!< PB4  I/O configuration register,                    Address offset: 0x50 */
  __IO uint32_t PB5;            /*!< PB5  I/O configuration register,                    Address offset: 0x54 */
  __IO uint32_t PB6;            /*!< PB6  I/O configuration register,                    Address offset: 0x58 */
  __IO uint32_t PB7;            /*!< PB7  I/O configuration register,                    Address offset: 0x5C */
  __IO uint32_t PB8;            /*!< PB8  I/O configuration register,                    Address offset: 0x60 */
  __IO uint32_t PB9;            /*!< PB9  I/O configuration register,                    Address offset: 0x64 */
  __IO uint32_t PB10;           /*!< PB10 I/O configuration register,                    Address offset: 0x68 */
  __IO uint32_t PB11;           /*!< PB11 I/O configuration register,                    Address offset: 0x6C */
  __IO uint32_t PB12;           /*!< PB12 I/O configuration register,                    Address offset: 0x70 */
  __IO uint32_t PB13;           /*!< PB13 I/O configuration register,                    Address offset: 0x74 */
  __IO uint32_t PB14;           /*!< PB14 I/O configuration register,                    Address offset: 0x78 */
  __IO uint32_t PB15;           /*!< PB15 I/O configuration register,                    Address offset: 0x7C */
  __IO uint32_t PC0;            /*!< PC0  I/O configuration register,                    Address offset: 0x80 */
  __IO uint32_t PC1;            /*!< PC1  I/O configuration register,                    Address offset: 0x84 */
  __IO uint32_t PC2;            /*!< PC2  I/O configuration register,                    Address offset: 0x88 */
  __IO uint32_t PC3;            /*!< PC3  I/O configuration register,                    Address offset: 0x8C */
  __IO uint32_t PC4;            /*!< PC4  I/O configuration register,                    Address offset: 0x90 */
  __IO uint32_t PC5;            /*!< PC5  I/O configuration register,                    Address offset: 0x94 */
  __IO uint32_t PC6;            /*!< PC6  I/O configuration register,                    Address offset: 0x98 */
  __IO uint32_t PC7;            /*!< PC7  I/O configuration register,                    Address offset: 0x9C */
  __IO uint32_t PC8;            /*!< PC8  I/O configuration register,                    Address offset: 0xA0 */
  __IO uint32_t PC9;            /*!< PC9  I/O configuration register,                    Address offset: 0xA4 */
  __IO uint32_t PC10;           /*!< PC10 I/O configuration register,                    Address offset: 0xA8 */
  __IO uint32_t PC11;           /*!< PC11 I/O configuration register,                    Address offset: 0xAC */
  __IO uint32_t PC12;           /*!< PC12 I/O configuration register,                    Address offset: 0xB0 */
  __IO uint32_t PC13;           /*!< PC13 I/O configuration register,                    Address offset: 0xB4 */
  __IO uint32_t PC14;           /*!< PC14 I/O configuration register,                    Address offset: 0xB8 */
  __IO uint32_t PC15;           /*!< PC15 I/O configuration register,                    Address offset: 0xBC */
} IOCFGx6_TypeDef;


/** 
  * @brief I/O Configuration
  */
typedef struct
{
  __IO uint32_t PC13;           /*!< PC13 I/O configuration register,                    Address offset: 0x00 */
  __IO uint32_t PC14;           /*!< PC14 I/O configuration register,                    Address offset: 0x04 */
  __IO uint32_t PC15;           /*!< PC15 I/O configuration register,                    Address offset: 0x08 */
  __IO uint32_t PC7;            /*!< PC7  I/O configuration register,                    Address offset: 0x0C */
  __IO uint32_t PC8;            /*!< PC8  I/O configuration register,                    Address offset: 0x10 */
  __IO uint32_t PC9;            /*!< PC9  I/O configuration register,                    Address offset: 0x14 */
  __IO uint32_t PA0;            /*!< PA0  I/O configuration register,                    Address offset: 0x18 */
  __IO uint32_t PA1;            /*!< PA1  I/O configuration register,                    Address offset: 0x1C */
  __IO uint32_t PA2;            /*!< PA2  I/O configuration register,                    Address offset: 0x20 */
  __IO uint32_t PA3;            /*!< PA3  I/O configuration register,                    Address offset: 0x24 */
  __IO uint32_t PA4;            /*!< PA4  I/O configuration register,                    Address offset: 0x28 */
  __IO uint32_t PA5;            /*!< PA5  I/O configuration register,                    Address offset: 0x2C */
  __IO uint32_t PA6;            /*!< PA6  I/O configuration register,                    Address offset: 0x30 */
  __IO uint32_t PA7;            /*!< PA7  I/O configuration register,                    Address offset: 0x34 */
       uint32_t RESERVED0[8];   /*!< Reserved,                                                    0x38 - 0x54 */
  __IO uint32_t PB0;            /*!< PB0  I/O configuration register,                    Address offset: 0x58 */
  __IO uint32_t PB1;            /*!< PB1  I/O configuration register,                    Address offset: 0x5C */
  __IO uint32_t PB2;            /*!< PB2  I/O configuration register,                    Address offset: 0x60 */
  __IO uint32_t PB10;           /*!< PB10 I/O configuration register,                    Address offset: 0x64 */
  __IO uint32_t PB11;           /*!< PB11 I/O configuration register,                    Address offset: 0x68 */
  __IO uint32_t PB12;           /*!< PB12 I/O configuration register,                    Address offset: 0x6C */
  __IO uint32_t PB13;           /*!< PB13 I/O configuration register,                    Address offset: 0x70 */
  __IO uint32_t PB14;           /*!< PB14 I/O configuration register,                    Address offset: 0x74 */
  __IO uint32_t PB15;           /*!< PB15 I/O configuration register,                    Address offset: 0x78 */
       uint32_t RESERVED1[4];   /*!< Reserved,                                                    0x7C - 0x88 */
  __IO uint32_t PA8;            /*!< PA8  I/O configuration register,                    Address offset: 0x8C */
  __IO uint32_t PA9;            /*!< PA9  I/O configuration register,                    Address offset: 0x90 */
  __IO uint32_t PA10;           /*!< PA10 I/O configuration register,                    Address offset: 0x94 */
  __IO uint32_t PA11;           /*!< PA11 I/O configuration register,                    Address offset: 0x98 */
  __IO uint32_t PA12;           /*!< PA12 I/O configuration register,                    Address offset: 0x9C */
  __IO uint32_t PA13;           /*!< PA13 I/O configuration register,                    Address offset: 0xA0 */
  __IO uint32_t PC10;           /*!< PC10 I/O configuration register,                    Address offset: 0xA4 */
  __IO uint32_t PC11;           /*!< PC11 I/O configuration register,                    Address offset: 0xA8 */
  __IO uint32_t PA14;           /*!< PA14 I/O configuration register,                    Address offset: 0xAC */
  __IO uint32_t PA15;           /*!< PA15 I/O configuration register,                    Address offset: 0xB0 */
       uint32_t RESERVED2[4];   /*!< Reserved,                                                    0xB4 - 0xC0 */
  __IO uint32_t PB3;            /*!< PB3  I/O configuration register,                    Address offset: 0xC4 */
  __IO uint32_t PB4;            /*!< PB4  I/O configuration register,                    Address offset: 0xC8 */
  __IO uint32_t PB5;            /*!< PB5  I/O configuration register,                    Address offset: 0xCC */
  __IO uint32_t PB6;            /*!< PB6  I/O configuration register,                    Address offset: 0xD0 */
  __IO uint32_t PB7;            /*!< PB7  I/O configuration register,                    Address offset: 0xD4 */
  __IO uint32_t PC12;           /*!< PC12 I/O configuration register,                    Address offset: 0xD8 */
  __IO uint32_t PB8;            /*!< PB8  I/O configuration register,                    Address offset: 0xDC */
  __IO uint32_t PB9;            /*!< PB9  I/O configuration register,                    Address offset: 0xE0 */
} IOCFGx8_TypeDef;


/** 
  * @brief UART
  */
typedef struct
{
  __IO uint32_t DR;             /*!< UART data register,                              Address offset: 0x00 */
  __IO uint32_t SR;             /*!< UART state register,                             Address offset: 0x04 */
  __IO uint32_t CR;             /*!< UART control register,                           Address offset: 0x08 */
  __IO uint32_t ISR;            /*!< UART interrupt status register,                  Address offset: 0x0C */
  __IO uint32_t BAUDDIV;        /*!< UART baud divide register,                       Address offset: 0x10 */
  __IO uint32_t FIFOCLR;        /*!< UART fifo clear register,                        Address offset: 0x14 */
} UART_TypeDef;


/** 
  * @brief Serial Peripheral Interface
  */
typedef struct
{
  __IO uint32_t CR0;            /*!< SPI Control register 0,                                  Address offset: 0x00 */
  __IO uint32_t CR1;            /*!< SPI Control register 1,                                  Address offset: 0x04 */
  __IO uint32_t DR;             /*!< SPI Data register,                                       Address offset: 0x08 */
  __I  uint32_t SR;             /*!< SPI Status register,                                     Address offset: 0x0C */
  __IO uint32_t CPSR;           /*!< SPI Clock Prescale register,                             Address offset: 0x10 */
  __IO uint32_t IMSC;           /*!< SPI Interrupt Mask Set and Clear register,               Address offset: 0x14 */
  __I  uint32_t RIS;            /*!< SPI Raw Interrupt Status register,                       Address offset: 0x18 */
  __I  uint32_t MIS;            /*!< SPI Masked Interrupt Status register,                    Address offset: 0x1C */
  __O  uint32_t ICR;            /*!< SPI Interrupt Clear register,                            Address offset: 0x20 */
} SPI_TypeDef;


/** 
  * @brief Inter－Integrated Circuit(I2C)
  */
typedef struct
{
  __IO uint32_t CONSET;		      /*!< I2C Control Set Register,                                Address offset: 0x00 */
  __I  uint32_t STAT;			      /*!< I2C Status Register,                                     Address offset: 0x04 */
  __IO uint32_t DAT;			      /*!< I2C Data Register,                                       Address offset: 0x08 */
  __IO uint32_t ADR0;			      /*!< I2C Slave Address Register 0,                            Address offset: 0x0C */
  __IO uint32_t SCLH;			      /*!< SCH Duty Cycle Register High Half Word,                  Address offset: 0x10 */
  __IO uint32_t SCLL;			      /*!< SCL Duty Cycle Register Low Half Word,                   Address offset: 0x14 */
  __O  uint32_t CONCLR;		      /*!< I2C Control Clear Register,                              Address offset: 0x18 */
       uint32_t RESERVED0;		  /*!< Reserved,                                                Address offset: 0x1C */
  __IO uint32_t ADR1;			      /*!< I2C Slave Address Register,                              Address offset: 0x20 */
  __IO uint32_t ADR2;			      /*!< I2C Slave Address Register,                              Address offset: 0x24 */
  __IO uint32_t ADR3;			      /*!< I2C Slave Address Register,                              Address offset: 0x28 */
  __I  uint32_t DATA_BUFFER;	  /*!< Data buffer register,                                    Address offset: 0x2C */
  __IO uint32_t MASK[4];		    /*!< I2C Slave address mask register 0..3,              Address offset: 0x30 - 0x3C*/
} I2C_TypeDef;


/** 
  * @brief Timer
  */
typedef struct
{
  __IO uint32_t IR;             /*!< TIM Interrupt Register,                                  Address offset: 0x00 */
  __IO uint32_t TCR;            /*!< TIM Control Register,                                    Address offset: 0x04 */
  __IO uint32_t TC;             /*!< TIM Counter Register,                                    Address offset: 0x08 */
  __IO uint32_t PR;             /*!< TIM Prescale Register,                                   Address offset: 0x0C */
  __IO uint32_t PC;             /*!< TIM Prescale Counter Register,                           Address offset: 0x10 */
  __IO uint32_t MCR;            /*!< TIM Match Control Register,                              Address offset: 0x14 */
  __IO uint32_t MR0;            /*!< TIM Match Register 0,                                    Address offset: 0x18 */
  __IO uint32_t MR1;            /*!< TIM Match Register 1,                                    Address offset: 0x1C */
  __IO uint32_t MR2;            /*!< TIM Match Register 2,                                    Address offset: 0x20 */
  __IO uint32_t MR3;            /*!< TIM Match Register 3,                                    Address offset: 0x24 */
  /************************************ Following is only used in TIM2 and TIM3 ************************************/
  __IO uint32_t CCR;            /*!< TIM Capture Control Register,                            Address offset: 0x28 */
  __I  uint32_t CR0;            /*!< TIM Capture Register 0,                                  Address offset: 0x2C */
  __I  uint32_t CR1;            /*!< TIM Capture Register 1,                                  Address offset: 0x30 */
  __I  uint32_t CR2;            /*!< TIM Capture Register 2,                                  Address offset: 0x34 */
  __I  uint32_t CR3;            /*!< TIM Capture Register 3,                                  Address offset: 0x38 */
  __IO uint32_t EMR;            /*!< TIM External Match Register,                             Address offset: 0x3C */
       uint32_t RESERVED0[12];  /*!< Reserved,                                                         0x40 - 0x6C */
  __IO uint32_t CTCR;           /*!< TIM Count Control Register,                              Address offset: 0x70 */
} TIM_TypeDef;


/** 
  * @brief Watchdog
  */
typedef struct
{
  __IO uint32_t MODE;           /*!< WDT mode register,                                       Address offset: 0x00 */
  __IO uint32_t TC;             /*!< WDT timer constant register,                             Address offset: 0x04 */
  __O  uint32_t FEED;           /*!< WDT feed sequence register,                              Address offset: 0x08 */
  __I  uint32_t TV;             /*!< WDT timer value register,                                Address offset: 0x0C */
  __IO uint32_t CLKSEL;         /*!< WDT clock select register,                               Address offset: 0x10 */
  __IO uint32_t WARNINT;        /*!< WDT warning interrupt register,                          Address offset: 0x14 */
  __IO uint32_t WINDOW;         /*!< WDT timer window register,                               Address offset: 0x18 */
} WDT_TypeDef;


/** 
  * @brief CRC
  */
typedef struct
{
  __IO uint32_t MODE;           /*!< CRC Mode register,                                   Address offset: 0x00 */
  __IO uint32_t SEED;           /*!< CRC Seed register,                                   Address offset: 0x04 */
  
  union {
    __I uint32_t SUM;
    __O uint32_t WR_DATA_DWORD;
    __O uint16_t WR_DATA_WORD;
        uint16_t RESERVED_WORD;
    __O uint8_t WR_DATA_BYTE;
        uint8_t RESERVED_BYTE[3];
  };                            /*!< CRC Sum register,                                    Address offset: 0x08 */
} CRC_TypeDef;

/** 
  * @brief ADC
  */
typedef struct
{
  __IO uint32_t CR;             /*!< ADC control register,                              Address offset: 0x00 */
  __IO uint32_t GDR;            /*!< ADC global data register,                          Address offset: 0x04 */
  __IO uint32_t CHSEL;          /*!< ADC channel select register,                       Address offset: 0x08 */
  __IO uint32_t INTEN;          /*!< ADC interrupt control register,                    Address offset: 0x0C */
  __IO uint32_t DR[8];          /*!< ADC data register,                          Address offset: 0x10 - 0x2C */
  __IO uint32_t STAT;           /*!< ADC status register,                               Address offset: 0x30 */
  __IO uint32_t HILMT;          /*!< ADC upper limit register,                          Address offset: 0x34 */
  __IO uint32_t LOLMT;          /*!< ADC lower limit register,                          Address offset: 0x38 */
       uint32_t RESERVED0;      /*!< Reserved,                                          Address offset: 0x3C */
  __IO uint32_t SSCR;           /*!< ADC software trigger register,                     Address offset: 0x40 */
} ADC_TypeDef;


/** 
  * @brief PWM
  */
typedef struct
{
  __IO uint32_t CTRL;           /*!< PWM ### register,                              Address offset: 0x00 */
  __IO uint32_t FCTRL;          /*!< PWM ### register,                              Address offset: 0x04 */
  __IO uint32_t FLTACK;         /*!< PWM ### register,                              Address offset: 0x08 */
  __IO uint32_t OUT;            /*!< PWM ### register,                              Address offset: 0x0C */
  __I  uint32_t CNTR;           /*!< PWM ### register,                              Address offset: 0x10 */
  __IO uint32_t CMOD;           /*!< PWM ### register,                              Address offset: 0x14 */
  __IO uint32_t VAL0;           /*!< PWM ### register,                              Address offset: 0x18 */
  __IO uint32_t VAL1;           /*!< PWM ### register,                              Address offset: 0x1C */
  __IO uint32_t VAL2;           /*!< PWM ### register,                              Address offset: 0x20 */
  __IO uint32_t VAL3;           /*!< PWM ### register,                              Address offset: 0x24 */
  __IO uint32_t VAL4;           /*!< PWM ### register,                              Address offset: 0x28 */
  __IO uint32_t VAL5;           /*!< PWM ### register,                              Address offset: 0x2C */
  __IO uint32_t VAL6;           /*!< PWM ### register,                              Address offset: 0x30 */
  __IO uint32_t VAL7;           /*!< PWM ### register,                              Address offset: 0x34 */
  __IO uint32_t DTIM0;          /*!< PWM ### register,                              Address offset: 0x38 */
  __IO uint32_t DTIM1;          /*!< PWM ### register,                              Address offset: 0x3C */
  __IO uint32_t DMAP0;          /*!< PWM ### register,                              Address offset: 0x40 */
  __IO uint32_t DMAP1;          /*!< PWM ### register,                              Address offset: 0x44 */
  __IO uint32_t CNFG;           /*!< PWM ### register,                              Address offset: 0x48 */
  __IO uint32_t CCTRL;          /*!< PWM ### register,                              Address offset: 0x4C */
  __IO uint32_t FPORTCTRL;      /*!< PWM ### register,                              Address offset: 0x50 */
  __IO uint32_t ICCTRL;         /*!< PWM ### register,                              Address offset: 0x54 */
       uint32_t RESERVED0[2];   /*!< Reserved,                                               0x58 - 0x5C */
  __IO uint32_t PSCR;           /*!< PWM ### register,                              Address offset: 0x60 */
  __O  uint32_t CNTRINI;        /*!< PWM ### register,                              Address offset: 0x64 */
} PWM_TypeDef;


/** 
  * @brief RTC
  */
typedef struct
{
  __I  uint32_t DR;             /*!< RTC ### register,                              Address offset: 0x00 */
  __IO uint32_t MR;             /*!< RTC ### register,                              Address offset: 0x04 */
  __IO uint32_t LR;             /*!< RTC ### register,                              Address offset: 0x08 */
  __IO uint32_t CR;             /*!< RTC ### register,                              Address offset: 0x0C */
  __IO uint32_t ICSC;           /*!< RTC ### register,                              Address offset: 0x10 */
  __I  uint32_t RIS;            /*!< RTC ### register,                              Address offset: 0x14 */
  __I  uint32_t MIS;            /*!< RTC ### register,                              Address offset: 0x18 */
  __O  uint32_t ICR;            /*!< RTC ### register,                              Address offset: 0x1C */
} RTC_TypeDef;


/** 
  * @brief Device Information Array
  */
typedef struct
{
      uint32_t RESERVED0;       /*!< Reserved,                                                 Address offset: 0x00 */
  __I uint32_t FUNCDESC;        /*!< Device function description,                              Address offset: 0x04 */
  __I uint32_t DID;             /*!< Device ID,                                                Address offset: 0x08 */
  __I uint32_t VERID;           /*!< Hardware version,                                         Address offset: 0x0C */
  __I uint32_t IRCTRIM;         /*!< Internal IRC trim bits                                    Address offset: 0x10 */
  __I uint32_t UNIQUEID0;       /*!< Unique device serial no- 32-bit,                          Address offset: 0x14 */
  __I uint32_t UNIQUEID1;       /*!< Unique device serial no- 32-bit,                          Address offset: 0x18 */
  __I uint32_t UNIQUEID2;       /*!< Unique device serial no- 32-bit,                          Address offset: 0x1C */
  __I uint32_t UNIQUEID3;       /*!< Unique device serial no- 32-bit,                          Address offset: 0x20 */
} DIA_TypeDef;



#if defined ( __CC_ARM )
#pragma no_anon_unions
#endif


/** @addtogroup Peripheral_declaration
  * @{
  */ 

#define I2C_BASE            ((uint32_t)0x40000000)
#define WDT_BASE            ((uint32_t)0x40004000)
#define UART0_BASE          ((uint32_t)0x40008000)
#define UART1_BASE          ((uint32_t)0x4000C000)
#define TIM0_BASE           ((uint32_t)0x40010000)
#define TIM1_BASE           ((uint32_t)0x40014000)
#define TIM2_BASE           ((uint32_t)0x40018000)
#define TIM3_BASE           ((uint32_t)0x4001C000)
#define ADC_BASE            ((uint32_t)0x40020000)

#define PMU_BASE            ((uint32_t)0x40038000)
#define SPI_BASE            ((uint32_t)0x40040000)
#define IOCFG_BASE          ((uint32_t)0x40044000)
#define SYSCON_BASE         ((uint32_t)0x40048000)
#define PWM_BASE            ((uint32_t)0x4004C000)
#define RTC_BASE            ((uint32_t)0x40050000)

#define GPIOA_BASE          ((uint32_t)0x50000000)
#define GPIOB_BASE          ((uint32_t)0x50010000)
#define GPIOC_BASE          ((uint32_t)0x50020000)

#define CRC_BASE            ((uint32_t)0x50070000)

#define DIA_BASE            ((uint32_t)0x1FFF07C0)


#define GPIO_BASE           GPIOA_BASE


/**
  * @}
  */


  
/** @addtogroup Peripheral_declaration
  * @{
  */ 

#define I2C                 ((I2C_TypeDef *) I2C_BASE)
#define WDT                 ((WDT_TypeDef *) WDT_BASE)
#define UART0               ((UART_TypeDef *) UART0_BASE)
#define UART1               ((UART_TypeDef *) UART1_BASE)
#define TIM0                ((TIM_TypeDef *) TIM0_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define ADC                 ((ADC_TypeDef *) ADC_BASE)

#define PMU                 ((PMU_TypeDef *) PMU_BASE)
#define SPI                 ((SPI_TypeDef *) SPI_BASE)


#define IOCFGx6             ((IOCFGx6_TypeDef *) IOCFG_BASE)
#define IOCFGx8             ((IOCFGx8_TypeDef *) IOCFG_BASE)

#if defined(GVM32F030x6)
#define IOCFG               IOCFGx6
#elif defined(GVM32F030x8)
#define IOCFG               IOCFGx8
#endif


#define SYSCON              ((SYSCON_TypeDef *) SYSCON_BASE)
#define PWM                 ((PWM_TypeDef *) PWM_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)

#define CRC                 ((CRC_TypeDef *) CRC_BASE)

#define DIA                 ((DIA_TypeDef *) DIA_BASE)

/**
  * @}
  */

  
  
  
  
  
/** @addtogroup Exported_constants
  * @{
  */
  
  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                                                                            */
/*                             System Control                                 */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCON_SYSMEMREMAP register  ******************/
#define SYSCON_SYSMEMREMAP_BOOTLOADER     ((uint32_t)0x00000000)        /*!< Boot Loader Mode. Interrupt vectors are re-mapped to Boot ROM */
#define SYSCON_SYSMEMREMAP_SRAM           ((uint32_t)0x00000001)        /*!< User RAM Mode. Interrupt vectors are re-mapped to Static RAM */
#define SYSCON_SYSMEMREMAP_FLASH          ((uint32_t)0x00000002)        /*!< User Flash mode. Interrupt vectors are not re-mapped and reside in Flash */

/******************  Bit definition for SYSCON_PRESETCTRL register  ******************/
#define SYSCON_PRESETCTRL_SPI_RST_N       ((uint32_t)0x00000001)        /*!< SYSCON_PRESETCTRL SPI reset bit */
#define SYSCON_PRESETCTRL_I2C_RST_N       ((uint32_t)0x00000002)        /*!< SYSCON_PRESETCTRL I2C reset bit */
#define SYSCON_PRESETCTRL_UART0_RST_N     ((uint32_t)0x00000004)        /*!< SYSCON_PRESETCTRL UART0 reset bit */
#define SYSCON_PRESETCTRL_UART1_RST_N     ((uint32_t)0x00000008)        /*!< SYSCON_PRESETCTRL UART1 reset bit */
#define SYSCON_PRESETCTRL_TIM0_RST_N      ((uint32_t)0x00000020)        /*!< SYSCON_PRESETCTRL TIM0 reset bit */
#define SYSCON_PRESETCTRL_TIM1_RST_N      ((uint32_t)0x00000040)        /*!< SYSCON_PRESETCTRL TIM1 reset bit */
#define SYSCON_PRESETCTRL_TIM2_RST_N      ((uint32_t)0x00000080)        /*!< SYSCON_PRESETCTRL TIM2 reset bit */
#define SYSCON_PRESETCTRL_TIM3_RST_N      ((uint32_t)0x00000100)        /*!< SYSCON_PRESETCTRL TIM3 reset bit */
#define SYSCON_PRESETCTRL_PWM_RST_N       ((uint32_t)0x00000200)        /*!< SYSCON_PRESETCTRL PWM reset bit */
#define SYSCON_PRESETCTRL_CRC_RST_N       ((uint32_t)0x00000400)        /*!< SYSCON_PRESETCTRL CRC reset bit */
#define SYSCON_PRESETCTRL_ADC_RST_N       ((uint32_t)0x00001000)        /*!< SYSCON_PRESETCTRL ADC reset bit */

/******************  Bit definition for SYSCON_WDTOSCCTRL register  ******************/
#define SYSCON_WDTOSCCTRL_DIVSEL          ((uint32_t)0x0000001F)        /*!< WDT clock divider mask bit */
#define SYSCON_WDTOSCCTRL_WDTCLKSRC       ((uint32_t)0x00001000)        /*!< 32KHz selected as WDT clock */

/******************  Bit definition for SYSCON_SYSRSTSTAT register  ******************/
#define SYSCON_SYSRSTSTAT_POR             ((uint32_t)0x00000001)         /*!< POR reset detected */
#define SYSCON_SYSRSTSTAT_EXTRST          ((uint32_t)0x00000002)         /*!< RESET pin reset event detected */
#define SYSCON_SYSRSTSTAT_WDT             ((uint32_t)0x00000004)         /*!< WDT reset detected */
#define SYSCON_SYSRSTSTAT_BOD             ((uint32_t)0x00000008)         /*!< Brown-out reset detected */
#define SYSCON_SYSRSTSTAT_SYSRST          ((uint32_t)0x00000010)         /*!< System reset (ARM software reset) detected */

/******************  Bit definition for SYSCON_MAINCLKSEL register  ******************/
#define SYSCON_MAINCLKSEL_IRC             ((uint32_t)0x00000000)        /*!< IRC oscillator selected as main clock source */
#define SYSCON_MAINCLKSEL_WDT             ((uint32_t)0x00000002)        /*!< WDT oscillator selected as main clock source */

/******************  Bit definition for SYSCON_MAINCLKUEN register  ******************/
#define SYSCON_MAINCLKUEN_ENA             ((uint32_t)0x00000001)        /*!< main clock source update and enable */

/******************  Bit definition for SYSCON_SYSAHBCLKCTRL register  ******************/
#define SYSCON_SYSAHBCLKCTRL_SYSEN        ((uint32_t)0x00000001)        /*!< SYS clock enable */
#define SYSCON_SYSAHBCLKCTRL_RAMEN        ((uint32_t)0x00000004)        /*!< RAM clock enable */
#define SYSCON_SYSAHBCLKCTRL_I2CEN        ((uint32_t)0x00000020)        /*!< I2C clock enable */
#define SYSCON_SYSAHBCLKCTRL_CRCEN        ((uint32_t)0x00000040)        /*!< CRC clock enable */
#define SYSCON_SYSAHBCLKCTRL_TIM0EN       ((uint32_t)0x00000080)        /*!< TIM0 clock enable */
#define SYSCON_SYSAHBCLKCTRL_TIM1EN       ((uint32_t)0x00000100)        /*!< TIM1 clock enable */
#define SYSCON_SYSAHBCLKCTRL_TIM2EN       ((uint32_t)0x00000200)        /*!< TIM2 clock enable */
#define SYSCON_SYSAHBCLKCTRL_TIM3EN       ((uint32_t)0x00000400)        /*!< TIM3 clock enable */
#define SYSCON_SYSAHBCLKCTRL_SPIEN        ((uint32_t)0x00000800)        /*!< SPI clock enable */
#define SYSCON_SYSAHBCLKCTRL_UART0EN      ((uint32_t)0x00001000)        /*!< UART0 clock enable */
#define SYSCON_SYSAHBCLKCTRL_UART1EN      ((uint32_t)0x00002000)        /*!< UART1 clock enable */
#define SYSCON_SYSAHBCLKCTRL_ADCEN        ((uint32_t)0x00004000)        /*!< ADC clock enable */
#define SYSCON_SYSAHBCLKCTRL_WDTEN        ((uint32_t)0x00008000)        /*!< Watchdog clock enable */
#define SYSCON_SYSAHBCLKCTRL_IOCONEN      ((uint32_t)0x00010000)        /*!< IOCON clock enable */
#define SYSCON_SYSAHBCLKCTRL_PWMEN        ((uint32_t)0x00020000)        /*!< PWM clock enable */
#define SYSCON_SYSAHBCLKCTRL_RTCEN        ((uint32_t)0x00080000)        /*!< RTC clock enable */
#define SYSCON_SYSAHBCLKCTRL_GPIOAEN      ((uint32_t)0x20000000)        /*!< GPIOA clock enable */
#define SYSCON_SYSAHBCLKCTRL_GPIOBEN      ((uint32_t)0x40000000)        /*!< GPIOB clock enable */
#define SYSCON_SYSAHBCLKCTRL_GPIOCEN      ((uint32_t)0x80000000)        /*!< GPIOC clock enable */

/******************  Bit definition for SYSCON_CLKOUTCLKSEL register  ******************/
#define SYSCON_CLKOUTCLKSEL_IRC           ((uint32_t)0x00000000)        /*!< IRC oscillator selected as clock out */
#define SYSCON_CLKOUTCLKSEL_SYSCLK        ((uint32_t)0x00000001)        /*!< SYSCLK selected as clock out */
#define SYSCON_CLKOUTCLKSEL_WDT           ((uint32_t)0x00000002)        /*!< WDT oscillator selected as clock out */
#define SYSCON_CLKOUTCLKSEL_MAINCLK       ((uint32_t)0x00000003)        /*!< MAINCLK selected as clock out */
#define SYSCON_CLKOUTCLKSEL_RTCOSC        ((uint32_t)0x00000004)        /*!< RTCOSC selected as clock out */

/******************  Bit definition for SYSCON_CLKOUTUEN register  ******************/
#define SYSCON_CLKOUTUEN_ENA              ((uint32_t)0x00000001)        /*!< clock out update and enable */

/******************  Bit definition for SYSCON_BODCTRL register  ******************/
#define SYSCON_BODCTRL_BODRSTENA          ((uint32_t)0x00000010)        /*!< enable BOD reset function */

/******************  Bit definition for SYSCON_INTNMI register  ******************/
#define SYSCON_INTNMI_WAKEUP              ((uint32_t)0x00000000)        /*!< Wake Up interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_PWM_FAULT           ((uint32_t)0x00000001)        /*!< PWM fault interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_I2C                 ((uint32_t)0x00000002)        /*!< I2C interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_TIM0                ((uint32_t)0x00000003)        /*!< TIM0 interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_TIM1                ((uint32_t)0x00000004)        /*!< TIM1 interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_TIM2                ((uint32_t)0x00000005)        /*!< TIM2 interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_TIM3                ((uint32_t)0x00000006)        /*!< TIM3 interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_UART0               ((uint32_t)0x00000007)        /*!< UART0 interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_UART1               ((uint32_t)0x00000008)        /*!< UART1 interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_ADC                 ((uint32_t)0x00000009)        /*!< ADC interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_WDT                 ((uint32_t)0x0000000A)        /*!< WDT interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_BOD                 ((uint32_t)0x0000000B)        /*!< BOD interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_GPIOA               ((uint32_t)0x0000000C)        /*!< GPIOA interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_GPIOB               ((uint32_t)0x0000000D)        /*!< GPIOB interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_GPIOC               ((uint32_t)0x0000000E)        /*!< GPIOC interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_RTC                 ((uint32_t)0x0000000F)        /*!< RTC interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_SPI                 ((uint32_t)0x00000010)        /*!< SPI interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_PWM_RELOAD          ((uint32_t)0x00000011)        /*!< PWM Reload interrupt select as NMI interrupt source */
#define SYSCON_INTNMI_DISABLE             ((uint32_t)0x0000003F)        /*!< Disable NMI interrupt */

/******************  Bit definition for SYSCON_DSWAKECTL register  ******************/
#define SYSCON_DSWAKECTL_CTLPA0           ((uint32_t)0x00000001)        /*!< PA0 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA1           ((uint32_t)0x00000002)        /*!< PA1 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA2           ((uint32_t)0x00000004)        /*!< PA2 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA3           ((uint32_t)0x00000008)        /*!< PA3 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA4           ((uint32_t)0x00000010)        /*!< PA4 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA5           ((uint32_t)0x00000020)        /*!< PA5 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA6           ((uint32_t)0x00000040)        /*!< PA6 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA7           ((uint32_t)0x00000080)        /*!< PA7 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA8           ((uint32_t)0x00000100)        /*!< PA8 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA9           ((uint32_t)0x00000200)        /*!< PA9 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA10          ((uint32_t)0x00000400)        /*!< PA10 rising edge select as wake up signal */
#define SYSCON_DSWAKECTL_CTLPA11          ((uint32_t)0x00000800)        /*!< PA11 rising edge select as wake up signal */

/******************  Bit definition for SYSCON_DSWAKEEN register  ******************/
#define SYSCON_DSWAKEEN_ERPA0             ((uint32_t)0x00000001)        /*!< Enable PA0 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA1             ((uint32_t)0x00000002)        /*!< Enable PA1 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA2             ((uint32_t)0x00000004)        /*!< Enable PA2 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA3             ((uint32_t)0x00000008)        /*!< Enable PA3 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA4             ((uint32_t)0x00000010)        /*!< Enable PA4 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA5             ((uint32_t)0x00000020)        /*!< Enable PA5 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA6             ((uint32_t)0x00000040)        /*!< Enable PA6 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA7             ((uint32_t)0x00000080)        /*!< Enable PA7 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA8             ((uint32_t)0x00000100)        /*!< Enable PA8 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA9             ((uint32_t)0x00000200)        /*!< Enable PA9 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA10            ((uint32_t)0x00000400)        /*!< Enable PA10 input as wake up signal */
#define SYSCON_DSWAKEEN_ERPA11            ((uint32_t)0x00000800)        /*!< Enable PA11 input as wake up signal */

/******************  Bit definition for SYSCON_DSWAKECLR register  ******************/
#define SYSCON_DSWAKECLR_RSRPA0           ((uint32_t)0x00000001)        /*!< clear PA0 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA1           ((uint32_t)0x00000002)        /*!< clear PA1 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA2           ((uint32_t)0x00000004)        /*!< clear PA2 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA3           ((uint32_t)0x00000008)        /*!< clear PA3 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA4           ((uint32_t)0x00000010)        /*!< clear PA4 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA5           ((uint32_t)0x00000020)        /*!< clear PA5 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA6           ((uint32_t)0x00000040)        /*!< clear PA6 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA7           ((uint32_t)0x00000080)        /*!< clear PA7 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA8           ((uint32_t)0x00000100)        /*!< clear PA8 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA9           ((uint32_t)0x00000200)        /*!< clear PA9 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA10          ((uint32_t)0x00000400)        /*!< clear PA10 trigger wake up signal */
#define SYSCON_DSWAKECLR_RSRPA11          ((uint32_t)0x00000800)        /*!< clear PA11 trigger wake up signal */

/******************  Bit definition for SYSCON_DSWAKE register  ******************/
#define SYSCON_DSWAKE_SRPA0               ((uint32_t)0x00000001)        /*!< PA0 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA1               ((uint32_t)0x00000002)        /*!< PA1 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA2               ((uint32_t)0x00000004)        /*!< PA2 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA3               ((uint32_t)0x00000008)        /*!< PA3 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA4               ((uint32_t)0x00000010)        /*!< PA4 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA5               ((uint32_t)0x00000020)        /*!< PA5 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA6               ((uint32_t)0x00000040)        /*!< PA6 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA7               ((uint32_t)0x00000080)        /*!< PA7 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA8               ((uint32_t)0x00000100)        /*!< PA8 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA9               ((uint32_t)0x00000200)        /*!< PA9 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA10              ((uint32_t)0x00000400)        /*!< PA10 trigger wake up signal pending */
#define SYSCON_DSWAKE_SRPA11              ((uint32_t)0x00000800)        /*!< PA11 trigger wake up signal pending */

/******************  Bit definition for SYSCON_PDSLEEPCFG register  ******************/
#define SYSCON_PDSLEEPCFG_BOD_PD          ((uint32_t)0x00000008)        /*!< BOD power down in deep sleep mode */
#define SYSCON_PDSLEEPCFG_RTCOSC_PD       ((uint32_t)0x00000020)        /*!< RTCOSC power down in deep sleep mode */
#define SYSCON_PDSLEEPCFG_WDTOSC_PD       ((uint32_t)0x00000040)        /*!< WDTOSC power down in deep sleep mode */

/******************  Bit definition for SYSCON_PDAWAKECFG register  ******************/
#define SYSCON_PDAWAKECFG_IRC_PD          ((uint32_t)0x00000002)        /*!< IRC power down when wake up */
#define SYSCON_PDAWAKECFG_BOD_PD          ((uint32_t)0x00000008)        /*!< BOD power down when wake up */
#define SYSCON_PDAWAKECFG_ADC_PD          ((uint32_t)0x00000010)        /*!< ADC power down when wake up */
#define SYSCON_PDAWAKECFG_RTCOSC_PD       ((uint32_t)0x00000020)        /*!< RTCOSC power down when wake up */
#define SYSCON_PDAWAKECFG_WDTOSC_PD       ((uint32_t)0x00000040)        /*!< WDTOSC power down when wake up */

/******************  Bit definition for SYSCON_PDRUNCFG register  ******************/
#define SYSCON_PDRUNCFG_IRC_PD            ((uint32_t)0x00000002)        /*!< IRC power down when wake up */
#define SYSCON_PDRUNCFG_BOD_PD            ((uint32_t)0x00000008)        /*!< BOD power down when wake up */
#define SYSCON_PDRUNCFG_ADC_PD            ((uint32_t)0x00000010)        /*!< ADC power down when wake up */
#define SYSCON_PDRUNCFG_RTCOSC_PD         ((uint32_t)0x00000020)        /*!< RTCOSC power down when wake up */
#define SYSCON_PDRUNCFG_WDTOSC_PD         ((uint32_t)0x00000040)        /*!< WDTOSC power down when wake up */



/******************************************************************************/
/*                                                                            */
/*                       Power Management Unit (PMU)                          */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for PMU_PCON register  *******************/
#define PMU_PCON_DPDEN                    ((uint32_t)0x00000002)        /*!< Deep power-down mode enable */
#define PMU_PCON_SLEEPFLAG                ((uint32_t)0x00000100)        /*!< Sleep mode flag */
#define PMU_PCON_DPDFLAG                  ((uint32_t)0x00000800)        /*!< Deep power-down flag */


/******************************************************************************/
/*                                                                            */
/*                       General Purpose IOs (GPIO)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for GPIO_MASK register  *******************/
#define GPIO_MASK_0                       ((uint32_t)0x00000001)
#define GPIO_MASK_1                       ((uint32_t)0x00000002)
#define GPIO_MASK_2                       ((uint32_t)0x00000004)
#define GPIO_MASK_3                       ((uint32_t)0x00000008)
#define GPIO_MASK_4                       ((uint32_t)0x00000010)
#define GPIO_MASK_5                       ((uint32_t)0x00000020)
#define GPIO_MASK_6                       ((uint32_t)0x00000040)
#define GPIO_MASK_7                       ((uint32_t)0x00000080)
#define GPIO_MASK_8                       ((uint32_t)0x00000100)
#define GPIO_MASK_9                       ((uint32_t)0x00000200)
#define GPIO_MASK_10                      ((uint32_t)0x00000400)
#define GPIO_MASK_11                      ((uint32_t)0x00000800)
#define GPIO_MASK_12                      ((uint32_t)0x00001000)
#define GPIO_MASK_13                      ((uint32_t)0x00002000)
#define GPIO_MASK_14                      ((uint32_t)0x00004000)
#define GPIO_MASK_15                      ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_PIN register  *******************/
#define GPIO_PIN_0                        ((uint32_t)0x00000001)
#define GPIO_PIN_1                        ((uint32_t)0x00000002)
#define GPIO_PIN_2                        ((uint32_t)0x00000004)
#define GPIO_PIN_3                        ((uint32_t)0x00000008)
#define GPIO_PIN_4                        ((uint32_t)0x00000010)
#define GPIO_PIN_5                        ((uint32_t)0x00000020)
#define GPIO_PIN_6                        ((uint32_t)0x00000040)
#define GPIO_PIN_7                        ((uint32_t)0x00000080)
#define GPIO_PIN_8                        ((uint32_t)0x00000100)
#define GPIO_PIN_9                        ((uint32_t)0x00000200)
#define GPIO_PIN_10                       ((uint32_t)0x00000400)
#define GPIO_PIN_11                       ((uint32_t)0x00000800)
#define GPIO_PIN_12                       ((uint32_t)0x00001000)
#define GPIO_PIN_13                       ((uint32_t)0x00002000)
#define GPIO_PIN_14                       ((uint32_t)0x00004000)
#define GPIO_PIN_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_OUT register  *******************/
#define GPIO_OUT_0                        ((uint32_t)0x00000001)
#define GPIO_OUT_1                        ((uint32_t)0x00000002)
#define GPIO_OUT_2                        ((uint32_t)0x00000004)
#define GPIO_OUT_3                        ((uint32_t)0x00000008)
#define GPIO_OUT_4                        ((uint32_t)0x00000010)
#define GPIO_OUT_5                        ((uint32_t)0x00000020)
#define GPIO_OUT_6                        ((uint32_t)0x00000040)
#define GPIO_OUT_7                        ((uint32_t)0x00000080)
#define GPIO_OUT_8                        ((uint32_t)0x00000100)
#define GPIO_OUT_9                        ((uint32_t)0x00000200)
#define GPIO_OUT_10                       ((uint32_t)0x00000400)
#define GPIO_OUT_11                       ((uint32_t)0x00000800)
#define GPIO_OUT_12                       ((uint32_t)0x00001000)
#define GPIO_OUT_13                       ((uint32_t)0x00002000)
#define GPIO_OUT_14                       ((uint32_t)0x00004000)
#define GPIO_OUT_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_SET register  *******************/
#define GPIO_SET_0                        ((uint32_t)0x00000001)
#define GPIO_SET_1                        ((uint32_t)0x00000002)
#define GPIO_SET_2                        ((uint32_t)0x00000004)
#define GPIO_SET_3                        ((uint32_t)0x00000008)
#define GPIO_SET_4                        ((uint32_t)0x00000010)
#define GPIO_SET_5                        ((uint32_t)0x00000020)
#define GPIO_SET_6                        ((uint32_t)0x00000040)
#define GPIO_SET_7                        ((uint32_t)0x00000080)
#define GPIO_SET_8                        ((uint32_t)0x00000100)
#define GPIO_SET_9                        ((uint32_t)0x00000200)
#define GPIO_SET_10                       ((uint32_t)0x00000400)
#define GPIO_SET_11                       ((uint32_t)0x00000800)
#define GPIO_SET_12                       ((uint32_t)0x00001000)
#define GPIO_SET_13                       ((uint32_t)0x00002000)
#define GPIO_SET_14                       ((uint32_t)0x00004000)
#define GPIO_SET_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_CLR register  *******************/
#define GPIO_CLR_0                        ((uint32_t)0x00000001)
#define GPIO_CLR_1                        ((uint32_t)0x00000002)
#define GPIO_CLR_2                        ((uint32_t)0x00000004)
#define GPIO_CLR_3                        ((uint32_t)0x00000008)
#define GPIO_CLR_4                        ((uint32_t)0x00000010)
#define GPIO_CLR_5                        ((uint32_t)0x00000020)
#define GPIO_CLR_6                        ((uint32_t)0x00000040)
#define GPIO_CLR_7                        ((uint32_t)0x00000080)
#define GPIO_CLR_8                        ((uint32_t)0x00000100)
#define GPIO_CLR_9                        ((uint32_t)0x00000200)
#define GPIO_CLR_10                       ((uint32_t)0x00000400)
#define GPIO_CLR_11                       ((uint32_t)0x00000800)
#define GPIO_CLR_12                       ((uint32_t)0x00001000)
#define GPIO_CLR_13                       ((uint32_t)0x00002000)
#define GPIO_CLR_14                       ((uint32_t)0x00004000)
#define GPIO_CLR_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_NOT register  *******************/
#define GPIO_NOT_0                        ((uint32_t)0x00000001)
#define GPIO_NOT_1                        ((uint32_t)0x00000002)
#define GPIO_NOT_2                        ((uint32_t)0x00000004)
#define GPIO_NOT_3                        ((uint32_t)0x00000008)
#define GPIO_NOT_4                        ((uint32_t)0x00000010)
#define GPIO_NOT_5                        ((uint32_t)0x00000020)
#define GPIO_NOT_6                        ((uint32_t)0x00000040)
#define GPIO_NOT_7                        ((uint32_t)0x00000080)
#define GPIO_NOT_8                        ((uint32_t)0x00000100)
#define GPIO_NOT_9                        ((uint32_t)0x00000200)
#define GPIO_NOT_10                       ((uint32_t)0x00000400)
#define GPIO_NOT_11                       ((uint32_t)0x00000800)
#define GPIO_NOT_12                       ((uint32_t)0x00001000)
#define GPIO_NOT_13                       ((uint32_t)0x00002000)
#define GPIO_NOT_14                       ((uint32_t)0x00004000)
#define GPIO_NOT_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_DIR register  *******************/
#define GPIO_DIR_0                        ((uint32_t)0x00000001)
#define GPIO_DIR_1                        ((uint32_t)0x00000002)
#define GPIO_DIR_2                        ((uint32_t)0x00000004)
#define GPIO_DIR_3                        ((uint32_t)0x00000008)
#define GPIO_DIR_4                        ((uint32_t)0x00000010)
#define GPIO_DIR_5                        ((uint32_t)0x00000020)
#define GPIO_DIR_6                        ((uint32_t)0x00000040)
#define GPIO_DIR_7                        ((uint32_t)0x00000080)
#define GPIO_DIR_8                        ((uint32_t)0x00000100)
#define GPIO_DIR_9                        ((uint32_t)0x00000200)
#define GPIO_DIR_10                       ((uint32_t)0x00000400)
#define GPIO_DIR_11                       ((uint32_t)0x00000800)
#define GPIO_DIR_12                       ((uint32_t)0x00001000)
#define GPIO_DIR_13                       ((uint32_t)0x00002000)
#define GPIO_DIR_14                       ((uint32_t)0x00004000)
#define GPIO_DIR_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_IS register  *******************/
#define GPIO_IS_0                         ((uint32_t)0x00000001)
#define GPIO_IS_1                         ((uint32_t)0x00000002)
#define GPIO_IS_2                         ((uint32_t)0x00000004)
#define GPIO_IS_3                         ((uint32_t)0x00000008)
#define GPIO_IS_4                         ((uint32_t)0x00000010)
#define GPIO_IS_5                         ((uint32_t)0x00000020)
#define GPIO_IS_6                         ((uint32_t)0x00000040)
#define GPIO_IS_7                         ((uint32_t)0x00000080)
#define GPIO_IS_8                         ((uint32_t)0x00000100)
#define GPIO_IS_9                         ((uint32_t)0x00000200)
#define GPIO_IS_10                        ((uint32_t)0x00000400)
#define GPIO_IS_11                        ((uint32_t)0x00000800)
#define GPIO_IS_12                        ((uint32_t)0x00001000)
#define GPIO_IS_13                        ((uint32_t)0x00002000)
#define GPIO_IS_14                        ((uint32_t)0x00004000)
#define GPIO_IS_15                        ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_IBE register  *******************/
#define GPIO_IBE_0                        ((uint32_t)0x00000001)
#define GPIO_IBE_1                        ((uint32_t)0x00000002)
#define GPIO_IBE_2                        ((uint32_t)0x00000004)
#define GPIO_IBE_3                        ((uint32_t)0x00000008)
#define GPIO_IBE_4                        ((uint32_t)0x00000010)
#define GPIO_IBE_5                        ((uint32_t)0x00000020)
#define GPIO_IBE_6                        ((uint32_t)0x00000040)
#define GPIO_IBE_7                        ((uint32_t)0x00000080)
#define GPIO_IBE_8                        ((uint32_t)0x00000100)
#define GPIO_IBE_9                        ((uint32_t)0x00000200)
#define GPIO_IBE_10                       ((uint32_t)0x00000400)
#define GPIO_IBE_11                       ((uint32_t)0x00000800)
#define GPIO_IBE_12                       ((uint32_t)0x00001000)
#define GPIO_IBE_13                       ((uint32_t)0x00002000)
#define GPIO_IBE_14                       ((uint32_t)0x00004000)
#define GPIO_IBE_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_IEV register  *******************/
#define GPIO_IEV_0                        ((uint32_t)0x00000001)
#define GPIO_IEV_1                        ((uint32_t)0x00000002)
#define GPIO_IEV_2                        ((uint32_t)0x00000004)
#define GPIO_IEV_3                        ((uint32_t)0x00000008)
#define GPIO_IEV_4                        ((uint32_t)0x00000010)
#define GPIO_IEV_5                        ((uint32_t)0x00000020)
#define GPIO_IEV_6                        ((uint32_t)0x00000040)
#define GPIO_IEV_7                        ((uint32_t)0x00000080)
#define GPIO_IEV_8                        ((uint32_t)0x00000100)
#define GPIO_IEV_9                        ((uint32_t)0x00000200)
#define GPIO_IEV_10                       ((uint32_t)0x00000400)
#define GPIO_IEV_11                       ((uint32_t)0x00000800)
#define GPIO_IEV_12                       ((uint32_t)0x00001000)
#define GPIO_IEV_13                       ((uint32_t)0x00002000)
#define GPIO_IEV_14                       ((uint32_t)0x00004000)
#define GPIO_IEV_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_IE register  *******************/
#define GPIO_IE_0                        ((uint32_t)0x00000001)
#define GPIO_IE_1                        ((uint32_t)0x00000002)
#define GPIO_IE_2                        ((uint32_t)0x00000004)
#define GPIO_IE_3                        ((uint32_t)0x00000008)
#define GPIO_IE_4                        ((uint32_t)0x00000010)
#define GPIO_IE_5                        ((uint32_t)0x00000020)
#define GPIO_IE_6                        ((uint32_t)0x00000040)
#define GPIO_IE_7                        ((uint32_t)0x00000080)
#define GPIO_IE_8                        ((uint32_t)0x00000100)
#define GPIO_IE_9                        ((uint32_t)0x00000200)
#define GPIO_IE_10                       ((uint32_t)0x00000400)
#define GPIO_IE_11                       ((uint32_t)0x00000800)
#define GPIO_IE_12                       ((uint32_t)0x00001000)
#define GPIO_IE_13                       ((uint32_t)0x00002000)
#define GPIO_IE_14                       ((uint32_t)0x00004000)
#define GPIO_IE_15                       ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_RIS register  *******************/
#define GPIO_RIS_0                       ((uint32_t)0x00000001)
#define GPIO_RIS_1                       ((uint32_t)0x00000002)
#define GPIO_RIS_2                       ((uint32_t)0x00000004)
#define GPIO_RIS_3                       ((uint32_t)0x00000008)
#define GPIO_RIS_4                       ((uint32_t)0x00000010)
#define GPIO_RIS_5                       ((uint32_t)0x00000020)
#define GPIO_RIS_6                       ((uint32_t)0x00000040)
#define GPIO_RIS_7                       ((uint32_t)0x00000080)
#define GPIO_RIS_8                       ((uint32_t)0x00000100)
#define GPIO_RIS_9                       ((uint32_t)0x00000200)
#define GPIO_RIS_10                      ((uint32_t)0x00000400)
#define GPIO_RIS_11                      ((uint32_t)0x00000800)
#define GPIO_RIS_12                      ((uint32_t)0x00001000)
#define GPIO_RIS_13                      ((uint32_t)0x00002000)
#define GPIO_RIS_14                      ((uint32_t)0x00004000)
#define GPIO_RIS_15                      ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_MIS register  *******************/
#define GPIO_MIS_0                       ((uint32_t)0x00000001)
#define GPIO_MIS_1                       ((uint32_t)0x00000002)
#define GPIO_MIS_2                       ((uint32_t)0x00000004)
#define GPIO_MIS_3                       ((uint32_t)0x00000008)
#define GPIO_MIS_4                       ((uint32_t)0x00000010)
#define GPIO_MIS_5                       ((uint32_t)0x00000020)
#define GPIO_MIS_6                       ((uint32_t)0x00000040)
#define GPIO_MIS_7                       ((uint32_t)0x00000080)
#define GPIO_MIS_8                       ((uint32_t)0x00000100)
#define GPIO_MIS_9                       ((uint32_t)0x00000200)
#define GPIO_MIS_10                      ((uint32_t)0x00000400)
#define GPIO_MIS_11                      ((uint32_t)0x00000800)
#define GPIO_MIS_12                      ((uint32_t)0x00001000)
#define GPIO_MIS_13                      ((uint32_t)0x00002000)
#define GPIO_MIS_14                      ((uint32_t)0x00004000)
#define GPIO_MIS_15                      ((uint32_t)0x00008000)

/*******************  Bit definition for GPIO_IC register  *******************/
#define GPIO_IC_0                        ((uint32_t)0x00000001)
#define GPIO_IC_1                        ((uint32_t)0x00000002)
#define GPIO_IC_2                        ((uint32_t)0x00000004)
#define GPIO_IC_3                        ((uint32_t)0x00000008)
#define GPIO_IC_4                        ((uint32_t)0x00000010)
#define GPIO_IC_5                        ((uint32_t)0x00000020)
#define GPIO_IC_6                        ((uint32_t)0x00000040)
#define GPIO_IC_7                        ((uint32_t)0x00000080)
#define GPIO_IC_8                        ((uint32_t)0x00000100)
#define GPIO_IC_9                        ((uint32_t)0x00000200)
#define GPIO_IC_10                       ((uint32_t)0x00000400)
#define GPIO_IC_11                       ((uint32_t)0x00000800)
#define GPIO_IC_12                       ((uint32_t)0x00001000)
#define GPIO_IC_13                       ((uint32_t)0x00002000)
#define GPIO_IC_14                       ((uint32_t)0x00004000)
#define GPIO_IC_15                       ((uint32_t)0x00008000)




/******************************************************************************/
/*                                                                            */
/*                          I/O Configuration                                 */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for IOCFG register  ******************/
#define IOCFG_FUNC                        ((uint32_t)0x00000007)        /*!< IOCFG FUNC Bit Mask */

#define IOCFG_PDE                         ((uint32_t)0x00000008)        /*!< IOCFG Power down Enable */
#define IOCFG_PUE                         ((uint32_t)0x00000010)        /*!< IOCFG Power up Enable */
#define IOCFG_CSE                         ((uint32_t)0x00000020)        /*!< IOCFG Schmitt trigger Enable */
#define IOCFG_INV                         ((uint32_t)0x00000040)        /*!< IOCFG inverse input Enable */
#define IOCFG_SRM                         ((uint32_t)0x00000080)        /*!< IOCFG Power up Enable */
#define IOCFG_ADM                         ((uint32_t)0x00000100)        /*!< IOCFG Analog mode */
#define IOCFG_DRV                         ((uint32_t)0x00000200)        /*!< IOCFG Low Current Mode */
#define IOCFG_OD                          ((uint32_t)0x00000400)        /*!< IOCFG Open drain mode */
#define IOCFG_SMODE                       ((uint32_t)0x00000800)        /*!< IOCFG Filter Mode */
#define IOCFG_IEN                         ((uint32_t)0x00001000)        /*!< IOCFG Input Enable */

#define IOCFG_DEFAULT                     (IOCFG_DRV | IOCFG_CSE | IOCFG_PUE)   /*!< IOCFG Default configuration */

/******************  Bit definition for IOCFG_PC13 register  ******************/
#define IOCFG_PC13_FUNC_PC13              ((uint32_t)0x00000000)        /*!< PC13 function selected as PC13 (default) */

/******************  Bit definition for IOCFG_PC14 register  ******************/
#define IOCFG_PC14_FUNC_PC14              ((uint32_t)0x00000000)        /*!< PC14 function selected as PC14 (default) */

/******************  Bit definition for IOCFG_PC15 register  ******************/
#define IOCFG_PC15_FUNC_PC15              ((uint32_t)0x00000000)        /*!< PC15 function selected as PC15 (default) */

/******************  Bit definition for IOCFG_PC7 register   ******************/
#define IOCFG_PC7_FUNC_PC7                ((uint32_t)0x00000000)        /*!< PC7 function selected as PC7 (default) */

/******************  Bit definition for IOCFG_PC8 register   ******************/
#define IOCFG_PC8_FUNC_PC8                ((uint32_t)0x00000000)        /*!< PC8 function selected as PC8 (default) */
#define IOCFG_PC8_FUNC_CLKOUT             ((uint32_t)0x00000001)        /*!< PC8 function selected as CLKOUT */

/******************  Bit definition for IOCFG_PC9 register   ******************/
#define IOCFG_PC9_FUNC_NRST               ((uint32_t)0x00000000)        /*!< PC9 function selected as NRST (default) */
#define IOCFG_PC9_FUNC_PC9                ((uint32_t)0x00000001)        /*!< PC9 function selected as PC9 */

/******************  Bit definition for IOCFG_PA0 register   ******************/
#define IOCFG_PA0_FUNC_PA0                ((uint32_t)0x00000000)        /*!< PA0 function selected as PA0 (default) */
#define IOCFG_PA0_FUNC_ADC_IN0            ((uint32_t)0x00000002)        /*!< PA0 function selected as ADC_IN0 */

/******************  Bit definition for IOCFG_PA1 register   ******************/
#define IOCFG_PA1_FUNC_PA1                ((uint32_t)0x00000000)        /*!< PA1 function selected as PA1 (default) */
#define IOCFG_PA1_FUNC_ADC_IN1            ((uint32_t)0x00000002)        /*!< PA1 function selected as ADC_IN1 */

/******************  Bit definition for IOCFG_PA2 register   ******************/
#define IOCFG_PA2_FUNC_PA2                ((uint32_t)0x00000000)        /*!< PA2 function selected as PA2 (default) */
#define IOCFG_PA2_FUNC_TIM3_CAP0          ((uint32_t)0x00000001)        /*!< PA2 function selected as TIM3_CAP0 */
#define IOCFG_PA2_FUNC_TIM3_MAT0          ((uint32_t)0x00000002)        /*!< PA2 function selected as TIM3_MAT0 */
#define IOCFG_PA2_FUNC_TXD0               ((uint32_t)0x00000003)        /*!< PA2 function selected as TXD0 */
#define IOCFG_PA2_FUNC_ADC_IN2            ((uint32_t)0x00000005)        /*!< PA2 function selected as ADC_IN2 */

/******************  Bit definition for IOCFG_PA3 register   ******************/
#define IOCFG_PA3_FUNC_PA3                ((uint32_t)0x00000000)        /*!< PA3 function selected as PA3 (default) */
#define IOCFG_PA3_FUNC_TIM3_CAP1          ((uint32_t)0x00000001)        /*!< PA3 function selected as TIM3_CAP1 */
#define IOCFG_PA3_FUNC_TIM3_MAT1          ((uint32_t)0x00000002)        /*!< PA3 function selected as TIM3_MAT1 */
#define IOCFG_PA3_FUNC_RXD0               ((uint32_t)0x00000003)        /*!< PA3 function selected as RXD0 */
#define IOCFG_PA3_FUNC_ADC_IN3            ((uint32_t)0x00000005)        /*!< PA3 function selected as ADC_IN3 */

/******************  Bit definition for IOCFG_PA4 register   ******************/
#define IOCFG_PA4_FUNC_PA4                ((uint32_t)0x00000000)        /*!< PA4 function selected as PA4 (default) */
#define IOCFG_PA4_FUNC_TIM3_CAP2          ((uint32_t)0x00000001)        /*!< PA4 function selected as TIM3_CAP2 */
#define IOCFG_PA4_FUNC_TIM3_MAT2          ((uint32_t)0x00000002)        /*!< PA4 function selected as TIM3_MAT2 */
#define IOCFG_PA4_FUNC_SPI_SSEL           ((uint32_t)0x00000003)        /*!< PA4 function selected as SPI_SSEL */
#define IOCFG_PA4_FUNC_ADC_IN4            ((uint32_t)0x00000005)        /*!< PA4 function selected as ADC_IN4 */

/******************  Bit definition for IOCFG_PA5 register   ******************/
#define IOCFG_PA5_FUNC_PA5                ((uint32_t)0x00000000)        /*!< PA5 function selected as PA5 (default) */
#define IOCFG_PA5_FUNC_SPI_SCK            ((uint32_t)0x00000001)        /*!< PA5 function selected as SPI_SCK */
#define IOCFG_PA5_FUNC_ADC_IN5            ((uint32_t)0x00000003)        /*!< PA5 function selected as ADC_IN5 */

/******************  Bit definition for IOCFG_PA6 register   ******************/
#define IOCFG_PA6_FUNC_PA6                ((uint32_t)0x00000000)        /*!< PA6 function selected as PA6 (default) */
#define IOCFG_PA6_FUNC_PWM_FAULT          ((uint32_t)0x00000001)        /*!< PA6 function selected as PWM_FAULT */
#define IOCFG_PA6_FUNC_TIM2_CAP0          ((uint32_t)0x00000002)        /*!< PA6 function selected as TIM2_CAP0 */
#define IOCFG_PA6_FUNC_TIM2_MAT0          ((uint32_t)0x00000003)        /*!< PA6 function selected as TIM2_MAT0 */
#define IOCFG_PA6_FUNC_SPI_MISO           ((uint32_t)0x00000004)        /*!< PA6 function selected as SPI_MISO */
#define IOCFG_PA6_FUNC_ADC_IN6            ((uint32_t)0x00000006)        /*!< PA6 function selected as ADC_IN6 */

/******************  Bit definition for IOCFG_PA7 register   ******************/
#define IOCFG_PA7_FUNC_PA7                ((uint32_t)0x00000000)        /*!< PA7 function selected as PA7 (default) */
#define IOCFG_PA7_FUNC_PWM_OUT1           ((uint32_t)0x00000001)        /*!< PA7 function selected as PWM_OUT1 */
#define IOCFG_PA7_FUNC_TIM2_CAP1          ((uint32_t)0x00000002)        /*!< PA7 function selected as TIM2_CAP1 */
#define IOCFG_PA7_FUNC_TIM2_MAT1          ((uint32_t)0x00000003)        /*!< PA7 function selected as TIM2_MAT1 */
#define IOCFG_PA7_FUNC_SPI_MOSI           ((uint32_t)0x00000004)        /*!< PA7 function selected as SPI_MOSI */
#define IOCFG_PA7_FUNC_ADC_IN7            ((uint32_t)0x00000006)        /*!< PA7 function selected as ADC_IN7 */

/******************  Bit definition for IOCFG_PB0 register   ******************/
#define IOCFG_PB0_FUNC_PB0                ((uint32_t)0x00000000)        /*!< PB0 function selected as PB0 (default) */
#define IOCFG_PB0_FUNC_PWM_OUT3           ((uint32_t)0x00000001)        /*!< PB0 function selected as PWM_OUT3 */
#define IOCFG_PB0_FUNC_TIM2_CAP2          ((uint32_t)0x00000002)        /*!< PB0 function selected as TIM2_CAP2 */
#define IOCFG_PB0_FUNC_TIM2_MAT2          ((uint32_t)0x00000003)        /*!< PB0 function selected as TIM2_MAT2 */

/******************  Bit definition for IOCFG_PB1 register   ******************/
#define IOCFG_PB1_FUNC_PB1                ((uint32_t)0x00000000)        /*!< PB1 function selected as PB1 (default) */
#define IOCFG_PB1_FUNC_PWM_OUT5           ((uint32_t)0x00000001)        /*!< PB1 function selected as PWM_OUT5 */
#define IOCFG_PB1_FUNC_TIM2_CAP3          ((uint32_t)0x00000002)        /*!< PB1 function selected as TIM2_CAP3 */
#define IOCFG_PB1_FUNC_TIM2_MAT3          ((uint32_t)0x00000003)        /*!< PB1 function selected as TIM2_MAT3 */

/******************  Bit definition for IOCFG_PB2 register   ******************/
#define IOCFG_PB2_FUNC_PB2                ((uint32_t)0x00000000)        /*!< PB2 function selected as PB2 (default) */

/******************  Bit definition for IOCFG_PB10 register   ******************/
#define IOCFG_PB10_FUNC_PB10              ((uint32_t)0x00000000)        /*!< PB10 function selected as PB10 (default) */
#define IOCFG_PB10_FUNC_I2C_SCL           ((uint32_t)0x00000001)        /*!< PB10 function selected as I2C_SCL */

/******************  Bit definition for IOCFG_PB11 register   ******************/
#define IOCFG_PB11_FUNC_PB11              ((uint32_t)0x00000000)        /*!< PB11 function selected as PB11 (default) */
#define IOCFG_PB11_FUNC_I2C_SDA           ((uint32_t)0x00000001)        /*!< PB11 function selected as I2C_SDA */

/******************  Bit definition for IOCFG_PB12 register   ******************/
#define IOCFG_PB12_FUNC_PB12              ((uint32_t)0x00000000)        /*!< PB12 function selected as PB12 (default) */
#define IOCFG_PB12_FUNC_SPI_SSEL          ((uint32_t)0x00000001)        /*!< PB12 function selected as SPI_SSEL */
#define IOCFG_PB12_FUNC_PWM_FAULT         ((uint32_t)0x00000002)        /*!< PB12 function selected as PWM_FAULT */

/******************  Bit definition for IOCFG_PB13 register   ******************/
#define IOCFG_PB13_FUNC_PB13              ((uint32_t)0x00000000)        /*!< PB13 function selected as PB13 (default) */
#define IOCFG_PB13_FUNC_SPI_SCK           ((uint32_t)0x00000001)        /*!< PB13 function selected as SPI_SCK */
#define IOCFG_PB13_FUNC_PWM_OUT1          ((uint32_t)0x00000002)        /*!< PB13 function selected as PWM_OUT1 */

/******************  Bit definition for IOCFG_PB14 register   ******************/
#define IOCFG_PB14_FUNC_PB14              ((uint32_t)0x00000000)        /*!< PB14 function selected as PB14 (default) */
#define IOCFG_PB14_FUNC_SPI_MISO          ((uint32_t)0x00000001)        /*!< PB14 function selected as SPI_MISO */
#define IOCFG_PB14_FUNC_TIM2_CAP3         ((uint32_t)0x00000002)        /*!< PB14 function selected as TIM2_CAP3 */
#define IOCFG_PB14_FUNC_TIM2_MAT3         ((uint32_t)0x00000003)        /*!< PB14 function selected as TIM2_MAT3 */
#define IOCFG_PB14_FUNC_PWM_OUT3          ((uint32_t)0x00000004)        /*!< PB14 function selected as PWM_OUT3 */

/******************  Bit definition for IOCFG_PB15 register   ******************/
#define IOCFG_PB15_FUNC_PB15              ((uint32_t)0x00000000)        /*!< PB15 function selected as PB15 (default) */
#define IOCFG_PB15_FUNC_SPI_MOSI          ((uint32_t)0x00000001)        /*!< PB15 function selected as SPI_MOSI */
#define IOCFG_PB15_FUNC_TIM3_CAP3         ((uint32_t)0x00000002)        /*!< PB15 function selected as TIM3_CAP3 */
#define IOCFG_PB15_FUNC_TIM3_MAT3         ((uint32_t)0x00000003)        /*!< PB15 function selected as TIM3_MAT3 */
#define IOCFG_PB15_FUNC_PWM_OUT5          ((uint32_t)0x00000004)        /*!< PB15 function selected as PWM_OUT5 */

/******************  Bit definition for IOCFG_PA8 register   ******************/
#define IOCFG_PA8_FUNC_PA8                ((uint32_t)0x00000000)        /*!< PA8 function selected as PA8 (default) */
#define IOCFG_PA8_FUNC_PWM_OUT0           ((uint32_t)0x00000001)        /*!< PA8 function selected as PWM_OUT0 */
#define IOCFG_PA8_FUNC_TIM3_CAP3          ((uint32_t)0x00000002)        /*!< PA8 function selected as TIM3_CAP3 */
#define IOCFG_PA8_FUNC_TIM3_MAT3          ((uint32_t)0x00000003)        /*!< PA8 function selected as TIM3_MAT3 */
#define IOCFG_PA8_FUNC_CLKOUT             ((uint32_t)0x00000005)        /*!< PA8 function selected as CLKOUT (only used in GVM32F030x6) */

/******************  Bit definition for IOCFG_PA9 register   ******************/
#define IOCFG_PA9_FUNC_PA9                ((uint32_t)0x00000000)        /*!< PA9 function selected as PA9 (default) */
#define IOCFG_PA9_FUNC_PWM_OUT2           ((uint32_t)0x00000001)        /*!< PA9 function selected as PWM_OUT2 */
#define IOCFG_PA9_FUNC_TXD0               ((uint32_t)0x00000002)        /*!< PA9 function selected as TXD0 */
#define IOCFG_PA9_FUNC_I2C_SCL            ((uint32_t)0x00000004)        /*!< PA9 function selected as I2C_SCL */

/******************  Bit definition for IOCFG_PA10 register   ******************/
#define IOCFG_PA10_FUNC_PA10              ((uint32_t)0x00000000)        /*!< PA10 function selected as PA10 (default) */
#define IOCFG_PA10_FUNC_PWM_OUT4          ((uint32_t)0x00000001)        /*!< PA10 function selected as PWM_OUT4 */
#define IOCFG_PA10_FUNC_RXD0              ((uint32_t)0x00000002)        /*!< PA10 function selected as RXD0 */
#define IOCFG_PA10_FUNC_I2C_SDA           ((uint32_t)0x00000004)        /*!< PA10 function selected as I2C_SDA */

/******************  Bit definition for IOCFG_PA11 register   ******************/
#define IOCFG_PA11_FUNC_PA11              ((uint32_t)0x00000000)        /*!< PA11 function selected as PA11 (default) */
#define IOCFG_PA11_FUNC_TIM2_CAP2         ((uint32_t)0x00000002)        /*!< PA11 function selected as TIM2_CAP2 */
#define IOCFG_PA11_FUNC_TIM2_MAT2         ((uint32_t)0x00000003)        /*!< PA11 function selected as TIM2_MAT2 */

/******************  Bit definition for IOCFG_PA12 register   ******************/
#define IOCFG_PA12_FUNC_PA12              ((uint32_t)0x00000000)        /*!< PA12 function selected as PA12 (default) */

/******************  Bit definition for IOCFG_PA13 register   ******************/
#define IOCFG_PA13_FUNC_SWDIO             ((uint32_t)0x00000000)        /*!< PA13 function selected as SWDIO (default) */
#define IOCFG_PA13_FUNC_PA13              ((uint32_t)0x00000001)        /*!< PA13 function selected as PA13 */

/******************  Bit definition for IOCFG_PC10 register   ******************/
#define IOCFG_PC10_FUNC_PC10              ((uint32_t)0x00000000)        /*!< PC10 function selected as PC10 (default) */
#define IOCFG_PC10_FUNC_I2C_SCL           ((uint32_t)0x00000001)        /*!< PC10 function selected as I2C_SCL */

/******************  Bit definition for IOCFG_PC11 register   ******************/
#define IOCFG_PC11_FUNC_PC11              ((uint32_t)0x00000000)        /*!< PC11 function selected as PC11 (default) */
#define IOCFG_PC11_FUNC_I2C_SDA           ((uint32_t)0x00000001)        /*!< PC11 function selected as I2C_SDA */

/******************  Bit definition for IOCFG_PA14 register   ******************/
#define IOCFG_PA14_FUNC_SWDCLK            ((uint32_t)0x00000000)        /*!< PA14 function selected as SWDCLK (default) */
#define IOCFG_PA14_FUNC_PA14              ((uint32_t)0x00000001)        /*!< PA14 function selected as PA14 */
#define IOCFG_PA14_FUNC_TXD1              ((uint32_t)0x00000002)        /*!< PA14 function selected as TXD1 */

/******************  Bit definition for IOCFG_PA15 register   ******************/
#define IOCFG_PA15_FUNC_PA15              ((uint32_t)0x00000000)        /*!< PA15 function selected as PA15 (default) */
#define IOCFG_PA15_FUNC_SPI_SSEL          ((uint32_t)0x00000001)        /*!< PA15 function selected as SPI_SSEL */
#define IOCFG_PA15_FUNC_RXD1              ((uint32_t)0x00000002)        /*!< PA15 function selected as RXD1 */

/******************  Bit definition for IOCFG_PB3 register   ******************/
/* Note: This pin functions as WAKEUP pin if the part is in Power-down mode regardless of the value of FUNC */
#define IOCFG_PB3_FUNC_PB3                ((uint32_t)0x00000000)        /*!< PB3 function selected as PB3 (default) */
#define IOCFG_PB3_FUNC_SPI_SCK            ((uint32_t)0x00000001)        /*!< PB3 function selected as SPI_SCK */

/******************  Bit definition for IOCFG_PB4 register   ******************/
#define IOCFG_PB4_FUNC_PB4                ((uint32_t)0x00000000)        /*!< PB4 function selected as PB4 (default) */
#define IOCFG_PB4_FUNC_SPI_MISO           ((uint32_t)0x00000001)        /*!< PB4 function selected as SPI_MISO */
#define IOCFG_PB4_FUNC_TIM2_CAP0          ((uint32_t)0x00000002)        /*!< PB4 function selected as TIM2_CAP0 */
#define IOCFG_PB4_FUNC_TIM2_MAT0          ((uint32_t)0x00000003)        /*!< PB4 function selected as TIM2_MAT0 */

/******************  Bit definition for IOCFG_PB5 register   ******************/
#define IOCFG_PB5_FUNC_PB5                ((uint32_t)0x00000000)        /*!< PB5 function selected as PB5 (default) */
#define IOCFG_PB5_FUNC_SPI_MOSI           ((uint32_t)0x00000001)        /*!< PB5 function selected as SPI_MOSI */
#define IOCFG_PB5_FUNC_TIM2_CAP1          ((uint32_t)0x00000002)        /*!< PB5 function selected as TIM2_CAP1 */
#define IOCFG_PB5_FUNC_TIM2_MAT1          ((uint32_t)0x00000003)        /*!< PB5 function selected as TIM2_MAT1 */

/******************  Bit definition for IOCFG_PB6 register   ******************/
#define IOCFG_PB6_FUNC_PB6                ((uint32_t)0x00000000)        /*!< PB6 function selected as PB6 (default) */
#define IOCFG_PB6_FUNC_I2C_SCL            ((uint32_t)0x00000001)        /*!< PB6 function selected as I2C_SCL */
#define IOCFG_PB6_FUNC_TXD1               ((uint32_t)0x00000002)        /*!< PB6 function selected as TXD1 */
#define IOCFG_PB6_FUNC_TIM3_CAP0          ((uint32_t)0x00000004)        /*!< PB6 function selected as TIM3_CAP0 */
#define IOCFG_PB6_FUNC_TIM3_MAT0          ((uint32_t)0x00000005)        /*!< PB6 function selected as TIM3_MAT0 */

/******************  Bit definition for IOCFG_PB7 register   ******************/
#define IOCFG_PB7_FUNC_PB7                ((uint32_t)0x00000000)        /*!< PB7 function selected as PB7 (default) */
#define IOCFG_PB7_FUNC_I2C_SDA            ((uint32_t)0x00000001)        /*!< PB7 function selected as I2C_SDA */
#define IOCFG_PB7_FUNC_RXD1               ((uint32_t)0x00000002)        /*!< PB7 function selected as RXD1 */
#define IOCFG_PB7_FUNC_TIM3_CAP1          ((uint32_t)0x00000004)        /*!< PB7 function selected as TIM3_CAP1 */
#define IOCFG_PB7_FUNC_TIM3_MAT1          ((uint32_t)0x00000005)        /*!< PB7 function selected as TIM3_MAT1 */

/******************  Bit definition for IOCFG_PB8 register   ******************/
#define IOCFG_PB8_FUNC_PB8                ((uint32_t)0x00000000)        /*!< PB8 function selected as PB8 (default) */
#define IOCFG_PB8_FUNC_I2C_SCL            ((uint32_t)0x00000001)        /*!< PB8 function selected as I2C_SCL */
#define IOCFG_PB8_FUNC_TIM3_CAP2          ((uint32_t)0x00000002)        /*!< PB8 function selected as TIM3_CAP2 */
#define IOCFG_PB8_FUNC_TIM3_MAT2          ((uint32_t)0x00000003)        /*!< PB8 function selected as TIM3_MAT2 */

/******************  Bit definition for IOCFG_PB9 register   ******************/
#define IOCFG_PB9_FUNC_PB9                ((uint32_t)0x00000000)        /*!< PB9 function selected as PB9 (default) */
#define IOCFG_PB9_FUNC_I2C_SDA            ((uint32_t)0x00000001)        /*!< PB9 function selected as I2C_SDA */

/******************  Bit definition for IOCFG_PC0 register   ******************/
#define IOCFG_PC0_FUNC_PC0                ((uint32_t)0x00000000)        /*!< PC0 function selected as PC0 (default) */

/******************  Bit definition for IOCFG_PC1 register   ******************/
#define IOCFG_PC1_FUNC_PC1                ((uint32_t)0x00000000)        /*!< PC1 function selected as PC1 (default) */

/******************  Bit definition for IOCFG_PC2 register   ******************/
#define IOCFG_PC2_FUNC_NRST               ((uint32_t)0x00000000)        /*!< PC2 function selected as NRST (default) */
#define IOCFG_PC2_FUNC_PC2                ((uint32_t)0x00000001)        /*!< PC2 function selected as PC2 */

/******************  Bit definition for IOCFG_PC3 register   ******************/
#define IOCFG_PC3_FUNC_PC3                ((uint32_t)0x00000000)        /*!< PC3 function selected as PC3 (default) */



/******************************************************************************/
/*                                                                            */
/*                 Universal Asynchronous Receiver Transmitter                */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for UART_SR register   ******************/
#define UART_SR_TXNE                      ((uint32_t)0x00000001)        /*!< UART state tx fifo not empty */
#define UART_SR_RXNE                      ((uint32_t)0x00000002)        /*!< UART state rx fifo not empty */
#define UART_SR_TXF                       ((uint32_t)0x00000004)        /*!< UART state tx fifo full */
#define UART_SR_RXF                       ((uint32_t)0x00000008)        /*!< UART state rx fifo full */
#define UART_SR_TXHLF                     ((uint32_t)0x00000010)        /*!< UART state tx fifo half */
#define UART_SR_RXHLF                     ((uint32_t)0x00000020)        /*!< UART state rx fifo half */
#define UART_SR_PARIERR                   ((uint32_t)0x00000040)        /*!< UART state parity check error */
#define UART_SR_OVERRUN                   ((uint32_t)0x00000080)        /*!< UART state rx fifo overflow */

/******************  Bit definition for UART_CR register   ******************/
#define UART_CR_TXEIE                     ((uint32_t)0x00000001)        /*!< UART enable tx fifo empty interrupt */
#define UART_CR_RXNEIE                    ((uint32_t)0x00000002)        /*!< UART enable rx fifo not empty interrupt */
#define UART_CR_TXFIE                     ((uint32_t)0x00000004)        /*!< UART enable tx fifo full interrupt */
#define UART_CR_RXFIE                     ((uint32_t)0x00000008)        /*!< UART enable rx fifo full interrupt */
#define UART_CR_TXHLFIE                   ((uint32_t)0x00000010)        /*!< UART enable tx fifo half interrupt */
#define UART_CR_RXHLFIE                   ((uint32_t)0x00000020)        /*!< UART enable rx fifo half interrupt */
#define UART_CR_PARIERRIE                 ((uint32_t)0x00000040)        /*!< UART enable parity check error interrupt */
#define UART_CR_OVERRUNIE                 ((uint32_t)0x00000080)        /*!< UART enable rx fifo overflow interrupt */
#define UART_CR_PARIODD                   ((uint32_t)0x00000100)        /*!< UART parity odd check */
#define UART_CR_PARIEN                    ((uint32_t)0x00000200)        /*!< UART parity enable */
#define UART_CR_IRDAEN                    ((uint32_t)0x00000400)        /*!< UART IrDA enable */
#define UART_CR_RXEN                      ((uint32_t)0x00400000)        /*!< UART rx enable */
#define UART_CR_TXEN                      ((uint32_t)0x00800000)        /*!< UART tx enable */

/******************  Bit definition for UART_ISR register   ******************/
#define UART_ISR_TXEINT                   ((uint32_t)0x00000001)        /*!< UART tx fifo empty interrupt */
#define UART_ISR_RXNEINT                  ((uint32_t)0x00000002)        /*!< UART rx fifo not empty interrupt */
#define UART_ISR_TXFINT                   ((uint32_t)0x00000004)        /*!< UART tx fifo full interrupt */
#define UART_ISR_RXFINT                   ((uint32_t)0x00000008)        /*!< UART rx fifo full interrupt */
#define UART_ISR_TXHLFINT                 ((uint32_t)0x00000010)        /*!< UART tx fifo half interrupt */
#define UART_ISR_RXHLFINT                 ((uint32_t)0x00000020)        /*!< UART rx fifo half interrupt */
#define UART_ISR_PARIERRINT               ((uint32_t)0x00000040)        /*!< UART parity check error interrupt */
#define UART_ISR_OVERRUNINT               ((uint32_t)0x00000080)        /*!< UART rx fifo overflow interrupt */

/******************  Bit definition for UART_FIFOCLR register   ******************/
#define UART_FIFOCLR_TXFIFOCLR            ((uint32_t)0x00000001)        /*!< UART tx fifo clear */
#define UART_FIFOCLR_RXFIFOCLR            ((uint32_t)0x00000002)        /*!< UART rx fifo clear */




/******************************************************************************/
/*                                                                            */
/*                          Serial Peripheral Interface                       */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SPI_CR0 register   ******************/
#define SPI_CR0_DSS_4Bit                  ((uint32_t)0x00000003)        /*!< SPI DSS 4 Bit */
#define SPI_CR0_DSS_5Bit                  ((uint32_t)0x00000004)        /*!< SPI DSS 5 Bit */
#define SPI_CR0_DSS_6Bit                  ((uint32_t)0x00000005)        /*!< SPI DSS 6 Bit */
#define SPI_CR0_DSS_7Bit                  ((uint32_t)0x00000006)        /*!< SPI DSS 7 Bit */
#define SPI_CR0_DSS_8Bit                  ((uint32_t)0x00000007)        /*!< SPI DSS 8 Bit */
#define SPI_CR0_DSS_9Bit                  ((uint32_t)0x00000008)        /*!< SPI DSS 9 Bit */
#define SPI_CR0_DSS_10Bit                 ((uint32_t)0x00000009)        /*!< SPI DSS 10 Bit */
#define SPI_CR0_DSS_11Bit                 ((uint32_t)0x0000000A)        /*!< SPI DSS 11 Bit */
#define SPI_CR0_DSS_12Bit                 ((uint32_t)0x0000000B)        /*!< SPI DSS 12 Bit */
#define SPI_CR0_DSS_13Bit                 ((uint32_t)0x0000000C)        /*!< SPI DSS 13 Bit */
#define SPI_CR0_DSS_14Bit                 ((uint32_t)0x0000000D)        /*!< SPI DSS 14 Bit */
#define SPI_CR0_DSS_15Bit                 ((uint32_t)0x0000000E)        /*!< SPI DSS 15 Bit */
#define SPI_CR0_DSS_16Bit                 ((uint32_t)0x0000000F)        /*!< SPI DSS 16 Bit */

#define SPI_CR0_CPOL                      ((uint32_t)0x00000040)        /*!< SPI Clock Polarity */
#define SPI_CR0_CPHA                      ((uint32_t)0x00000080)        /*!< SPI Clock Phase */

/******************  Bit definition for SPI_CR1 register   ******************/
#define SPI_CR1_LBM                       ((uint32_t)0x00000001)        /*!< SPI Loop Back Mode */
#define SPI_CR1_SSE                       ((uint32_t)0x00000002)        /*!< SPI Enable */
#define SPI_CR1_MS                        ((uint32_t)0x00000004)        /*!< SPI Slave mode */
#define SPI_CR1_SOD                       ((uint32_t)0x00000008)        /*!< SPI Slave Output Disable */
#define SPI_CR1_CSFL                      ((uint32_t)0x00000010)        /*!< SPI CS  */
#define SPI_CR1_RSFR                      ((uint32_t)0x00000020)        /*!< SPI Clear FIFO */

/******************  Bit definition for SPI_SR register   ******************/
#define SPI_SR_TFE                        ((uint32_t)0x00000001)        /*!< SPI TX FIFO empty */
#define SPI_SR_TNF                        ((uint32_t)0x00000002)        /*!< SPI TX FIFO not full */
#define SPI_SR_RNE                        ((uint32_t)0x00000004)        /*!< SPI RX FIFO not empty */
#define SPI_SR_RFF                        ((uint32_t)0x00000008)        /*!< SPI RX FIFO full */
#define SPI_SR_BSY                        ((uint32_t)0x00000010)        /*!< SPI Busy */

/******************  Bit definition for SPI_IMSC register   ******************/
#define SPI_IMSC_RORIM                    ((uint32_t)0x00000001)        /*!< SPI RX FIFO overrun interrupt enable */
#define SPI_IMSC_RTIM                     ((uint32_t)0x00000002)        /*!< SPI receive time-out interrupt enable */
#define SPI_IMSC_RXIM                     ((uint32_t)0x00000004)        /*!< SPI RX FIFO is at least half full interrupt enable */
#define SPI_IMSC_TXIM                     ((uint32_t)0x00000008)        /*!< SPI TX FIFO is at least half empty interrupt enable */

/******************  Bit definition for SPI_RIS register   ******************/
#define SPI_RIS_RORRIS                    ((uint32_t)0x00000001)        /*!< SPI RX FIFO overrun flag */
#define SPI_RIS_RTRIS                     ((uint32_t)0x00000002)        /*!< SPI receive time-out flag */
#define SPI_RIS_RXRIS                     ((uint32_t)0x00000004)        /*!< SPI RX FIFO is at least half full flag */
#define SPI_RIS_TXRIS                     ((uint32_t)0x00000008)        /*!< SPI TX FIFO is at least half empty flag */

/******************  Bit definition for SPI_MIS register   ******************/
#define SPI_MIS_RORMIS                    ((uint32_t)0x00000001)        /*!< SPI RX FIFO overrun masked flag */
#define SPI_MIS_RTMIS                     ((uint32_t)0x00000002)        /*!< SPI receive time-out masked flag */
#define SPI_MIS_RXMIS                     ((uint32_t)0x00000004)        /*!< SPI RX FIFO is at least half full masked flag */
#define SPI_MIS_TXMIS                     ((uint32_t)0x00000008)        /*!< SPI TX FIFO is at least half empty masked flag */

/******************  Bit definition for SPI_ICR register   ******************/
#define SPI_ICR_RORIC                     ((uint32_t)0x00000001)        /*!< SPI clear RX FIFO overrun interrupt */
#define SPI_ICR_RTIC                      ((uint32_t)0x00000002)        /*!< SPI clear RX timeout interrupt */



/******************************************************************************/
/*                                                                            */
/*                           Inter－Integrated Circuit                        */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for I2C_CONSET register   ******************/
#define I2C_CONSET_TXRX                   ((uint32_t)0x00000001)        /*!< I2C */
#define I2C_CONSET_MASL                   ((uint32_t)0x00000002)        /*!< I2C */
#define I2C_CONSET_AA                     ((uint32_t)0x00000004)        /*!< I2C */
#define I2C_CONSET_SI                     ((uint32_t)0x00000008)        /*!< I2C */
#define I2C_CONSET_STO                    ((uint32_t)0x00000010)        /*!< I2C */
#define I2C_CONSET_STA                    ((uint32_t)0x00000020)        /*!< I2C */
#define I2C_CONSET_I2CEN                  ((uint32_t)0x00000040)        /*!< I2C */

/******************  Bit definition for I2C_CONCLR register   ******************/
#define I2C_CONCLR_TXRX                   ((uint32_t)0x00000001)        /*!< I2C */
#define I2C_CONCLR_MASL                   ((uint32_t)0x00000002)        /*!< I2C */
#define I2C_CONCLR_AAC                    ((uint32_t)0x00000004)        /*!< I2C */
#define I2C_CONCLR_SIC                    ((uint32_t)0x00000008)        /*!< I2C */
#define I2C_CONCLR_STAC                   ((uint32_t)0x00000020)        /*!< I2C */
#define I2C_CONCLR_I2CENC                 ((uint32_t)0x00000040)        /*!< I2C */

#define I2C_CONCLR_SLVADDMATCH            ((uint32_t)0x00000100)        /*!< I2C */

/******************  Bit definition for I2C_STAT register   ******************/
#define I2C_STAT_SLVADDMATCH              ((uint32_t)0x00000100)        /*!< I2C */
#define I2C_STAT_SLVRXBUFFULL             ((uint32_t)0x00000200)        /*!< I2C */
#define I2C_STAT_SLVTXBUFEMPTY            ((uint32_t)0x00000400)        /*!< I2C */






/******************************************************************************/
/*                                                                            */
/*                                 Timer                                      */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for TIM_IR register   ******************/
#define TIM_IR_MR0INT                     ((uint32_t)0x00000001)        /*!< TIM Interrupt flag for match channel 0 */
#define TIM_IR_MR1INT                     ((uint32_t)0x00000002)        /*!< TIM Interrupt flag for match channel 1 */
#define TIM_IR_MR2INT                     ((uint32_t)0x00000004)        /*!< TIM Interrupt flag for match channel 2 */
#define TIM_IR_MR3INT                     ((uint32_t)0x00000008)        /*!< TIM Interrupt flag for match channel 3 */
#define TIM_IR_CR0INT                     ((uint32_t)0x00000010)        /*!< TIM Interrupt flag for capture channel 0 event */
#define TIM_IR_CR1INT                     ((uint32_t)0x00000020)        /*!< TIM Interrupt flag for capture channel 1 event */
#define TIM_IR_CR2INT                     ((uint32_t)0x00000040)        /*!< TIM Interrupt flag for capture channel 2 event */
#define TIM_IR_CR3INT                     ((uint32_t)0x00000080)        /*!< TIM Interrupt flag for capture channel 3 event */

/******************  Bit definition for TIM_TCR register   ******************/
#define TIM_TCR_CEN                       ((uint32_t)0x00000001)        /*!< TIM Counter enable */
#define TIM_TCR_CRST                      ((uint32_t)0x00000002)        /*!< TIM Counter reset */

/******************  Bit definition for TIM_MCR register   ******************/
#define TIM_MCR_MR0I                      ((uint32_t)0x00000001)        /*!< TIM Interrupt on MR0 */
#define TIM_MCR_MR0R                      ((uint32_t)0x00000002)        /*!< TIM Reset on MR0 */
#define TIM_MCR_MR0S                      ((uint32_t)0x00000004)        /*!< TIM Stop on MR0 */
#define TIM_MCR_MR1I                      ((uint32_t)0x00000008)        /*!< TIM Interrupt on MR1 */
#define TIM_MCR_MR1R                      ((uint32_t)0x00000010)        /*!< TIM Reset on MR1 */
#define TIM_MCR_MR1S                      ((uint32_t)0x00000020)        /*!< TIM Stop on MR1 */
#define TIM_MCR_MR2I                      ((uint32_t)0x00000040)        /*!< TIM Interrupt on MR2 */
#define TIM_MCR_MR2R                      ((uint32_t)0x00000080)        /*!< TIM Reset on MR2 */
#define TIM_MCR_MR2S                      ((uint32_t)0x00000100)        /*!< TIM Stop on MR2 */
#define TIM_MCR_MR3I                      ((uint32_t)0x00000200)        /*!< TIM Interrupt on MR3 */
#define TIM_MCR_MR3R                      ((uint32_t)0x00000400)        /*!< TIM Reset on MR3 */
#define TIM_MCR_MR3S                      ((uint32_t)0x00000800)        /*!< TIM Stop on MR3 */

/******************  Bit definition for TIM_CCR register   ******************/
#define TIM_CCR_CAP0RE                    ((uint32_t)0x00000001)        /*!< Capture on TIMx_CAP0 rising edge */
#define TIM_CCR_CAP0FE                    ((uint32_t)0x00000002)        /*!< Capture on TIMx_CAP0 falling edge */
#define TIM_CCR_CAP0I                     ((uint32_t)0x00000004)        /*!< Interrupt on TIMx_CAP0 event */
#define TIM_CCR_CAP1RE                    ((uint32_t)0x00000008)        /*!< Capture on TIMx_CAP1 rising edge */
#define TIM_CCR_CAP1FE                    ((uint32_t)0x00000010)        /*!< Capture on TIMx_CAP1 falling edge */
#define TIM_CCR_CAP1I                     ((uint32_t)0x00000020)        /*!< Interrupt on TIMx_CAP1 event */
#define TIM_CCR_CAP2RE                    ((uint32_t)0x00000040)        /*!< Capture on TIMx_CAP2 rising edge */
#define TIM_CCR_CAP2FE                    ((uint32_t)0x00000080)        /*!< Capture on TIMx_CAP2 falling edge */
#define TIM_CCR_CAP2I                     ((uint32_t)0x00000100)        /*!< Interrupt on TIMx_CAP2 event */
#define TIM_CCR_CAP3RE                    ((uint32_t)0x00000200)        /*!< Capture on TIMx_CAP3 rising edge */
#define TIM_CCR_CAP3FE                    ((uint32_t)0x00000400)        /*!< Capture on TIMx_CAP3 falling edge */
#define TIM_CCR_CAP3I                     ((uint32_t)0x00000800)        /*!< Interrupt on TIMx_CAP3 event */

/******************  Bit definition for TIM_EMR register   ******************/
#define TIM_EMR_EM0                       ((uint32_t)0x00000001)        /*!< External Match 0 */
#define TIM_EMR_EM1                       ((uint32_t)0x00000002)        /*!< External Match 1 */
#define TIM_EMR_EM2                       ((uint32_t)0x00000004)        /*!< External Match 2 */
#define TIM_EMR_EM3                       ((uint32_t)0x00000008)        /*!< External Match 3 */

#define TIM_EMR_EMC_DO_NOTHING            ((uint32_t)0x00000000)        /*!< Timer match state does nothing on match pin */
#define TIM_EMR_EMC_CLEAR                 ((uint32_t)0x00000001)        /*!< Timer match state sets match pin low */
#define TIM_EMR_EMC_SET                   ((uint32_t)0x00000002)        /*!< Timer match state sets match pin high */
#define TIM_EMR_EMC_TOGGLE                ((uint32_t)0x00000003)        /*!< Timer match state toggles match pin */

#define TIM_EMR_EMC0_DO_NOTHING           ((uint32_t)0x00000000)        /*!< Timer match0 state does nothing on match pin */
#define TIM_EMR_EMC0_CLEAR                ((uint32_t)0x00000010)        /*!< Timer match0 state sets match pin low */
#define TIM_EMR_EMC0_SET                  ((uint32_t)0x00000020)        /*!< Timer match0 state sets match pin high */
#define TIM_EMR_EMC0_TOGGLE               ((uint32_t)0x00000030)        /*!< Timer match0 state toggles match pin */
#define TIM_EMR_EMC1_DO_NOTHING           ((uint32_t)0x00000000)        /*!< Timer match1 state does nothing on match pin */
#define TIM_EMR_EMC1_CLEAR                ((uint32_t)0x00000040)        /*!< Timer match1 state sets match pin low */
#define TIM_EMR_EMC1_SET                  ((uint32_t)0x00000080)        /*!< Timer match1 state sets match pin high */
#define TIM_EMR_EMC1_TOGGLE               ((uint32_t)0x000000C0)        /*!< Timer match1 state toggles match pin */
#define TIM_EMR_EMC2_DO_NOTHING           ((uint32_t)0x00000000)        /*!< Timer match2 state does nothing on match pin */
#define TIM_EMR_EMC2_CLEAR                ((uint32_t)0x00000100)        /*!< Timer match2 state sets match pin low */
#define TIM_EMR_EMC2_SET                  ((uint32_t)0x00000200)        /*!< Timer match2 state sets match pin high */
#define TIM_EMR_EMC2_TOGGLE               ((uint32_t)0x00000300)        /*!< Timer match2 state toggles match pin */
#define TIM_EMR_EMC3_DO_NOTHING           ((uint32_t)0x00000000)        /*!< Timer match3 state does nothing on match pin */
#define TIM_EMR_EMC3_CLEAR                ((uint32_t)0x00000400)        /*!< Timer match3 state sets match pin low */
#define TIM_EMR_EMC3_SET                  ((uint32_t)0x00000800)        /*!< Timer match3 state sets match pin high */
#define TIM_EMR_EMC3_TOGGLE               ((uint32_t)0x00000C00)        /*!< Timer match3 state toggles match pin */


/******************  Bit definition for TIM_CTCR register   ******************/
#define TIM_CTCR_CTM                      ((uint32_t)0x00000007)        /*!< TIM ### */
#define TIM_CTCR_CTM_TIMER                ((uint32_t)0x00000000)        /*!< TIM ### */
#define TIM_CTCR_CTM_COUNTER_RE           ((uint32_t)0x00000001)        /*!< TIM ### */
#define TIM_CTCR_CTM_COUNTER_FE           ((uint32_t)0x00000002)        /*!< TIM ### */
#define TIM_CTCR_CTM_COUNTER_RFE          ((uint32_t)0x00000003)        /*!< TIM ### */
#define TIM_CTCR_CTM_ENCODER              ((uint32_t)0x00000004)        /*!< TIM ### */
/*############################################*/

#define TIM_CTCR_ENCC                     ((uint32_t)0x00000008)        /*!< TIM ### */

#define TIM_CTCR_SELCC                    ((uint32_t)0x000000F0)        /*!< TIM ### */
/*############################################*/

#define TIM_CTCR_PRISEL                   ((uint32_t)0x00000F00)        /*!< TIM ### */
#define TIM_CTCR_PRISEL_CAP0              ((uint32_t)0x00000000)        /*!< TIM ### */
#define TIM_CTCR_PRISEL_CAP1              ((uint32_t)0x00000100)        /*!< TIM ### */
#define TIM_CTCR_PRISEL_CAP2              ((uint32_t)0x00000200)        /*!< TIM ### */
#define TIM_CTCR_PRISEL_CAP3              ((uint32_t)0x00000300)        /*!< TIM ### */
#define TIM_CTCR_PRISEL_PWM_RELOAD        ((uint32_t)0x00000400)        /*!< TIM ### */
#define TIM_CTCR_PRISEL_TIM23_MAT0        ((uint32_t)0x00000500)        /*!< TIM ### */

#define TIM_CTCR_SECSEL                   ((uint32_t)0x0000F000)        /*!< TIM ### */
#define TIM_CTCR_SECSEL_CAP0              ((uint32_t)0x00000000)        /*!< TIM ### */
#define TIM_CTCR_SECSEL_CAP1              ((uint32_t)0x00001000)        /*!< TIM ### */
#define TIM_CTCR_SECSEL_CAP2              ((uint32_t)0x00002000)        /*!< TIM ### */
#define TIM_CTCR_SECSEL_CAP3              ((uint32_t)0x00003000)        /*!< TIM ### */
#define TIM_CTCR_SECSEL_PWM_RELOAD        ((uint32_t)0x00004000)        /*!< TIM ### */
#define TIM_CTCR_SECSEL_TIM23_MAT0        ((uint32_t)0x00005000)        /*!< TIM ### */

#define TIM_CTCR_IPS                      ((uint32_t)0x00010000)        /*!< TIM ### */




/******************************************************************************/
/*                                                                            */
/*                          Cyclic Redundancy Code (CRC)                      */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for CRC_MODE register   ******************/
#define CRC_MODE_POLY                     ((uint32_t)0x00000003)        /*!< CRC Poly */
#define CRC_MODE_POLY_CRC_CCITT           ((uint32_t)0x00000000)        /*!< CRC Mode CRC-CCITT */
#define CRC_MODE_POLY_CRC_16              ((uint32_t)0x00000001)        /*!< CRC Mode CRC16 */
#define CRC_MODE_POLY_CRC_32              ((uint32_t)0x00000002)        /*!< CRC Mode CRC32 */

#define CRC_MODE_BIT_RVS_WR               ((uint32_t)0x00000004)        /*!< CRC bit reverse */
#define CRC_MODE_CMPL_WR                  ((uint32_t)0x00000008)        /*!< CRC ### */
#define CRC_MODE_BIT_RVS_SUM              ((uint32_t)0x00000010)        /*!< CRC ### */
#define CRC_MODE_CMPL_SUM                 ((uint32_t)0x00000020)        /*!< CRC ### */
#define CRC_MODE_SEED_OP                  ((uint32_t)0x00000040)        /*!< CRC ### */
#define CRC_MODE_SEED_SET                 ((uint32_t)0x00000080)        /*!< CRC ### */




/******************************************************************************/
/*                                                                            */
/*                              Watchdog Timer (WDT)                          */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for WDT_MODE register   ******************/
#define WDT_MODE_WDEN                     ((uint32_t)0x00000001)        /*!< Watchdog timer Enable */
#define WDT_MODE_WDRESET                  ((uint32_t)0x00000002)        /*!< Watchdog timer trigger reset */
#define WDT_MODE_WDTOF                    ((uint32_t)0x00000004)        /*!< Watchdog timer timeout flag */
#define WDT_MODE_WDINT                    ((uint32_t)0x00000008)        /*!< Watchdog timer interrupt flag */
#define WDT_MODE_WDPROTECT                ((uint32_t)0x00000010)        /*!< Watchdog timer protect mode */
#define WDT_MODE_WDLOCKCLK                ((uint32_t)0x00000030)        /*!< Watchdog timer lock WDCLK */
#define WDT_MODE_WDLOCKDP                 ((uint32_t)0x00000040)        /*!< Watchdog timer disable MCU enter power down mode */
#define WDT_MODE_WDLOCKEN                 ((uint32_t)0x00000080)        /*!< Watchdog timer lock */

#define WDT_FEED_CMD1                     ((uint32_t)0x000000AA)        /*!< Watchdog timer Feed sequence 1 */
#define WDT_FEED_CMD2                     ((uint32_t)0x00000055)        /*!< Watchdog timer Feed sequence 2 */


/******************************************************************************/
/*                                                                            */
/*                                   Flash                                    */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for FLASH_CR register   ******************/
#define FLASH_CR_WRITE                    ((uint32_t)0x00000002)        /*!< Flash write */
#define FLASH_CR_ERASE                    ((uint32_t)0x00000004)        /*!< Flash erase */
#define FLASH_CR_BUSY                     ((uint32_t)0x00000100)        /*!< Flash busy bit */



/******************************************************************************/
/*                                                                            */
/*                                    ADC                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for ADC_CR register  *********************/
#define ADC_CR_CNVEN                      ((uint32_t)0x000000FF)        /*!< ADC channel convert enable mask */
#define ADC_CR_CLKDIV                     ((uint32_t)0x0000FF00)        /*!< ADC convert clock divide mask */
#define ADC_CR_BURST                      ((uint32_t)0x00010000)        /*!< ADC burst mode control */
#define ADC_CR_START                      ((uint32_t)0x0F000000)        /*!< ADC start control in trigger mode mask */

#define ADC_CR_START_SOFTWARE             ((uint32_t)0x01000000)        /*!< ADC start when software trigger */
#define ADC_CR_START_TIM2_CAP0            ((uint32_t)0x02000000)        /*!< ADC start when TIM2 capture 0 */
#define ADC_CR_START_TIM2_CAP1            ((uint32_t)0x03000000)        /*!< ADC start when TIM2 capture 1 */
#define ADC_CR_START_TIM2_MAT0            ((uint32_t)0x04000000)        /*!< ADC start when TIM2 match 0 */
#define ADC_CR_START_TIM2_MAT1            ((uint32_t)0x05000000)        /*!< ADC start when TIM2 match 1 */
#define ADC_CR_START_TIM3_MAT0            ((uint32_t)0x06000000)        /*!< ADC start when TIM3 match 0 */
#define ADC_CR_START_TIM3_MAT1            ((uint32_t)0x07000000)        /*!< ADC start when TIM3 match 1 */
#define ADC_CR_START_PWM_RELOAD           ((uint32_t)0x08000000)        /*!< ADC start when PWM reload */

#define ADC_CR_EDGE                       ((uint32_t)0x10000000)        /*!< ADC edge control in trigger mode */
#define ADC_CR_SCMODE                     ((uint32_t)0x20000000)        /*!< ADC converter sample clock selection */


/*******************  Bit definition for ADC_GDR register  *********************/
#define ADC_GDR_RESULT                    ((uint32_t)0x00000FFF)        /*!< ADC global data convert result mask */
#define ADC_GDR_CHN                       ((uint32_t)0x00007000)        /*!< ADC global data channel number mask */
#define ADC_GDR_OVERRUN                   ((uint32_t)0x00008000)        /*!< ADC global data override flag in burst mode */
#define ADC_GDR_DONE                      ((uint32_t)0x00010000)        /*!< ADC global data convert done flag */


/*******************  Bit definition for ADC_CHSEL register  *********************/
#define ADC_CHSEL_DR0CHSEL                ((uint32_t)0x00000007)        /*!< ADC select channel convert use DR0 */
#define ADC_CHSEL_DR1CHSEL                ((uint32_t)0x00000070)        /*!< ADC select channel convert use DR1 */
#define ADC_CHSEL_DR2CHSEL                ((uint32_t)0x00000700)        /*!< ADC select channel convert use DR2 */
#define ADC_CHSEL_DR3CHSEL                ((uint32_t)0x00007000)        /*!< ADC select channel convert use DR3 */
#define ADC_CHSEL_DR4CHSEL                ((uint32_t)0x00070000)        /*!< ADC select channel convert use DR4 */
#define ADC_CHSEL_DR5CHSEL                ((uint32_t)0x00700000)        /*!< ADC select channel convert use DR5 */
#define ADC_CHSEL_DR6CHSEL                ((uint32_t)0x07000000)        /*!< ADC select channel convert use DR6 */
#define ADC_CHSEL_DR7CHSEL                ((uint32_t)0x70000000)        /*!< ADC select channel convert use DR7 */


/*******************  Bit definition for ADC_INTEN register  *********************/
#define ADC_INTEN_INTEN                   ((uint32_t)0x000000FF)        /*!< ADC channel convert interrupt mask */
#define ADC_INTEN_GINTEN                  ((uint32_t)0x00000100)        /*!< ADC global convert done interrupt */


/*******************  Bit definition for ADC_DR register  *********************/
#define ADC_DR_RESULT                     ((uint32_t)0x00000FFF)        /*!< ADC data convert result mask */
#define ADC_DR_OVERRUN                    ((uint32_t)0x40000000)        /*!< ADC data override flag in burst mode */
#define ADC_DR_DONE                       ((uint32_t)0x80000000)        /*!< ADC data convert done flag */


/*******************  Bit definition for ADC_STAT register  *********************/
#define ADC_STAT_DONE                     ((uint32_t)0x000000FF)        /*!< ADC state channel convert done mask */
#define ADC_STAT_OVERRUN                  ((uint32_t)0x0000FF00)        /*!< ADC state channel convert override mask */

#define ADC_STAT_DR0_DONE                 ((uint32_t)0x00000001)        /*!< ADC CH0 Done flag */
#define ADC_STAT_DR1_DONE                 ((uint32_t)0x00000002)        /*!< ADC CH1 Done flag */
#define ADC_STAT_DR2_DONE                 ((uint32_t)0x00000004)        /*!< ADC CH2 Done flag */
#define ADC_STAT_DR3_DONE                 ((uint32_t)0x00000008)        /*!< ADC CH3 Done flag */
#define ADC_STAT_DR4_DONE                 ((uint32_t)0x00000010)        /*!< ADC CH4 Done flag */
#define ADC_STAT_DR5_DONE                 ((uint32_t)0x00000020)        /*!< ADC CH5 Done flag */
#define ADC_STAT_DR6_DONE                 ((uint32_t)0x00000040)        /*!< ADC CH6 Done flag */
#define ADC_STAT_DR7_DONE                 ((uint32_t)0x00000080)        /*!< ADC CH7 Done flag */
#define ADC_STAT_ADINT                    ((uint32_t)0x00010000)        /*!< ADC state interrupt flag */
#define ADC_STAT_HILMTFLAG0               ((uint32_t)0x00020000)        /*!< ADC state value > HILMT0 flag */
#define ADC_STAT_HILMTFLAG1               ((uint32_t)0x00040000)        /*!< ADC state value > HILMT1 flag */
#define ADC_STAT_LOLMTFLAG0               ((uint32_t)0x00080000)        /*!< ADC state value < LOLMT0 flag */
#define ADC_STAT_LOLMTFLAG1               ((uint32_t)0x00100000)        /*!< ADC state value < LOLMT1 flag */
#define ADC_STAT_ADCRDY                   ((uint32_t)0x00200000)        /*!< ADC ready flag */


/*******************  Bit definition for ADC_HILMT register  *********************/
#define ADC_HILMT_HILMT0                  ((uint32_t)0x00000FFF)        /*!< ADC upper limit 0 value */
#define ADC_HILMT_CHNSEL0                 ((uint32_t)0x00007000)        /*!< ADC upper limit 0 channel select */
#define ADC_HILMT_INTEN0                  ((uint32_t)0x00008000)        /*!< ADC upper limit 0 interrupt enable */
#define ADC_HILMT_HILMT1                  ((uint32_t)0x0FFF0000)        /*!< ADC upper limit 1 value */
#define ADC_HILMT_CHNSEL1                 ((uint32_t)0x70000000)        /*!< ADC upper limit 1 channel select */
#define ADC_HILMT_INTEN1                  ((uint32_t)0x80000000)        /*!< ADC upper limit 1 interrupt enable */


/*******************  Bit definition for ADC_LOLMT register  *********************/
#define ADC_LOLMT_LOLMT0                  ((uint32_t)0x00000FFF)        /*!< ADC lower limit 0 value */
#define ADC_LOLMT_CHNSEL0                 ((uint32_t)0x00007000)        /*!< ADC lower limit 0 channel select */
#define ADC_LOLMT_INTEN0                  ((uint32_t)0x00008000)        /*!< ADC lower limit 0 interrupt enable */
#define ADC_LOLMT_LOLMT1                  ((uint32_t)0x0FFF0000)        /*!< ADC lower limit 1 value */
#define ADC_LOLMT_CHNSEL1                 ((uint32_t)0x70000000)        /*!< ADC lower limit 1 channel select */
#define ADC_LOLMT_INTEN1                  ((uint32_t)0x80000000)        /*!< ADC lower limit 1 interrupt enable */


/*******************  Bit definition for ADC_SSCR register  *********************/
#define ADC_SSCR_ADCTRIG                  ((uint32_t)0x00000001)        /*!< ADC trigger convert */




/******************************************************************************/
/*                                                                            */
/*                           Pulse Width Modulation                           */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for PWM_CTRL register   ******************/
#define PWM_CTRL_PWMEN                    ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_CTRL_LDOK                     ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_CTRL_PWMF                     ((uint32_t)0x00000004)        /*!< PWM ### */
#define PWM_CTRL_PWMRIE                   ((uint32_t)0x00000008)        /*!< PWM ### */
#define PWM_CTRL_PRSC                     ((uint32_t)0x00000060)        /*!< PWM ### */
#define PWM_CTRL_IPOL0                    ((uint32_t)0x00000080)        /*!< PWM ### */
#define PWM_CTRL_IPOL1                    ((uint32_t)0x00000100)        /*!< PWM ### */
#define PWM_CTRL_IPOL2                    ((uint32_t)0x00000200)        /*!< PWM ### */
#define PWM_CTRL_HALF                     ((uint32_t)0x00000800)        /*!< PWM ### */
#define PWM_CTRL_LDFQ                     ((uint32_t)0x0000F000)        /*!< PWM ### */
#define PWM_CTRL_SOFTFAULT                ((uint32_t)0x00010000)        /*!< PWM ### */
#define PWM_CTRL_INIDIR                   ((uint32_t)0x00020000)        /*!< PWM ### */
#define PWM_CTRL_TSCNT                    ((uint32_t)0x00300000)        /*!< PWM ### */
#define PWM_CTRL_TSSEL                    ((uint32_t)0x00C00000)        /*!< PWM ### */
#define PWM_CTRL_CH0OUTEN                 ((uint32_t)0x01000000)        /*!< PWM ### */
#define PWM_CTRL_CH1OUTEN                 ((uint32_t)0x02000000)        /*!< PWM ### */
#define PWM_CTRL_CH2OUTEN                 ((uint32_t)0x04000000)        /*!< PWM ### */
#define PWM_CTRL_CH3OUTEN                 ((uint32_t)0x08000000)        /*!< PWM ### */
#define PWM_CTRL_CH4OUTEN                 ((uint32_t)0x10000000)        /*!< PWM ### */
#define PWM_CTRL_CH5OUTEN                 ((uint32_t)0x20000000)        /*!< PWM ### */

/******************  Bit definition for PWM_FCTRL register   ******************/
#define PWM_FCTRL_FMODE0                  ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_FCTRL_FIE0                    ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_FCTRL_FMODE1                  ((uint32_t)0x00000004)        /*!< PWM ### */
#define PWM_FCTRL_FIE1                    ((uint32_t)0x00000008)        /*!< PWM ### */
#define PWM_FCTRL_FMODE2                  ((uint32_t)0x00000010)        /*!< PWM ### */
#define PWM_FCTRL_FIE2                    ((uint32_t)0x00000020)        /*!< PWM ### */


/******************  Bit definition for PWM_FLTACK register   ******************/
#define PWM_FLTACK_FTACK0                 ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_FLTACK_FTACK6                 ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_FLTACK_FTACK7                 ((uint32_t)0x00000004)        /*!< PWM ### */
#define PWM_FLTACK_FFLAG0                 ((uint32_t)0x00010000)        /*!< PWM ### */
#define PWM_FLTACK_FPIN0                  ((uint32_t)0x00020000)        /*!< PWM ### */
#define PWM_FLTACK_FFLAG6                 ((uint32_t)0x00040000)        /*!< PWM ### */
#define PWM_FLTACK_FPIN6                  ((uint32_t)0x00080000)        /*!< PWM ### */
#define PWM_FLTACK_FFLAG7                 ((uint32_t)0x00100000)        /*!< PWM ### */
#define PWM_FLTACK_FPIN7                  ((uint32_t)0x00200000)        /*!< PWM ### */

/******************  Bit definition for PWM_OUT register   ******************/
#define PWM_OUT_OUT0                      ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_OUT_OUT1                      ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_OUT_OUT2                      ((uint32_t)0x00000004)        /*!< PWM ### */
#define PWM_OUT_OUT3                      ((uint32_t)0x00000008)        /*!< PWM ### */
#define PWM_OUT_OUT4                      ((uint32_t)0x00000010)        /*!< PWM ### */
#define PWM_OUT_OUT5                      ((uint32_t)0x00000020)        /*!< PWM ### */
#define PWM_OUT_OUTCTL0                   ((uint32_t)0x00000100)        /*!< PWM ### */
#define PWM_OUT_OUTCTL1                   ((uint32_t)0x00000200)        /*!< PWM ### */
#define PWM_OUT_OUTCTL2                   ((uint32_t)0x00000400)        /*!< PWM ### */
#define PWM_OUT_OUTCTL3                   ((uint32_t)0x00000800)        /*!< PWM ### */
#define PWM_OUT_OUTCTL4                   ((uint32_t)0x00001000)        /*!< PWM ### */
#define PWM_OUT_OUTCTL5                   ((uint32_t)0x00002000)        /*!< PWM ### */


/******************  Bit definition for PWM_CNTR register   ******************/
#define PWM_CNTR_CNT                      ((uint32_t)0x0000FFFF)        /*!< PWM ### */

/******************  Bit definition for PWM_CMOD register   ******************/
#define PWM_CMOD_CM                       ((uint32_t)0x0000FFFF)        /*!< PWM ### */

/******************  Bit definition for PWM_VAL register   ******************/
#define PWM_VAL_VAL                       ((uint32_t)0x0000FFFF)        /*!< PWM ### */

/******************  Bit definition for PWM_DTIM register   ******************/
#define PWM_DTIM_DTIM                     ((uint32_t)0x0000FFFF)        /*!< PWM ### */


/******************  Bit definition for PWM_DMAP0 register   ******************/
#define PWM_DMAP0_P0FLTMAP0               ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_DMAP0_P0FLTMAP1               ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_DMAP0_P0FLTMAP2               ((uint32_t)0x00000004)        /*!< PWM ### */


/******************  Bit definition for PWM_CNFG register   ******************/
#define PWM_CNFG_WP                       ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_CNFG_NDEP01                   ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_CNFG_NDEP23                   ((uint32_t)0x00000004)        /*!< PWM ### */
#define PWM_CNFG_NDEP45                   ((uint32_t)0x00000008)        /*!< PWM ### */
#define PWM_CNFG_BOTNEG01                 ((uint32_t)0x00000020)        /*!< PWM ### */
#define PWM_CNFG_BOTNEG23                 ((uint32_t)0x00000040)        /*!< PWM ### */
#define PWM_CNFG_BOTNEG45                 ((uint32_t)0x00000080)        /*!< PWM ### */
#define PWM_CNFG_TOPNEG01                 ((uint32_t)0x00000200)        /*!< PWM ### */
#define PWM_CNFG_TOPNEG23                 ((uint32_t)0x00000400)        /*!< PWM ### */
#define PWM_CNFG_TOPNEG45                 ((uint32_t)0x00000800)        /*!< PWM ### */
#define PWM_CNFG_EDG                      ((uint32_t)0x00002000)        /*!< PWM ### */


/******************  Bit definition for PWM_CCTRL register   ******************/
#define PWM_CCTRL_SWP01                   ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_CCTRL_SWP23                   ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_CCTRL_SWP45                   ((uint32_t)0x00000004)        /*!< PWM ### */
#define PWM_CCTRL_VLMODE                  ((uint32_t)0x00000030)        /*!< PWM ### */
#define PWM_CCTRL_MSK0                    ((uint32_t)0x00000040)        /*!< PWM ### */
#define PWM_CCTRL_MSK1                    ((uint32_t)0x00000080)        /*!< PWM ### */
#define PWM_CCTRL_MSK2                    ((uint32_t)0x00000100)        /*!< PWM ### */
#define PWM_CCTRL_MSK3                    ((uint32_t)0x00000200)        /*!< PWM ### */
#define PWM_CCTRL_MSK4                    ((uint32_t)0x00000400)        /*!< PWM ### */
#define PWM_CCTRL_MSK5                    ((uint32_t)0x00000800)        /*!< PWM ### */
#define PWM_CCTRL_ENHA                    ((uint32_t)0x00008000)        /*!< PWM ### */

/******************  Bit definition for PWM_FPORTCTRL register   ******************/
#define PWM_FPORTCTRL_FAULTPORT           ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_FPORTCTRL_FAULTVAL0           ((uint32_t)0x00000100)        /*!< PWM ### */
#define PWM_FPORTCTRL_FAULTVAL1           ((uint32_t)0x00000200)        /*!< PWM ### */
#define PWM_FPORTCTRL_FAULTVAL2           ((uint32_t)0x00000400)        /*!< PWM ### */
#define PWM_FPORTCTRL_FAULTVAL3           ((uint32_t)0x00000800)        /*!< PWM ### */
#define PWM_FPORTCTRL_FAULTVAL4           ((uint32_t)0x00001000)        /*!< PWM ### */
#define PWM_FPORTCTRL_FAULTVAL5           ((uint32_t)0x00002000)        /*!< PWM ### */
#define PWM_FPORTCTRL_FPSEL0              ((uint32_t)0x00010000)        /*!< PWM ### */
#define PWM_FPORTCTRL_FPSEL1              ((uint32_t)0x00020000)        /*!< PWM ### */
#define PWM_FPORTCTRL_FPSEL2              ((uint32_t)0x00040000)        /*!< PWM ### */
#define PWM_FPORTCTRL_FPSEL3              ((uint32_t)0x00080000)        /*!< PWM ### */
#define PWM_FPORTCTRL_FPSEL4              ((uint32_t)0x00100000)        /*!< PWM ### */
#define PWM_FPORTCTRL_FPSEL5              ((uint32_t)0x00200000)        /*!< PWM ### */


/******************  Bit definition for PWM_ICCTRL register   ******************/
#define PWM_ICCTRL_ICC0                   ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_ICCTRL_ICC1                   ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_ICCTRL_ICC2                   ((uint32_t)0x00000004)        /*!< PWM ### */
#define PWM_ICCTRL_PAD_EN                 ((uint32_t)0x00000010)        /*!< PWM ### */

/******************  Bit definition for PWM_PSCR register   ******************/
#define PWM_PSCR_CINV0                    ((uint32_t)0x00000001)        /*!< PWM ### */
#define PWM_PSCR_CINV1                    ((uint32_t)0x00000002)        /*!< PWM ### */
#define PWM_PSCR_CINV2                    ((uint32_t)0x00000004)        /*!< PWM ### */
#define PWM_PSCR_CINV3                    ((uint32_t)0x00000008)        /*!< PWM ### */
#define PWM_PSCR_CINV4                    ((uint32_t)0x00000010)        /*!< PWM ### */
#define PWM_PSCR_CINV5                    ((uint32_t)0x00000020)        /*!< PWM ### */



  
  /**
  * @}
  */
  
/**
  * @}
  */




//#define USE_FULL_ASSERT
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */


#ifdef __cplusplus
}
#endif

#endif /* __GVM32F030_H */

