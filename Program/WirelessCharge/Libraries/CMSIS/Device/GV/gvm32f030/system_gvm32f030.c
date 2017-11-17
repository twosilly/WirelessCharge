/**************************************************************************//**
 * @file     system_gvm32f030.c
 * @brief    CMSIS Device System Source File for
 *           ARMCM0 Device Series
 * @version  V2.00
 * @date     18. August 2015
 ******************************************************************************/
/* Copyright (c) 2011 - 2015 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#include "gvm32f030.h"

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define  __WDT_OSC_CLOCK               (32000U)       /* Internal WDT oscillator frequency */
#define  __IRC_OSC_CLOCK_50MHz         (50000000U)    /* Internal RC oscillator frequency (sample) */
#define  __IRC_OSC_CLOCK_40MHz         (40000000U)    /* Internal RC oscillator frequency (sample) */
#define  __IRC_OSC_CLOCK   __IRC_OSC_CLOCK_50MHz      /* Internal RC oscillator default frequency  */


#define  MAIN_CLOCK       __IRC_OSC_CLOCK
#define  SYSTEM_CLOCK     (MAIN_CLOCK / 2)
/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = SYSTEM_CLOCK;   /* System Core Clock Frequency  */
uint32_t MainClock = MAIN_CLOCK;        /* Chip Main Clock Frequency    */


void SystemCoreClockUpdate (void)
{
  /* Get MainClock Source*/
  switch(SYSCON->MAINCLKSEL & 0x03)
  {
  case 0:   //IRC oscillator
    if(SYSCON->IRCCTRL & 0x20)  MainClock = __IRC_OSC_CLOCK_50MHz;
    else  MainClock = __IRC_OSC_CLOCK_40MHz;
    break;
  case 1:
    break;
  case 2:   //WDT oscillator
    MainClock = __WDT_OSC_CLOCK;
    break;
  case 3:
    break;
  }

  /* Update SystemCoreClock */
  SystemCoreClock = MainClock / SYSCON->SYSAHBCLKDIV;
}

void SystemInit (void)
{
  SYSCON->MAINCLKUEN = 1;
  SYSCON->MAINCLKSEL = SYSCON_MAINCLKSEL_IRC;
  SYSCON->MAINCLKUEN = 0;

  SYSCON->SYSAHBCLKDIV = 2;
}
