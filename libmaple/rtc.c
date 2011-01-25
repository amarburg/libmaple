/******************************************************************************
 * Functions for dealing with the RTC module.
 * Based heavily on Martin Thomas' functions, which relied
 * on the CMSIS.  This code does not use the CMSIS.
 *
 * The MIT License
 *
 * Copyright (c) 2010 Aaron Marburg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *****************************************************************************/

/**
 * @brief Rudimentary RTC access and control.
 */

#include "libmaple.h"
#include "rcc.h"
#include "rtc.h"



#ifdef RTC_USE_DST
// A big block of daylight-savings related functions.
// Drawn largely from Martin Thomas, who in turn
// drew it from around the internet.

/**
 * @brief Is the given time within the Daylight Savings time period.
 * @param t  An RTC_t time structure
 * @return   true if DST, false if not
 */
static bool isDST( const RTC_t *t )
{
  uint8_t wday, month;            // locals for faster access

  month = t->month;

  if( month < 3 || month > 10 ) {         // month 1, 2, 11, 12
    return false;                                   // -> Winter
  }

  wday  = t->wday;

  if( t->mday - wday >= 25 && (wday || t->hour >= 2) ) { // after last Sunday 2:00
    if( month == 10 ) {                             // October -> Winter
      return false;
    }
  } else {                                                        // before last Sunday 2:00
    if( month == 3 ) {                              // March -> Winter
      return false;
    }
  }

  return true;
}

/**
 * @brief Adjusts and RTC_t for DST if needed
 * @param t  A non-DST RTC_t time structure
 * @return   true if DST, false if not
 */
static bool adjustDST( RTC_t *t )
{
  uint8_t hour, day, wday, month;                 // locals for faster access

  hour  = t->hour;
  day   = t->mday;
  wday  = t->wday;
  month = t->month;

  if ( isDST(t) ) {
    t->dst = 1;
    hour++;                                                         // add one hour
    if( hour == 24 ){                                       // next day
      hour = 0;
      wday++;                                                 // next weekday
      if( wday == 7 ) {
        wday = 0;
      }
      if( day == DaysInMonth[month-1] ) {             // next month
        day = 0;
        month++;
      }
      day++;
    }
    t->month = month;
    t->hour  = hour;
    t->mday  = day;
    t->wday  = wday;
    return true;
  } else {
    t->dst = 0;
    return false;

#endif

/**
 * @brief Initialize the realtime clock.
 * @param 
 */
void rtc_init(uint32_t sysclk_src, uint32_t pll_src, uint32_t pll_mul)
{
  int i=0;

  rcc_clk_enable( RCC_PWR );
  rcc_clk_enable( RCC_BKP );

  /* LSI clock stabilization time */
  for(i ;i<5000;i++) { ; }
//
//  if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5) {
//    /* Backup data register value is not correct or not yet programmed (when
//     *        the first time the program is executed) */
//
//    /* Allow access to BKP Domain */
//    PWR_BackupAccessCmd(ENABLE);
//
//    /* Reset Backup Domain */
//    BKP_DeInit();
//
//    /* Enable LSE */
//    RCC_LSEConfig(RCC_LSE_ON);
//
//    /* Wait till LSE is ready */
//    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) { ; }
//
//    /* Select LSE as RTC Clock Source */
//    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
//
//    /* Enable RTC Clock */
//    RCC_RTCCLKCmd(ENABLE);
//
//    /* Wait for RTC registers synchronization */
//    RTC_WaitForSynchro();
//
//    /* Wait until last write operation on RTC registers has finished */
//    RTC_WaitForLastTask();
//
//    /* Set RTC prescaler: set RTC period to 1sec */
//    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
//
//    /* Wait until last write operation on RTC registers has finished */
//    RTC_WaitForLastTask();
//
//    /* Set initial value */
//    RTC_SetCounter( (uint32_t)((11*60+55)*60) ); // here: 1st January 2000 11:55:00
//
//    /* Wait until last write operation on RTC registers has finished */
//    RTC_WaitForLastTask();
//
//    BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
//
//    /* Lock access to BKP Domain */
//    PWR_BackupAccessCmd(DISABLE);
//
//  } else {
//
//    /* Wait for RTC registers synchronization */
//    RTC_WaitForSynchro();
//
//  }
//
//
//    /* Assume that we're going to clock the chip off the PLL, fed by
//     * the HSE */
//    ASSERT(sysclk_src == RCC_CLKSRC_PLL &&
//           pll_src    == RCC_PLLSRC_HSE);
//
//    uint32 cfgr = 0;
//    uint32 cr = RCC_READ_CR();
//
//    cfgr =  (pll_src | pll_mul);
//    RCC_WRITE_CFGR(cfgr);
//
//    /* Turn on the HSE  */
//    cr |= RCC_CR_HSEON;
//    RCC_WRITE_CR(cr);
//    while (!(RCC_READ_CR() & RCC_CR_HSERDY))
//        ;
//
//    /* Now the PLL  */
//    cr |= RCC_CR_PLLON;
//    RCC_WRITE_CR(cr);
//    while (!(RCC_READ_CR() & RCC_CR_PLLRDY))
//        ;
//
//    /* Finally, let's switch over to the PLL  */
//    cfgr &= ~RCC_CFGR_SW;
//    cfgr |= RCC_CFGR_SW_PLL;
//    RCC_WRITE_CFGR(cfgr);
//    while ((RCC_READ_CFGR() & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
//        ;
}

bool rtc_gettime( RTC_t *rtc )
{
  rtc->year = 2011;
  rtc->month = 1;
  rtc->mday = 1;
  rtc->wday = 0;
  rtc->hour = 12;
  rtc->min = 34;
  rtc->sec = 56;
  rtc->dst = 1;
   return true;
}

bool rtc_settime( const RTC_t *rtc )
{
  return true;  
}
