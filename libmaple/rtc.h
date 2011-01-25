/******************************************************************************
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
 * @file rtc.h
 * @brief Control of realtime clock module
 */

#ifndef _RTC_H_
#define _RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


// If defined, the stored time will accomodate daylight
// savings.  If not defined, the stored time will
// ignore daylight savings (i.e. if storing GST)
//#define RTC_USE_DST

typedef struct {
  uint16_t year;  /* 1..4095 */
  uint8_t  month; /* 1..12 */
  uint8_t  mday;  /* 1..31 */
  uint8_t  wday;  /* 0..6, Sunday = 0*/
  uint8_t  hour;  /* 0..23 */
  uint8_t  min;   /* 0..59 */
  uint8_t  sec;   /* 0..59 */
  uint8_t  dst;   /* 0 Winter, !=0 Summer */
} RTC_t;

void rtc_init(uint32_t sysclk_src, uint32_t pll_src, uint32_t pll_mul);
bool rtc_gettime( RTC_t *rtc );
bool rtc_settime( const RTC_t *rtc );

#ifdef __cplusplus
}
#endif
#endif

