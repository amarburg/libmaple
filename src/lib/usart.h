/* *****************************************************************************
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Created: 12/18/09 02:38:35
 *  Copyright (c) 2009 Perry L. Hung. All rights reserved.
 *
 * ****************************************************************************/

/**
 *  @file usart.h
 *
 *  @brief USART Definitions
 */

#ifndef _USART_H_
#define _USART_H_

/* Transmit procedure:
 * 1.  Enable the USART by writing the UE bit in USART_CR1 register to 1.
 *
 * 2.  Program the M bit in USART_CR1 to define the word length.
 *
 * 3.  Program the number of stop bits in USART_CR2.
 *
 * 4.  Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is
 *     to take place. Configure the DMA register as explained in multibuffer
 *     communication.
 *
 * 5.  Select the desired baud rate using the USART_BRR register.
 *
 * 6.  Set the TE bit in USART_CR1 to send an idle frame as first transmission.
 *
 * 7.  Write the data to send in the USART_DR register (this clears the TXE
 *     bit). Repeat this for each data to be transmitted in case of single buffer.
 *
 * 8.  After writing the last data into the USART_DR register, wait until TC=1.
 *     This indicates that the transmission of the last frame is complete. This is
 *     required for instance when the USART is disabled or enters the Halt mode to
 *     avoid corrupting the last transmission.
 *
 * Single byte communication
 * Clearing the TXE bit is always performed by a write to the data register.
 *
 * The TXE bit is set by hardware and it indicates:
 * ?   The data has been moved from TDR to the shift register and the data transmission has
 *     started.
 * ?   The TDR register is empty.
 * ?   The next data can be written in the USART_DR register without overwriting the
 *     previous data.
 * This flag generates an interrupt if the TXEIE bit is set.
 *
 * When a transmission is taking place, a write instruction to the USART_DR
 * register stores the data in the TDR register and which is copied in the
 * shift register at the end of the current transmission.
 *
 * When no transmission is taking place, a write instruction to the USART_DR
 * register places the data directly in the shift register, the data
 * transmission starts, and the TXE bit is immediately set.
 *
 * If a frame is transmitted (after the stop bit) and the TXE bit is set, the
 * TC bit goes high. An interrupt is generated if the TCIE bit is set in the
 * USART_CR1 register.  After writing the last data into the USART_DR register,
 * it is mandatory to wait for TC=1 before disabling the USART or causing the
 * microcontroller to enter the low power mode (see Figure 241: TC/TXE behavior
 * when transmitting).
 *
 * Clearing the TC bit is performed by the following software sequence:
 * 1.     A read from the USART_SR register
 * 2.     A write to the USART_DR register
 *
 *
 * For now, use 8N1
 *
 * Baud rate is generated by programming the mantissa and fraction values of
 * USARTDIV
 *
 * baud = fck / 16*USARTDIV
 * Fck = PLK1 for USART2 and USART3 = 36MHz
 * Fck = PCLK2 for USART1 = 72MHz
 *
 * Baud    Actual  USARTDIV_36MHZ  Error     USARTDIV_72MHZ Error
 * 2400    2.400   937.5           0%        1875           0%
 * 9600    9.600   234.375         0%        468.75         0%
 * 19200   19.2    117.1875        0%        234.375        0%
 * 57600   57.6    39.0625         0%        78.125         0.%
 * 115200  115.384 19.5            0.15%     39.0625        0%
 * 230400  230.769 9.75            0.16%     19.5           0.16%
 * 460800  461.538 4.875           0.16%     9.75           0.16%
 * 921600  923.076 2.4375          0.16%     4.875          0.16%
 * 225000  2250    1               0%        2              0%
 *
 * */
#define NR_USARTS           0x3

#ifdef __cplusplus
extern "C"{
#endif

#define USART_MAX_BAUD      225000

void usart_init(uint8 usart_num, uint32 baud);
void usart_disable(uint8 usart_num);

void usart_putstr(uint8 usart_num, const char*);
void usart_putudec(uint8 usart_num, uint32 val);
void usart_putc(uint8 usart_num, uint8 ch);

uint32 usart_data_available(uint8 usart_num);
uint8 usart_getc(uint8 usart_num);

#ifdef __cplusplus
} // extern "C"
#endif


#endif
