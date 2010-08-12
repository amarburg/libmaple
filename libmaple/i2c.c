/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Bryan Newbold.
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
 * ****************************************************************************/

#include "libmaple.h"
#include "rcc.h"
#include "nvic.h"
#include "gpio.h"
#include "i2c.h"

#define I2C1_BASE         0x40005400
#define I2C2_BASE         0x40005800

/* i2c descriptor table  */
struct i2c_dev i2c_dev_table[] = {
   [I2C1] = {
      .base = (i2c_port*)I2C1_BASE,
      .rcc_dev_num = RCC_I2C1,
      .nvic_ev_num = NVIC_I2C1_EV,
      .nvic_er_num = NVIC_I2C1_ER,
      .state = I2C_SUCCESS,
      .target_address = 0,
      .data = 0,
      .length = 0,
      .offset = 0,
      .slave_handler = 0,
   },
   [I2C2] = {
      .base = (i2c_port*)I2C2_BASE,
      .rcc_dev_num = RCC_I2C2,
      .nvic_ev_num = NVIC_I2C2_EV,
      .nvic_er_num = NVIC_I2C2_ER,
      .state = I2C_SUCCESS,
      .target_address = 0,
      .data = 0,
      .length = 0,
      .offset = 0,
      .slave_handler = 0,
   },
};

void i2c_init(uint8 i2c_num, uint32 freq) {
    i2c_port *port = i2c_dev_table[i2c_num].base;
   
    // Configure the GPIO pins
    switch (i2c_num) {
        case I2C1:
            // Configure I2C pins: SCL and SDA 
            // On the maple, I2C1 SCL is header pin D5 and I2C1 SDA is D9
            gpio_set_mode(GPIOB_BASE, 6, GPIO_MODE_AF_OUTPUT_OD);
            gpio_set_mode(GPIOB_BASE, 7, GPIO_MODE_AF_OUTPUT_OD);
            break;
        case I2C2: 
            // TODO: copy above
        default:   // should never get here
            ASSERT(0);
    }

    // Setup peripheral clock and interrupts
    rcc_clk_enable(i2c_dev_table[i2c_num].rcc_dev_num);
    nvic_irq_enable(i2c_dev_table[i2c_num].nvic_ev_num);
    nvic_irq_enable(i2c_dev_table[i2c_num].nvic_er_num);

    port->CR1 |= I2C_CR1_SWRST;  // reset the peripheral
    port->CR1 &= ~I2C_CR1_SWRST; // re-enable
    port->CR1 &= (~ I2C_CR1_PE); // disable the hardware to configure

    port->CR2 |= (port->CR2 & 0xFFC0) | 36; // set clock to 36MHz

    // configure clock control registers
    // Set speed value for standard mode; this is equal to the number of peripheral clock cycles
    // required for a full T_high or T_low (half a clock period):
    //port->CCR = 0xB4; // 100kHz w/ 36MHz clock rate
    // NOTE: calculations get unreliable around 1MHz
    if(freq <= 100000) {
        port->CCR &= (~ I2C_CCR_FS); // standard mode
        // CCR_VAL = periph_clk / (2*i2c_speed)
        port->CCR = 36000000/(2*freq);
    } else {
        port->CCR |= I2C_CCR_FS; // fast mode
        port->CCR &= (~ I2C_CCR_DUTY);
        // CCR_VAL = periph_clk / (3*i2c_speed)
        // TODO: don't get why this isn't actually 3*freq...
        port->CCR = 36000000/(2*freq);
    }
    
    // configure rise time register
    // this is equal the number of cycles to meet a given rise time
    // TRISE = (periph_clk * rise_time) + 1
    if(freq <= 100000) {
        port->TRISE = (36000000 * (1/1000000)) + 1; // <1000ns for Standard
        //port->TRISE = 37; // for 100kHz standard w/ 36MHz clock rate
    } else if (freq <= 400000) {
        port->TRISE = (36000000 * (1/3400000)) + 1; // <300ns for Fast-mode
    } else {
        port->TRISE = (36000000 * (1/8200000)) + 1; // <120ns for Fast-mode+
    }
    // configure I2C_CR1, I2C_CR2
    port->CR1 &= (~ (I2C_CR1_PEC | I2C_CR1_SMBUS | I2C_CR1_SMBTYPE | I2C_CR1_NOSTRETCH | I2C_CR1_POS)); 
    // enable all interrupts on the peripheral side
    port->CR2 |= (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

    // set ACK flag low: won't be a slave until addr is set
    port->CR1 &= (~ I2C_CR1_ACK);

/*
    // setup state
    state->busy = 0;                            // 0 = ready, 1 = busy
    state->ack= 0;                            // not until slave address is set
    state->slave_addr = 0;                       // the remote target
    state->data = 0;                             // by the byte! chars! TODO: bad, null?
    state->len = 0;                           // how much data?
    state->index = 0;                           // where we're at
    state->slave_begin_handler = 0;
    state->slave_rx_handler = 0;
    state->slave_tx_handler = 0;
    state->slave_end_handler = 0;
*/
    
    // Re-enable port after configuration
    port->CR1 |= I2C_CR1_PE; 
}

// polling; for testing
void i2c_send1(uint32 addr, uint32 data) {
    i2c_port *port = i2c_dev_table[I2C1].base;
    int I2C_TIMEOUT = 100;

    uint32 startmillis = millis();
    
    port->CR1 |= I2C_CR1_START; 

    while (! (port->SR1 & I2C_SR1_SB)) { 
        asm volatile("nop");
        if(millis() - startmillis > I2C_TIMEOUT) {
            port->CR1 |= I2C_CR1_SWRST;     // timed out, freak out
            port->CR1 &= (~I2C_CR1_SWRST);
            return;
        }
    }
    port->DR = (addr & 0x7F) << 1;
    //port->DR = 0x8; // ADDR of slave device: "4" plus LSB is low for transmit
    while (! ((port->SR1 & I2C_SR1_ADDR) && (port->SR2 & I2C_SR2_TRA))) { // addr accepted? transmitting?
        asm volatile("nop");
        if(millis() - startmillis > I2C_TIMEOUT) {
            port->CR1 |= I2C_CR1_SWRST;     // timed out, freak out
            port->CR1 &= (~I2C_CR1_SWRST);
            return;
        }
    }
    port->DR = (data & 0xFF); // set data
    port->SR1 &= ~ I2C_SR1_ADDR; // clear addr
    port->CR1 |= I2C_CR1_STOP; // only sending the one byte; this could be inserted above the DR write?

    while (port->SR1 & I2C_SR1_STOPF) { 
        asm volatile("nop");
        if(millis() - startmillis > I2C_TIMEOUT) {
            port->CR1 |= I2C_CR1_SWRST;     // timed out, freak out
            port->CR1 &= (~I2C_CR1_SWRST);
            return;
        }
    }
}



// polling; for testing
uint8 i2c_read1(uint32 addr) {
    i2c_port *port = i2c_dev_table[I2C1].base;

    uint8 ret;
    uint8 oldack = port->CR1 & I2C_CR1_ACK;
    uint32 startmillis = millis();
    int I2C_TIMEOUT = 100;

    // set ACK flag low b/c we're going to NACK after the first byte
    port->CR1 &= (~ I2C_CR1_ACK);

    // start
    port->CR1 |= I2C_CR1_START; 

    port->DR = 0x9; // ADDR of slave device: "4" plus LSB is low for transmit
    while (! (port->SR1 & I2C_SR1_SB)) { 
        asm volatile("nop");
        if(millis() - startmillis > I2C_TIMEOUT) {
            port->CR1 |= I2C_CR1_SWRST;     // timed out, freak out
            port->CR1 &= (~I2C_CR1_SWRST);
            return 0; 
        }
    }
    port->DR = ((addr & 0x7F) << 1) | 0x1;
    //port->DR = 0x9; // ADDR of slave device: "4" plus LSB is low for transmit
    
    // addr accepted? transmitting?
    while (! ((port->SR1 & I2C_SR1_ADDR) && (! (port->SR2 & I2C_SR2_TRA)))) { 
        asm volatile("nop");
        if(millis() - startmillis > I2C_TIMEOUT) {
            port->CR1 |= I2C_CR1_SWRST;     // timed out, freak out
            port->CR1 &= (~I2C_CR1_SWRST);
            return 0;
        }
    }
    port->SR1 &= (~ I2C_SR1_ADDR); // clear it
    port->CR1 |= I2C_CR1_STOP;     // only reading the one byte
    port->CR1 &= (~ I2C_CR1_ACK);

    // stuff to read?
    while (! (port->SR1 & I2C_SR1_RXNE)) { 
        asm volatile("nop");
        if(millis() - startmillis > I2C_TIMEOUT) {
            port->CR1 |= I2C_CR1_SWRST;     // timed out, freak out
            port->CR1 &= (~I2C_CR1_SWRST);
            return 0;
        }
    }
    ret = port->DR & 0xFF;          // get data

    // ok we're done; return to previous state
    if(oldack) {
        port->CR1 |= I2C_CR1_ACK;
    } else {
        port->CR1 &= (~ I2C_CR1_ACK);
    }
    return ret;
}

/* -----------------------------------------------------------------------*/
// completely tears down the periph?
void i2c_disable(uint8 i2c_num) {
    i2c_port *port = i2c_dev_table[i2c_num].base;

    port->CR1 &= (~ I2C_CR1_PE); // unset enable pin
    port->CR1 |= I2C_CR1_SWRST;  // reset the peripheral
    port->CR1 &= ~I2C_CR1_SWRST; // re-enable
    port->CR1 &= (~ I2C_CR1_PE); // unset enable pin again after reset
}

/* -----------------------------------------------------------------------*/
void i2c_master_start_write(uint8 i2c_num, uint16 addr, uint8 *data, uint32 len) {
    i2c_port *port = i2c_dev_table[i2c_num].base;

    uint32 startmillis = millis();

    if(!len) {
        return;
    }

    switch (i2c_num) {
    case I2C1:
        port = (i2c_port*)I2C1_BASE;
    case I2C2:
        // TODO: copy above
    default:
        ASSERT(0); // shouldn't ever get here
    }

    // need to make sure any previous activity is wrapped up
    /*
    if((state->busy) || (port->SR2 & I2C_SR2_BUSY)) {
        while(port->SR2 & I2C_SR2_BUSY) { 
            delayMicroseconds(1);
            if((millis() - startmillis) > I2C_TIMEOUT) {
                // we timed out; probably some device is pulling SCL down
                // indefinately
                i2c_disable(i2c_num);
                return; 
            }
        }
        // this will increase the START high hold time, required for
        // some slave devices
        delayMicroseconds(10);
    }

    state->busy = 1;
    state->slave_addr = (addr & 0x8F) << 1;  // 7bit plus LSB low for write
    state->len = len;
    state->index = 0;
    state->data = data;
    */

    // ok, kick things off
    port->CR2 |= (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
    port->CR1 |= I2C_CR1_ACK;
    port->CR1 &= (~ I2C_CR1_STOP);
    port->CR1 |= I2C_CR1_START; 
}


/* -----------------------------------------------------------------------*/
void i2c_master_start_read(uint8 i2c_num, uint16 addr, uint8 *data, uint32 len) {
    i2c_port *port = i2c_dev_table[i2c_num].base;

    if(!len) {
        return;
    }

    // need to make sure any previous activity is wrapped up
    /*
    if((state->busy) || (port->SR2 & I2C_SR2_BUSY)) {
        while(port->SR2 & I2C_SR2_BUSY) { 
            delayMicroseconds(1);
            if((millis() - startmillis) > I2C_TIMEOUT) {
                // we timed out; probably some device is pulling SCL down
                // indefinately
                i2c_disable(i2c_num);
                return; 
            }
        }
        // this will increase the START high hold time, required for
        // some slave devices
        delayMicroseconds(10);
    }

    // if we just wrapped up a transmission we gotta wait for it to complete
    state->busy = 1;
    state->slave_addr = (addr & 0x8F) << 1 | 0x01;  // 7bit plus LSB high for read
    state->len = len;
    state->index = 0;
    state->data = data;
    if(i2c1_state.len == 2) {
        port->CR1 |= I2C_CR1_POS;       // ugh
    }
    */

    // ok, kick things off
    port->CR2 |= (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
    port->CR1 |= I2C_CR1_ACK;
    port->CR1 &= (~ I2C_CR1_STOP);
    port->CR1 |= I2C_CR1_START; 
}

// sets up the periph as a slave device
// addr is expected to be 7bits
void i2c_slave_set_addr(uint8 i2c_num, uint16 addr) {
    i2c_port *port;

    switch (i2c_num) {
    case I2C1:
        port = (i2c_port*)I2C1_BASE;
        break;
    case I2C2:
        port = (i2c_port*)I2C2_BASE;
        break;
    default:
        ASSERT(0); // shouldn't ever get here
    }

    port->OAR1 &= (~ I2C_OAR1_ADDMODE); // 7-bit mode
    port->OAR1 = (addr & 0x7F) << 1;

    // set ACK flag high to start ACK-ing
    port->CR1 |= I2C_CR1_ACK;
}

void i2c_slave_set_handler(uint8 port, void (*handler)(uint8, uint8*)) {
    //TODO
    return;
}

/* -----------------------------------------------------------------------*/
void I2C1_EV_IRQHandler(void) {
    /*
    i2c_port *port = (i2c_port*)I2C1_BASE;
    
    uint16 SR1 = port->SR1;
    uint16 SR2 = port->SR2;

    // are we master or slave?
    if(port->SR2 & I2C_SR2_MSL) { // we're MASTER
        if(SR1 & I2C_SR1_SB) {
            // Start has happened:
            // - when START actually happens, SB is set and interrupt happens; hardware
            //   waits until address is written to DR
            port->DR = i2c1_state.slave_addr;    // LSB already set
            port->CR1 |= I2C_CR1_ACK;
        } else if((SR2 & I2C_SR2_TRA) && (SR1 & I2C_SR1_ADDR)) {
            // Address sent and acknowledged; we are transmitting first byte
            // - address shifts out and an interrupt is thrown with ADDR high; if LSB of
            //   address was low, in transmitter mode. TRA reflects this
            // - software writes to the first byte to DR and clears ADDR
            port->DR = i2c1_state.data[0];
            port->SR1 &= ~ I2C_SR1_ADDR;    // clear ADDR bit
            i2c1_state.index++;
            if(i2c1_state.len == 1) {
                port->CR1 |= I2C_CR1_STOP;  // only 1 byte so stop here
                i2c1_state.busy = 0;
                if(i2c1_state.ack) {
                    port->CR1 |= I2C_CR1_ACK;
                } else {
                    port->CR1 &= ~ I2C_CR1_ACK;
                }
            }
        //} else if((SR2 & I2C_SR2_TRA) && (SR1 & I2C_SR1_TXE)) {
        } else if(SR1 & I2C_SR1_TXE) {
            // Byte sent; time to send another
            port->DR = i2c1_state.data[i2c1_state.index];
            i2c1_state.index++;
            if(i2c1_state.index >= i2c1_state.len) {   // this will be the last byte
                port->CR1 |= I2C_CR1_STOP;
                i2c1_state.busy = 0;
                if(i2c1_state.ack) {
                    port->CR1 |= I2C_CR1_ACK;
                } else {
                    port->CR1 &= ~ I2C_CR1_ACK;
                }
            }
        } else if(!(SR2 & I2C_SR2_TRA) && (SR1 & I2C_SR1_ADDR)) {
            // Address sent and acknowledged; we are recieving first byte
            // - address shifts out and an interrupt is thrown with ADDR high; if LSB of
            //   address was low, in transmitter mode. TRA reflects this
            if(i2c1_state.len == 1) {
                port->CR1 &= ~ I2C_CR1_ACK;
                port->SR1 &= ~ I2C_SR1_ADDR;    // clear ADDR bit
                port->CR1 |= I2C_CR1_STOP;  // only 1 byte so stop here
            } else if(i2c1_state.len == 2) {
                port->CR1 &= ~ I2C_CR1_ACK;
                port->SR1 &= ~ I2C_SR1_ADDR;    // clear ADDR bit
            } else {
                port->CR1 |= I2C_CR1_ACK;
                port->SR1 &= ~ I2C_SR1_ADDR;    // clear ADDR bit
            }
        } else if(!(SR2 & I2C_SR2_TRA) && (SR1 & I2C_SR1_BTF)) {
            // TODO: there is an error here; the final RXNE interrupt never gets thrown
            // stuff to read!
            if(i2c1_state.len == 2) {
                port->CR1 |= I2C_CR1_STOP;  // only 1 byte so stop here
            }
            i2c1_state.data[i2c1_state.index] = port->DR;
            i2c1_state.index++;
            port->CR1 &= ~ I2C_CR1_POS;       // ugh
            if(i2c1_state.index + 2 >= i2c1_state.len) {         // next will be the last byte
                port->CR1 &= ~ I2C_CR1_ACK;
                port->CR1 |= I2C_CR1_STOP;
            } else if(i2c1_state.index + 1 >= i2c1_state.len) {  // thie was the last byte
                i2c1_state.busy = 0;
            }
        } else {
            // shouldn't ever get here
            ASSERT(0);
        }
    } else { // we're SLAVE
        ASSERT(0);
    } 
    return;
    */
}

/* -----------------------------------------------------------------------*/
void I2C1_ER_IRQHandler(void) {
    //i2c_disable(I2C_PORT1);     // TODO: something more reasonable
    ASSERT(0);  // for testing/debugging
    return;
}

/* -----------------------------------------------------------------------*/
void I2C2_EV_IRQHandler(void) {
    ASSERT(0);  // for testing/debugging
    return;
}

/* -----------------------------------------------------------------------*/
void I2C2_ER_IRQHandler(void) {
    ASSERT(0);  // for testing/debugging
    return;
}


