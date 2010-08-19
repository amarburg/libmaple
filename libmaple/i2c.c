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

// for millis()
#include "systick.h"

#define I2C1_BASE         0x40005400
#define I2C2_BASE         0x40005800

// i2c descriptor table; some values set during initialization
struct i2c_dev i2c_dev_table[] = {
   [I2C1] = {
      .base = (i2c_port*)I2C1_BASE,
      .rcc_dev_num = RCC_I2C1,
      .nvic_ev_num = NVIC_I2C1_EV,
      .nvic_er_num = NVIC_I2C1_ER,
      .scl_pin_port = GPIOB_BASE,
      .scl_pin_num = 6,
      .sda_pin_port = GPIOB_BASE,
      .sda_pin_num = 7,
   },
   [I2C2] = {
      .base = (i2c_port*)I2C2_BASE,
      .rcc_dev_num = RCC_I2C2,
      .nvic_ev_num = NVIC_I2C2_EV,
      .nvic_er_num = NVIC_I2C2_ER,
      .scl_pin_port = GPIOB_BASE,
      .scl_pin_num = 10,
      .sda_pin_port = GPIOB_BASE,
      .sda_pin_num = 11,
   },
};

void i2c_init(uint8 i2c_num, uint32 freq) {
    i2c_port *port = i2c_dev_table[i2c_num].base;
    struct i2c_dev dev = i2c_dev_table[i2c_num];
   
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
    port->CR2 |= (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    // set ACK flag low: won't be a slave until addr is set
    port->CR1 &= (~ I2C_CR1_ACK);

    // setup state
    dev.state = I2C_SUCCESS;
    dev.target_address = 0;
    dev.data = 0;
    dev.length = 0;
    dev.offset = 0;
    dev.slave_handler = 0x0;
    
    // Re-enable port after configuration
    port->CR1 |= I2C_CR1_PE; 
}

// completely tears down the periph?
void i2c_disable(uint8 i2c_num) {
    i2c_port *port = i2c_dev_table[i2c_num].base;
    struct i2c_dev dev = i2c_dev_table[i2c_num];

    port->CR1 &= (~ I2C_CR1_PE); // unset enable pin
    port->CR1 |= I2C_CR1_SWRST;  // reset the peripheral
    port->CR1 &= ~I2C_CR1_SWRST; // re-enable
    port->CR1 &= (~ I2C_CR1_PE); // unset enable pin again after reset

    dev.state = I2C_SUCCESS;
    dev.target_address = 0;
    dev.data = 0;
    dev.length = 0;
    dev.offset = 0;
    dev.slave_handler = 0x0;
}

uint8 i2c_status(uint8 i2c_num) {
    struct i2c_dev dev = i2c_dev_table[i2c_num];
    return dev.state;
}

/* -----------------------------------------------------------------------*/
void i2c_master_start_write(uint8 i2c_num, uint16 addr, uint8 *data, uint32 len) {
    i2c_port *port = i2c_dev_table[i2c_num].base;
    struct i2c_dev dev = i2c_dev_table[i2c_num];

    if(!len) { return; }

    // need to make sure any previous activity is wrapped up
    if(dev.state != I2C_SUCCESS) { return; }

    // setup the device software state
    dev.state = I2C_BUSY_WRITE;
    dev.target_address = addr;
    dev.data = data;
    dev.length = len;
    dev.offset = 0;

    //state->slave_addr = (addr & 0x8F) << 1;  // 7bit plus LSB low for write
    
    // ok, kick things off
    port->CR2 |= (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
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
//    ASSERT(0);
    i2c_port *port = (i2c_port*)I2C1_BASE;
    uint16 SR1 = port->SR1;
    uint16 SR2 = port->SR2;
    struct i2c_dev dev = i2c_dev_table[1];

    // are we master or slave?
    if(port->SR2 & I2C_SR2_MSL) { // we're MASTER
        if(SR1 & I2C_SR1_SB) {
            // Start has happened:
            // - when START actually happens, SB is set and interrupt happens; hardware
            //   waits until address is written to DR
            port->DR = (dev.target_address & 0x007F) << 1;
            port->CR1 |= I2C_CR1_ACK;
        } else if((SR2 & I2C_SR2_TRA) && (SR1 & I2C_SR1_ADDR)) {
            // Address sent and acknowledged; we are transmitting first byte
            // - address shifts out and an interrupt is thrown with ADDR high; if LSB of
            //   address was low, in transmitter mode. TRA reflects this
            // - software writes to the first byte to DR and clears ADDR
            port->DR = dev.data[0];
            port->SR1 &= ~ I2C_SR1_ADDR;    // clear ADDR bit
            dev.offset++;
            if(dev.length == 1) {
                port->CR1 |= I2C_CR1_STOP;  // only 1 byte so stop here
                /*
                //dev.state = I2C_SUCCESS;
                if(i2c1_state.ack) {
                    port->CR1 |= I2C_CR1_ACK;
                } else {
                    port->CR1 &= ~ I2C_CR1_ACK;
                }
                */
                port->CR1 |= I2C_CR1_ACK;
            }
        //} else if((SR2 & I2C_SR2_TRA) && (SR1 & I2C_SR1_TXE)) {
        } else if(SR1 & I2C_SR1_TXE) {
            // Byte sent; time to send another
            port->DR = dev.data[dev.offset];
            dev.offset++;
            if(dev.offset >= dev.length) {   // this will be the last byte
                port->CR1 |= I2C_CR1_STOP;
                /*
                i2c1_state.busy = 0;
                if(i2c1_state.ack) {
                    port->CR1 |= I2C_CR1_ACK;
                } else {
                    port->CR1 &= ~ I2C_CR1_ACK;
                }
                */
                port->CR1 |= I2C_CR1_ACK;
            }
        } else if(!(SR2 & I2C_SR2_TRA) && (SR1 & I2C_SR1_ADDR)) {
            // Address sent and acknowledged; we are recieving first byte
            // - address shifts out and an interrupt is thrown with ADDR high; if LSB of
            //   address was low, in transmitter mode. TRA reflects this
            if(dev.length == 1) {
                port->CR1 &= ~ I2C_CR1_ACK;
                port->SR1 &= ~ I2C_SR1_ADDR;    // clear ADDR bit
                port->CR1 |= I2C_CR1_STOP;  // only 1 byte so stop here
            } else if(dev.length == 2) {
                port->CR1 &= ~ I2C_CR1_ACK;
                port->SR1 &= ~ I2C_SR1_ADDR;    // clear ADDR bit
            } else {
                port->CR1 |= I2C_CR1_ACK;
                port->SR1 &= ~ I2C_SR1_ADDR;    // clear ADDR bit
            }
        } else if(!(SR2 & I2C_SR2_TRA) && (SR1 & I2C_SR1_BTF)) {
            // TODO: there is an error here; the final RXNE interrupt never gets thrown
            // stuff to read!
            if(dev.length == 2) {
                port->CR1 |= I2C_CR1_STOP;  // only 1 byte so stop here
            }
            dev.data[dev.offset] = port->DR;
            dev.offset++;
            port->CR1 &= ~ I2C_CR1_POS;       // ugh
            if(dev.offset + 2 >= dev.length) {         // next will be the last byte
                port->CR1 &= ~ I2C_CR1_ACK;
                port->CR1 |= I2C_CR1_STOP;
            } else if(dev.offset + 1 >= dev.length) {  // thie was the last byte
                //i2c1_state.busy = 0;
            }
        } else {
            // shouldn't ever get here
            ASSERT(0);
        }
    } else { // we're SLAVE
        ASSERT(0);
    } 
    return;
}

/* -----------------------------------------------------------------------*/
void I2C1_ER_IRQHandler(void) {
    //i2c_disable(I2C_PORT1);     // TODO: something more reasonable
    struct i2c_dev dev = i2c_dev_table[I2C1];
    dev.state = I2C_ERROR;
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
    struct i2c_dev dev = i2c_dev_table[I2C2];
    dev.state = I2C_ERROR;
    ASSERT(0);  // for testing/debugging
    return;
}


