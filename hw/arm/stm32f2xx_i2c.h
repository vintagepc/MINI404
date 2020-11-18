/*
 * STM32F2XX I2C
 *
 * Copyright (c) 2020 VintagePC <github.com/vintagepc>
 * Portions based on Pebble qemu stm32f2xx_i2c.c, rewritten for qemu 5.x
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_STM32F2XX_I2C_H
#define HW_STM32F2XX_I2C_H

#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define R_I2C_CR1_PE_BIT          0x00001
#define R_I2C_CR1_SMBUS_BIT       0x00002
#define R_I2C_CR1_SMBTYPE_BIT     0x00008
#define R_I2C_CR1_ENARB_BIT       0x00010
#define R_I2C_CR1_ENPEC_BIT       0x00020
#define R_I2C_CR1_ENGC_BIT        0x00040
#define R_I2C_CR1_NOSTRETCH_BIT   0x00080
#define R_I2C_CR1_START_BIT       0x00100
#define R_I2C_CR1_STOP_BIT        0x00200
#define R_I2C_CR1_ACK_BIT         0x00400
#define R_I2C_CR1_POS_BIT         0x00800
#define R_I2C_CR1_PEC_BIT         0x01000
#define R_I2C_CR1_ALERT_BIT       0x02000
#define R_I2C_CR1_SWRTS_BIT       0x08000


#define R_I2C_CR2_ITERREN_BIT     0x00100
#define R_I2C_CR2_ITEVTEN_BIT     0x00200
#define R_I2C_CR2_ITBUFEN_BIT     0x00400
#define R_I2C_CR2_DMAEN_BIT       0x00800
#define R_I2C_CR2_LAST_BIT        0x01000

#define R_I2C_SR1_SB_BIT          0x00001
#define R_I2C_SR1_ADDR_BIT        0x00002
#define R_I2C_SR1_BTF_BIT         0x00004
#define R_I2C_SR1_ADD10_BIT       0x00008
#define R_I2C_SR1_STOPF_BIT       0x00010
#define R_I2C_SR1_RxNE_BIT        0x00040
#define R_I2C_SR1_TxE_BIT         0x00080
#define R_I2C_SR1_BERR_BIT        0x00100
#define R_I2C_SR1_ARLO_BIT        0x00200
#define R_I2C_SR1_AF_BIT          0x00400
#define R_I2C_SR1_OVR_BIT         0x00800
#define R_I2C_SR1_PECERR_BIT      0x01000
#define R_I2C_SR1_TIMEOUT_BIT     0x04000
#define R_I2C_SR1_SMBALERT_BIT    0x08000

#define TYPE_STM32F2XX_I2C "stm32f2xx-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F2XXI2CState, STM32F2XX_I2C)

struct STM32F2XXI2CState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;
    
    union {
        struct {
            uint16_t i2c_cr1;
            uint16_t i2c_cr2;
            uint16_t i2c_oar1;
            uint16_t i2c_oar2;
            uint16_t i2c_dr;
            uint16_t i2c_sr1;
            uint16_t i2c_sr2;
            uint16_t i2c_ccr;
            uint16_t i2c_trise;
        };
        uint16_t regs[9];
    };

    uint8_t last_read;

    qemu_irq evt_irq;
    qemu_irq err_irq;
    uint8_t slave_address;
    int32_t rx;
    int rx_full; 
    I2CBus *bus;
};

#endif /* HW_STM32F2XX_I2C_H */
