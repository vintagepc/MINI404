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


#define TYPE_STM32F2XX_I2C "stm32f2xx-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F2XXI2CState, STM32F2XX_I2C);

#define R_I2C_COUNT 9u

struct STM32F2XXI2CState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;
    
    union {
        struct {
            struct {
                uint16_t PE :1;
                uint16_t SMBUS :1;
                uint16_t _res :1;
                uint16_t SMBTYPE :1;
                uint16_t ENARP :1;
                uint16_t ENPEC :1;
                uint16_t ENGC :1;
                uint16_t NOSTRETCH :1;
                uint16_t START :1;
                uint16_t STOP :1;
                uint16_t ACK :1;
                uint16_t POS :1;
                uint16_t PEC :1;
                uint16_t ALERT :1;
                uint16_t _res1 :1;
                uint16_t SWRST :1;
            } CR1 QEMU_PACKED;
            struct {
                uint16_t FREQ :6;
                uint16_t _res :2;
                uint16_t ITERREN :1;
                uint16_t ITEVTEN :1;
                uint16_t ITBUFEN :1;
                uint16_t DMAEN :1;                                
                uint16_t LAST :1;
                uint16_t :3;
            } CR2 QEMU_PACKED;
            struct {
                uint16_t ADD0 :1;
                uint16_t ADDLO  :7;
                uint16_t ADDHI :2;
                uint16_t _res :5;
                uint16_t ADDMODE :1;
            } OAR1 QEMU_PACKED;
            struct {
                uint16_t ENDUAL :1;
                uint16_t ADD2 :7;
                uint16_t :8;
            } OAR2 QEMU_PACKED;
            uint16_t DR;
            struct {
                uint16_t SB :1;
                uint16_t ADDR :1;
                uint16_t BTF :1;
                uint16_t ADD10 :1;
                uint16_t STOPF :1;
                uint16_t :1;
                uint16_t RxNE :1;
                uint16_t TxE :1;
                uint16_t BERR :1;
                uint16_t ARLO :1;
                uint16_t AF :1;
                uint16_t OVR :1;
                uint16_t PECERR :1;
                uint16_t :1;
                uint16_t TIMEOUT :1;
                uint16_t SMBALERT :1;
            } SR1 QEMU_PACKED;
            struct {
                uint16_t MSL :1;
                uint16_t BUSY :1;
                uint16_t TRA :1;
                uint16_t :1;
                uint16_t GENCALL:1;
                uint16_t SMBDEFAULT:1;
                uint16_t SMBHOST:1;
                uint16_t DUALF:1;
                uint16_t PEC :8;
            } SR2 QEMU_PACKED;
            uint16_t CCR;
            uint16_t TRISE;
        } defs QEMU_PACKED;
        uint16_t regs[R_I2C_COUNT];
    } QEMU_PACKED;

    uint8_t last_read;

    qemu_irq evt_irq;
    qemu_irq err_irq;
    uint8_t slave_address;
    uint8_t shiftreg; // DR shift register. 
    int32_t rx;
    bool shift_full; 
    bool is_read;
    bool dr_unread; 
    I2CBus *bus;
};

#endif /* HW_STM32F2XX_I2C_H */
