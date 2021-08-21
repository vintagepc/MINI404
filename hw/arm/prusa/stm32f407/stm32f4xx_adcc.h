/*
 * STM32F4XX ADC Common registers
 *
 * Copyright (c) 2021 by VintagePC <http://github.com/vintagepc> for Mini404
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

#ifndef HW_STM32F4XX_ADCC_H
#define HW_STM32F4XX_ADCC_H

#include "qom/object.h"
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "../utility/macros.h"

#define TYPE_STM32F4XX_ADCC "stm32f4xx-adcc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XXADCCState, STM32F4XX_ADCC)

#define R_ADCC_MAX    (0x0C/4)

struct STM32F4XXADCCState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    union {
        uint32_t regs[3]; // Common registers. There's only 3 but #4 is used for initialization/sanity.
        struct {
            union {
                uint8_t CSRBLK[3];
                struct {
                    REG_B32(AWD1);
                    REG_B32(EOC1);
                    REG_B32(JEOC1);
                    REG_B32(JSTRT1);
                    REG_B32(STRT1);
                    REG_B32(OVR1);
                    uint32_t _unused :2;
                    REG_B32(AWD2);
                    REG_B32(EOC2);
                    REG_B32(JEOC2);
                    REG_B32(JSTRT2);
                    REG_B32(STRT2);
                    REG_B32(OVR2);
                    uint32_t _unused2 :2;
                    REG_B32(AWD3);
                    REG_B32(EOC3);
                    REG_B32(JEOC3);
                    REG_B32(JSTRT3);
                    REG_B32(STRT3);
                    REG_B32(OVR3);
                    uint32_t :10;
                } QEMU_PACKED CSR;
            };
            struct {
                uint32_t MULTI :5;
                uint32_t _unused :3;
                uint32_t DELAY :4;
                REG_B32(_unused2);
                REG_B32(DDS);
                uint32_t DMA :2;
                uint32_t ADCPRE :2;
                uint32_t _unused3 :4;
                REG_B32(VBATE);
                REG_B32(TSVREFE);
                uint32_t :8;
            } QEMU_PACKED CCR;
            struct {
                uint32_t DATA1 :16;
                uint32_t DATA2 :16;
            } QEMU_PACKED CDR;
        } QEMU_PACKED defs;
    };
};

uint8_t stm32f4xx_adcc_get_adcpre(STM32F4XXADCCState *s);

#endif /* HW_STM32F4XX_ADCC_H */
