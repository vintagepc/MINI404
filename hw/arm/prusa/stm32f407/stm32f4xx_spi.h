/*
 * STM32F4XX SPI
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified/bugfixed for Mini404 by VintagePC 2021 (http://github.com/vintagepc/)
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

#ifndef HW_STM32F4XX_SPI_H
#define HW_STM32F4XX_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"
#include "../stm32_common/stm32_common.h"

// #define R_SPI_CR1     0x00
// #define R_SPI_CR2     0x04
// #define R_SPI_SR      0x08
// #define R_SPI_DR      0x0C
// #define R_SPI_CRCPR   0x10
// #define R_SPI_RXCRCR  0x14
// #define R_SPI_TXCRCR  0x18
// #define R_SPI_I2SCFGR 0x1C
// #define R_SPI_I2SPR   0x20

// #define R_SPI_CR1_SPE  (1 << 6)
// #define R_SPI_CR1_MSTR (1 << 2)

// #define R_SPI_SR_RXNE   1
#define R_MAX      (0x24 / 4)

#define TYPE_STM32F4XX_SPI "stm32f4xx-spi"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XXSPIState, STM32F4XX_SPI)

struct STM32F4XXSPIState {
    /* <private> */
    STM32Peripheral parent;

    /* <public> */
    MemoryRegion mmio;

    uint16_t regs[R_MAX];

    qemu_irq irq;
    SSIBus *ssi;
};

#endif /* HW_STM32F4XX_SPI_H */
