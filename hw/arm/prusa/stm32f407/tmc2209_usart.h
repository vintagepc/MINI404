/*
 * TMC2209 USART - based off stm usart, modfied by VintagePC
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
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

#ifndef HW_TMC2209_USART_H
#define HW_TMC2209_USART_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/ssi/ssi.h"


#define USART_SR   0x00
#define USART_DR   0x04
#define USART_BRR  0x08
#define USART_CR1  0x0C
#define USART_CR2  0x10
#define USART_CR3  0x14
#define USART_GTPR 0x18

/*
 * NB: The reset value mentioned in "24.6.1 Status register" seems bogus.
 * Looking at "Table 98 USART register map and reset values", it seems it
 * should be 0xc0, and that's how real hardware behaves.
 */
#define USART_SR_RESET (USART_SR_TXE | USART_SR_TC)

#define USART_SR_TXE  (1 << 7)
#define USART_SR_TC   (1 << 6)
#define USART_SR_RXNE (1 << 5)

#define USART_CR1_UE  (1 << 13)
#define USART_CR1_RXNEIE  (1 << 5)
#define USART_CR1_TE  (1 << 3)
#define USART_CR1_RE  (1 << 2)

#define TYPE_TMC2209_USART "tmc2209-usart"
OBJECT_DECLARE_SIMPLE_TYPE(TMC2209UsartState, TMC2209_USART)

typedef union { 
    uint8_t bytes[8];
    uint32_t dwords[2];
    uint64_t raw;
} cmd;

struct TMC2209UsartState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    uint32_t usart_sr;
    uint32_t usart_dr;
    uint32_t usart_brr;
    uint32_t usart_cr1;
    uint32_t usart_cr2;
    uint32_t usart_cr3;
    uint32_t usart_gtpr;

    // Rather than try to deal with muxing chardevs I'm opting to treat it like a fake SSI bus since
    // it's a very nicely formed exchange with 64 bit messages and replies. 

    cmd cmdTx;
    cmd cmdRx;
    uint8_t cmdTxIdx;
    uint8_t cmdLen;
    uint8_t cmdRxLen;
    SSIBus *spi;

    qemu_irq irq;
    qemu_irq irqCS[4]; // Motor CS irqs. 0-3 = Z/X/E/Y

};
#endif /* HW_TMC2209_USART_H */
