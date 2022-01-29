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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "stm32f4xx_spi.h"
#include "migration/vmstate.h"

#ifndef R_SPI_ERR_DEBUG
#define R_SPI_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (R_SPI_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define	R_CR1             (0x00 / 4)
#define	R_CR1_DFF      (1 << 11)
#define	R_CR1_LSBFIRST (1 <<  7)
#define	R_CR1_SPE      (1 <<  6)
#define	R_CR2             (0x04 / 4)

#define	R_SR       (0x08 / 4)
#define	R_SR_RESET    0x0002
#define	R_SR_MASK     0x01FF
#define R_SR_OVR     (1 << 6)
#define R_SR_TXE     (1 << 1)
#define R_SR_RXNE    (1 << 0)

#define	R_DR       (0x0C / 4)
#define	R_CRCPR    (0x10 / 4)
#define	R_CRCPR_RESET 0x0007
#define	R_RXCRCR   (0x14 / 4)
#define	R_TXCRCR   (0x18 / 4)
#define	R_I2SCFGR  (0x1C / 4)
#define	R_I2SPR    (0x20 / 4)
#define	R_I2SPR_RESET 0x0002

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

static void stm32f4xx_spi_reset(DeviceState *dev)
{
    STM32F4XXSPIState *s = STM32F4XX_SPI(dev);

    s->regs[R_CR1] = 0x00000000;
    s->regs[R_CR2] = 0x00000000;
    s->regs[R_SR] = 0x0000000A;
    s->regs[R_DR] = 0x0000000C;
    s->regs[R_CRCPR] = 0x00000007;
    s->regs[R_RXCRCR] = 0x00000000;
    s->regs[R_TXCRCR] = 0x00000000;
    s->regs[R_I2SCFGR] = 0x00000000;
    s->regs[R_I2SPR] = 0x00000002;
}

static uint64_t stm32f4xx_spi_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    STM32F4XXSPIState *s = opaque;

    DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);

    uint16_t r = UINT16_MAX;

    if (!(size == 1 || size == 2 || size == 4 || (addr & 0x3) != 0)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad register access 0x%" HWADDR_PRIx "\n",
             __func__, addr);
    }
    addr >>= 2;
    if (addr < R_MAX) {
        r = s->regs[addr];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "Out of range SPI write, addr 0x%"HWADDR_PRIx"\n", addr<<2);
    }
    switch (addr) {
    case R_DR:
        s->regs[R_SR] &= ~R_SR_RXNE;
    }
    return r;
}

static uint8_t bitswap(uint8_t val)
{
    return ((val * 0x0802LU & 0x22110LU) | (val * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}


static void stm32f4xx_spi_write(void *opaque, hwaddr addr,
                                uint64_t data, unsigned int size)
{
    STM32F4XXSPIState *s = opaque;

    DB_PRINT("Address: 0x%" HWADDR_PRIx ", Value: 0x%"PRIx64"\n", addr, data);

    int offset = addr & 0x3;

    /* SPI registers are all at most 16 bits wide */
    data &= 0xFFFFF;
    addr >>= 2;

    switch (size) {
        case 1:
            data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
            break;
        case 2:
            data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
            break;
        case 4:
            break;
        default:
            abort();
    }

    switch (addr) {
        case R_CR1:
            if ((data & R_CR1_DFF) != s->regs[R_CR1] && (s->regs[R_CR1] & R_CR1_SPE) != 0)
                qemu_log_mask(LOG_GUEST_ERROR, "cannot change DFF with SPE set\n");
            if (data & R_CR1_DFF)
                qemu_log_mask(LOG_UNIMP, "f2xx DFF 16-bit mode not implemented\n");
            s->regs[R_CR1] = data;
            break;
        case R_DR:
            s->regs[R_SR] &= ~R_SR_TXE;
            if (s->regs[R_SR] & R_SR_RXNE) {
                s->regs[R_SR] |= R_SR_OVR;
            }
            if (s->regs[R_CR1] & R_CR1_LSBFIRST) {
                s->regs[R_DR] = bitswap(ssi_transfer(s->ssi, bitswap(data)));
            } else {
                s->regs[R_DR] = ssi_transfer(s->ssi, data);
            }
            
            s->regs[R_SR] |= R_SR_RXNE;
            s->regs[R_SR] |= R_SR_TXE;
            break;
    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            s->regs[addr] = data;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad addr 0x%" HWADDR_PRIx "\n", __func__, addr);
        }
    }
}

static const MemoryRegionOps stm32f4xx_spi_ops = {
    .read = stm32f4xx_spi_read,
    .write = stm32f4xx_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_stm32f4xx_spi = {
    .name = TYPE_STM32F4XX_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs,STM32F4XXSPIState, R_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f4xx_spi_init(Object *obj)
{
    STM32F4XXSPIState *s = STM32F4XX_SPI(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &stm32f4xx_spi_ops, s,
                          TYPE_STM32F4XX_SPI, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    s->ssi = ssi_create_bus(dev, "ssi");
}

static void stm32f4xx_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32f4xx_spi_reset;
    dc->vmsd = &vmstate_stm32f4xx_spi;
}

static const TypeInfo stm32f4xx_spi_info = {
    .name          = TYPE_STM32F4XX_SPI,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32F4XXSPIState),
    .instance_init = stm32f4xx_spi_init,
    .class_init    = stm32f4xx_spi_class_init,
};

static void stm32f4xx_spi_register_types(void)
{
    type_register_static(&stm32f4xx_spi_info);
}

type_init(stm32f4xx_spi_register_types)
