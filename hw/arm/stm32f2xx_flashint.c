/*-
 * Copyright (c) 2020 VintagePC
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
 * QEMU model of the stm32f2xx FINT module
 */
#include "stm32f2xx_flashint.h"
#include "hw/irq.h"
#include "qemu/log.h"

//#define DEBUG_STM32_FINT
#ifdef DEBUG_STM32_FINT
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F2XX_FINT: " fmt , ## __VA_ARGS__); \
         usleep(100); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


static uint64_t
stm32f2xx_fint_read(void *arg, hwaddr offset, unsigned int size)
{
    stm32f2xx_fint *s = arg;
    uint32_t r;

    offset >>= 2;
    r = s->regs[offset];
//printf("FINT unit %d reg %x return 0x%x\n", s->periph, (int)offset << 2, r);
    return r;
}

static void
stm32f2xx_fint_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    stm32f2xx_fint *s = arg;
    int offset = addr % 3;

    addr >>= 2;
    if (addr > STM32_FINT_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid FINT write reg 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    switch(size) {
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
        default:
            qemu_log_mask(LOG_UNIMP, "f2xx FINT reg 0x%x:%d write (0x%x) unimplemented\n",
         (int)addr << 2, offset, (int)data);
            s->regs[addr] = data;
            break;
    }
}

static const MemoryRegionOps stm32f2xx_fint_ops = {
    .read = stm32f2xx_fint_read,
    .write = stm32f2xx_fint_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32f2xx_fint_reset(DeviceState *dev)
{
    stm32f2xx_fint *s = STM32F2XX_FINT(dev);
    s->regs[STM32_FINT_CR] = 0x80000000;
    s->regs[STM32_FINT_OPTCR] = 0x0FFFAAED;
}

static void
stm32f2xx_fint_init(Object *obj)
{
    stm32f2xx_fint *s = STM32F2XX_FINT(obj);

    memory_region_init_io(&s->iomem, obj, &stm32f2xx_fint_ops, s, "fint", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void
stm32f2xx_fint_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32f2xx_fint_reset;
}

static const TypeInfo stm32f2xx_fint_info = {
    .name = TYPE_STM32F2XX_FINT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(stm32f2xx_fint),
    .instance_init = stm32f2xx_fint_init,
    .class_init = stm32f2xx_fint_class_init
};

static void
stm32f2xx_fint_register_types(void)
{
    type_register_static(&stm32f2xx_fint_info);
}

type_init(stm32f2xx_fint_register_types)
