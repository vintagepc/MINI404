/*
    stm32f2xx_flashint.c - Flash I/F Configuration block for STM32

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "stm32f2xx_flashint.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
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
    int offset = addr & 0x03;

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
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_stm32f2xx_fint = {
    .name = TYPE_STM32F2XX_FINT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, stm32f2xx_fint,STM32_FINT_MAX),
        VMSTATE_END_OF_LIST()
    }
};


static void
stm32f2xx_fint_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32f2xx_fint_reset;
    dc->vmsd = &vmstate_stm32f2xx_fint;
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
