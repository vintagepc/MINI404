/*
    stm32f4xx_rng.c - RNG block for STM32F4xx

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

#include "stm32f4xx_rng.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "../utility/macros.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"

#define R_OFF_CR 0U
#define R_OFF_SR 1U
#define R_OFF_DR 2U

static uint64_t
stm32f4xx_rng_read(void *arg, hwaddr addr, unsigned int size)
{
    Stm32f4xxRNGState *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_RNG_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f4xx_rng register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    }

    switch (addr)
    {
        case R_OFF_SR:
            // No DRDY until both RNG is enabled and the required time has elapsed.
            if (s->regs.defs.CR.RNGEN && (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->next_drdy))
            {
                s->regs.defs.SR.DRDY = 1;
            }
            break;
        case R_OFF_DR:
            // Clear DRDY
            if (!s->regs.defs.SR.DRDY)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "Guest attempted to read RNG DR when DRDY = 0!");
            }
            s->regs.defs.SR.DRDY = 0;
            qemu_guest_getrandom_nofail(&s->regs.defs.DR, sizeof(&s->regs.defs.DR));
            // Calculate next DRDY:
            uint32_t clk_freq = 84000000;
            if (s->rcc)
            {
                clk_freq = stm32_rcc_get_periph_freq(s->rcc, STM32_RNG_PERIPH);
            }
            // Datasheet says this is 40 clock ticks or less. 
            s->next_drdy = (1000000000LLU/(clk_freq/40U)) + qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            break;
        case R_OFF_CR:
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "invalid read from RNG register 0x%"HWADDR_PRIx"\n", addr);

    }

    uint32_t value = s->regs.raw[addr];

    r = (value >> offset * 8) & ((1ull << (8 * size)) - 1);

    return r;
}

static void
stm32f4xx_rng_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    // int offset = addr & 0x3;
    addr >>= 2;

    Stm32f4xxRNGState *s = STM32F4XX_RNG(arg);

    switch (addr)
    {
        case R_OFF_CR:
            s->regs.raw[addr] = data;
            if (s->regs.defs.CR.IE)
            {
                printf("FIXME: RNG interrupt not implemented\n");
            }
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "invalid write to RNG register 0x%"HWADDR_PRIx"\n", addr);
    }
}

static const MemoryRegionOps stm32f4xx_rng_ops = {
    .read = stm32f4xx_rng_read,
    .write = stm32f4xx_rng_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(Stm32f4xxRNGState, stm32f4xx_rng, STM32F4XX_RNG, SYS_BUS_DEVICE, {NULL});

static void stm32f4xx_rng_reset(DeviceState *ds)
{
    Stm32f4xxRNGState *s = STM32F4XX_RNG(ds);
    memset(&s->regs.raw, 0, sizeof(s->regs.raw));
    s->next_drdy = 0;
}

static void stm32f4xx_rng_finalize(Object *obj) {

}

static void stm32f4xx_rng_realize(DeviceState *dev, Error **errp)
{
    // Stm32f4xxRNGState *s = STM32F4XX_OTP(dev);

}

static void
stm32f4xx_rng_init(Object *obj)
{
    Stm32f4xxRNGState *s = STM32F4XX_RNG(obj);
    memory_region_init_io(&s->iomem, obj, &stm32f4xx_rng_ops, s, "rng", R_RNG_MAX * 4U);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj),&s->irq, 1);

}

static const VMStateDescription vmstate_stm32f4xx_rng = {
    .name = TYPE_STM32F4XX_RNG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,Stm32f4xxRNGState, R_RNG_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32f4xx_rng_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = &stm32f4xx_rng_realize;
    dc->reset = &stm32f4xx_rng_reset;
    dc->vmsd = &vmstate_stm32f4xx_rng;

}
