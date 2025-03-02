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
#include "../stm32_common/stm32_rcc_if.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "../utility/macros.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"

#include "stm32f4xx_rng_regdata.h"

static void stm32f4xx_update_irq(Stm32f4xxRNGState* s)
{
    if (s->regs.defs.CR.IE)
    {
        qemu_set_irq(s->irq, s->regs.defs.SR.DRDY | s->regs.defs.SR.SEIS | s->regs.defs.SR.CEIS );
    }
}


static void stm32f4xx_rng_set_next_drdy(Stm32f4xxRNGState* s)
{
	uint32_t clk_freq = s->parent.clock_freq;
	// Datasheet says this is 40 clock ticks or less.
    timer_mod_ns(s->next_drdy, (NANOSECONDS_PER_SECOND/(clk_freq/40U)) + qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void stm32f4xx_rng_deadline(void *opaque) {

    Stm32f4xxRNGState *s = STM32F4XX_RNG(opaque);
    if (s->regs.defs.CR.RNGEN)
    {
        s->regs.defs.SR.DRDY = 1;
        stm32f4xx_update_irq(s);
    }
}

static uint64_t
stm32f4xx_rng_read(void *arg, hwaddr addr, unsigned int size)
{
    Stm32f4xxRNGState *s = arg;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= RI_END) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f4xx_rng register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    }

    switch (addr)
    {
        case RI_SR:
			s->regs.defs.SR.CECS = (stm32_rcc_if_check_periph_clk(&s->parent) == false);
			s->regs.defs.SR.CEIS |= s->regs.defs.SR.CECS;
            stm32f4xx_update_irq(s);
            break;
        case RI_DR:
            // Clear DRDY
            if (!s->regs.defs.SR.DRDY)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "Guest attempted to read RNG DR when DRDY = 0!");
				return 0;
            }
            s->regs.defs.SR.DRDY = 0;
            stm32f4xx_update_irq(s);
            qemu_guest_getrandom_nofail(&s->regs.defs.DR, sizeof(&s->regs.defs.DR));
            stm32f4xx_rng_set_next_drdy(s);

            break;
        case RI_CR:
            break;
    }

    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b100);

    return value;
}

static void
stm32f4xx_rng_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    Stm32f4xxRNGState *s = STM32F4XX_RNG(arg);
    int offset = addr & 0x3;
    addr >>= 2;
    ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b100);

    switch (addr)
    {
        case RI_CR:
            s->regs.raw[addr] = data;
			if (s->regs.defs.CR.RNGEN)
			{
				stm32f4xx_rng_set_next_drdy(s);
			}
            break;
		case RI_SR:
		{
			uint32_t wc = data & 0x60;
			s->regs.raw[addr] &= ~wc;
            stm32f4xx_update_irq(s);

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

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(Stm32f4xxRNGState, stm32f4xx_rng, STM32F4XX_RNG, STM32_PERIPHERAL, {NULL});

static void stm32f4xx_rng_reset(DeviceState *ds)
{
    Stm32f4xxRNGState *s = STM32F4XX_RNG(ds);
    memset(&s->regs.raw, 0, sizeof(s->regs.raw));
}

static void stm32f4xx_rng_finalize(Object *obj) {

}

static void stm32f4xx_rng_realize(DeviceState *dev, Error **errp)
{
    Stm32f4xxRNGState *s = STM32F4XX_RNG(dev);
    s->next_drdy = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32f4xx_rng_deadline, s);
}

static void
stm32f4xx_rng_init(Object *obj)
{
    Stm32f4xxRNGState *s = STM32F4XX_RNG(obj);
    STM32_MR_IO_INIT(&s->iomem, obj, &stm32f4xx_rng_ops, s, R_RNG_MAX * 4U);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
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
