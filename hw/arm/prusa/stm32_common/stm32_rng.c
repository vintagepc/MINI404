/*
    stm32_common_rng.c - RNG block for STM32F4xx

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

#include "stm32_rng.h"
#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "stm32_common.h"
#include "stm32_rcc_if.h"
#include "stm32_shared.h"
#include "stm32_types.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "../utility/macros.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Rng), COM_CLASS_NAME(Rng), STM32COM_RNG);

typedef union {
	struct {
		uint32_t _reserved0    : 2;
		uint32_t RNGEN         : 1;
		uint32_t IE            : 1;
		uint32_t _reserved4    : 1;
		uint32_t CED           : 1;
		uint32_t _reserved6    : 1;
		uint32_t ARDIS         : 1;
		uint32_t RNG_CONFIG3   : 4;
		uint32_t NISTC         : 1;
		uint32_t RNG_CONFIG2   : 3;
		uint32_t CLKDIV        : 4;
		uint32_t RNG_CONFIG1   : 6;
		uint32_t _reserved26   : 4;
		uint32_t CONDRST       : 1;
		uint32_t CONFIGLOCK    : 1;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(rng,cr);
CHECK_TYPEDEF_u32(REGDEF_NAME(rng,cr),bits);

typedef union {
	struct {
		uint32_t DRDY          : 1;
		uint32_t CECS          : 1;
		uint32_t SECS          : 1;
		uint32_t _reserved3    : 2;
		uint32_t CEIS          : 1;
		uint32_t SEIS          : 1;
		uint32_t _reserved7    :25;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(rng,sr);
CHECK_TYPEDEF_u32(REGDEF_NAME(rng,sr),bits);

typedef union {
	struct {
		uint32_t DR            :32; // Data register
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(rng,dr);
CHECK_TYPEDEF_u32(REGDEF_NAME(rng,dr),bits);

typedef union {
	struct {
		uint32_t HTCFG         :32;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(rng,htcr);
CHECK_TYPEDEF_u32(REGDEF_NAME(rng,htcr),bits);

typedef union {
	struct {
		REGDEF_NAME(rng,cr) CR;
		REGDEF_NAME(rng,sr) SR;
		REGDEF_NAME(rng,dr) DR;
		uint32_t _reserved0xC;
		REGDEF_NAME(rng,htcr) HTCR;
	} /*QEMU_PACKED*/;
	uint32_t raw[RI_END];
}  REGDEF_NAME(com,rng);
QEMU_BUILD_BUG_MSG(sizeof(REGDEF_NAME(com,rng)) != sizeof(uint32_t)*RI_END , "Structure Size mismatch - expected uint32[RI_END]");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(com,rng), CR) != 0, "Offset mismatch for CR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(com,rng), SR) != 4, "Offset mismatch for SR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(com,rng), DR) != 8, "Offset mismatch for DR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(com,rng), HTCR) != 16, "Offset mismatch for HTCR");

#include "../stm32_registers/generated/stm32f427/RNG_reginfo.h"
#include "../stm32_registers/generated/stm32h503/RNG_reginfo.h"

static const stm32_periph_variant_t stm32_rng_variants[2] = {
	{TYPE_STM32H503_RNG, stm32_h503_rng_reginfo},
	{TYPE_STM32F4xx_RNG, stm32_f427_rng_reginfo}
};

typedef struct COM_STRUCT_NAME(Rng) {
    STM32Peripheral  parent;
    MemoryRegion  iomem;

	REGDEF_NAME(com,rng) regs;

    qemu_irq irq;

	const stm32_reginfo_t* reginfo;

    QEMUTimer* next_drdy;

} COM_STRUCT_NAME(Rng);

typedef struct COM_CLASS_NAME(Rng){
	STM32PeripheralClass parent_class;
	stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Rng);


static void stm32_common_rng_update_irq(COM_STRUCT_NAME(Rng)* s)
{
    if (s->regs.CR.bits.IE)
    {
        qemu_set_irq(s->irq, s->regs.SR.bits.DRDY | s->regs.SR.bits.SEIS | s->regs.SR.bits.CEIS );
    }
}


static void stm32_common_rng_set_next_drdy(COM_STRUCT_NAME(Rng)* s)
{
	uint32_t clk_freq = s->parent.clock_freq;
	// Datasheet says this is 40 clock ticks or less.
    timer_mod_ns(s->next_drdy, (NANOSECONDS_PER_SECOND/(clk_freq/40U)) + qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void stm32_common_rng_deadline(void *opaque) {

	COM_STRUCT_NAME(Rng) *s = STM32COM_RNG(opaque);

    if (s->regs.CR.bits.RNGEN)
    {
        s->regs.SR.bits.DRDY = 1;
        stm32_common_rng_update_irq(s);
    }
}

static uint64_t
stm32_common_rng_read(void *arg, hwaddr addr, unsigned int size)
{
	COM_STRUCT_NAME(Rng) *s = STM32COM_RNG(arg);

    int offset = addr & 0x3;
    addr >>= 2;

	CHECK_BOUNDS_R_V2(addr, RI_END, s->reginfo);

    switch (addr)
    {
        case RI_SR:
			s->regs.SR.bits.CECS = (stm32_rcc_if_check_periph_clk(&s->parent) == false);
			s->regs.SR.bits.CEIS |= s->regs.SR.bits.CECS;
            stm32_common_rng_update_irq(s);
            break;
        case RI_DR:
            // Clear DRDY
            if (!s->regs.SR.bits.DRDY)
            {
                qemu_log_mask(LOG_GUEST_ERROR, "Guest attempted to read RNG DR when DRDY = 0!");
				return 0;
            }
            s->regs.SR.bits.DRDY = 0;
            stm32_common_rng_update_irq(s);
            qemu_guest_getrandom_nofail(&s->regs.raw[RI_DR], sizeof(&s->regs.raw[RI_DR]));
            stm32_common_rng_set_next_drdy(s);

            break;
        case RI_CR:
            break;
    }

    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b100);

    return value;
}

static void
stm32_common_rng_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
	COM_STRUCT_NAME(Rng) *s = STM32COM_RNG(arg);

    int offset = addr & 0x3;
    addr >>= 2;

	CHECK_BOUNDS_W_V2(addr, data, RI_END);

	ENFORCE_RESERVED_V2(data, s->regs.raw[addr], s->reginfo, addr);

    ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b100);

    switch (addr)
    {
        case RI_CR:
            s->regs.raw[addr] = data;
			if (s->regs.CR.bits.RNGEN)
			{
				stm32_common_rng_set_next_drdy(s);
			}
            break;
		case RI_SR:
		{
			uint32_t wc = data & 0x60;
			s->regs.raw[addr] &= ~wc;
            stm32_common_rng_update_irq(s);

		}
		break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "invalid write to RNG register 0x%"HWADDR_PRIx"\n", addr);
    }
}

static const MemoryRegionOps stm32_common_rng_ops = {
    .read = stm32_common_rng_read,
    .write = stm32_common_rng_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void stm32_common_rng_reset(DeviceState *ds)
{
	COM_STRUCT_NAME(Rng) *s = STM32COM_RNG(ds);
	for (int i=0;i<RI_END; i++)
	{
		s->regs.raw[i] = s->reginfo[i].reset_val;
	}
}

static void stm32_common_rng_realize(DeviceState *dev, Error **errp)
{
	COM_STRUCT_NAME(Rng) *s = STM32COM_RNG(dev);
    s->next_drdy = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32_common_rng_deadline, s);
}

static void
stm32_common_rng_init(Object *obj)
{
	COM_STRUCT_NAME(Rng) *s = STM32COM_RNG(obj);

    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_rng_ops, s, 1U* KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

	COM_CLASS_NAME(Rng) *k = STM32COM_RNG_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;
}

static const VMStateDescription vmstate_stm32_common_rng = {
    .name = TYPE_STM32COM_RNG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,COM_STRUCT_NAME(Rng), RI_END),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32_common_rng_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = &stm32_common_rng_realize;
    device_class_set_legacy_reset(dc, stm32_common_rng_reset);
    dc->vmsd = &vmstate_stm32_common_rng;

	COM_CLASS_NAME(Rng) *k = STM32COM_RNG_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");

}

static const TypeInfo stm32_com_rng_info = {
    .name          = TYPE_STM32COM_RNG,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Rng)),
	.class_size = sizeof(COM_CLASS_NAME(Rng)),
	.abstract = true,
};

static void stm32_common_rng_register_types(void)
{
    type_register_static(&stm32_com_rng_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_rng_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_rng_variants[i].variant_name,
            .parent     = TYPE_STM32COM_RNG,
			.instance_init = stm32_common_rng_init,
    		.class_init    = stm32_common_rng_class_init,
            .class_data = (void *)stm32_rng_variants[i].variant_regs,
        };
        type_register(&ti);
    }
}

type_init(stm32_common_rng_register_types)
