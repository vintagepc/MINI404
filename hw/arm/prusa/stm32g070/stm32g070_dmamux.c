/*
    stm32g070_dmamux.c - DMAMUX for STM32G070.

	Copyright 2022 VintagePC <https://github.com/vintagepc/>

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


#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "exec/memory.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "../stm32_common/stm32_common.h"
#include "../stm32_common/stm32_shared.h"

enum reg_index
{
	RI_CxCR_BASE,
	RI_C0CR = 0,
	RI_C1CR,
	RI_C2CR,
	RI_C3CR,
	RI_C4CR,
	RI_C5CR,
	RI_C6CR,
	RI_C7CR,
	RI_C8CR,
	RI_C9CR,
	RI_C10CR,
	RI_C11CR,
	RI_CxCR_END = RI_C11CR,
	RI_CSR = 0x80/4,
	RI_CFR,
	RI_RG0CR = 0x100/4,
	RI_RGxCR_BASE = RI_RG0CR,
	RI_RG1CR,
	RI_RG2CR,
	RI_RG3CR,
	RI_RGxCR_END = RI_RG3CR,
	RI_RGSR = 0x140/4,
	RI_RGCFGR,
	RI_END,
};

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Dmamux), COM_CLASS_NAME(Dmamux), STM32COM_DMAMUX);

REGDEF_BLOCK_BEGIN()
	REG_K32(ID, 7);
	REG_RB();
	REG_B32(SOIE);
	REG_B32(EGE);
	REG_R(6);
	REG_B32(SE);
	REG_K32(SPOL,2);
	REG_K32(NBREQ, 5);
	REG_K32(SYNC_ID, 5);
	REG_R(3);
REGDEF_BLOCK_END(dmamux, cxcr)

REGDEF_BLOCK_BEGIN()
	REG_K32(SIG_ID, 5);
	REG_R(3);
	REG_B32(OIE);
	REG_R(7);
	REG_B32(GE);
	REG_K32(GPOL,2);
	REG_K32(GNBREQ, 5);
	REG_R(8);
REGDEF_BLOCK_END(dmamux, rgxcr)

typedef struct COM_STRUCT_NAME(Dmamux) {
    STM32Peripheral parent;
    MemoryRegion  iomem;

    union {
        struct {
			REGDEF_NAME(dmamux, cxcr) C0CR;		//0x00
			REGDEF_NAME(dmamux, cxcr) C1CR;		//0x04
			REGDEF_NAME(dmamux, cxcr) C2CR;		//0x08
			REGDEF_NAME(dmamux, cxcr) C3CR;		//0x0C
			REGDEF_NAME(dmamux, cxcr) C4CR;		//0x10
			REGDEF_NAME(dmamux, cxcr) C5CR;		//0x14
			REGDEF_NAME(dmamux, cxcr) C6CR;		//0x18
			REGDEF_NAME(dmamux, cxcr) C7CR;		//0x1C
			REGDEF_NAME(dmamux, cxcr) C8CR;		//0x20
			REGDEF_NAME(dmamux, cxcr) C9CR;		//0x24
			REGDEF_NAME(dmamux, cxcr) C10CR;		//0x28
			REGDEF_NAME(dmamux, cxcr) C11CR;		//0x2C
			uint32_t _unused[RI_CSR - RI_CxCR_END - 1];
			uint32_t CSR;
			uint32_t CFR;
			uint32_t _unused2[RI_RGxCR_BASE - RI_CFR - 1];
			REGDEF_NAME(dmamux, rgxcr) RG0CR;
			REGDEF_NAME(dmamux, rgxcr) RG1CR;
			REGDEF_NAME(dmamux, rgxcr) RG2CR;
			REGDEF_NAME(dmamux, rgxcr) RG3CR;
			uint32_t _unused3[RI_RGSR - RI_RGxCR_END - 1];
			uint32_t RGSR;
			uint32_t RGCFGR;
		} defs;
        uint32_t raw[RI_END];
    } QEMU_PACKED regs;

	qemu_irq irq;

	const stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Dmamux);

typedef struct COM_CLASS_NAME(Dmamux) {
	STM32PeripheralClass parent_class;
    stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Dmamux);

static const stm32_reginfo_t stm32g070_dmamux_reginfo[RI_END] =
{
	[RI_CxCR_BASE ... RI_CxCR_END] = {.mask = 0x1FFF037F, .unimp_mask = 0x1FFF037F},
	[RI_CxCR_END+1 ... RI_CSR -1] = { .is_reserved = true },
	[RI_CSR ... RI_CFR] = { .mask = 0xFFF, .unimp_mask = 0xFFF },
	[RI_CFR+1 ... RI_RGxCR_BASE -1] = { .is_reserved = true },
	[RI_RGxCR_BASE ... RI_RGxCR_END] = {.mask = 0xFF011F, .unimp_mask = 0xFF011F },
	[RI_RGxCR_END + 1 ... RI_RGSR - 1] = {.is_reserved = true},
	[RI_RGSR ... RI_RGCFGR] = {.mask = 0xF, .unimp_mask = 0xF}
};

static const stm32_periph_variant_t stm32_dmamux_variants[] = {
 	{TYPE_STM32G070_DMAMUX, stm32g070_dmamux_reginfo},
};

static uint64_t
stm32_common_dmamux_read(void *opaque, hwaddr addr, unsigned int size)
{
	COM_STRUCT_NAME(Dmamux) *s = STM32COM_DMAMUX(opaque);
    int offset = addr & 0x3;

    addr >>= 2;

	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "STM32G070 DMAMUX");

    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b110);
    return value;
}

// static void
// stm32_common_dmamux_chan_write(COM_STRUCT_NAME(Dmamux) *s, hwaddr addr, uint64_t data, unsigned int size)
// {
// 	uint8_t chan = (addr - RI_CxCR_BASE);
// 	// printf("Wrote DMA channel %u/off %u with value %0"PRIx64"\n", chan, offset, data);
// }

static void
stm32_common_dmamux_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	COM_STRUCT_NAME(Dmamux) *s = STM32COM_DMAMUX(opaque);

    int offset = addr & 0x3;

    addr >>= 2;

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 6);

	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "STM32G070 DMAMUX");
	CHECK_UNIMP_RESVD(data, s->reginfo, addr);

    switch(addr) {
		// case RI_CxCR_BASE ... RI_CxCR_END:
			// stm32_common_dmamux_chan_write(s, addr, data, size);
			// break;
		default:
			s->regs.raw[addr] = data;
    }
}

static const MemoryRegionOps stm32_common_dmamux_ops = {
    .read = stm32_common_dmamux_read,
    .write = stm32_common_dmamux_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 4,
    }
};


static void stm32_common_dmamux_reset(DeviceState *dev)
{
	COM_STRUCT_NAME(Dmamux) *s = STM32COM_DMAMUX(dev);
    memset(&s->regs, 0, sizeof(s->regs));
}

static void
stm32_common_dmamux_init(Object *obj)
{
	COM_STRUCT_NAME(Dmamux) *s = STM32COM_DMAMUX(obj);

    CHECK_REG_u32(s->regs.defs.C0CR);
    CHECK_REG_u32(s->regs.defs.RG0CR);
    QEMU_BUILD_BUG_MSG(sizeof(s->regs.defs)!=sizeof(s->regs.raw), __FILE__" definitions/unions do not align!"); // Make sure packing is correct.

    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_dmamux_ops, s, 2U*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

	COM_CLASS_NAME(Dmamux) *k = STM32COM_DMAMUX_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;
}

static const VMStateDescription vmstate_stm32_common_dmamux = {
    .name = TYPE_STM32COM_DMAMUX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,COM_STRUCT_NAME(Dmamux), RI_END),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32_common_dmamux_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32_common_dmamux;
    dc->reset = stm32_common_dmamux_reset;

	COM_CLASS_NAME(Dmamux) *k = STM32COM_DMAMUX_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
};

static const TypeInfo
stm32_common_dmamux_info = {
    .name          = TYPE_STM32COM_DMAMUX,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Dmamux)),
	.class_size    = sizeof(COM_CLASS_NAME(Dmamux)),
	.abstract = true,
};

static void
stm32_common_dmamux_register_types(void)
{
    type_register_static(&stm32_common_dmamux_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_dmamux_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_dmamux_variants[i].variant_name,
            .parent     = TYPE_STM32COM_DMAMUX,
			.instance_init = stm32_common_dmamux_init,
			.class_init    = stm32_common_dmamux_class_init,
            .class_data = (void *)stm32_dmamux_variants[i].variant_regs,
        };
        type_register(&ti);
    }
}

type_init(stm32_common_dmamux_register_types)
