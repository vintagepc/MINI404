/*
    stm32_dbg.c - DBG block for STM32

	Copyright 2023 VintagePC <https://github.com/vintagepc/>

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
#include "migration/vmstate.h"
#include "stm32_common.h"
#include "qemu/log.h"
#include "../utility/macros.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "exec/memory.h"

typedef struct COM_CLASS_NAME(Dbg) {
	STM32PeripheralClass parent_class;
	const stm32_reginfo_t* reginfo;
} COM_CLASS_NAME(Dbg);

enum REGINDEX
{
	RI_IDCODE,
	RI_CR,
	RI_APB1FZ,
	RI_APB2FZ,
	RI_END
};

static const stm32_reginfo_t stm32g070_dbg_reginfo[RI_END] =
{
	[RI_IDCODE] = {.mask = 0xFFFF0FFF, .unimp_mask = UINT32_MAX, .reset_val = 0x10000460}, // 1.0 Rev A
	[RI_CR] = {.mask = 0b110, .unimp_mask = UINT32_MAX},
	[RI_APB1FZ] = {.mask = 0x601C32, .unimp_mask = UINT32_MAX},
	[RI_APB2FZ] = {.mask = 0x78800, .unimp_mask = UINT32_MAX},
};

static const stm32_reginfo_t stm32f40x_dbg_reginfo[RI_END] =
{
	[RI_IDCODE] = {.mask = 0xFFFF0FFF, .unimp_mask = UINT32_MAX, .reset_val = 0x10000413}, // Rev A
};

static const stm32_reginfo_t stm32f42x_dbg_reginfo[RI_END] =
{
	[RI_IDCODE] = {.mask = 0xFFFF0FFF, .unimp_mask = UINT32_MAX, .reset_val = 0x10000419}, // Rev A
};

static const stm32_periph_variant_t stm32_common_dbg_variants[] = {
	{TYPE_STM32G070_DBG, stm32g070_dbg_reginfo},
	{TYPE_STM32F40x_F41x_DBG , stm32f40x_dbg_reginfo},
	{TYPE_STM32F42x_F43x_DBG , stm32f42x_dbg_reginfo}
};

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Dbg), COM_CLASS_NAME(Dbg), STM32COM_DBG);

typedef struct COM_STRUCT_NAME(Dbg) {
    STM32Peripheral  parent;
    MemoryRegion  iomem;

	union {
		struct {
			// TBD...
		} defs;
    	uint32_t raw[RI_END];
	} regs;
	const stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Dbg);

static uint64_t
stm32_common_dbg_read(void *arg, hwaddr addr, unsigned int size)
{
    COM_STRUCT_NAME(Dbg) *s = STM32COM_DBG(arg);
    int offset = addr & 0x3;
	addr >>=2;
	CHECK_BOUNDS_R(addr,RI_END, s->reginfo, "STM32 Common DBG"); // LCOV_EXCL_LINE
	uint32_t data = s->regs.raw[addr];
	ADJUST_FOR_OFFSET_AND_SIZE_R(data, size, offset, 0b100);
    return data;
}

static void
stm32_common_dbg_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    COM_STRUCT_NAME(Dbg) *s = STM32COM_DBG(arg);
    uint8_t offset = addr & 0x3;
	addr >>= 2;
	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "STM32 Common DBG"); // LCOV_EXCL_LINE

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b100);
	if (addr == RI_IDCODE)
	{
		qemu_log_mask(LOG_GUEST_ERROR, "Tried to write read-only IDCODE DBG register");
	}
	else
	{
		s->regs.raw[addr] = data;
	}
}

static const MemoryRegionOps stm32_common_dbg_ops = {
    .read =  stm32_common_dbg_read,
    .write = stm32_common_dbg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void stm32_common_dbg_realize(DeviceState *dev, Error **errp)
{
    COM_STRUCT_NAME(Dbg) *s = STM32COM_DBG(dev);
    for (int i=0; i<RI_END; i++)
    {
        s->regs.raw[i] = s->reginfo[i].reset_val;
    }
}

static void
stm32_common_dbg_init(Object *obj)
{
    COM_STRUCT_NAME(Dbg) *s = STM32COM_DBG(obj);
	COM_CLASS_NAME(Dbg) *c = STM32COM_DBG_GET_CLASS(s);
    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_dbg_ops, s, 1U*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	s->reginfo = c->reginfo;
}

static Property stm32_common_dbg_props[] = {
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32_common_dbg = {
    .name = TYPE_STM32COM_DBG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw, COM_STRUCT_NAME(Dbg), RI_END),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32_common_dbg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = &stm32_common_dbg_realize;
    dc->vmsd = &vmstate_stm32_common_dbg;
    device_class_set_props(dc, stm32_common_dbg_props);

	COM_CLASS_NAME(Dbg) *k = STM32COM_DBG_CLASS(klass);
	k->reginfo = data;
}

static const TypeInfo
stm32_common_dbg_info = {
    .name          = TYPE_STM32COM_DBG,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Dbg)),
	.class_size    = sizeof(COM_CLASS_NAME(Dbg)),
	.abstract	   = true,
};

static void
stm32_common_dbg_register_types(void)
{
    type_register_static(&stm32_common_dbg_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_common_dbg_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_common_dbg_variants[i].variant_name,
            .parent     = TYPE_STM32COM_DBG,
			.instance_init = stm32_common_dbg_init,
    		.class_init    = stm32_common_dbg_class_init,
            .class_data = (void *)stm32_common_dbg_variants[i].variant_regs,
        };
        type_register(&ti);
    }

}

type_init(stm32_common_dbg_register_types)
