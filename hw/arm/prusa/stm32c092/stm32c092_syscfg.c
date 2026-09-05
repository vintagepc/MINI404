/*
 * STM32 Microcontroller SysCFG module
 * (STM32C092x variants)
 *
 * Copyright 2026 by VintagePC <http://github.com/vintagepc>
 *
 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0360 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */


#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "system/memory.h"
#include "../stm32_common/stm32_common.h"
#include "../stm32_common/stm32_shared.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/core/qdev-properties.h"

OBJECT_DECLARE_SIMPLE_TYPE(STM32C092_STRUCT_NAME(Syscfg), STM32C092_SYSCFG);

#include "../stm32_registers/generated/stm32c092/SYSCFG_index.h"
#include "../stm32_registers/generated/stm32c092/SYSCFG_registers.h"

typedef struct STM32C092_STRUCT_NAME(Syscfg) {
    STM32Peripheral  parent;

    MemoryRegion  iomem;

	REGDEF_NAME(c092,syscfg) regs;

	MemoryRegion *sram;
	MemoryRegion *flash;

} STM32C092_STRUCT_NAME(Syscfg);

static uint64_t
stm32_c092_syscfg_read(void *arg, hwaddr addr, unsigned int size)
{
	STM32C092_STRUCT_NAME(Syscfg) *s = STM32C092_SYSCFG(arg);

    int offset = addr & 0x3;
    addr >>= 2;

	CHECK_BOUNDS_R_V2(addr, RI_END, stm32_c092_syscfg_reginfo);

    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b100);

    return value;
}

static void
stm32_c092_syscfg_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
	STM32C092_STRUCT_NAME(Syscfg) *s = STM32C092_SYSCFG(arg);

    int offset = addr & 0x3;
    addr >>= 2;

	CHECK_BOUNDS_W_V2(addr, data, RI_END);

	CHECK_UNIMP_RESVD_V2(data, s->regs.raw[addr], stm32_c092_syscfg_reginfo, addr); // LCOV_EXCL_LINE

    ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b100);

	s->regs.raw[addr] = data;
    switch (addr)
    {
        case RI_CFGR1:
			switch (s->regs.CFGR1.bits.MEM_MODE)
			{
				case 0b00:
				case 0b01:
				{
					printf("# Syscfg: mapped main flash to 0x0\n");
					memory_region_set_enabled(s->sram, false);
					memory_region_set_enabled(s->flash, true);
				}
				break;
				case 0b11:
				{
					printf("# Syscfg: mapped SRAM to 0x0\n");
					memory_region_set_enabled(s->sram, true);
					memory_region_set_enabled(s->flash, false);
				}
				break;
				default:
					qemu_log_mask(LOG_UNIMP, "UNHANDLED: Remap SysFlash to 0x00\n");
				break;
			}
            break;
    }
}

static const MemoryRegionOps stm32_c092_syscfg_ops = {
    .read = stm32_c092_syscfg_read,
    .write = stm32_c092_syscfg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void stm32_c092_syscfg_reset(DeviceState *ds)
{
	//STM32C092_STRUCT_NAME(Syscfg) *s = STM32C092_SYSCFG(ds);
	for (int i=0;i<RI_END; i++)
	{
		if (stm32_c092_syscfg_reginfo[i].not_reserved)
		{
			stm32_c092_syscfg_write(ds, i<<2U, stm32_c092_syscfg_reginfo[i].reset_val, 4U);
		}
	}
}

static void
stm32_c092_syscfg_init(Object *obj)
{
	STM32C092_STRUCT_NAME(Syscfg) *s = STM32C092_SYSCFG(obj);

    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_c092_syscfg_ops, s, 1U* KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

}

static const VMStateDescription vmstate_stm32_c092_syscfg = {
    .name = TYPE_STM32C092_SYSCFG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,STM32C092_STRUCT_NAME(Syscfg), RI_END),
        VMSTATE_END_OF_LIST()
    }
};

static const Property stm32_c092_syscfg_properties[] = {
    DEFINE_PROP_LINK("flash", STM32C092_STRUCT_NAME(Syscfg), flash, TYPE_MEMORY_REGION, MemoryRegion*),
    DEFINE_PROP_LINK("sram",  STM32C092_STRUCT_NAME(Syscfg), sram, TYPE_MEMORY_REGION, MemoryRegion*),
};

static void
stm32_c092_syscfg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32_c092_syscfg_reset);
    dc->vmsd = &vmstate_stm32_c092_syscfg;
	device_class_set_props(dc, stm32_c092_syscfg_properties);
}

static const TypeInfo stm32_c092_syscfg_info = {
    .name          = TYPE_STM32C092_SYSCFG,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32C092_STRUCT_NAME(Syscfg)),
	.instance_init = stm32_c092_syscfg_init,
	.class_init    = stm32_c092_syscfg_class_init,
};

static void stm32_c092_syscfg_register_types(void)
{
    type_register_static(&stm32_c092_syscfg_info);
}

type_init(stm32_c092_syscfg_register_types)
