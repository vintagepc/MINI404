/*
 * STM32 Microcontroller Power module
 * (STM32H503x variants)
 *
 * Copyright 2022 by VintagePC <http://github.com/vintagepc>
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
#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include <stdio.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/units.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_common/stm32_common.h"
#include "../utility/macros.h"
#include "../stm32_registers/generated/stm32h503/PWR_index.h"
#include "../stm32_registers/generated/stm32h503/PWR_registers.h"


OBJECT_DECLARE_SIMPLE_TYPE(STM32H503_STRUCT_NAME(Pwr), STM32H503_PWR);

typedef struct STM32H503_STRUCT_NAME(Pwr) {
    /* <private> */
    STM32Peripheral parent;

	/* <public> */
	/* Memory region */
	MemoryRegion mmio;

	stm32reg_h503_pwr_t regs;

} STM32H503_STRUCT_NAME(Pwr);


static uint64_t stm32_h503_pwr_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    STM32H503_STRUCT_NAME(Pwr) *s = STM32H503_PWR(opaque);
  	int shift = offset & 0x3;
	int index = offset >>2;
	CHECK_BOUNDS_R_V2(index,RI_END, stm32_h503_pwr_reginfo); // LCOV_EXCL_LINE
	uint32_t data = s->regs.raw[index];
	ADJUST_FOR_OFFSET_AND_SIZE_R(data, size, shift, 0b111);
    return data;
}

static void stm32_h503_pwr_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    STM32H503_STRUCT_NAME(Pwr) *s = STM32H503_PWR(opaque);
  	int shift = offset & 0x3;
	int index = offset >>2;
	CHECK_BOUNDS_W_V2(index,value, RI_END); // LCOV_EXCL_LINE
	CHECK_UNIMP_RESVD_V2(value, s->regs.raw[index], stm32_h503_pwr_reginfo, index);
	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[index], value, size, shift, 0b111);
	s->regs.raw[index] = value;
}

static const MemoryRegionOps stm32_h503_pwr_ops = {
    .read =  stm32_h503_pwr_read,
    .write = stm32_h503_pwr_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_h503_pwr_reset(DeviceState *dev)
{
    STM32H503_STRUCT_NAME(Pwr) *s = STM32H503_PWR(dev);

	memset(&s->regs.raw, 0, sizeof(s->regs.raw));

	for (int i=0; i< RI_END; i++)
	{
		if (stm32_h503_pwr_reginfo[i].not_reserved)
			stm32_h503_pwr_write(dev, i<<2U, stm32_h503_pwr_reginfo[i].reset_val, true);
	}
}

/* DEVICE INITIALIZATION */
static void stm32_h503_pwr_realize(DeviceState *dev, Error **errp)
{
    // STM32H503_STRUCT_NAME(Pwr) *s = STM32H503_PWR(dev);
}

static void stm32_h503_pwr_init(Object *obj)
{

    STM32H503_STRUCT_NAME(Pwr) *s = STM32H503_PWR(obj);
    STM32_MR_IO_INIT(&s->mmio, obj, &stm32_h503_pwr_ops, s, 1U * KiB);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

}

static const VMStateDescription vmstate_stm32h503_pwr = {
    .name = TYPE_STM32H503_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs.raw, STM32H503_STRUCT_NAME(Pwr), RI_END),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_h503_pwr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32_h503_pwr_reset);
    dc->realize = stm32_h503_pwr_realize;
    dc->vmsd = &vmstate_stm32h503_pwr;
}

static TypeInfo stm32_h503_pwr_info = {
    .name  = TYPE_STM32H503_PWR,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size  = sizeof(STM32H503_STRUCT_NAME(Pwr)),
    .class_init = stm32_h503_pwr_class_init,
    .instance_init = stm32_h503_pwr_init,
};

static void stm32_h503_pwr_register_types(void)
{
    type_register_static(&stm32_h503_pwr_info);
}

type_init(stm32_h503_pwr_register_types)
