/*
    stm32_exti.c - EXTI for STM32.
	- Currently modeled on STM32G070

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
#include "hw/core/irq.h"
#include "../stm32_common/stm32_common.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "../stm32_registers/generated/common/EXTI_TYPE_A_registers.h"

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Exti), COM_CLASS_NAME(Exti), STM32COM_EXTI);

typedef struct COM_STRUCT_NAME(Exti) {
    STM32Peripheral  parent;
    MemoryRegion  iomem;

    REGDEF_NAME(stm32com,exti_type_a) regs;

	const stm32_reginfo_t* reginfo;

	qemu_irq exti_out[16];

} COM_STRUCT_NAME(Exti);

typedef struct COM_CLASS_NAME(Exti) {
	STM32PeripheralClass parent_class;
	stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Exti);

#include "../stm32_registers/generated/stm32c092/EXTI_reginfo.h"
#include "../stm32_registers/generated/stm32g070/EXTI_reginfo.h"

static const stm32_periph_variant_t stm32_exti_variants[] = {
	{TYPE_STM32C092_EXTI, stm32_c092_exti_reginfo},
	{TYPE_STM32G070_EXTI, stm32_g070_exti_reginfo}
};


static void stm32_common_exti_in(void *opaque, int n, int level)
{
	COM_STRUCT_NAME(Exti) *s = STM32COM_EXTI(opaque);
	uint8_t port = n/16U; // the GPIO bank it's on.
	uint8_t pin = n%16U; // the EXTI line.
	uint8_t reg = pin/4U; // The register index.
	uint8_t shift = 8U*(pin%4U); // The shift into the register.
	uint32_t pin_mask = 1U << pin;

	g_assert(RI_EXTICR1 + reg <= (RI_EXTICR4)); //LCOV_EXCL_LINE
	uint32_t regval = s->regs.raw[RI_EXTICR1 + reg];
    if (((regval >> shift)&0xF) == port) {
		switch (level)
		{
			case EXTI_RISING:
				if (s->regs.RTSR1.raw & pin_mask)
				{
					qemu_irq_pulse(s->exti_out[pin]);
					s->regs.RPR1.raw |= pin_mask;
				}
				break;
			case EXTI_FALLING:
			if (s->regs.FTSR1.raw & pin_mask)
				{
					qemu_irq_pulse(s->exti_out[pin]);
					s->regs.FPR1.raw |= pin_mask;
				}
				break;
			default:
				// No level change, ignore it.
				break;
		}
   }
}

static uint64_t
stm32_common_exti_read(void *opaque, hwaddr addr, unsigned int size)
{
	COM_STRUCT_NAME(Exti) *s = STM32COM_EXTI(opaque);
	int offset = addr & 0x3;
	addr >>= 2;
	CHECK_BOUNDS_R_V2(addr, RI_END, s->reginfo);

	uint32_t value = s->regs.raw[addr];

	ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b111);

    return value;
}


static void
stm32_common_exti_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	COM_STRUCT_NAME(Exti) *s = STM32COM_EXTI(opaque);

	int offset = addr & 0x3;

	addr >>= 2;
	uint32_t raw_data = data;

	CHECK_BOUNDS_W_V2(addr, data , RI_END);

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b110);

	CHECK_UNIMP_RESVD_V2(data, s->regs.raw[addr], s->reginfo, addr);

	switch (addr) {
		case RI_SWIER1:
			// SW generates a rising edge and HW auto clears the bit, so we may as well not set it...
			s->regs.RPR1.raw |= raw_data;
			int index = 0;
			while (raw_data){
				if (raw_data & 1U)
				{
					qemu_irq_pulse(s->exti_out[index]);
				}
				raw_data >>= 1U;
				index++;
			}
			break;
		case RI_RTSR1 ... RI_FTSR1:
		case RI_EXTICR1 ... RI_EXTICR4:
			s->regs.raw[addr] = data;
			break;
		case RI_RPR1 ... RI_FPR1: // These are w1_c registers.
			s->regs.raw[addr] &= ~raw_data;
			break;;
	}
}

static const MemoryRegionOps stm32_common_exti_ops = {
	.read = stm32_common_exti_read,
	.write = stm32_common_exti_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.impl = {
		.min_access_size = 2,
		.max_access_size = 4,
	}
};

static void stm32_common_exti_reset(DeviceState *dev)
{
	COM_STRUCT_NAME(Exti) *s = STM32COM_EXTI(dev);
	for (int i=0;i<RI_END; i++)
	{
		if (s->reginfo[i].not_reserved)
		{
			s->regs.raw[i] = s->reginfo[i].reset_val;
		}
	}
}

static void
stm32_common_exti_init(Object *obj)
{
	COM_STRUCT_NAME(Exti) *s = STM32COM_EXTI(obj);
	assert(sizeof(s->regs)==sizeof(s->regs.raw)); // Make sure packing is correct.
	CHECK_REG_u32(s->regs.EXTICR1);
	CHECK_REG_u32(s->regs.EXTICR2);
	CHECK_REG_u32(s->regs.EXTICR3);
	CHECK_REG_u32(s->regs.EXTICR4);
	CHECK_UNION(COM_STRUCT_NAME(Exti), regs.EXTICR1, regs.raw[RI_EXTICR1]);
	CHECK_UNION(COM_STRUCT_NAME(Exti), regs.RPR1, regs.raw[RI_RPR1]);
	CHECK_UNION(COM_STRUCT_NAME(Exti), regs.IMR1, regs.raw[RI_IMR1]);
	CHECK_UNION(COM_STRUCT_NAME(Exti), regs.EMR1, regs.raw[RI_EMR1]);

	COM_CLASS_NAME(Exti) *k = STM32COM_EXTI_GET_CLASS(obj);
	s->reginfo = k->var_reginfo;

	STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_exti_ops, s, 1U*KiB);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	for (int i = 0; i < 16; i++) {
		sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->exti_out[i]);
	}

	qdev_init_gpio_in(DEVICE(obj), stm32_common_exti_in, (16U *(STM32G070_GPIO_END - STM32_P_GPIO_BEGIN)));

	}

	static const VMStateDescription vmstate_stm32_exti = {
	.name = TYPE_STM32G070_EXTI,
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = (const VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs.raw, COM_STRUCT_NAME(Exti), RI_END),
		VMSTATE_END_OF_LIST()
	}
};

static void
stm32_common_exti_class_init(ObjectClass *klass, const void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	dc->vmsd = &vmstate_stm32_exti;
	device_class_set_legacy_reset(dc, stm32_common_exti_reset);
	COM_CLASS_NAME(Exti) *k = STM32COM_EXTI_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	//k->var_reginfo = (const stm32_reginfo_t*)data;
}

static const TypeInfo
stm32_common_exti_info = {
	.name          = TYPE_STM32COM_EXTI,
	.parent        = TYPE_STM32_PERIPHERAL,
	.instance_size = sizeof(COM_STRUCT_NAME(Exti)),
	.class_size = sizeof(COM_CLASS_NAME(Exti)),
	.abstract = true,
};

static void
stm32_common_exti_register_types(void)
{
	type_register_static(&stm32_common_exti_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_exti_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_exti_variants[i].variant_name,
            .parent     = TYPE_STM32COM_EXTI,
			.instance_init = stm32_common_exti_init,
			.class_init = stm32_common_exti_class_init,
            .class_data = (void *)stm32_exti_variants[i].variant_regs,
		};
		type_register_static(&ti);
	}
}

type_init(stm32_common_exti_register_types)
