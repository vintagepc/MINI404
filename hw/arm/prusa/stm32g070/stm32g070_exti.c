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
#include "hw/irq.h"
#include "../stm32_common/stm32_common.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "stm32g070_exti_regdata.h"

// Mostly just convenience for easier debugging while inspecting the struct.
// All the logic takes advantage of the regularity using bitshifts and indexing.
REGDEF_BLOCK_BEGIN()
	REG_K32(EXTI0,8);
	REG_K32(EXTI1,8);
	REG_K32(EXTI2,8);
	REG_K32(EXTI3,8);
REGDEF_BLOCK_END(exti, exticr1);

REGDEF_BLOCK_BEGIN()
	REG_K32(EXTI4,8);
	REG_K32(EXTI5,8);
	REG_K32(EXTI6,8);
	REG_K32(EXTI7,8);
REGDEF_BLOCK_END(exti, exticr2);

REGDEF_BLOCK_BEGIN()
	REG_K32(EXTI8,8);
	REG_K32(EXTI9,8);
	REG_K32(EXTI10,8);
	REG_K32(EXTI11,8);
REGDEF_BLOCK_END(exti, exticr3);

REGDEF_BLOCK_BEGIN()
	REG_K32(EXTI12,8);
	REG_K32(EXTI13,8);
	REG_K32(EXTI14,8);
	REG_K32(EXTI15,8);
REGDEF_BLOCK_END(exti, exticr4);

OBJECT_DECLARE_SIMPLE_TYPE(STM32G070_STRUCT_NAME(Exti), STM32G070_EXTI);

typedef struct STM32G070_STRUCT_NAME(Exti) {
    STM32Peripheral  parent;
    MemoryRegion  iomem;

    union {
        struct {
            REG_S32(RT, 16) RTSR; // 0x00
            REG_S32(FT, 16) FTSR; // 0x04
            REG_S32(SWI, 16) SWIER; // 0x08
            REG_S32(RPIF, 16) RPR; // 0x0C
            REG_S32(FPIF, 16) FPR; // 0x10
			REGDEF_RANGE32(0x14,0x5C);
			REGDEF_NAME(exti,exticr1) EXTICR1;
			REGDEF_NAME(exti,exticr2) EXTICR2;
			REGDEF_NAME(exti,exticr3) EXTICR3;
			REGDEF_NAME(exti,exticr4) EXTICR4;
           	REGDEF_RANGE32(0x70, 0x7C);
			uint32_t IMR1;
			uint32_t EMR1;
        } defs;
        uint32_t raw[RI_END];
    } regs;

	const stm32_reginfo_t* reginfo;

	qemu_irq exti_out[16];

} STM32G070_STRUCT_NAME(Exti);

static const stm32_reginfo_t stm32g070_exti_reginfo[RI_END] =
{
	[RI_RTSR] = {.mask = UINT16_MAX },
	[RI_FTSR] = {.mask = UINT16_MAX },
	[RI_SWIER] = {.mask = UINT16_MAX },
	[RI_RPR] = {.mask = UINT16_MAX },
	[RI_FPR] = {.mask = UINT16_MAX },
	[RI_FPR + 1U ... RI_EXTICR_BEGIN -1U] = { .is_reserved = true },
	[RI_EXTICR1 ... RI_EXTICR4] = { .mask = UINT32_MAX },
	[RI_EXTICR_END ... RI_IMR - 1U] = { .is_reserved = true },
	[RI_IMR ... RI_EMR ] = {.unimp_mask= UINT32_MAX, .mask = 0x87E8FFFF}
};

static void stm32_g070_exti_in(void *opaque, int n, int level)
{
	STM32G070_STRUCT_NAME(Exti) *s = STM32G070_EXTI(opaque);
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
				if (s->regs.defs.RTSR.RT & pin_mask)
				{
					qemu_irq_pulse(s->exti_out[pin]);
					s->regs.defs.RPR.RPIF |= pin_mask;
				}
				break;
			case EXTI_FALLING:
			if (s->regs.defs.FTSR.FT & pin_mask)
				{
					qemu_irq_pulse(s->exti_out[pin]);
					s->regs.defs.FPR.FPIF |= pin_mask;
				}
				break;
			default:
				// No level change, ignore it.
				break;
		}
   }
}

static uint64_t
stm32_g070_exti_read(void *opaque, hwaddr addr, unsigned int size)
{
	STM32G070_STRUCT_NAME(Exti) *s = STM32G070_EXTI(opaque);
	int offset = addr & 0x3;

	addr >>= 2;
	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "STM32G070 EXTI"); // LCOV_EXCL_LINE

	uint32_t value = s->regs.raw[addr];

	ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b111);

    return value;
}


static void
stm32_g070_exti_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	STM32G070_STRUCT_NAME(Exti) *s = STM32G070_EXTI(opaque);

	int offset = addr & 0x3;

	addr >>= 2;
	uint32_t raw_data = data;
	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "STM32G070 EXTI "); // LCOV_EXCL_LINE
	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b111);
	CHECK_UNIMP_RESVD(data, s->reginfo, addr);

	switch (addr) {
		case RI_SWIER:
			// SW generates a rising edge and HW auto clears the bit, so we may as well not set it...
			s->regs.defs.RPR.RPIF |= raw_data;
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
		case RI_RTSR ... RI_FTSR:
		case RI_EXTICR1 ... RI_EXTICR4:
			s->regs.raw[addr] = data;
			break;
		case RI_RPR ... RI_FPR: // These are w1_c registers.
			s->regs.raw[addr] &= ~raw_data;
			break;
		default: // LCOV_EXCL_LINE
			qemu_log_mask(LOG_UNIMP, "STM32 exti unimplemented write 0x%x+%u size %u val 0x%x\n", // LCOV_EXCL_LINE
			(unsigned int)addr << 2, offset, size, (unsigned int)data);
		break;
	}
}

static const MemoryRegionOps stm32_g070_exti_ops = {
	.read = stm32_g070_exti_read,
	.write = stm32_g070_exti_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.impl = {
		.min_access_size = 2,
		.max_access_size = 4,
	}
};

static void stm32_g070_exti_reset(DeviceState *dev)
{
	STM32G070_STRUCT_NAME(Exti) *s = STM32G070_EXTI(dev);
	for (int i=0;i<RI_END; i++)
	{
		s->regs.raw[i] = s->reginfo[i].reset_val;
	}
}

static void
stm32_g070_exti_init(Object *obj)
{
	STM32G070_STRUCT_NAME(Exti) *s = STM32G070_EXTI(obj);
	assert(sizeof(s->regs)==sizeof(s->regs.raw)); // Make sure packing is correct.
	CHECK_REG_u32(s->regs.defs.EXTICR1);
	CHECK_REG_u32(s->regs.defs.EXTICR2);
	CHECK_REG_u32(s->regs.defs.EXTICR3);
	CHECK_REG_u32(s->regs.defs.EXTICR4);
	CHECK_UNION(STM32G070_STRUCT_NAME(Exti), regs.defs.EXTICR1, regs.raw[RI_EXTICR1]);
	CHECK_UNION(STM32G070_STRUCT_NAME(Exti), regs.defs.RPR, regs.raw[RI_RPR]);
	CHECK_UNION(STM32G070_STRUCT_NAME(Exti), regs.defs.IMR1, regs.raw[RI_IMR]);
	CHECK_UNION(STM32G070_STRUCT_NAME(Exti), regs.defs.EMR1, regs.raw[RI_EMR]);

	s->reginfo = stm32g070_exti_reginfo;

	STM32_MR_IO_INIT(&s->iomem, obj, &stm32_g070_exti_ops, s, 1U*KiB);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	for (int i = 0; i < 16; i++) {
		sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->exti_out[i]);
	}

	qdev_init_gpio_in(DEVICE(obj), stm32_g070_exti_in, (16U *(STM32G070_GPIO_END - STM32_P_GPIO_BEGIN)));

	}

	static const VMStateDescription vmstate_stm32g070_exti = {
	.name = TYPE_STM32G070_EXTI,
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs.raw, STM32G070_STRUCT_NAME(Exti), RI_END),
		VMSTATE_END_OF_LIST()
	}
};

static void
stm32_g070_exti_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	dc->vmsd = &vmstate_stm32g070_exti;
	dc->reset = stm32_g070_exti_reset;
	QEMU_BUILD_BUG_MSG(sizeof(stm32g070_exti_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
}

static const TypeInfo
stm32_g070_exti_info = {
	.name          = TYPE_STM32G070_EXTI,
	.parent        = TYPE_STM32_PERIPHERAL,
	.instance_size = sizeof(STM32G070_STRUCT_NAME(Exti)),
	.instance_init = stm32_g070_exti_init,
	.class_init = stm32_g070_exti_class_init,
};

static void
stm32_g070_exti_register_types(void)
{
	type_register_static(&stm32_g070_exti_info);
}

type_init(stm32_g070_exti_register_types)
