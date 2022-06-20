/*
    stm32_syscfg.c - SYSCFG for STM32F4xx.

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
#include "hw/irq.h"
#include "exec/memory.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"

OBJECT_DECLARE_TYPE(STM32F4XX_STRUCT_NAME(Syscfg), COM_CLASS_NAME(Syscfg),  STM32F4xx_SYSCFG);

REGDEF_BLOCK_BEGIN()
	REG_K32(MEM_MODE,2);
	REG_R(5);
	REG_B32(FB_MODE);
	REG_K32(SWP_FMC,2);
	REG_R(20);
REGDEF_BLOCK_END(syscfg, memrmp);

REGDEF_BLOCK_BEGIN()
	REG_R(16);
	REG_B32(ADC1DC2);
	REG_B32(ADC2DC2);
	REG_B32(ADC3DC2);
	REG_R(4);
	REG_B32(MII_RMII_SEL);
	REG_R(8);
REGDEF_BLOCK_END(syscfg, pmc);

REGDEF_BLOCK_BEGIN()
	REG_K32(EXTI0,4);
	REG_K32(EXTI1,4);
	REG_K32(EXTI2,4);
	REG_K32(EXTI3,4);
	REG_R(16);
REGDEF_BLOCK_END(syscfg, exticr1);

REGDEF_BLOCK_BEGIN()
	REG_K32(EXTI4,4);
	REG_K32(EXTI5,4);
	REG_K32(EXTI6,4);
	REG_K32(EXTI7,4);
	REG_R(16);
REGDEF_BLOCK_END(syscfg, exticr2);

REGDEF_BLOCK_BEGIN()
	REG_K32(EXTI8,4);
	REG_K32(EXTI9,4);
	REG_K32(EXTI10,4);
	REG_K32(EXTI11,4);
	REG_R(16);
REGDEF_BLOCK_END(syscfg, exticr3);

REGDEF_BLOCK_BEGIN()
	REG_K32(EXTI12,4);
	REG_K32(EXTI13,4);
	REG_K32(EXTI14,4);
	REG_K32(EXTI15,4);
	REG_R(16);
REGDEF_BLOCK_END(syscfg, exticr4);

REGDEF_BLOCK_BEGIN()
	REG_B32(CMP_PD);
	REG_R(6);
	REG_B32(READY);
	REG_R(24);
REGDEF_BLOCK_END(syscfg, cmpcr);

enum reg_index {
	RI_MEMRMP,
	RI_PMC,
	RI_EXTICR1,
	RI_EXTICR2,
	RI_EXTICR3,
	RI_EXTICR4,
	RI_CMPCR = 0x20/4,
	RI_END
};

typedef struct STM32F4XX_STRUCT_NAME(Syscfg) {
    STM32Peripheral parent;
    MemoryRegion  iomem;

    union {
        struct {
			REGDEF_NAME(syscfg,memrmp) MEMRMP;
			REGDEF_NAME(syscfg, pmc) PMC;
			REGDEF_NAME(syscfg,exticr1) EXTICR1;
			REGDEF_NAME(syscfg,exticr2) EXTICR2;
			REGDEF_NAME(syscfg,exticr3) EXTICR3;
			REGDEF_NAME(syscfg,exticr4) EXTICR4;
			REGDEF_NAME(syscfg,cmpcr) CMPCR;
        } QEMU_PACKED defs;
        uint32_t raw[RI_END];
    } QEMU_PACKED regs;

	MemoryRegion *sram;
	MemoryRegion *flash;

	qemu_irq gpio_exti_out[16];

	stm32_reginfo_t* reginfo;

} STM32F4XX_STRUCT_NAME(Syscfg);

typedef struct COM_CLASS_NAME(Syscfg) {
	STM32PeripheralClass parent;
	stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Syscfg);

enum MEMMODE
{
	MEMMODE_MAIN,
	MEMMODE_SYSFLASH,
	MEMMODE_FMC_BANK1,
	MEMMODE_SRAM1,
	MEMMODE_SDRAM1
};

static const stm32_reginfo_t stm32f40x_41x_syscfg_reginfo[RI_END] =
{
	[RI_MEMRMP] = {.mask = 0b11},
	[RI_PMC] = {.mask = 0x87<<16U, .unimp_mask = UINT32_MAX},
	[RI_EXTICR1 ... RI_EXTICR4] = {.mask = UINT16_MAX },
	[RI_EXTICR4 + 1U] = {.is_reserved = true},
	[RI_CMPCR] = {.mask = 0x81, .unimp_mask = UINT32_MAX}

};

static const stm32_periph_variant_t stm32_f4xx_syscfg_variants[] = {
 	{TYPE_STM32F40x_F41x_SYSCFG, stm32f40x_41x_syscfg_reginfo},
 };


static void stm32_common_syscfg_set_irq(void *opaque, int n, int level)
{
   	STM32F4XX_STRUCT_NAME(Syscfg) *s = STM32F4xx_SYSCFG(opaque);
	uint8_t port = n/16U; // the GPIO bank it's on.
	uint8_t pin = n%16; // the EXTI line.
	uint8_t reg = pin/4; // The register index.
    uint8_t shift = pin%4; // The shift into the register.

    g_assert(RI_EXTICR1 + reg <= (RI_EXTICR4));
	uint32_t regval = s->regs.raw[RI_EXTICR1 + reg];
    if (((regval >> shift)&0xF) == port) {
        qemu_set_irq(s->gpio_exti_out[pin], level);
   }
}


static uint64_t
stm32_common_syscfg_read(void *opaque, hwaddr addr, unsigned int size)
{
	STM32F4XX_STRUCT_NAME(Syscfg) *s = STM32F4xx_SYSCFG(opaque);
    int offset = addr & 0x3;
    addr >>= 2;

	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "Syscfg");

    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b110);

    return value;
}


static void
stm32_common_syscfg_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	STM32F4XX_STRUCT_NAME(Syscfg) *s = STM32F4xx_SYSCFG(opaque);
    int offset = addr & 0x3;
    addr >>= 2;

	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "Syscfg");

	uint32_t old = s->regs.raw[addr];

	ADJUST_FOR_OFFSET_AND_SIZE_W(old, data, size, offset, 0b110)

    switch(addr) {
		case RI_MEMRMP:
			s->regs.defs.MEMRMP.raw = data;
			switch (s->regs.defs.MEMRMP.MEM_MODE)
			{
				case MEMMODE_MAIN:
				case MEMMODE_FMC_BANK1:
				{
					printf("Syscfg: mapped main flash to 0x0\n");
					memory_region_set_enabled(s->sram, false);
					memory_region_set_enabled(s->flash, true);
				}
				break;
				case MEMMODE_SRAM1:
				{
					printf("Syscfg: mapped SRAM to 0x0\n");
					memory_region_set_enabled(s->sram, true);
					memory_region_set_enabled(s->flash, false);
				}
				break;
				default:
					printf("UNHANDLED: Remap SysFlash to 0x00\n");
				break;
			}
		break;
		case RI_EXTICR1 ... RI_EXTICR4:
			s->regs.raw[addr] = data;
			break;
    default:
        qemu_log_mask(LOG_UNIMP, "stm32_common syscfg unimplemented write 0x%x+%u size %u val 0x%x\n",
        (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
}

static const MemoryRegionOps stm32_common_syscfg_ops = {
    .read = stm32_common_syscfg_read,
    .write = stm32_common_syscfg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 4,
    }
};


static void stm32_common_syscfg_reset(DeviceState *dev)
{
	STM32F4XX_STRUCT_NAME(Syscfg) *s = STM32F4xx_SYSCFG(dev);
    memset(&s->regs, 0, sizeof(s->regs));
}


static void
stm32_common_syscfg_init(Object *obj)
{
	STM32F4XX_STRUCT_NAME(Syscfg) *s = STM32F4xx_SYSCFG(obj);
    assert(sizeof(s->regs)==sizeof(s->regs.raw)); // Make sure packing is correct.
    CHECK_REG_u32(s->regs.defs.MEMRMP);
    CHECK_REG_u32(s->regs.defs.PMC);
    CHECK_REG_u32(s->regs.defs.EXTICR1);
    CHECK_REG_u32(s->regs.defs.EXTICR2);
    CHECK_REG_u32(s->regs.defs.EXTICR3);
    CHECK_REG_u32(s->regs.defs.EXTICR4);
    CHECK_REG_u32(s->regs.defs.CMPCR);

    memory_region_init_io(&s->iomem, obj, &stm32_common_syscfg_ops, s, "syscfg", 1U*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

	// Input lines from the GPIOS for mapping EXTI out.
	qdev_init_gpio_in(DEVICE(obj), stm32_common_syscfg_set_irq, (16U *(STM32_P_GPIO_END - STM32_P_GPIO_BEGIN)));
	// exti output lines.
    qdev_init_gpio_out(DEVICE(obj), s->gpio_exti_out, 16);
	COM_CLASS_NAME(Syscfg) *k = STM32F4xx_SYSCFG_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;
}

static const VMStateDescription vmstate_stm32_common_syscfg = {
    .name = TYPE_STM32F4xx_SYSCFG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,STM32F4XX_STRUCT_NAME(Syscfg), RI_END),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32_common_syscfg_properties[] = {
    DEFINE_PROP_LINK("flash", STM32F4XX_STRUCT_NAME(Syscfg), flash, TYPE_MEMORY_REGION, MemoryRegion*),
    DEFINE_PROP_LINK("sram",  STM32F4XX_STRUCT_NAME(Syscfg), sram, TYPE_MEMORY_REGION, MemoryRegion*),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32_common_syscfg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32_common_syscfg;
    dc->reset = stm32_common_syscfg_reset;
	device_class_set_props(dc, stm32_common_syscfg_properties);

	COM_CLASS_NAME(Syscfg) *k = STM32F4xx_SYSCFG_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
}

static const TypeInfo
stm32_common_syscfg_info = {
    .name          = TYPE_STM32F4xx_SYSCFG,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32F4XX_STRUCT_NAME(Syscfg)),
	.class_size	   = sizeof(COM_CLASS_NAME(Syscfg)),
};

static void
stm32_common_syscfg_register_types(void)
{
    type_register_static(&stm32_common_syscfg_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_f4xx_syscfg_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_f4xx_syscfg_variants[i].variant_name,
            .parent     = TYPE_STM32F4xx_SYSCFG,
			.instance_init = stm32_common_syscfg_init,
			.class_init    = stm32_common_syscfg_class_init,
            .class_data = (void *)stm32_f4xx_syscfg_variants[i].variant_regs,
        };
        type_register(&ti);
    }
}

type_init(stm32_common_syscfg_register_types)
