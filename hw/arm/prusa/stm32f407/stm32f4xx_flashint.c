/*
    stm32f4xx_flashint.c - Flash I/F Configuration block for STM32

	Copyright 2021-2 VintagePC <https://github.com/vintagepc/>

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
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "../utility/macros.h"
#include "../stm32_common/stm32_common.h"
#include "stm32f4xx_flashint_regdata.h"


OBJECT_DECLARE_TYPE(STM32F4XX_STRUCT_NAME(FlashIF), COM_CLASS_NAME(F4xxFlashIF), STM32F4xx_FINT)

REGDEF_BLOCK_BEGIN()
    REG_K32(LATENCY, 4);
    REG_R(5);
    REG_B32(PRFTEN);
    REG_B32(ICEN);
    REG_B32(DCEN);
    REG_B32(ICRST);
    REG_B32(DCRST);
REGDEF_BLOCK_END(flashif, acr);

REGDEF_BLOCK_BEGIN()
    REG_B32(OPTLOCK);
    REG_B32(OPTSTRT);
    REG_K32(BOR_LEV,2);
    REG_B32(BFB2);
    REG_B32(WDG_SW);
    REG_B32(nRST_STOP);
    REG_B32(nRST_STDBY);
    REG_K32(RDP,8);
    REG_K32(nWRP, 12);
    REG_R(2);
	REG_B32(DB1M);
	REG_B32(SPRMOD);
REGDEF_BLOCK_END(flashif, optcr);

REGDEF_BLOCK_BEGIN()
	REG_R(16);
    REG_K32(nWRP, 12);
	REG_R(4);
REGDEF_BLOCK_END(flashif, optcr1);


typedef struct STM32F4XX_STRUCT_NAME(FlashIF) {
    STM32Peripheral parent;
    MemoryRegion iomem;

    union {
        struct {
			REGDEF_NAME(flashif, acr) ACR;
            uint32_t KEYR;
            uint32_t OPTKEYR;
            REGDEF_NAME(flashif, sr) SR;
            REGDEF_NAME(flashif, cr) CR;
            REGDEF_NAME(flashif, optcr) OPTCR;
            REGDEF_NAME(flashif, optcr1) OPTCR1;
		} defs;
        uint32_t raw[RI_END];
    } regs;


    uint8_t flash_state;

    MemoryRegion* flash;

	stm32_reginfo_t* reginfo;

    qemu_irq irq;

} STM32F4XX_STRUCT_NAME(FlashIF);

enum wp_state
{
    LOCKED,
    KEY1_OK,
    UNLOCKED
};

// Common to the F405/7 and 415/7
static const stm32_reginfo_t stm32f40x_41x_flashif_reginfo[RI_END] = {
    [RI_ACR] = {.mask = 0x1F07, .unimp_mask = 0x1F07},
    [RI_KEYR] = {.mask = UINT32_MAX},
    [RI_OPTKEYR] = {.unimp_mask = UINT32_MAX },
    [RI_SR]  = {.mask = 0x100F3, .unimp_mask = 0x100F3},
    [RI_CR] = {.mask = 0x8101037F, .unimp_mask = 0x1010304},
    [RI_OPTCR] = {.unimp_mask = UINT32_MAX, .reset_val =0x0FFFAAED },
	[RI_OPTCR1] = {.is_reserved = true }
};

static const stm32_reginfo_t stm32f42xxx_flashif_reginfo[RI_END] = {
    [RI_ACR] = {.mask = 0x1F0F, .unimp_mask = 0x1F0F},
    [RI_KEYR] = {.mask = UINT32_MAX},
    [RI_OPTKEYR] = {.unimp_mask = UINT32_MAX },
    [RI_SR]  = {.mask = 0x100F3, .unimp_mask = 0x100F3},
    [RI_CR] = {.mask = 0x810183FF, .unimp_mask = 0x1018304},
    [RI_OPTCR] = {.unimp_mask = UINT32_MAX, .reset_val =0x0FFFAAED },
	[RI_OPTCR1] = {.mask = 0x0FFF0000, .unimp_mask = UINT32_MAX, .reset_val =0x0FFF0000 },
};

static const stm32_periph_variant_t stm32f4xx_flashif_variants[] = {
	{TYPE_STM32F40x_F41x_FINT, stm32f40x_41x_flashif_reginfo },
	{TYPE_STM32F42x_F43x_FINT, stm32f42xxx_flashif_reginfo },
};

#define KEY1 0x45670123UL
#define KEY2 0xCDEF89ABUL

#define OPTKEY1 0x08192A3BUL
#define OPTKEY2 0x4C5D6E7FUL

static uint32_t sector_boundaries[][2] =
{
    { 0U*KiB, ( 16U*KiB)-1U},
    { 16U*KiB, ( 32U*KiB)-1U},
    { 32U*KiB, ( 48U*KiB)-1U},
    { 48U*KiB, ( 64U*KiB)-1U},
    { 64U*KiB, (128U*KiB)-1U},
    {128U*KiB, (256U*KiB)-1U},
    {256U*KiB, (384U*KiB)-1U},
    {384U*KiB, (512U*KiB)-1U},
    {512U*KiB, (640U*KiB)-1U},
    {640U*KiB, (768U*KiB)-1U},
    {768U*KiB, (896U*KiB)-1U},
    {896U*KiB, (1U*MiB)-1U},
	{(1U*MiB) +   0U*KiB, ((1U * MiB) +  16U*KiB)-1U},
    {(1U*MiB) +  16U*KiB, ((1U * MiB) +  32U*KiB)-1U},
    {(1U*MiB) +  32U*KiB, ((1U * MiB) +  48U*KiB)-1U},
    {(1U*MiB) +  48U*KiB, ((1U * MiB) +  64U*KiB)-1U},
    {(1U*MiB) +  64U*KiB, ((1U * MiB) + 128U*KiB)-1U},
    {(1U*MiB) + 128U*KiB, ((1U * MiB) + 256U*KiB)-1U},
    {(1U*MiB) + 256U*KiB, ((1U * MiB) + 384U*KiB)-1U},
    {(1U*MiB) + 384U*KiB, ((1U * MiB) + 512U*KiB)-1U},
    {(1U*MiB) + 512U*KiB, ((1U * MiB) + 640U*KiB)-1U},
    {(1U*MiB) + 640U*KiB, ((1U * MiB) + 768U*KiB)-1U},
    {(1U*MiB) + 768U*KiB, ((1U * MiB) + 896U*KiB)-1U},
    {(1U*MiB) + 896U*KiB, ((1U * MiB) + 1U*MiB)-1U},
};

typedef struct COM_CLASS_NAME(F4xxFlashIF) {
	STM32PeripheralClass parent_class;
    stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(F4xxFlashIF);

static uint64_t
stm32f4xx_fint_read(void *arg, hwaddr offset, unsigned int size)
{
    STM32F4XX_STRUCT_NAME(FlashIF) *s = arg;
    uint32_t r;

    uint32_t index = offset >> 2U;
    offset&= 0x3;

	CHECK_BOUNDS_R(index, RI_END, s->reginfo, "F4xx Flash Interface"); // LCOV_EXCL_LINE

    switch (index)
    {
        case RI_CR:
            s->regs.defs.CR.LOCK = (s->flash_state != UNLOCKED);
            /* FALLTHRU */
        default:
            r = s->regs.raw[index];
            break;
    }
    ADJUST_FOR_OFFSET_AND_SIZE_R(r, size, offset, 0b111);
//printf("FINT unit %d reg %x return 0x%x\n", s->periph, (int)offset << 2, r);
    return r;
}

static void stm32f4xx_flashif_sector_erase(STM32F4XX_STRUCT_NAME(FlashIF) *s)
{
    if (s->regs.defs.CR.LOCK)
    {
        s->regs.defs.SR.WRPERR = 1;
        return;
    }
    printf("# Erasing flash sector %u\n", s->regs.defs.CR.SNB);
    uint32_t (*p)[2] = &sector_boundaries[s->regs.defs.CR.SNB];
    uint32_t buff = 0xFFFFFFFFU;
    for (int i=(*p)[0]; i<=(*p)[1]; i+=4)
    {
        cpu_physical_memory_write(s->flash->addr +i, &buff, 4);
    }
}

static void
stm32f4xx_fint_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    STM32F4XX_STRUCT_NAME(FlashIF) *s = arg;
    int offset = addr & 0x03;

    addr >>= 2;
    CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "F4xx Flash IF"); // LCOV_EXCL_LINE
    ADJUST_FOR_OFFSET_AND_SIZE_W(stm32f4xx_fint_read(arg, addr<<2U, 4U), data, size, offset, 0b111)
    CHECK_UNIMP_RESVD(data, s->reginfo, addr);

    if (s->flash_state == KEY1_OK && addr != RI_KEYR)
    {
        s->flash_state = LOCKED;
    }

    switch (addr) {
        case RI_KEYR:
        {
            if (data == KEY1)
                s->flash_state = KEY1_OK;
            else if (data == KEY2 && s->flash_state == KEY1_OK)
            {
                s->flash_state = UNLOCKED;
                printf("# Flash unlocked!\n");
                memory_region_set_readonly(s->flash, false);
            }
        }
        break;
		case RI_SR:
		{
            REGDEF_NAME(flashif, sr) r = {.raw = data};
			if (r.WRPERR)
			{
				s->regs.defs.SR.WRPERR = false;
			}

		}
		break;
        case RI_CR:
        {
            REGDEF_NAME(flashif, cr) r = {.raw = data};
            if ( s->flash_state != UNLOCKED && !r.LOCK)
            {
                qemu_log_mask(LOG_GUEST_ERROR, __FILE__":Guest tried to clear the set-only flask LOCK bit.");
                r.LOCK = 1;
    }
            else if (s->flash_state == UNLOCKED && r.LOCK)
            {
                printf("# Flash LOCKED\n");
                memory_region_set_readonly(s->flash, true);
                s->flash_state = LOCKED;
            }
            s->regs.defs.CR.raw = r.raw;
            s->regs.defs.CR.STRT = false;
            if (r.STRT)
            {
                if (r.SER)
                {
                    stm32f4xx_flashif_sector_erase(s);
                } //else if (r.PG)
            }
        }
        break;
        default: // LCOV_EXCL_START
            qemu_log_mask(LOG_UNIMP, "f2xx FINT reg 0x%x:%d write (0x%x) unimplemented\n",
         (int)addr << 2, offset, (int)data);
            s->regs.raw[addr] = data;
            break;
    } // LCOV_EXCL_STOP
}

static const MemoryRegionOps stm32f4xx_fint_ops = {
    .read = stm32f4xx_fint_read,
    .write = stm32f4xx_fint_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32f4xx_fint_reset(DeviceState *dev)
{
    STM32F4XX_STRUCT_NAME(FlashIF) *s = STM32F4xx_FINT(dev);
    s->flash_state = LOCKED;
    if (s->flash)
    {
        memory_region_set_readonly(s->flash, true);
    }
    for (int i=0; i<RI_END; i++)
    {
        s->regs.raw[i] = s->reginfo[i].reset_val;
    }
}

static void
stm32f4xx_fint_init(Object *obj)
{
    STM32F4XX_STRUCT_NAME(FlashIF) *s = STM32F4xx_FINT(obj);
    STM32_MR_IO_INIT(&s->iomem, obj, &stm32f4xx_fint_ops, s, 1U *KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
	COM_CLASS_NAME(F4xxFlashIF) *k = STM32F4xx_FINT_GET_CLASS(obj);
	s->reginfo = k->var_reginfo;
}

static const VMStateDescription vmstate_stm32f4xx_fint = {
    .name = TYPE_STM32F4xx_FINT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw, STM32F4XX_STRUCT_NAME(FlashIF),RI_END),
        VMSTATE_UINT8(flash_state, STM32F4XX_STRUCT_NAME(FlashIF)),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32f4xx_fint_properties[] = {
    DEFINE_PROP_LINK("flash", STM32F4XX_STRUCT_NAME(FlashIF), flash, TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32f4xx_fint_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32f4xx_fint_reset;
    dc->vmsd = &vmstate_stm32f4xx_fint;
	device_class_set_props(dc, stm32f4xx_fint_properties);
	COM_CLASS_NAME(F4xxFlashIF) *k = STM32F4xx_FINT_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
}

static const TypeInfo stm32f4xx_fint_info = {
    .name = TYPE_STM32F4xx_FINT,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32F4XX_STRUCT_NAME(FlashIF)),
	.class_size = sizeof(COM_CLASS_NAME(F4xxFlashIF)),
	.abstract = true,
};

static void
stm32f4xx_fint_register_types(void)
{
    type_register_static(&stm32f4xx_fint_info);
		for (int i = 0; i < ARRAY_SIZE(stm32f4xx_flashif_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32f4xx_flashif_variants[i].variant_name,
            .parent     = TYPE_STM32F4xx_FINT,
			.instance_init = stm32f4xx_fint_init,
			.class_init = stm32f4xx_fint_class_init,
            .class_data = (void *)stm32f4xx_flashif_variants[i].variant_regs,
        };
        type_register(&ti);
    }
}

type_init(stm32f4xx_fint_register_types)
