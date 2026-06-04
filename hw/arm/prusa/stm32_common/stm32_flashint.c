/*
    stm32g070_flashint.c - Flash I/F Configuration block for STM32G0x0, STM32C092

	Copyright 2022-6 VintagePC <https://github.com/vintagepc/>

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
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "stm32_common.h"
#include "hw/core/irq.h"
#include "stm32_flashint_regdata.h"
#include "exec/cpu-common.h"

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(FlashIF), COM_CLASS_NAME(FlashIF), STM32COM_FINT);

REGDEF_BLOCK_BEGIN()
    REG_K32(LATENCY, 3);
    REG_R(5);
    REG_B32(PRFTEN);
    REG_B32(ICEN);
    REG_RB();
    REG_B32(ICRST);
	REG_R(4);
    REG_B32(EMPTY);
REGDEF_BLOCK_END(flashif, acr);

REGDEF_BLOCK_BEGIN()
	REG_K32(ADDR_ECC,14);
	REG_R(6);
	REG_B32(SYSF_ECC);
	REG_R(3);
	REG_B32(ECCCIE);
	REG_R(5);
	REG_B32(ECCC);
	REG_B32(ECCD);
REGDEF_BLOCK_END(flashif, eccr);

REGDEF_BLOCK_BEGIN()
	REG_K32(RDP,8);
	REG_R(5);
    REG_B32(nRST_STOP);
    REG_B32(nRST_STDBY);
	REG_RB();
	REG_B32(IWDG_SW);
	REG_B32(IWDG_STOP);
	REG_B32(IWDG_STBY);
	REG_B32(WWDG_SW);
	REG_RB();
	REG_B32(DUAL_BANK);
	REG_B32(RAM_PARITY_CHECK);
	REG_RB();
	REG_B32(nBOOT_SEL);
	REG_B32(nBOOT1);
	REG_B32(nBOOT0);
    REG_R(5);
REGDEF_BLOCK_END(flashif, optr);

REGDEF_BLOCK_BEGIN()
	REG_K32(START,7);
	REG_R(9);
    REG_K32(END,7);
	REG_R(9);
REGDEF_BLOCK_END(flashif, wrp);


typedef struct COM_STRUCT_NAME(FlashIF) {
    STM32Peripheral parent;
    MemoryRegion iomem;

    union {
        struct {
			REGDEF_NAME(flashif, acr) ACR; // 0x00
			REGDEF_R(0x04);
            uint32_t KEYR;	//0x08
            uint32_t OPTKEYR; //0x0C
            REGDEF_NAME(flashif, sr) SR; //0x10
            REGDEF_NAME(flashif, cr) CR; //0x14
			REGDEF_NAME(flashif, eccr) ECCR; // 0x18
			REGDEF_R(0x1C);
            REGDEF_NAME(flashif, optr) OPTR; // 0x20
            REGDEF_R(0x24);
            REGDEF_R(0x28);
            REGDEF_NAME(flashif, wrp) WRP1AR; // 0x2C
            REGDEF_NAME(flashif, wrp) WRP1BR; // 0x30
			REGDEF_R(0x34);
			REGDEF_R(0x38);
			REGDEF_R(0x3C);
			REGDEF_R(0x40);
			REGDEF_R(0x44);
			REGDEF_R(0x48);
            REGDEF_NAME(flashif, wrp) WRP2AR; // 0x4C
            REGDEF_NAME(flashif, wrp) WRP2BR; // 0x50
		} defs;
        uint32_t raw[RI_END];
    } regs;


    uint8_t flash_state;

    MemoryRegion* flash;

    qemu_irq irq;

	const stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(FlashIF);

typedef struct COM_CLASS_NAME(FlashIF) {
	STM32PeripheralClass parent_class;
	const stm32_reginfo_t* var_reginfo;
} COM_CLASS_NAME(FlashIF);

enum wp_state
{
    LOCKED,
    KEY1_OK,
    UNLOCKED
};

// Common to the F405/7 and 415/7
static const stm32_reginfo_t stm32g070_flashif_reginfo[RI_END] = {
    [RI_ACR] = {.mask = 0x10B07, .unimp_mask = 0x10B07},
    [RI_KEYR] = {.mask = UINT32_MAX},
    [RI_OPTKEYR] = {.unimp_mask = UINT32_MAX },
    [RI_SR]  = {.mask = 0x783FB, .unimp_mask = 0x683FB},
    [RI_CR] = {.mask = 0xCB07BFFF, .unimp_mask = 0x4B068004, .reset_val = 0xC0000000},
	[RI_ECCR] = {.mask = 0xC1103FFF, .unimp_mask = 0xC1103FFF},
	[RI_OPTR] = {.mask = 0x76F60FF, .unimp_mask = 0x76F60FF},
	[RI_WRP1AR ... RI_WRP1BR] = {.mask = 0x7F007F, .unimp_mask = 0x7F007F},
	[RI_WRP2AR ... RI_WRP2BR] = {.mask = 0x7F007F, .unimp_mask = 0x7F007F},
};

#include "../stm32_registers/generated/stm32c092/FLASH_reginfo.h"
#include "../stm32_registers/generated/stm32g070/FLASH_reginfo.h"


static const stm32_periph_variant_t stm32_fint_variants[] = {
	{TYPE_STM32C092_FINT, stm32_c092_flash_reginfo},
	{TYPE_STM32G070_FINT, stm32_g070_flash_reginfo},
};


#define KEY1 0x45670123UL
#define KEY2 0xCDEF89ABUL

#define OPTKEY1 0x08192A3BUL
#define OPTKEY2 0x4C5D6E7FUL

#define BOUNDARY_LN(start,size) {  (start)*KiB, (  (start+size)*KiB)-1U},

#define BOUNDARY_2K(start) BOUNDARY_LN(start+0,2)

#define BOUNDARY_10K(start) \
	BOUNDARY_2K(start+0) \
	BOUNDARY_2K(start+2) \
	BOUNDARY_2K(start+4) \
	BOUNDARY_2K(start+6) \
	BOUNDARY_2K(start+8)

#define BOUNDARY_100K(start) \
	BOUNDARY_10K(start) \
	BOUNDARY_10K(start+10) \
	BOUNDARY_10K(start+20) \
	BOUNDARY_10K(start+30) \
	BOUNDARY_10K(start+40) \
	BOUNDARY_10K(start+50) \
	BOUNDARY_10K(start+60) \
	BOUNDARY_10K(start+70) \
	BOUNDARY_10K(start+80) \
	BOUNDARY_10K(start+90)

// Up to 256K (127 2k pages)
static uint32_t sector_boundaries[][2] =
{
	BOUNDARY_100K(0)
	BOUNDARY_100K(100)
	BOUNDARY_10K(200)
	BOUNDARY_10K(210)
	BOUNDARY_10K(220)
	BOUNDARY_10K(230)
	BOUNDARY_10K(240)
	BOUNDARY_2K(250)
	BOUNDARY_2K(252)
	BOUNDARY_2K(254)
};

static uint64_t
stm32_common_fint_read(void *arg, hwaddr offset, unsigned int size)
{
    COM_STRUCT_NAME(FlashIF) *s = arg;
    uint32_t r;

    uint32_t index = offset >> 2U;
    offset&= 0x3;

	CHECK_BOUNDS_R(index, RI_END, stm32g070_flashif_reginfo, "G070 Flash Interface"); // LCOV_EXCL_LINE

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

static void stm32g070_flashif_sector_erase(COM_STRUCT_NAME(FlashIF) *s)
{
    if (s->regs.defs.CR.LOCK)
    {
		qemu_log_mask(LOG_GUEST_ERROR, "Tried to erase page %u while locked!\n",s->regs.defs.CR.PNB);
        s->regs.defs.SR.WRPERR = 1;
        return;
    }
    printf("# Erasing flash sector %u\n", s->regs.defs.CR.PNB);
    uint32_t (*p)[2] = &sector_boundaries[s->regs.defs.CR.PNB];
    uint32_t buff = 0xFFFFFFFFU;
    for (int i=(*p)[0]; i<=(*p)[1]; i+=4)
    {
        cpu_physical_memory_write(s->flash->addr +i, &buff, 4);
    }
}

static void
stm32_common_fint_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    COM_STRUCT_NAME(FlashIF) *s = arg;
    int offset = addr & 0x03;

    addr >>= 2;
    CHECK_BOUNDS_W(addr, data, RI_END, stm32g070_flashif_reginfo, "F4xx Flash IF"); // LCOV_EXCL_LINE
    ADJUST_FOR_OFFSET_AND_SIZE_W(stm32_common_fint_read(arg, addr<<2U, 4U), data, size, offset, 0b111)
    CHECK_UNIMP_RESVD(data, stm32g070_flashif_reginfo, addr);

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
                if (r.PER)
                {
                    stm32g070_flashif_sector_erase(s);
                } //else if (r.PG)
            }
        }
		break;
		case RI_SR:
		{
			s->regs.defs.SR.raw &= ~(data&0xFFFF); // handle w1c bits.
		}
        break;
        default:
            qemu_log_mask(LOG_UNIMP, "G070 FINT reg 0x%x:%d write (0x%x) unimplemented\n",
         (int)addr << 2, offset, (int)data);
            s->regs.raw[addr] = data;
            break;
    }
}

static const MemoryRegionOps stm32_common_fint_ops = {
    .read = stm32_common_fint_read,
    .write = stm32_common_fint_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32_common_fint_reset(DeviceState *dev)
{
    COM_STRUCT_NAME(FlashIF) *s = STM32COM_FINT(dev);
    s->flash_state = LOCKED;
    if (s->flash)
    {
        memory_region_set_readonly(s->flash, true);
    }
    for (int i=0; i<RI_END; i++)
    {
        s->regs.raw[i] = stm32g070_flashif_reginfo[i].reset_val;
    }
}

static void
stm32_common_fint_init(Object *obj)
{
    COM_STRUCT_NAME(FlashIF) *s = STM32COM_FINT(obj);
    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_fint_ops, s, 1U *KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

	COM_CLASS_NAME(FlashIF) *k = STM32COM_FINT_GET_CLASS(obj);
	s->reginfo = k->var_reginfo;

	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.ACR, regs.raw[RI_ACR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.KEYR, regs.raw[RI_KEYR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.OPTKEYR, regs.raw[RI_OPTKEYR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.SR, regs.raw[RI_SR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.CR, regs.raw[RI_CR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.ECCR, regs.raw[RI_ECCR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.OPTR, regs.raw[RI_OPTR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.WRP1AR, regs.raw[RI_WRP1AR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.WRP1BR, regs.raw[RI_WRP1BR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.WRP2AR, regs.raw[RI_WRP2AR]);
	CHECK_UNION(COM_STRUCT_NAME(FlashIF), regs.defs.WRP2BR, regs.raw[RI_WRP2BR]);

}

static const VMStateDescription vmstate_stm32_common_fint = {
    .name = TYPE_STM32G070_FINT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw, COM_STRUCT_NAME(FlashIF),RI_END),
        VMSTATE_UINT8(flash_state, COM_STRUCT_NAME(FlashIF)),
        VMSTATE_END_OF_LIST()
    }
};

static const Property stm32_common_fint_properties[] = {
    DEFINE_PROP_LINK("flash", COM_STRUCT_NAME(FlashIF), flash, TYPE_MEMORY_REGION, MemoryRegion *),

};

static void
stm32_common_fint_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32_common_fint_reset);
    dc->vmsd = &vmstate_stm32_common_fint;
	device_class_set_props(dc, stm32_common_fint_properties);

	COM_CLASS_NAME(FlashIF) *k = STM32COM_FINT_CLASS(klass);
	k->var_reginfo = (const stm32_reginfo_t*)data;
}

static const TypeInfo stm32_common_fint_info = {
    .name = TYPE_STM32COM_FINT,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(FlashIF)),
	.class_size = sizeof(COM_CLASS_NAME(FlashIF)),
	.instance_init = stm32_common_fint_init,
	.class_init = stm32_common_fint_class_init,
	.abstract = true
};

static void
stm32_common_fint_register_types(void)
{
    type_register_static(&stm32_common_fint_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_fint_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_fint_variants[i].variant_name,
            .parent     = TYPE_STM32COM_FINT,
            .class_data = (void *)stm32_fint_variants[i].variant_regs,
		};
		type_register_static(&ti);
	}
}

type_init(stm32_common_fint_register_types)
