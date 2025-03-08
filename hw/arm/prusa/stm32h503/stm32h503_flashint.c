/*
    stm32h503_flashint.c - Flash I/F Configuration block for STM32H5

	Copyright 2025 VintagePC <https://github.com/vintagepc/>

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
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "../stm32_common/stm32_common.h"
#include "hw/irq.h"
#include "../stm32_registers/generated/stm32h503/FLASH_registers.h"
#include "trace.h"


OBJECT_DECLARE_SIMPLE_TYPE(STM32H503_STRUCT_NAME(FlashIF), STM32H503_FINT);


typedef struct STM32H503_STRUCT_NAME(FlashIF) {
    STM32Peripheral parent;
    MemoryRegion iomem;

    REGDEF_NAME(h503,flash) regs;

    uint8_t flash_state;

    MemoryRegion* flash;

    qemu_irq irq;

} STM32H503_STRUCT_NAME(FlashIF);

enum wp_state
{
    LOCKED,
    KEY1_OK,
    UNLOCKED
};

#define KEY1 0x45670123UL
#define KEY2 0xCDEF89ABUL

#define OPTKEY1 0x08192A3BUL
#define OPTKEY2 0x4C5D6E7FUL

#define BOUNDARY_LN(start,size) {  (start)*KiB, (  (start+size)*KiB)-1U},

// 128KB flash, 8KB sectors. Can be erased individually, one bank of 8 sectors, or both banks.
static uint32_t sector_boundaries[][2] =
{
	BOUNDARY_LN(0, 8)
	BOUNDARY_LN(8, 8)
	BOUNDARY_LN(16, 8)
	BOUNDARY_LN(24, 8)
	BOUNDARY_LN(32, 8)
	BOUNDARY_LN(40, 8)
	BOUNDARY_LN(48, 8)
	BOUNDARY_LN(56, 8)
	BOUNDARY_LN(64, 8)
	BOUNDARY_LN(72, 8)
	BOUNDARY_LN(80, 8)
	BOUNDARY_LN(88, 8)
	BOUNDARY_LN(96, 8)
	BOUNDARY_LN(104, 8)
	BOUNDARY_LN(112, 8)
	BOUNDARY_LN(120, 8)
};

enum ERASE_TYPE
{
	ET_SECTOR,
	ET_BANK,
	ET_MASS
};

static uint64_t
stm32h503_fint_read(void *arg, hwaddr offset, unsigned int size)
{
    STM32H503_STRUCT_NAME(FlashIF) *s = arg;
    uint32_t r;

    uint32_t index = offset >> 2U;
    offset&= 0x3;

	CHECK_BOUNDS_R_V2(index, RI_END, stm32_h503_flash_reginfo); // LCOV_EXCL_LINE

    switch (index)
    {
        case RI_CR:
            s->regs.CR.bits.LOCK = (s->flash_state != UNLOCKED);
            /* FALLTHRU */
        default:
            r = s->regs.raw[index];
            break;
    }
    ADJUST_FOR_OFFSET_AND_SIZE_R(r, size, offset, 0b111);
//printf("FINT unit %d reg %x return 0x%x\n", s->periph, (int)offset << 2, r);
    return r;
}

static void stm32h503_flashif_erase(STM32H503_STRUCT_NAME(FlashIF) *s, int type)
{
    if (s->regs.CR.bits.LOCK)
    {
		qemu_log_mask(LOG_GUEST_ERROR, "Tried to erase flash while locked!\n");
        s->regs.SR.bits.WRPERR = 1;
        return;
    }
	trace_stm32h503_fint_erase(s->regs.CR.bits.SNB, s->regs.CR.bits.BKSEL, s->regs.CR.bits.MER << 2U | s->regs.CR.bits.BER << 1U | s->regs.CR.bits.SER);
    const uint32_t buff = 0xFFFFFFFFU;
	uint32_t start = 0;
	uint32_t end = 0;
	switch (type)
	{
		case ET_SECTOR:
		{
			int sector = s->regs.CR.bits.SNB + (8U * s->regs.CR.bits.BKSEL);
			start = sector_boundaries[sector][0];
			end = sector_boundaries[sector][1];
		}
		break;
		case ET_BANK:
		{
			int sector = s->regs.CR.bits.BKSEL ? 8U : 0U;
			start = sector_boundaries[sector][0];
			end = sector_boundaries[sector + 7U][1];
		}
		break;
		case ET_MASS:
		{
			start = sector_boundaries[0][0];
			end = sector_boundaries[15][1];
		}
		break;
	}
	trace_stm32h503_fint_erase_range(start, end);
    for (int i=start; i<=end; i+=4)
    {
        cpu_physical_memory_write(s->flash->addr +i, &buff, 4);
    }
}

static void
stm32h503_fint_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    STM32H503_STRUCT_NAME(FlashIF) *s = arg;
    int offset = addr & 0x03;

    addr >>= 2;
    CHECK_BOUNDS_W_V2(addr, data, RI_END); // LCOV_EXCL_LINE
    ADJUST_FOR_OFFSET_AND_SIZE_W(stm32h503_fint_read(arg, addr<<2U, 4U), data, size, offset, 0b111)
    CHECK_UNIMP_RESVD_V2(data, s->regs.raw[addr], stm32_h503_flash_reginfo, addr);

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
                trace_stm32h503_fint_unlock();
                memory_region_set_readonly(s->flash, false);
            }
        }
        break;
        case RI_CR:
        {
            REGDEF_NAME(h503_flash, cr) r = {.raw = data};
            if ( s->flash_state != UNLOCKED && !r.bits.LOCK)
            {
                qemu_log_mask(LOG_GUEST_ERROR, __FILE__":Guest tried to clear the set-only flask LOCK bit.");
                r.bits.LOCK = 1;
    		}
            else if (s->flash_state == UNLOCKED && r.bits.LOCK)
            {
                trace_stm32h503_fint_lock();
                memory_region_set_readonly(s->flash, true);
                s->flash_state = LOCKED;
            }
            s->regs.CR.raw = r.raw;
            s->regs.CR.bits.START = false;
            if (r.bits.START)
            {
                if (r.bits.SER + r.bits.BER + r.bits.MER > 1)
				{
					qemu_log_mask(LOG_GUEST_ERROR, "Multiple of SER, BER, MER set same time!\n");
					s->regs.SR.bits.PGSERR = 1;
				}
				else if (r.bits.SER)
                {
                    stm32h503_flashif_erase(s, ET_SECTOR);
                }
				else if (r.bits.BER)
				{
					stm32h503_flashif_erase(s, ET_BANK);
				}
				else if (r.bits.MER)
				{
					stm32h503_flashif_erase(s, ET_MASS);
				}
            }
        }
		break;
		case RI_SR:
		{
			s->regs.SR.raw &= ~(data&0xFFFF); // handle w1c bits.
		}
        break;
        default:
            qemu_log_mask(LOG_UNIMP, "H503 FINT reg 0x%x:%d write (0x%x) unimplemented\n",
         (int)addr << 2, offset, (int)data);
            s->regs.raw[addr] = data;
            break;
    }
}

static const MemoryRegionOps stm32h503_fint_ops = {
    .read = stm32h503_fint_read,
    .write = stm32h503_fint_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32h503_fint_reset(DeviceState *dev)
{
    STM32H503_STRUCT_NAME(FlashIF) *s = STM32H503_FINT(dev);
    s->flash_state = LOCKED;
    if (s->flash)
    {
        memory_region_set_readonly(s->flash, true);
    }
    for (int i=0; i<RI_END; i++)
    {
        s->regs.raw[i] = stm32_h503_flash_reginfo[i].reset_val;
    }
}

static void
stm32h503_fint_init(Object *obj)
{
    STM32H503_STRUCT_NAME(FlashIF) *s = STM32H503_FINT(obj);
    STM32_MR_IO_INIT(&s->iomem, obj, &stm32h503_fint_ops, s, 1U *KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

}

static const VMStateDescription vmstate_stm32h503_fint = {
    .name = TYPE_STM32H503_FINT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw, STM32H503_STRUCT_NAME(FlashIF),RI_END),
        VMSTATE_UINT8(flash_state, STM32H503_STRUCT_NAME(FlashIF)),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32h503_fint_properties[] = {
    DEFINE_PROP_LINK("flash", STM32H503_STRUCT_NAME(FlashIF), flash, TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32h503_fint_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32h503_fint_reset);
    dc->vmsd = &vmstate_stm32h503_fint;
	device_class_set_props(dc, stm32h503_fint_properties);
}

static const TypeInfo stm32h503_fint_info = {
    .name = TYPE_STM32H503_FINT,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32H503_STRUCT_NAME(FlashIF)),
	.instance_init = stm32h503_fint_init,
	.class_init = stm32h503_fint_class_init,
};

static void
stm32h503_fint_register_types(void)
{
    type_register_static(&stm32h503_fint_info);
}

type_init(stm32h503_fint_register_types)
