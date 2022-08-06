/*
    stm32f2xx_flashint.c - Flash I/F Configuration block for STM32

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

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


#include "stm32f2xx_flashint.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"

enum RegIndex{
    RI_ACR, 
    RI_KEYR, 
    RI_OPTKEYR, 
    RI_SR, 
    RI_CR, 
    RI_OPTCR, 
    RI_END
};

enum wp_state
{
    LOCKED, 
    KEY1_OK,
    UNLOCKED
};

static stm32_reginfo_t stm32f4xx_flashif_reginfo[RI_END] = {
    [RI_ACR] = {.mask = 0x1F07, .unimp_mask = 0x1F07},
    [RI_KEYR] = {.mask = UINT32_MAX},
    [RI_OPTKEYR] = {.unimp_mask = UINT32_MAX },
    [RI_SR]  = {.mask = 0x100F3, .unimp_mask = 0x100F3},
    [RI_CR] = {.mask = 0x8101037F, .unimp_mask = 0x1010304},
    [RI_OPTCR] = {.unimp_mask = UINT32_MAX, .reset_val =0x0FFFAAED },
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
};


QEMU_BUILD_BUG_MSG(RI_END != STM32_FINT_MAX, "Register index misaligned in " __FILE__);

static uint64_t
stm32f2xx_fint_read(void *arg, hwaddr offset, unsigned int size)
{
    STM32F4XX_STRUCT_NAME(FlashIF) *s = arg;
    uint32_t r;

    uint32_t index = offset >> 2U;
    offset&= 0x3;

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
    printf("Erasing flash sector %u\n", s->regs.defs.CR.SNB);
    uint32_t (*p)[2] = &sector_boundaries[s->regs.defs.CR.SNB];
    uint32_t buff = 0xFFFFFFFFU;
    for (int i=(*p)[0]; i<=(*p)[1]; i+=4)
    {
        cpu_physical_memory_write(s->flash->addr +i, &buff, 4);
    }
}

static void
stm32f2xx_fint_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    STM32F4XX_STRUCT_NAME(FlashIF) *s = arg;
    int offset = addr & 0x03;

    addr >>= 2;
    CHECK_BOUNDS_W(addr, data, RI_END, stm32f4xx_flashif_reginfo, "F4xx Flash IF");
    ADJUST_FOR_OFFSET_AND_SIZE_W(stm32f2xx_fint_read(arg, addr<<2U, 4U), data, size, offset, 0b111)
    CHECK_UNIMP_RESVD(data, stm32f4xx_flashif_reginfo, addr);

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
                printf("FLASH unlocked!\n");
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
                printf("FLASH LOCKED\n");
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
        default:
            qemu_log_mask(LOG_UNIMP, "f2xx FINT reg 0x%x:%d write (0x%x) unimplemented\n",
         (int)addr << 2, offset, (int)data);
            s->regs.raw[addr] = data;
            break;
    }
}

static const MemoryRegionOps stm32f2xx_fint_ops = {
    .read = stm32f2xx_fint_read,
    .write = stm32f2xx_fint_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32f2xx_fint_reset(DeviceState *dev)
{
    STM32F4XX_STRUCT_NAME(FlashIF) *s = STM32F4xx_FINT(dev);
    s->flash_state = LOCKED;
    if (s->flash)
    {
        memory_region_set_readonly(s->flash, true);
    }
    for (int i=0; i<RI_END; i++)
    {
        s->regs.raw[i] = stm32f4xx_flashif_reginfo[i].reset_val;
    }
}

static void
stm32f2xx_fint_init(Object *obj)
{
    STM32F4XX_STRUCT_NAME(FlashIF) *s = STM32F4xx_FINT(obj);
    memory_region_init_io(&s->iomem, obj, &stm32f2xx_fint_ops, s, "flash_if", 1U *KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_stm32f2xx_fint = {
    .name = TYPE_STM32F4xx_FINT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw, STM32F4XX_STRUCT_NAME(FlashIF),STM32_FINT_MAX),
        VMSTATE_UINT8(flash_state, STM32F4XX_STRUCT_NAME(FlashIF)),
        VMSTATE_END_OF_LIST()
    }
};


static void
stm32f2xx_fint_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32f2xx_fint_reset;
    dc->vmsd = &vmstate_stm32f2xx_fint;
}

static const TypeInfo stm32f2xx_fint_info = {
    .name = TYPE_STM32F4xx_FINT,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32F4XX_STRUCT_NAME(FlashIF)),
    .instance_init = stm32f2xx_fint_init,
    .class_init = stm32f2xx_fint_class_init
};

static void
stm32f2xx_fint_register_types(void)
{
    type_register_static(&stm32f2xx_fint_info);
}

type_init(stm32f2xx_fint_register_types)
