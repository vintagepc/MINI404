/*
    stm32f4xx_itm.h - ITM Debug channel for STM32

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

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "chardev/char-fe.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "../stm32_common/stm32_common.h"

enum reg_index {
	RI_PORT_BASE,
	RI_TER = 0xE00/4,
	RI_TPR = 0xE40/4,
	RI_TCR = 0xE80/4,
	RI_LAR = 0xFB0/4,
	RI_END = 0xFFC/4,
};

OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XX_STRUCT_NAME(Itm), STM32F4xx_ITM)

typedef struct STM32F4XX_STRUCT_NAME(Itm) {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t regs[RI_END];
    bool unlocked; // Set when LAR is written properly

	CharBackend chr;

}STM32F4XX_STRUCT_NAME(Itm);


static uint64_t
stm32f4xx_itm_read(void *arg, hwaddr offset, unsigned int size)
{
    STM32F4XX_STRUCT_NAME(Itm) *s = STM32F4xx_ITM(arg);
    uint32_t r;
    offset >>= 2;
    r = s->regs[offset];
    if (offset == RI_PORT_BASE)
	{
        r = 1; // never return 0 since we can always take data.
	}
	ADJUST_FOR_OFFSET_AND_SIZE_R(r, size, offset, 0b111);
    return r;
}

static void
stm32f4xx_itm_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    STM32F4XX_STRUCT_NAME(Itm) *s = STM32F4xx_ITM(arg);
    int offset = addr & 0x03;

    addr >>= 2;
    if (addr > RI_END) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid ITM write reg 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs[addr], data, size, offset, 0b111);

    switch (addr) {
        case RI_LAR:
            s->unlocked = s->regs[RI_LAR] == 0xC5ACCE55;
            break;
        case RI_TER:
            if (data>1)
                qemu_log_mask(LOG_UNIMP, "f4xx-ITM - ports >0 are not implemented.\n");
            break;
        case RI_PORT_BASE:
		{
			uint8_t ch = data&0xFF;
			qemu_chr_fe_write(&s->chr, &ch, 1);
		}
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "f2xx ITM reg 0x%x:%d write (0x%x) unimplemented\n",
            (int)addr << 2, offset, (int)data);
            break;
    }
}

static const MemoryRegionOps stm32f4xx_itm_ops = {
    .read = stm32f4xx_itm_read,
    .write = stm32f4xx_itm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32f4xx_itm_init(Object *obj)
{
    STM32F4XX_STRUCT_NAME(Itm) *s = STM32F4xx_ITM(obj);
    STM32_MR_IO_INIT(&s->iomem, obj, &stm32f4xx_itm_ops, s, 4U*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->regs[RI_TCR] = 1; // actually enable it
    s->regs[RI_TER] = 1;
}

static const VMStateDescription vmstate_stm32f4xx_itm = {
    .name = TYPE_STM32F4xx_ITM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs,STM32F4XX_STRUCT_NAME(Itm), RI_END),
        VMSTATE_BOOL(unlocked, STM32F4XX_STRUCT_NAME(Itm)),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32_itm_properties[] = {
    DEFINE_PROP_CHR("chardev", STM32F4XX_STRUCT_NAME(Itm), chr),
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32f4xx_itm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32f4xx_itm;
	device_class_set_props(dc, stm32_itm_properties);
}

static const TypeInfo stm32f4xx_itm_info = {
    .name = TYPE_STM32F4xx_ITM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F4XX_STRUCT_NAME(Itm)),
    .instance_init = stm32f4xx_itm_init,
    .class_init = stm32f4xx_itm_class_init
};

static void
stm32f4xx_itm_register_types(void)
{
    type_register_static(&stm32f4xx_itm_info);
}

type_init(stm32f4xx_itm_register_types)
