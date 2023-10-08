/*
    stm32_otp.c - OTP block for STM32F

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
#include "migration/vmstate.h"
#include "stm32_common.h"
#include "qemu/log.h"
#include "../utility/macros.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "sysemu/block-backend.h"

typedef struct COM_CLASS_NAME(Otp) {
	STM32PeripheralClass parent_class;
    hwaddr otp_size;
} COM_CLASS_NAME(Otp);

// Not actual register, just to store info for the variant.
enum REGINDEX
{
	DATA_SIZE,
	RI_END
};

static const stm32_reginfo_t stm32g070_otp_reginfo[RI_END] =
{
	// Temporary- second block is engineering data region.
	[DATA_SIZE] = {.mask = 1U*KiB + 1U*KiB},
};

static const stm32_reginfo_t stm32f4xx_otp_reginfo[RI_END] =
{
	[DATA_SIZE] = {.mask = 0x220},
};

static const stm32_periph_variant_t stm32_common_otp_variants[] = {
	{TYPE_STM32G070_OTP, stm32g070_otp_reginfo},
	{TYPE_STM32F4xx_OTP, stm32f4xx_otp_reginfo}
};

#define MAX_OTP_SIZE_BYTES 0x7FF

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Otp), COM_CLASS_NAME(Otp), STM32COM_OTP);

typedef struct COM_STRUCT_NAME(Otp) {
    STM32Peripheral  parent;
    MemoryRegion  iomem;

    BlockBackend *blk;

    uint32_t data[MAX_OTP_SIZE_BYTES/sizeof(uint32_t)];

	hwaddr cfg_otp_end_bytes;

	uint32_t nr_init;
    uint32_t *init_data;

} COM_STRUCT_NAME(Otp);

static uint64_t
stm32_common_otp_read(void *arg, hwaddr addr, unsigned int size)
{
    COM_STRUCT_NAME(Otp) *s = STM32COM_OTP(arg);
    int offset = addr & 0x3;

    addr >>= 2;

    if (addr >= s->cfg_otp_end_bytes) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f4xx_otp register 0x%x\n",
          (unsigned int)addr);
        return 0;
    }

    uint32_t value = s->data[addr];
	ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 7);
    return value;
}

static void
stm32_common_otp_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    qemu_log_mask(LOG_GUEST_ERROR,"OTP: Tried to write READ ONLY region!");
}

static const MemoryRegionOps stm32_common_otp_ops = {
    .read =  stm32_common_otp_read,
    .write = stm32_common_otp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void stm32_common_otp_realize(DeviceState *dev, Error **errp)
{
    COM_STRUCT_NAME(Otp) *s = STM32COM_OTP(dev);
    s->blk = blk_by_name("stm32-otp");
    if (s->blk) {

        int64_t len = blk_getlength(s->blk);

        if (blk_set_perm(s->blk, BLK_PERM_CONSISTENT_READ,
                         BLK_PERM_ALL, &error_fatal) < 0)
        {
            error_setg(errp, "%s: Backing file incorrect permission",
                       TYPE_STM32COM_OTP);
            return;
        }

        if (blk_pread(s->blk, 0, MIN(sizeof(s->data), len), &s->data, 0) < 0) {
            error_setg(errp, "%s: failed to read backing file.",
                TYPE_STM32COM_OTP);
        }
    }
	else if (s->nr_init) // Otherwise, there were properties set
	{
		// Some defaults for Mini404.
		// TODO - abstract this out as properties or use file backend
		// if you want anything other than blank.
		for (int i=0; i< MIN(s->nr_init, sizeof(s->data)/sizeof(uint32_t)); i++)
		{
			s->data[i] = s->init_data[i];
		}
	}
	else // Not initialized, fill first 1k with FFs.
	{
		memset(s->data, 0xFF, MIN(sizeof(s->data), 1U*KiB));
	}
}

static void
stm32_common_otp_init(Object *obj)
{
    COM_STRUCT_NAME(Otp) *s = STM32COM_OTP(obj);
	COM_CLASS_NAME(Otp) *c = STM32COM_OTP_GET_CLASS(s);
	printf("# FIXME: OTP and engineering bytes should be split.\n");
    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_otp_ops, s, MAX_OTP_SIZE_BYTES);
    s->iomem.readonly = true;
	s->cfg_otp_end_bytes = c->otp_size;
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static Property stm32_common_otp_props[] = {
    DEFINE_PROP_DRIVE("drive", COM_STRUCT_NAME(Otp), blk),
	DEFINE_PROP_ARRAY("otp-data", COM_STRUCT_NAME(Otp),nr_init, init_data, qdev_prop_uint32, uint32_t),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32_common_otp = {
    .name = TYPE_STM32COM_OTP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(data, COM_STRUCT_NAME(Otp), MAX_OTP_SIZE_BYTES/sizeof(uint32_t)),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32_common_otp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = &stm32_common_otp_realize;
    dc->vmsd = &vmstate_stm32_common_otp;
    device_class_set_props(dc, stm32_common_otp_props);

	COM_CLASS_NAME(Otp) *k = STM32COM_OTP_CLASS(klass);
	stm32_reginfo_t* d = data;
	k->otp_size = d[DATA_SIZE].mask;

}

static const TypeInfo
stm32_common_otp_info = {
    .name          = TYPE_STM32COM_OTP,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Otp)),
	.class_size    = sizeof(COM_CLASS_NAME(Otp)),
	.abstract	   = true,
};

static void
stm32_common_otp_register_types(void)
{
    type_register_static(&stm32_common_otp_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_common_otp_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_common_otp_variants[i].variant_name,
            .parent     = TYPE_STM32COM_OTP,
			.instance_init = stm32_common_otp_init,
    		.class_init    = stm32_common_otp_class_init,
            .class_data = (void *)stm32_common_otp_variants[i].variant_regs,
        };
        type_register(&ti);
    }

}

type_init(stm32_common_otp_register_types)
