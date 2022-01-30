/*
    stm32f4xx_otp.c - OTP block for STM32F4xx

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

#include "stm32f4xx_otp.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "../utility/macros.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qapi/error.h"



static uint64_t
stm32f4xx_otp_read(void *arg, hwaddr addr, unsigned int size)
{
    Stm32f4xx_OTP *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= OTP_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f4xx_otp register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    }

    // HACK ALERT: the custom board ver (4.0.4) we set 
    // for the sim upsets the bootloader. So pretend to be 
    // HW ver 0 for the first read only.
    if (addr == 0 && !s->blk && s->first_read && size == 1)
    {
        s->first_read = false;
        return 0;
    }

    uint32_t value = s->data[addr];

    r = (value >> offset * 8) & ((1ull << (8 * size)) - 1);

    return r;
}

static void
stm32f4xx_otp_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    qemu_log_mask(LOG_GUEST_ERROR,"OTP: Tried to write READ ONLY region!");
}

static const MemoryRegionOps stm32f4xx_otp_ops = {
    .read = stm32f4xx_otp_read,
    .write = stm32f4xx_otp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(Stm32f4xx_OTP, stm32f4xx_otp, STM32F4XX_OTP, SYS_BUS_DEVICE, {NULL});

static void stm32f4xx_otp_finalize(Object *obj) {

}

static void stm32f4xx_otp_realize(DeviceState *dev, Error **errp)
{
    Stm32f4xx_OTP *s = STM32F4XX_OTP(dev);
    s->blk = blk_by_name("stm32-otp");
    if (s->blk) {
        
        int64_t len = blk_getlength(s->blk);

        // Note - some OSes do not allow files under 1k, so as long as the source is larger it's fine. 
        if (len <= sizeof(s->data)) {
            error_setg(errp, "%s: Backing file size %" PRId64 " != %" PRIu64,
                       TYPE_STM32F4XX_OTP, len, sizeof(s->data));
            return;
        }

        if (blk_set_perm(s->blk, BLK_PERM_CONSISTENT_READ,
                         BLK_PERM_ALL, &error_fatal) < 0)
        {
            error_setg(errp, "%s: Backing file incorrect permission",
                       TYPE_STM32F4XX_OTP);
            return;
        }
        len = blk_pread(s->blk, 0, &s->data, sizeof(s->data));

        if (len != sizeof(s->data)) {
            error_setg(errp, "%s: failed to read backing file!",
                TYPE_STM32F4XX_OTP);;
        }
    }
}

static void
stm32f4xx_otp_init(Object *obj)
{
    Stm32f4xx_OTP *s = STM32F4XX_OTP(obj);
    memory_region_init_io(&s->iomem, obj, &stm32f4xx_otp_ops, s, "otp", OTP_SIZE * 4u);
    s->iomem.readonly = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->first_read = true;

    // Some defaults for Mini404. 
    // TODO - abstract this out as properties or use file backend
    // if you want anything other than blank. 
    s->data[0] = 0x00040004;
    s->data[1] = 1081065844;
    s->data[2] = 0x56207942;
    s->data[3] = 0x61746e69;
    s->data[4] = 0x43506567;
    s->data[5] = 0x00000000;
    s->data[6] = 0x04040000;
    s->data[7] = 0x04040404;

}

static void
stm32f4xx_otp_reset(DeviceState *dev)
{
    Stm32f4xx_OTP *s = STM32F4XX_OTP(dev);
    s->first_read = true;
}


static Property stm32f4xx_otp_props[] = {
    DEFINE_PROP_DRIVE("drive", Stm32f4xx_OTP, blk),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32f4xx_otp = {
    .name = TYPE_STM32F4XX_OTP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(data,Stm32f4xx_OTP, OTP_SIZE),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32f4xx_otp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = &stm32f4xx_otp_realize;
    dc->reset = &stm32f4xx_otp_reset;
    dc->vmsd = &vmstate_stm32f4xx_otp;
    device_class_set_props(dc, stm32f4xx_otp_props);

}
