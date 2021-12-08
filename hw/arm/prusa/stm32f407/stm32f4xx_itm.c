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

#include "stm32f4xx_itm.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"

//#define DEBUG_STM32_ITM
#ifdef DEBUG_STM32_ITM
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F4XX_ITM: " fmt , ## __VA_ARGS__); \
         usleep(100); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


static uint64_t
stm32f4xx_itm_read(void *arg, hwaddr offset, unsigned int size)
{
    stm32f4xx_itm *s = arg;
    uint32_t r;

    offset >>= 2;
    r = s->regs[offset];
    if (offset == R_ITM_PORT_BASE)
        r = 1; // never return 0 since we can always take data.
    return r;
}

static void
stm32f4xx_itm_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    stm32f4xx_itm *s = arg;
    int offset = addr & 0x03;  

    addr >>= 2;
    if (addr > R_ITM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid ITM write reg 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    switch(size) {
    case 1:
       data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
       break;
    case 2:
        qemu_log_mask(LOG_UNIMP, "f4xx-ITM - writes !=32bits are not implemented.\n");
        return;
        break;
    case 4:
        break;
    default:
        abort();
    }
    s->regs[addr] = data;
    switch (addr) {
        case R_ITM_LAR:
            s->unlocked = s->regs[R_ITM_LAR] == 0xC5ACCE55;
            break;
        case R_ITM_TER:
            if (data>1)
                qemu_log_mask(LOG_UNIMP, "f4xx-ITM - ports >0 are not implemented.\n");
            break;
        case R_ITM_PORT_BASE:
            s->port_buffer[s->buffer_pos] = data;
            s->buffer_pos++;
            if (s->buffer_pos>=100 || data == '\n') // Dump to terminal if out of buffer space or newline.
            {
                if (data == '\n') // null terminate the current string so we can printf it. 
                    s->port_buffer[s->buffer_pos] = '\0';
                printf("STM32F4XX_ITM Port 0:"); //" %s \n",s->port_buffer); // ugly, but the prusa implementation is just a serial print
                for (int i=0; i<s->buffer_pos; i++)
                    printf("%c", s->port_buffer[i]);

                s->buffer_pos = 0;
                if (data != '\n') 
                    printf("\n");
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
stm32f4xx_itm_reset(DeviceState *dev)
{
    stm32f4xx_itm *s = STM32F4XX_ITM(dev);
    memset(s->port_buffer,'\0',101*sizeof(uint32_t));
    s->buffer_pos = 0;
}

static void
stm32f4xx_itm_init(Object *obj)
{
    stm32f4xx_itm *s = STM32F4XX_ITM(obj);
    memset(s->port_buffer,'\0',101*sizeof(uint32_t));
    s->buffer_pos = 0;
    memory_region_init_io(&s->iomem, obj, &stm32f4xx_itm_ops, s, "itm", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->regs[R_ITM_TCR] = 1; // actually enable it
    s->regs[R_ITM_TER] = 1;
}

static int stm32f4xx_post_load(void* opaque, int version) {
    stm32f4xx_itm *s = STM32F4XX_ITM(opaque);
    if (s->buffer_pos>= 101)
        return -EINVAL;

    return 0;
}

static const VMStateDescription vmstate_stm32f4xx_itm = {
    .name = TYPE_STM32F4XX_ITM,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = stm32f4xx_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs,stm32f4xx_itm, R_ITM_MAX),
        VMSTATE_UINT32_ARRAY(port_buffer,stm32f4xx_itm, 101),
        VMSTATE_UINT8(buffer_pos, stm32f4xx_itm),
        VMSTATE_BOOL(unlocked, stm32f4xx_itm),
        VMSTATE_END_OF_LIST()
    }
};


static void
stm32f4xx_itm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32f4xx_itm_reset;
    dc->vmsd = &vmstate_stm32f4xx_itm;
}

static const TypeInfo stm32f4xx_itm_info = {
    .name = TYPE_STM32F4XX_ITM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(stm32f4xx_itm),
    .instance_init = stm32f4xx_itm_init,
    .class_init = stm32f4xx_itm_class_init
};

static void
stm32f4xx_itm_register_types(void)
{
    type_register_static(&stm32f4xx_itm_info);
}

type_init(stm32f4xx_itm_register_types)
