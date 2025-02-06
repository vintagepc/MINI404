/*
    demux.c - a simple 2 line SPI CS BCD demux

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
#include "../utility/macros.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"

#define TYPE_GPIODEMUX "cs-demux"

OBJECT_DECLARE_SIMPLE_TYPE(DemuxState, GPIODEMUX)

struct DemuxState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    uint8_t address; // Current selected channel.
    qemu_irq cs_out[4]; // CS output propagation
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(DemuxState, demux, GPIODEMUX, SYS_BUS_DEVICE, {NULL})

static void demux_finalize(Object *obj)
{
}

static void demux_reset(DeviceState *dev)
{
    DemuxState *s = GPIODEMUX(dev);
    s->address = 0;
    for (int i=0; i<4; i++)
    {
        qemu_irq_lower(s->cs_out[i]);
    }
}

static void demux_select(void *opaque, int n, int level){
    DemuxState *s = GPIODEMUX(opaque);
    // Clear CS state on current output
    qemu_irq_lower(s->cs_out[s->address]);
    if (n==0)
    {
        s->address = (s->address & 0x02) | (level ? 0x1 : 0);
    }
    else
    {
        s->address = (s->address & 0x01) | (level ? 0x2 : 0);
    }
    qemu_irq_raise(s->cs_out[s->address]);
}

static void demux_init(Object *obj)
{
    DemuxState *s = GPIODEMUX(obj);
    qdev_init_gpio_out(DEVICE(obj), s->cs_out, 4);
    qdev_init_gpio_in(DEVICE(obj), demux_select, 2);
}

static const VMStateDescription vmstate_demux = {
    .name = TYPE_GPIODEMUX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(address,DemuxState), // Current selected channel.
        VMSTATE_END_OF_LIST()
    }
};

static void demux_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = demux_reset;
    dc->vmsd = &vmstate_demux;
}
