/*
    74CH4052.c - Sim demux for Mini404.

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
#include "qemu/timer.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"

#define TYPE_HC4052 "hc4052"

OBJECT_DECLARE_SIMPLE_TYPE(HC4052State, HC4052)

struct HC4052State {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    uint8_t channel; // Current selected channel.
    uint8_t start_channel; // starting selected channel.

    int levels[2][4]; //first set of inputs.

	// Read requests for downstream devices
	qemu_irq mux_read[2][4];

    qemu_irq irq[2];

};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(HC4052State, hc4052, HC4052, SYS_BUS_DEVICE, {NULL})

static void hc4052_finalize(Object *obj)
{
}

static void hc4052_reset(DeviceState *dev)
{
    HC4052State *s = HC4052(dev);
    s->channel = s->start_channel;
    qemu_set_irq(s->irq[0],0);
    qemu_set_irq(s->irq[1],0);

}

static void hc4052_ch1_in(void *opaque, int n, int level){
    HC4052State *s = HC4052(opaque);
    s->levels[0][n] = level;
    // printf("Ch1 %d changed to %d\n",n,level);
}

static void hc4052_ch2_in(void *opaque, int n, int level){
    HC4052State *s = HC4052(opaque);
    s->levels[1][n] = level;
    // printf("Ch2 %d changed to %d\n",n,level);
}

static void hc4052_ch1_read_request(void *opaque, int n, int level)
{
    HC4052State *s = HC4052(opaque);
	qemu_irq_raise(s->mux_read[n][s->channel]);
    qemu_set_irq(s->irq[n], s->levels[n][s->channel]);
}

static void hc4052_select(void *opaque, int n, int level){
    HC4052State *s = HC4052(opaque);
    if (n==0) {
        s->channel = (s->channel& 0x02) | (level & 0x1);
    } else {
        s->channel = (s->channel& 0x01) | ((level & 0x1)<<1);
    }
}

static void hc4052_init(Object *obj)
{
    HC4052State *s = HC4052(obj);
    qdev_init_gpio_out(DEVICE(obj), s->irq, 2); // 1Z, 2Z
    qdev_init_gpio_out_named(DEVICE(obj), s->mux_read[0],"1Y_read", 4);
    qdev_init_gpio_out_named(DEVICE(obj), s->mux_read[1],"2Y_read", 4);
    qdev_init_gpio_in_named(DEVICE(obj),hc4052_select,"select",2);// S1, S2
    qdev_init_gpio_in_named(DEVICE(obj),hc4052_ch1_in,"1Y",4);// 1Y
    qdev_init_gpio_in_named(DEVICE(obj),hc4052_ch2_in,"2Y",4);// 2Y
    qdev_init_gpio_in_named(DEVICE(obj),hc4052_ch1_read_request,"adc_read_request",2);

}

static const VMStateDescription vmstate_hc4052 = {
    .name = TYPE_HC4052,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(channel,HC4052State), // Current selected channel.
        VMSTATE_INT32_2DARRAY(levels, HC4052State, 2,4),
        VMSTATE_END_OF_LIST()
    }
};

static Property hc4052_properties[] = {
    DEFINE_PROP_UINT8("start_channel", HC4052State, start_channel, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void hc4052_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = hc4052_reset;
    dc->vmsd = &vmstate_hc4052;
	device_class_set_props(dc, hc4052_properties);
}
