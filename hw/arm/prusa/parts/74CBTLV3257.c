/*
    74CBTLV.c - Sim demux for Mini404.

	Copyright 2022 VintagePC <https://github.com/vintagepc/>

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

#define TYPE_CBTLV3257 "cbtl3257"

OBJECT_DECLARE_SIMPLE_TYPE(CBTLV3257State, CBTLV3257)

struct CBTLV3257State {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
	bool select;
	bool oe;
	int levels[2][4];
    qemu_irq irq[4];

};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(CBTLV3257State, cbtl3257, CBTLV3257, SYS_BUS_DEVICE, {NULL})

static void cbtl3257_finalize(Object *obj)
{
}

static void cbtl3257_reset(DeviceState *dev)
{
    CBTLV3257State *s = CBTLV3257(dev);
	s->oe = true; // default to 1...
}

static void cbtl3257_ch1_in(void *opaque, int n, int level){
    CBTLV3257State *s = CBTLV3257(opaque);
	s->levels[0][n] = level;
    if (s->oe && !s->select)
	{
		qemu_set_irq(s->irq[n],level);
	}
}

static void cbtl3257_ch2_in(void *opaque, int n, int level){
    CBTLV3257State *s = CBTLV3257(opaque);
	s->levels[1][n] = level;
    if (s->oe && s->select)
	{
		qemu_set_irq(s->irq[n],level);
	}
    // printf("Ch2 %d changed to %d\n",n,level);
}

static void cbtl3257_select(void *opaque, int n, int level){
    CBTLV3257State *s = CBTLV3257(opaque);
	bool old_sel = s->select;
	s->select = level;
	// Update outputs if they are changed by select.
	for (int i=0; i<4; i++)
	{
		if (s->levels[old_sel][i] != s->levels[s->select][i])
		{
			qemu_set_irq(s->irq[n],s->levels[s->select][i]);
		}
	}
}

static void cbtl3257_nOE(void *opaque, int n, int level){
    CBTLV3257State *s = CBTLV3257(opaque);
	s->oe = level==0;
	for (int i=0; i<4; i++)
	{
		if (s->oe)
		{
			qemu_set_irq(s->irq[n],s->levels[s->select][i]);
		}
		else
		{
			qemu_irq_lower(s->irq[n]);
		}
	}
}

static void cbtl3257_init(Object *obj)
{
    CBTLV3257State *s = CBTLV3257(obj);
    qdev_init_gpio_out(DEVICE(obj), s->irq, 4); // outputs.

    qdev_init_gpio_in_named(DEVICE(obj),cbtl3257_nOE,"nOE",1);
    qdev_init_gpio_in_named(DEVICE(obj),cbtl3257_select,"select",1);
    qdev_init_gpio_in_named(DEVICE(obj),cbtl3257_ch1_in,"B1",4);// 1Y
    qdev_init_gpio_in_named(DEVICE(obj),cbtl3257_ch2_in,"B2",4);// 2Y

}

static const VMStateDescription vmstate_cbtl3257 = {
    .name = TYPE_CBTLV3257,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(oe,CBTLV3257State),
        VMSTATE_BOOL(select,CBTLV3257State),
		VMSTATE_INT32_2DARRAY(levels, CBTLV3257State, 2,4),
        VMSTATE_END_OF_LIST()
    }
};

static void cbtl3257_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = cbtl3257_reset;
    dc->vmsd = &vmstate_cbtl3257;
}
