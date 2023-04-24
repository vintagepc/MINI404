/*
    corexy_helper.c - Helper for A/B -> X/Y translation w/ steppers

	Copyright 2023 VintagePC <https://github.com/vintagepc/>

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
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "../utility/p404_motor_if.h"
#include "hw/sysbus.h"

#define TYPE_COREXY "corexy-helper"

OBJECT_DECLARE_SIMPLE_TYPE(CoreXYState, COREXY)

struct CoreXYState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    int32_t pos_a_um;
	int32_t pos_b_um;

    uint32_t x_max_um;
	uint32_t y_max_um;

	bool irq_state;
    qemu_irq endstop[2];

	p404_motorif_status_t vis_x;
	p404_motorif_status_t vis_y;
	p404_motorif_status_t* vis;

};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(CoreXYState, corexy, COREXY, SYS_BUS_DEVICE, {TYPE_P404_MOTOR_IF}, {NULL});

static void corexy_finalize(Object *obj)
{
}

static void corexy_reset(DeviceState *dev)
{
    CoreXYState *s = COREXY(dev);
    qemu_set_irq(s->endstop[0],0);
    qemu_set_irq(s->endstop[1],0);
}

static void corexy_move(void *opaque, int n, int level)
{
    CoreXYState *s = COREXY(opaque);
    if (n == 0)
	{
		s->pos_a_um = level;
	}
	else
	{
		s->pos_b_um = level;
	}
	int32_t xpos = (s->pos_a_um + s->pos_b_um)/2.f;
	int32_t ypos = (s->pos_a_um - s->pos_b_um)/2.f;
	s->vis_x.current_pos = (float)xpos/1000.f;
	s->vis_y.current_pos = (float)ypos/1000.f;
	s->vis_x.status.stalled = xpos > s->x_max_um || xpos < 0;
	s->vis_y.status.stalled = ypos < 0 || ypos > s->y_max_um;
	bool hit = ( s->vis_x.status.stalled || s->vis_y.status.stalled );

	if (hit ^ s->irq_state || hit)
	{
		// printf("Stall flag %u at pos %d %d\n", hit, xpos, ypos);
		qemu_set_irq(s->endstop[0], hit);
		qemu_set_irq(s->endstop[1], hit);
		s->irq_state = hit;
	}
	s->vis_x.status.changed = true;
	s->vis_y.status.changed = true;
}

static const p404_motorif_status_t* corexy_get_status(P404MotorIF* p)
{
    CoreXYState *s = COREXY(p);
	if (s->vis == & s->vis_x)
	{
		s->vis = &s->vis_y;
	}
	else
	{
		s->vis = &s->vis_x;
	}
    return s->vis;
}

static void corexy_init(Object *obj)
{
    CoreXYState *s = COREXY(obj);
	s->x_max_um = 365*1000;
	s->y_max_um = 365*1000;
	s->vis_x.max_pos = 365;
	s->vis_x.label = 'X';
	s->vis_x.status.enabled = true;
	s->vis_x.status.changed = true;
	s->vis_y.max_pos = 365;
	s->vis_y.label = 'Y';
	s->vis_y.status.enabled = true;
	s->vis_y.status.changed = true;
    qdev_init_gpio_out(DEVICE(obj), s->endstop, 2);
    qdev_init_gpio_in(DEVICE(obj),corexy_move,2);
}

static const VMStateDescription vmstate_corexy = {
    .name = TYPE_COREXY,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_INT32(pos_a_um,CoreXYState),
        VMSTATE_INT32(pos_b_um,CoreXYState),
        VMSTATE_BOOL(irq_state, CoreXYState),
        VMSTATE_END_OF_LIST(),
    }
};

static void corexy_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = corexy_reset;
    dc->vmsd = &vmstate_corexy;

	P404MotorIFClass *mc = P404_MOTOR_IF_CLASS(oc);
    mc->get_current_status = corexy_get_status;
}
