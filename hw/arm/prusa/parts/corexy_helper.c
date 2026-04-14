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
#include "../utility/p404_keyclient.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"

#define TYPE_COREXY "corexy-helper"

OBJECT_DECLARE_SIMPLE_TYPE(CoreXYState, COREXY)

typedef struct dock_pos
{
    unsigned char key;
    int32_t x;
    int32_t y;
} dock_pos;

// Coordinates need to be "offset" by the amount the axes are allowed
// to move in the negative direction.
#define Y_NEG_SIZE 10
#define X_NEG_SIZE 8

// Per comments in code, x is ~25+*n*82, y is ~451
static const dock_pos dock_positions[] = {
    {'z', (X_NEG_SIZE + 25+(0*82)),    455 + Y_NEG_SIZE},
    {'x', (X_NEG_SIZE + 25+(1*82)),    455 + Y_NEG_SIZE},
    {'c', (X_NEG_SIZE + 25+(2*82)),    455 + Y_NEG_SIZE},
    {'v', (X_NEG_SIZE + 25+(3*82)),    455 + Y_NEG_SIZE},
    {'b', (X_NEG_SIZE + 25+(4*82)),    455 + Y_NEG_SIZE}
};

struct CoreXYState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    int32_t pos_a_um;
	int32_t pos_b_um;

    uint32_t x_max_um;
	uint32_t y_max_um;

    bool swap_calc;

	bool irq_state;
    qemu_irq endstop[2];
    qemu_irq pos_change[2];
    qemu_irq um_out[2];
    qemu_irq tool_pick_ctl[ARRAY_SIZE(dock_positions)];
    qemu_irq cal_pin;
	p404_motorif_status_t vis_x;
	p404_motorif_status_t vis_y;
	p404_motorif_status_t* vis;

    bool cal_pin_present;

    bool pos_set;

    uint8_t tool_states[ARRAY_SIZE(dock_positions)];

};

#define MAG_SENSE_DISTANCE_MM 1

// These values should be consistent with the 
// tool mag sensor states, i.e. docked = p0 set, picked = p1 set, in-a-change = both set.
enum TOOL_STATES {
    TOOL_LOST = 0,
    TOOL_PARKED = 1,
    TOOL_PICKED = 2,
    TOOL_CHANGING = 3,
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(CoreXYState, corexy, COREXY, SYS_BUS_DEVICE, {TYPE_P404_MOTOR_IF}, {TYPE_P404_KEYCLIENT}, {NULL});

static void corexy_finalize(Object *obj)
{
}

static void corexy_reset(DeviceState *dev)
{
    CoreXYState *s = COREXY(dev);
    qemu_set_irq(s->endstop[0],0);
    qemu_set_irq(s->endstop[1],0);
    s->cal_pin_present = false;
    if (!s->pos_set)
    {
        // Start with the head in the middle of the print area.
        s->vis_x.current_pos = s->vis_x.max_pos/2;
        s->vis_y.current_pos = s->vis_y.max_pos/2;
        float pos_a = (s->vis_x.current_pos + s->vis_y.current_pos);
        float pos_b = (s->vis_x.current_pos - s->vis_y.current_pos);
        qemu_set_irq(s->pos_change[0], pos_a*1000);
        qemu_set_irq(s->pos_change[1], pos_b*1000);
        s->pos_set = true;
    }
}

static void update_tool_states(CoreXYState *s)
{
    // No picking/parking can happen if the tool doesn't travel far enough to the rear.
    if (s->vis_y.current_pos < 445)
    {
        return;
    }

    // First, check if the tool head is within 1mm of a parked tool position, and if so, it means both the dock 
    // and carriage sensors must be active.
    for(int i = 0; i < ARRAY_SIZE(dock_positions); i++)
    {
        bool x_in_range = abs(s->vis_x.current_pos - dock_positions[i].x) <= MAG_SENSE_DISTANCE_MM;
        bool y_in_range = abs(s->vis_y.current_pos - dock_positions[i].y) <= MAG_SENSE_DISTANCE_MM;
        if (y_in_range && x_in_range)
        {
            if (s->tool_states[i] != TOOL_CHANGING)
            {
                for (int j = 0; j < ARRAY_SIZE(dock_positions); j++)
                {
                    // all other tools must be parked...
                    s->tool_states[j] = j == i ? TOOL_CHANGING : TOOL_PARKED;
                    qemu_set_irq(s->tool_pick_ctl[j], s->tool_states[j]);
                    printf("Tool %u state: %u\n", j, s->tool_states[j]);
                }
            }
        }
        // If the tool is in the changing state (picked and parked and we have moved out of the "sense zone",
        // decide new state based on whether:
        // 1. X has remained the same: carriage moved in -Y and the tool has been unloaded.
        // 2. Y has remained steady: carriage has moved in -X and tool has been picked up
        // This is stupidly simple, but it should work and results in simpler logic
        // than trying to interpret the direction the head is moving.
        else if (s->tool_states[i] == TOOL_CHANGING)
        {
            if (x_in_range)
            {
                printf("Tool %u state: parked\n", i);
                s->tool_states[i] = TOOL_PARKED;
                qemu_set_irq(s->tool_pick_ctl[i], TOOL_PARKED);
            }
            else if (y_in_range)
            {
                printf("Tool %u state: picked\n", i);
                s->tool_states[i] = TOOL_PICKED;
                qemu_set_irq(s->tool_pick_ctl[i], TOOL_PICKED);
            }
        }
    }
    
}

static void corexy_move(void *opaque, int n, int level)
{
    CoreXYState *s = COREXY(opaque);
    if (n == 0)
	{
		s->pos_a_um = level;
	}
	else if (n == 1)
	{
		s->pos_b_um = level;
	}
	int32_t xpos = (s->pos_a_um + s->pos_b_um)/2.f;
	int32_t ypos = (s->pos_a_um - s->pos_b_um)/2.f;
    if (s->swap_calc)
    {
        int32_t tmp = xpos;
        xpos = ypos;
        ypos = tmp;
    }

    qemu_set_irq(s->um_out[0], xpos);
    qemu_set_irq(s->um_out[1], ypos);

    float newx = ((float)xpos/1000.f);    
    float newy = ((float)ypos/1000.f);

    float delta_x = newx - s->vis_x.current_pos;
    float delta_y = newy - s->vis_y.current_pos;

	s->vis_x.current_pos = newx;
	s->vis_y.current_pos = newy;

    update_tool_states(s);

	s->vis_x.status.stalled = (xpos > s->x_max_um && delta_x > 0) || 
                                (xpos < 0 && delta_x < 0);
    s->vis_y.status.stalled = (ypos > s->y_max_um && delta_y > 0) ||
                                (ypos < 0 && delta_y < 0);

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

static void corexy_handle_key(P404KeyIF *opaque, Key keycode)
{
    CoreXYState *s = COREXY(opaque);

    if (keycode == 'm')
    {
        s->cal_pin_present = !s->cal_pin_present;
        qemu_set_irq(s->cal_pin, s->cal_pin_present);
        printf("Cal pin %s\n", s->cal_pin_present? "Installed":"Removed");
        return;
    }

    for (int i = 0; i < ARRAY_SIZE(dock_positions); i++)
    {
        if (dock_positions[i].key == keycode)
        {
            printf("Key %c - dock %u\n", keycode, i);
            // Need to reverse the X/Y calculations to determine A/B absolute positions.
            float pos_a = (dock_positions[i].x + dock_positions[i].y);
            float pos_b = (dock_positions[i].x - dock_positions[i].y);
            if (s->swap_calc)
            {
                float tmp = pos_a;
                pos_a = pos_b;
                pos_b = tmp;
            }

            qemu_set_irq(s->pos_change[0], pos_a*1000);
            qemu_set_irq(s->pos_change[1], pos_b*1000);
            break;
        }
    }
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

static void corexy_realize(DeviceState *dev, Error **errp)
{
    CoreXYState *s = COREXY(dev);
    for (int i = 0; i < ARRAY_SIZE(dock_positions); i++)
    {
        s->tool_states[i] = TOOL_PARKED;
        qemu_set_irq(s->tool_pick_ctl[i], TOOL_PARKED);
    }
	s->vis_x.max_pos = s->x_max_um/(1000U);
	s->vis_x.status.changed = true;
	s->vis_y.max_pos = s->y_max_um/(1000U);
	s->vis_y.status.changed = true;
    s->pos_set = false;
}

static void corexy_init(Object *obj)
{
    CoreXYState *s = COREXY(obj);
	s->vis_x.max_pos = s->x_max_um/(1000U);
	s->vis_x.label = 'X';
	s->vis_x.status.enabled = true;
	s->vis_x.status.changed = true;
	s->vis_y.max_pos = s->y_max_um/(1000U);
	s->vis_y.label = 'Y';
	s->vis_y.status.enabled = true;
	s->vis_y.status.changed = true;
    qdev_init_gpio_out(DEVICE(obj), s->endstop, 2);
    qdev_init_gpio_out_named(DEVICE(obj), s->pos_change, "motor-move", 2);
    qdev_init_gpio_out_named(DEVICE(obj), s->um_out, "um-out", 2);
    qdev_init_gpio_out_named(DEVICE(obj), &s->cal_pin, "cal-pin", 1);
    qdev_init_gpio_out_named(DEVICE(obj), s->tool_pick_ctl, "tool-pick", ARRAY_SIZE(dock_positions));
    qdev_init_gpio_in(DEVICE(obj),corexy_move,2);

    p404_key_handle pKey = p404_new_keyhandler(P404_KEYCLIENT(obj));
    p404_register_keyhandler(pKey, 'z',"Places head at dock 1");
    p404_register_keyhandler(pKey, 'x',"Places head at dock 2");
    p404_register_keyhandler(pKey, 'c',"Places head at dock 3");
    p404_register_keyhandler(pKey, 'v',"Places head at dock 4");
    p404_register_keyhandler(pKey, 'b',"Places head at dock 5");
    p404_register_keyhandler(pKey, 'm',"Installs/removes the calibration pin");

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

static Property corexy_properties[] = {
    DEFINE_PROP_UINT32("x-max-um", CoreXYState, x_max_um, 375*1000),
    DEFINE_PROP_UINT32("y-max-um", CoreXYState, y_max_um, 466*1000),
    DEFINE_PROP_BOOL("swap-calc", CoreXYState, swap_calc, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void corexy_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, corexy_reset);
    dc->realize = corexy_realize;
    dc->vmsd = &vmstate_corexy;
    device_class_set_props(dc, corexy_properties);

	P404MotorIFClass *mc = P404_MOTOR_IF_CLASS(oc);
    mc->get_current_status = corexy_get_status;

    P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(oc);
    kc->KeyHandler = corexy_handle_key;
}
