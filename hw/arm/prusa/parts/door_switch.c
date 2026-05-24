/*
    door_input.c - GPIO keyboard door_input for Mini404

	Copyright 2024 VintagePC <https://github.com/vintagepc/>

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
#include "../utility/p404scriptable.h"
#include "../utility/p404_keyclient.h"
#include "../utility/macros.h"
#include "../utility/ScriptHost_C.h"
#include "../utility/ArgHelper.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "qom/object.h"
#include "hw/core/sysbus.h"
#include "trace.h"

#define TYPE_DOOR_INPUT "door-switch"

OBJECT_DECLARE_SIMPLE_TYPE(DoorInputState, DOOR_INPUT)

struct DoorInputState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    qemu_irq irq;
    bool is_open;
    script_handle handle;
};

enum {
    ACT_OPEN,
    ACT_CLOSE,
    ACT_DISCONNECT,
};

static void door_input_handle_key(P404KeyIF *opaque, Key keycode)
{
	DoorInputState *s = DOOR_INPUT(opaque);
    switch (keycode & ~ P404_KEYCLIENT_RELEASE_MASK)
    {
        case 'd': // down
            s->is_open ^= 1;
            trace_door_switch_state(s->is_open);
			qemu_set_irq(s->irq, s->is_open ? 2000 : 20);
            break;
    }
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(DoorInputState, door_input, DOOR_INPUT, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {TYPE_P404_KEYCLIENT}, {NULL})

static void door_input_finalize(Object *obj)
{
}

static void door_input_reset(DeviceState *dev)
{
    DoorInputState *s = DOOR_INPUT(dev);
    qemu_set_irq(s->irq, 20); // Init to connected, closed
    s->is_open = false;
}

static int door_input_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    switch (action)
    {
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;
}

static void door_input_init(Object *obj)
{
    DoorInputState *s = DOOR_INPUT(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);

    p404_key_handle pKey = p404_new_keyhandler(P404_KEYCLIENT(obj));
    p404_register_keyhandler(pKey, 'd',"Toggles door state");
    // TODO: add scripting open/close/disconnect.
}

static void door_input_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, door_input_reset);
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = door_input_process_action;

    P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(oc);
    kc->KeyHandler = door_input_handle_key;
}
