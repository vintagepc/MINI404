/*
    dwarf_button.c - GPIO keyboard dwarf_button for Mini404

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
#include "../utility/p404scriptable.h"
#include "../utility/p404_keyclient.h"
#include "../utility/macros.h"
#include "../utility/ScriptHost_C.h"
#include "../utility/ArgHelper.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_DWARF_INPUT "dwarf-input"

OBJECT_DECLARE_SIMPLE_TYPE(DwarfInputState, DWARF_INPUT)

struct DwarfInputState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    qemu_irq irq[2];

    script_handle handle;
};

enum {
    ACT_PUSH,
    ACT_RELEASE,
};

static void dwarf_button_handle_key(P404KeyIF *opaque, Key keycode)
{
	DwarfInputState *s = DWARF_INPUT(opaque);
	bool release = keycode & P404_KEYCLIENT_RELEASE_MASK;
    switch (keycode & ~ P404_KEYCLIENT_RELEASE_MASK)
    {
        case 's': // down
			qemu_set_irq(s->irq[1], release);
            break;
        case 'w': // up
			qemu_set_irq(s->irq[0], release);
            break;
    }
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(DwarfInputState, dwarf_button, DWARF_INPUT, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {TYPE_P404_KEYCLIENT}, {NULL})

static void dwarf_button_finalize(Object *obj)
{
}

static void dwarf_button_reset(DeviceState *dev)
{
    DwarfInputState *s = DWARF_INPUT(dev);
    qemu_irq_raise(s->irq[0]);
    qemu_irq_raise(s->irq[1]);
}

static int dwarf_button_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    switch (action)
    {
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;
}

static void dwarf_button_init(Object *obj)
{
    DwarfInputState *s = DWARF_INPUT(obj);
    qdev_init_gpio_out(DEVICE(obj), s->irq, 2);

    p404_key_handle pKey = p404_new_keyhandler(P404_KEYCLIENT(obj));
    p404_register_keyhandler(pKey, 'w',"Pushes top button");
    p404_register_keyhandler(pKey, 's',"Pushes bottom button");
	p404_register_keyhandler(pKey, 'w' | P404_KEYCLIENT_RELEASE_MASK, "");
    p404_register_keyhandler(pKey, 's' | P404_KEYCLIENT_RELEASE_MASK, "");
}

static void dwarf_button_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = dwarf_button_reset;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = dwarf_button_process_action;

    P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(oc);
    kc->KeyHandler = dwarf_button_handle_key;
}
