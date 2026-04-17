/*
    p404_key_inpuit.c - Keyboard receiver and dispatch for Mini404
    Mini404.

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
#include "ui/console.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_P404_KEY_INPUT "p404-key-input"

OBJECT_DECLARE_SIMPLE_TYPE(P404KeyState, P404_KEY_INPUT)

struct P404KeyState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
};

extern void p404_keyctl_handle_key(int keycode, bool bDown);

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(P404KeyState, p404_key, P404_KEY_INPUT, SYS_BUS_DEVICE, {NULL})


static void p404_key_input_keyevent(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    InputKeyEvent *key = evt->u.key.data;
    int qcode = qemu_input_key_value_to_qcode(key->key);
    p404_keyctl_handle_key(qcode, key->down);
}

static void p404_key_finalize(Object *obj)
{
}

static const QemuInputHandler p404_keyboard_handler = {
    .name  = "MINI404 Key",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = p404_key_input_keyevent,
};

static void p404_key_init(Object *obj)
{
    P404KeyState *s = P404_KEY_INPUT(obj);
    qemu_input_handler_register(DEVICE(s), &p404_keyboard_handler);
}

static void p404_key_class_init(ObjectClass *oc, void *data)
{
}
