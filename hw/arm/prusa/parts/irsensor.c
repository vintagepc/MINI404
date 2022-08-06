/*
    irsensor.c - Sim IR filament sensor for
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
#include "../utility/p404scriptable.h"
#include "../utility/macros.h"
#include "../utility/ScriptHost_C.h"
#include "../utility/ArgHelper.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_IRSENSOR "ir-sensor"

OBJECT_DECLARE_SIMPLE_TYPE(IRState, IRSENSOR)

struct IRState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    bool state;
    qemu_irq irq;
    script_handle handle;
};

enum {
    ACT_SET, 
    ACT_TOGGLE,
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(IRState, irsensor, IRSENSOR, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL})


static void irsensor_finalize(Object *obj)
{
}

static void irsensor_reset(DeviceState *dev)
{
    IRState *s = IRSENSOR(dev);
    qemu_set_irq(s->irq,0);
}

static void irsensor_update(IRState *s) {
    qemu_set_irq(s->irq,s->state);
}

static int irsensor_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    IRState *s = IRSENSOR(obj);
    switch (action)
    {
        case ACT_TOGGLE:
        {
            s->state ^=1;
            break;
        }
        case ACT_SET:
            s->state = scripthost_get_bool(args, 0);
            break;
        default:
            return ScriptLS_Unhandled;
    }
    irsensor_update(s);
    return ScriptLS_Finished;
}

static void irsensor_init(Object *obj)
{
    IRState *s = IRSENSOR(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);

    s->handle = script_instance_new(P404_SCRIPTABLE(obj), TYPE_IRSENSOR);

    script_register_action(s->handle, "Set", "Sets the IR sensor to the given value", ACT_SET);
    script_add_arg_bool(s->handle, ACT_SET);
    script_register_action(s->handle, "Toggle",  "Toggles IR sensor state", ACT_TOGGLE);

    scripthost_register_scriptable(s->handle);
}

static const VMStateDescription vmstate_irsensor = {
    .name = TYPE_IRSENSOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_BOOL(state, IRState),
        VMSTATE_END_OF_LIST(),
    }
};

static void irsensor_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = irsensor_reset;
    dc->vmsd = &vmstate_irsensor;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = irsensor_process_action;
}
