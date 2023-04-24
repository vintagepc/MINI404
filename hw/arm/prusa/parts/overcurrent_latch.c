/*
    oc_latch.c - Overcurrent detection part sim for
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

#define TYPE_OCLATCH "oc-latch"

OBJECT_DECLARE_SIMPLE_TYPE(OCLatchState, OCLATCH)

struct OCLatchState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    bool state;
    qemu_irq irq;

};

enum {
    ACT_RESET,
    ACT_SET,
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(OCLatchState, oc_latch, OCLATCH, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL})


static void oc_latch_finalize(Object *obj)
{
}

static void oc_latch_update(OCLatchState *s) {
    qemu_set_irq(s->irq,s->state);
}
static void oc_latch_reset(DeviceState *dev)
{
    OCLatchState *s = OCLATCH(dev);
	s->state = 0;
	oc_latch_update(s);
}

static void oc_latch_reset_in(void* opaque, int n, int level) {
	if (level==0)
	{
   		oc_latch_reset(DEVICE(opaque));
	}
}

static int oc_latch_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    OCLatchState *s = OCLATCH(obj);
    switch (action)
    {
        case ACT_RESET:
        case ACT_SET:
			s->state = action;
			oc_latch_update(s);
             break;
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;
}

static void oc_latch_init(Object *obj)
{
    OCLatchState *s = OCLATCH(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
	qdev_init_gpio_in(DEVICE(obj), oc_latch_reset_in,1);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_OCLATCH);

    script_register_action(pScript, "Set", "Sets the overcurrent line to fault", ACT_SET);
    script_register_action(pScript, "Toggle",  "Clears the overcurrent fault line", ACT_RESET);

    scripthost_register_scriptable(pScript);
}

static const VMStateDescription vmstate_oc_latch = {
    .name = TYPE_OCLATCH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_BOOL(state, OCLatchState),
        VMSTATE_END_OF_LIST(),
    }
};

static void oc_latch_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = oc_latch_reset;
    dc->vmsd = &vmstate_oc_latch;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = oc_latch_process_action;
}
