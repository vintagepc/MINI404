/*
    pinda.c - Sim IR filament sensor for
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

#define TYPE_PINDA "pinda"

OBJECT_DECLARE_SIMPLE_TYPE(PindaState, PINDA)

struct PindaState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    float mesh[4][4];
    int step_mesh[4][4];
    int current_pos[3];
    bool state;
    qemu_irq irq;

};

enum {
    ACT_SET, 
    ACT_TOGGLE,
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(PindaState, pinda, PINDA, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL})


static void pinda_finalize(Object *obj)
{
}

static void pinda_reset(DeviceState *dev)
{
    PindaState *s = PINDA(dev);
    qemu_set_irq(s->irq,0);
}

static void pinda_update(PindaState *s) {
    uint8_t x = s->current_pos[0]/(46*100*16);
    uint8_t y = s->current_pos[1]/(46*100*16);
    bool newstate = s->current_pos[2] < (s->step_mesh[x][y]);
    if (newstate != s->state) {
        qemu_set_irq(s->irq, newstate);
            printf("PINDA toggled to %u at %u, %u, %u (%u,%u @ %i)\n", newstate, s->current_pos[0],s->current_pos[1],s->current_pos[2],x,y,s->step_mesh[x][y]);
        s->state = newstate;
    }

}

static void pinda_move(void *opaque, int n, int level) 
{
    PindaState *s = PINDA(opaque);
    s->current_pos[n] = level;
    if (n==2 && level < 400*16*5) {
        pinda_update(s);
    }
}

static int pinda_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    PindaState *s = PINDA(obj);
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
    pinda_update(s);
    return ScriptLS_Finished;
}

static void pinda_init(Object *obj)
{
    PindaState *s = PINDA(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in_named(DEVICE(obj),pinda_move,"position_xyz",3);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_PINDA);

    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            s->step_mesh[i][j] = (.2F + s->mesh[i][j])*400.F*16.F;
        }
    }

    //script_register_action(pScript, "Set", "Sets the IR sensor to the given value", ACT_SET);
    //script_add_arg_bool(pScript, ACT_SET);
    //script_register_action(pScript, "Toggle",  "Toggles IR sensor state", ACT_TOGGLE);

    scripthost_register_scriptable(pScript);
}

// static const VMStateDescription vmstate_pinda = {
//     .name = TYPE_PINDA,
//     .version_id = 1,
//     .minimum_version_id = 1,
//     .fields      = (VMStateField []) {
//         VMSTATE_BOOL(state, PindaState),
//         VMSTATE_END_OF_LIST(),
//     }
// };

static void pinda_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = pinda_reset;
    //dc->vmsd = &vmstate_pinda;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = pinda_process_action;
}
