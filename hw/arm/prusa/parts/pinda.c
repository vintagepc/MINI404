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
    float mesh_mm[4][4];
    uint32_t step_mesh[4][4];
    int32_t current_pos[3];
    bool state;
    bool first_fired;
    qemu_irq irq;
    script_handle handle;
};

enum {
    ACT_SET_MBL, 

};

#define Z_MM_TO_STEPS 400.F*16.F
#define XY_TO_STEPS 100.F*16.F

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(PindaState, pinda, PINDA, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL})


static void pinda_finalize(Object *obj)
{
}

static void pinda_reset(DeviceState *dev)
{
    PindaState *s = PINDA(dev);
    qemu_set_irq(s->irq,0);
    s->state = false;
}

static void pinda_update(PindaState *s) {
    uint8_t x = s->current_pos[0]/(46*XY_TO_STEPS);
    uint8_t y = s->current_pos[1]/(46*XY_TO_STEPS);
    bool newstate = s->current_pos[2] <= (s->step_mesh[y][x]);
    // if (newstate) printf("PINDA update at %u, %u, %u\n", s->current_pos[0],s->current_pos[1],s->current_pos[2]);

    if (newstate != s->state || !s->first_fired) {
        qemu_set_irq(s->irq, newstate);
        // if (newstate) printf("PINDA toggled to %u at %u, %u, %u (%u,%u @ %i)\n", newstate, s->current_pos[0],s->current_pos[1],s->current_pos[2],x,y,s->step_mesh[x][y]);
        s->state = newstate;
        s->first_fired = true;
    }

}

static void pinda_move(void *opaque, int n, int level) 
{
    PindaState *s = PINDA(opaque);
    s->current_pos[n] = level;
    if (n==2){
        pinda_update(s);
    }
}

static void pinda_rebuild_mesh(PindaState *s) {
    printf("MBL Grid: \n");
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            s->step_mesh[i][j] = (s->mesh_mm[i][j])*Z_MM_TO_STEPS;
            printf("%f ",s->mesh_mm[i][j]);
        }
        printf("\n");
    }
}


static int pinda_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    PindaState *s = PINDA(obj);
    switch (action)
    {
        case ACT_SET_MBL:
        {
            int iX = scripthost_get_int(args, 0);
            int iY = scripthost_get_int(args, 1);
            float fZ = scripthost_get_float(args, 2);
            if (iX<0 || iY <0 || iX>3 || iY>3 || fZ<-0.99) {
                fprintf(stderr, "Scripting error - MBL argument out of range! (index 0-4, z > -1) \n");
                return ScriptLS_Error; // Bad input. 
            } else {
                s->mesh_mm[iY][iX] = 1.F + fZ;
                pinda_rebuild_mesh(s);
            }

            break;
        }
        default:
            return ScriptLS_Unhandled;
    }
    pinda_update(s);
    return ScriptLS_Finished;
}


// Note: Grid gets a +1.0 so that we don't hit Z stallguard before we actually trigger the pinda.
// You can edit this to an initial grid if you want - values should NOT be <=-1 for the above reason.
static const float pinda_start_grid[4][4] =  {
    {0, 0, 0, 0 },
    {0, 0, 0, 0 },
    {0, 0, 0, 0 },
    {0, 0, 0, 0 },
};



static void pinda_init(Object *obj)
{
    PindaState *s = PINDA(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in_named(DEVICE(obj),pinda_move,"position_xyz",3);

    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            s->mesh_mm[i][j] = 1.F + pinda_start_grid[i][j];//0.5F * (i);
        }
    }
    pinda_rebuild_mesh(s);

    s->handle = script_instance_new(P404_SCRIPTABLE(obj), TYPE_PINDA);


    script_register_action(s->handle, "SetMBL", "Sets the given block of the 4x4 z-trigger grid to the specified value. (x,y,mm)", ACT_SET_MBL);
    script_add_arg_int(s->handle,ACT_SET_MBL);
    script_add_arg_int(s->handle,ACT_SET_MBL);
    script_add_arg_bool(s->handle,ACT_SET_MBL);
    scripthost_register_scriptable(s->handle);
}

static int pinda_post_load(void *opaque, int version_id)
{
    PindaState *s = PINDA(opaque);
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            s->mesh_mm[i][j] = (float)s->step_mesh[i][j]/Z_MM_TO_STEPS;
        }
    }

    return 0;
}

static const VMStateDescription vmstate_pinda = {
    .name = TYPE_PINDA,
    .version_id = 1,
    .post_load = pinda_post_load,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_UINT32_2DARRAY(step_mesh,PindaState, 4,4),
        VMSTATE_INT32_ARRAY(current_pos, PindaState, 3),
        VMSTATE_BOOL(state, PindaState),
        VMSTATE_END_OF_LIST(),
    }
};

static void pinda_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = pinda_reset;
    dc->vmsd = &vmstate_pinda;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = pinda_process_action;
}
