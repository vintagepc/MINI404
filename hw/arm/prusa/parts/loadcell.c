/*
    loadcell.c - Sim loadcell sensor for
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
#include "../utility/p404_keyclient.h"
#include "../utility/macros.h"
#include "../utility/ScriptHost_C.h"
#include "../utility/ArgHelper.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_LOADCELL "loadcell"

OBJECT_DECLARE_SIMPLE_TYPE(LoadcellState, LOADCELL)

struct LoadcellState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    qemu_irq irq; //
    bool is_zero;
    int last_pos;

	uint8_t tap;
	p404_key_handle key;
	QEMUTimer* timer;
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(LoadcellState, loadcell, LOADCELL, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {TYPE_P404_KEYCLIENT}, {NULL})

static void loadcell_finalize(Object *obj)
{
}

static void loadcell_reset(DeviceState *dev)
{
    LoadcellState *s = LOADCELL(dev);
    qemu_set_irq(s->irq,10);
    s->is_zero = true;
}

static void loadcell_zpos_in(void *opaque, int n, int level){
    LoadcellState *s = LOADCELL(opaque);
    // This is just a SWAG... loadcell will start at 0.75mm above "reference"
    // zero.
    bool dir = s->last_pos<level;
    #define START_HEIGHT 750
    if (dir) {
        if (level>START_HEIGHT && !s->is_zero) {
			qemu_set_irq(s->irq,10);
            s->is_zero = true;
        } else if (level <=START_HEIGHT) {
            s->is_zero = false;
            int val_out = START_HEIGHT-(level);
            qemu_set_irq(s->irq, val_out*-250);
            // printf("LC out: %d\n",val_out);
        }
    } else {
        if (level>START_HEIGHT+20 && !s->is_zero) {
            qemu_set_irq(s->irq,10);
            s->is_zero = true;
        } else if (level <=START_HEIGHT-20) {
            s->is_zero = false;
            int val_out = START_HEIGHT-20-(level);
            qemu_set_irq(s->irq, val_out*-250);
            // printf("LC out: %d\n",val_out);
        }
    }
    s->last_pos = level;
}

static void loadcell_tap_timer(void *opaque)
{
    LoadcellState *s = LOADCELL(opaque);
	uint32_t value = 0;
	if (s->tap < 25)
	{
		value = s->tap*-20000;
	}
	else
	{
		value = (50-s->tap)*-20000;
	}
	qemu_set_irq(s->irq, value);
	s->tap++;

	if (s->tap > 50)
	{
		s->tap = 0;
		qemu_set_irq(s->irq, 10);

	}

	if (s->tap)
	{
		timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+10);
	}
}

static void loadcell_input_handle_key(P404KeyIF *opaque, Key keycode)
{
    LoadcellState *s = LOADCELL(opaque);
    if (keycode == 't')
	{
		printf("Tapped loadcell\n");
		timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+1);
	}
}

static int loadcell_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    // LoadcellState *s = LOADCELL(obj);
    switch (action)
    {
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;
}

static void loadcell_init(Object *obj)
{
    LoadcellState *s = LOADCELL(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in(DEVICE(obj),loadcell_zpos_in,1);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_LOADCELL);
    scripthost_register_scriptable(pScript);

	s->key = p404_new_keyhandler(P404_KEYCLIENT(obj));
    p404_register_keyhandler(s->key, 't',"Taps the loadcell");

	s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, loadcell_tap_timer, s);
}

static const VMStateDescription vmstate_loadcell = {
    .name = TYPE_LOADCELL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(is_zero,LoadcellState),
        VMSTATE_INT32(last_pos, LoadcellState),
        VMSTATE_END_OF_LIST()
    }
};

static void loadcell_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = loadcell_reset;
    dc->vmsd = &vmstate_loadcell;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = loadcell_process_action;

	P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(oc);
	kc->KeyHandler = loadcell_input_handle_key;
}
