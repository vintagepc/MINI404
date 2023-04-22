/*
    hall_sensor.c - Sim hall filament sensor for
    Mini404.

	Copyright 2021-3 VintagePC <https://github.com/vintagepc/>

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

#define TYPE_HALL_SENSOR "hall-sensor"

OBJECT_DECLARE_SIMPLE_TYPE(HallState, HALL_SENSOR)

struct HallState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    uint8_t state;
    qemu_irq irq;
    qemu_irq status;

};

enum {
    ACT_SET,
    ACT_TOGGLE,
};

enum {
	STATE_MISSING,
	STATE_PRESENT,
	STATE_RUNOUT
};

static const int hall_values[3] = {2, 1048575, 250};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(HallState, hall_sensor, HALL_SENSOR, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {TYPE_P404_KEYCLIENT},{NULL})


static void hall_sensor_finalize(Object *obj)
{
}

static void hall_sensor_update(HallState *s) {
    qemu_set_irq(s->irq,hall_values[s->state]);
	printf("Hall FS state: %d\n",hall_values[s->state]);
	qemu_set_irq(s->status,s->state == STATE_PRESENT);
}



static void hall_sensor_toggle(HallState *s)
{
	if (s->state == STATE_RUNOUT)
	{
		s->state = STATE_PRESENT;
	}
	else
	{
		s->state = STATE_RUNOUT;
	}
	hall_sensor_update(s);
}

static void hall_sensor_reset(DeviceState *dev)
{
    HallState *s = HALL_SENSOR(dev);
    s->state = STATE_PRESENT;
	hall_sensor_update(s);
}

static int hall_sensor_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    HallState *s = HALL_SENSOR(obj);
    switch (action)
    {
        case ACT_TOGGLE:
        {
			hall_sensor_toggle(s);
            break;
        }
        case ACT_SET:
           	s->state = scripthost_get_int(args, 0);
    		hall_sensor_update(s);
            break;
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;
}

static void hall_sensor_input_handle_key(P404KeyIF *opaque, Key keycode)
{
    HallState *s = HALL_SENSOR(opaque);
    if (keycode == 'f')
	{
		hall_sensor_toggle(s);
	}
}

static void hall_sensor_init(Object *obj)
{
    HallState *s = HALL_SENSOR(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->status, "status",1);


    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_HALL_SENSOR);

    script_register_action(pScript, "Set", "Sets the hall sensor to the given value", ACT_SET);
    script_add_arg_int(pScript, ACT_SET);
    script_register_action(pScript, "Toggle",  "Toggles hall filament sensor enum state", ACT_TOGGLE);

    scripthost_register_scriptable(pScript);
	p404_key_handle pKey = p404_new_keyhandler(P404_KEYCLIENT(obj));
    p404_register_keyhandler(pKey, 'f',"Toggles filament sensor state");
}

static const VMStateDescription vmstate_hall_sensor = {
    .name = TYPE_HALL_SENSOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_UINT8(state, HallState),
        VMSTATE_END_OF_LIST(),
    }
};

static void hall_sensor_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = hall_sensor_reset;
    dc->vmsd = &vmstate_hall_sensor;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = hall_sensor_process_action;

	P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(oc);
    kc->KeyHandler = hall_sensor_input_handle_key;
}
