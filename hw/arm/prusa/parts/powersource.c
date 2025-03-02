/*
	powersource.c

    Written for Mini404 in 2021 by VintagePC <https://github.com/vintagepc/>

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
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "../utility/macros.h"
#include "../utility/p404scriptable.h"
#include "../utility/p404_keyclient.h"
#include "../utility/ScriptHost_C.h"
#include "migration/vmstate.h"

#define TYPE_POWERSOURCE "powersource"
OBJECT_DECLARE_SIMPLE_TYPE(PSState, POWERSOURCE)

struct PSState {
    SysBusDevice parent;

    uint32_t r1;
    uint32_t r2;
    uint32_t start_voltage;
    uint32_t voltage_x1000;
    float vdiv_ratio;
    float voltage;

    qemu_irq irq;
	qemu_irq panic;
};

enum {
    ActSetV,
    ActSetA,
	ActSetPanic,
};

static void powersource_update_irqs(PSState *s) {
    int v_sense = (s->voltage*s->vdiv_ratio)*(4096.f/3.35f);
    qemu_set_irq(s->irq,v_sense);
}

static void powersource_read_request(void *opaque, int n, int level)
{
    PSState *s = POWERSOURCE(opaque);
	powersource_update_irqs(s);
}

static int powersource_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    PSState *s = POWERSOURCE(obj);
    switch (action)
    {
        case ActSetV:
            s->voltage = scripthost_get_float(args, 0);
            powersource_update_irqs(s);
            break;
		case ActSetPanic:
			qemu_irq_lower(s->panic);
			break;
        default:
            return ScriptLS_Unhandled;

    }
    return ScriptLS_Finished;
}

static void powersource_handle_key(P404KeyIF *opaque, Key keycode)
{
	PSState *s = POWERSOURCE(opaque);
    if (keycode == 'p')
	{
		printf("Panic tripped!\n");
		qemu_irq_lower(s->panic);
	}
}

static void powersource_reset(DeviceState *dev)
{
    PSState *s = POWERSOURCE(dev);
    s->vdiv_ratio = (float)s->r2/((float)s->r2+(float)s->r1);
    s->voltage = (float)s->start_voltage/1000.f;
    powersource_update_irqs(s);
	qemu_irq_raise(s->panic);
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(PSState, powersource, POWERSOURCE, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {TYPE_P404_KEYCLIENT},{NULL});

static void powersource_finalize(Object *obj)
{

}

static void powersource_init(Object *obj)
{
    PSState *s = POWERSOURCE(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "v_sense", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->panic, "panic", 1);
    qdev_init_gpio_in_named(DEVICE(obj),powersource_read_request,"adc_read_request",1);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_POWERSOURCE);
    script_register_action(pScript, "SetV","Sets the voltage readout to a given value.",ActSetV);
    script_add_arg_float(pScript, ActSetV);
	script_register_action(pScript, "SetPanic", "Trips the Power Panic line", ActSetPanic);
    scripthost_register_scriptable(pScript);

	p404_key_handle pKey = p404_new_keyhandler(P404_KEYCLIENT(obj));
    p404_register_keyhandler(pKey, 'p',"Trips power panic");
}

static int powersource_pre_save(void *opaque) {
    PSState *s = POWERSOURCE(opaque);
    s->voltage_x1000 = s->voltage*1000.f;
    return 0;
}

static int powersource_post_load(void *opaque, int version) {
    PSState *s = POWERSOURCE(opaque);
    s->vdiv_ratio = (float)s->r2/((float)s->r2+(float)s->r1);
    s->voltage = (float)s->voltage_x1000/1000.f;
    return 0;
}

static Property powersource_properties[] = {
    DEFINE_PROP_UINT32("mV", PSState, start_voltage,0),
    DEFINE_PROP_UINT32("R1", PSState, r1,10000),
    DEFINE_PROP_UINT32("R2", PSState, r2,1000),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_powersource = {
    .name = TYPE_POWERSOURCE,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = powersource_pre_save,
    .post_load = powersource_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(r1,PSState),
        VMSTATE_UINT32(r2,PSState),
        VMSTATE_UINT32(start_voltage,PSState),
        VMSTATE_UINT32(voltage_x1000,PSState),
        VMSTATE_END_OF_LIST()
    }
};

static void powersource_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = powersource_reset;
    dc->vmsd = &vmstate_powersource;
    device_class_set_props(dc, powersource_properties);

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = powersource_process_action;

	P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(klass);
    kc->KeyHandler = powersource_handle_key;

}
