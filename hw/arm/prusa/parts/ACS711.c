/*
	acs711.c - current amp

    Written for Mini404 in 2022 by VintagePC <https://github.com/vintagepc/>

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
#include "../utility/ScriptHost_C.h"
#include "migration/vmstate.h"

#define TYPE_ACS711 "acs711"
OBJECT_DECLARE_SIMPLE_TYPE(ACS711State, ACS711)

// Number of items contributing to the reading...
#define NUM_ITEMS 10

struct ACS711State {
    SysBusDevice parent;

	bool is_on[NUM_ITEMS];
	uint8_t on_count;

    qemu_irq irq;
};

enum {
    ActSetA,
};

static void acs711_update_irqs(ACS711State *s) {
    // ADC expects values directly in "ADC units, 12 bit."
	float amps = s->on_count * 1.7f; // 1.7A per bed turned on...
	amps += 0.01;
	// 90 mV/Amp, scaled to 3.3v AREF
    int adc_readout = 4095.f * ((amps * 0.09f)/3.3f);
    qemu_set_irq(s->irq,adc_readout);
	// if (s->on_count) printf("current out (%u on): %d\n",s->on_count, adc_readout);
}

static void acs711_read_request(void *opaque, int n, int level)
{
    ACS711State *s = ACS711(opaque);
	acs711_update_irqs(s);
}

static void acs711_bed_in(void *opaque, int n, int level)
{
    ACS711State *s = ACS711(opaque);
	s->is_on[n] = level>0;
	s->on_count = 0 ;
	for (int i=0; i<NUM_ITEMS; i++)
	{
		s->on_count += s->is_on[i];
	}
}

static int acs711_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    // ACS711State *s = ACS711(obj);
    switch (action)
    {
        // case ActSetA:
        //     s->current = scripthost_get_float(args, 0);
        //     acs711_update_irqs(s);
        //     break;
        default:
            return ScriptLS_Unhandled;

    }
    return ScriptLS_Finished;
}

static void acs711_reset(DeviceState *dev)
{
    ACS711State *s = ACS711(dev);
	for (int i=0; i< NUM_ITEMS; i++)
	{
		s->is_on[i] = false;
	}
	s->on_count = 0;
    acs711_update_irqs(s);
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(ACS711State, acs711, ACS711, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL});

static void acs711_finalize(Object *obj)
{

}

static void acs711_init(Object *obj)
{
    ACS711State *s = ACS711(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "adc_out", 1);
    qdev_init_gpio_in_named(DEVICE(obj),acs711_read_request,"adc_read_request",1);
    qdev_init_gpio_in_named(DEVICE(obj),acs711_bed_in,"bed_on",NUM_ITEMS);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_ACS711);
    script_register_action(pScript, "SetA","Sets the current readout to a given value.",ActSetA);
    script_add_arg_float(pScript, ActSetA);

    scripthost_register_scriptable(pScript);
}

static Property acs711_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_acs711 = {
    .name = TYPE_ACS711,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(on_count, ACS711State),
        VMSTATE_BOOL_ARRAY(is_on, ACS711State,NUM_ITEMS),
        VMSTATE_END_OF_LIST()
    }
};

static void acs711_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = acs711_reset;
    dc->vmsd = &vmstate_acs711;
    device_class_set_props(dc, acs711_properties);

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = acs711_process_action;
}
