/*
	cs30bl.c - current amplifier implementation.

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
#include "../utility/ScriptHost_C.h"
#include "migration/vmstate.h"

#define TYPE_CS30BL "cs30bl"
OBJECT_DECLARE_SIMPLE_TYPE(CS30BLState, CS30BL)

struct CS30BLState {
    SysBusDevice parent;

    uint32_t rcsense;
    uint32_t start_current;
    uint32_t gain;
    float current;
    uint32_t current_x1000;

    qemu_irq irq;
};

enum {
    ActSetV,
    ActSetA,
};

static void cs30bl_update_irqs(CS30BLState *s) {
    // ADC expects values directly in "ADC units"
    int ma_sense = (s->current*((float)s->rcsense/1000.f)*(float)s->gain)/3.35f*4096.f*1.95f/2.f;
    qemu_set_irq(s->irq,ma_sense);
}

static void cs30bl_read_request(void *opaque, int n, int level)
{
    CS30BLState *s = CS30BL(opaque);
	cs30bl_update_irqs(s);
}

static int cs30bl_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    CS30BLState *s = CS30BL(obj);
    switch (action)
    {
        case ActSetA:
            s->current = scripthost_get_float(args, 0);
            cs30bl_update_irqs(s);
            break;
        default:
            return ScriptLS_Unhandled;

    }
    return ScriptLS_Finished;
}

static void cs30bl_reset(DeviceState *dev)
{
    CS30BLState *s = CS30BL(dev);
    s->current = (float)s->start_current/1000.f;
    cs30bl_update_irqs(s);
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(CS30BLState, cs30bl, CS30BL, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL});

static void cs30bl_finalize(Object *obj)
{

}

static void cs30bl_init(Object *obj)
{
    CS30BLState *s = CS30BL(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "a_sense", 1);
    qdev_init_gpio_in_named(DEVICE(obj),cs30bl_read_request,"adc_read_request",1);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_CS30BL);
    script_register_action(pScript, "SetA","Sets the voltage readout to a given value.",ActSetA);
    script_add_arg_float(pScript, ActSetA);

    scripthost_register_scriptable(pScript);
}

static Property cs30bl_properties[] = {
    DEFINE_PROP_UINT32("mA", CS30BLState, start_current,0),
    DEFINE_PROP_UINT32("mOhmR", CS30BLState, rcsense,22),
    DEFINE_PROP_UINT32("gain", CS30BLState, gain,50),
    DEFINE_PROP_END_OF_LIST(),
};

static int cs30bl_pre_save(void *opaque) {
    CS30BLState *s = CS30BL(opaque);
    s->current_x1000 = s->current*1000.f;
    return 0;
}

static int cs30bl_post_load(void *opaque, int version) {
    CS30BLState *s = CS30BL(opaque);
    s->current = (float)s->current_x1000/1000.f;
    return 0;
}

static const VMStateDescription vmstate_cs30bl = {
    .name = TYPE_CS30BL,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = cs30bl_pre_save,
    .post_load = cs30bl_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(rcsense, CS30BLState),
        VMSTATE_UINT32(start_current, CS30BLState),
        VMSTATE_UINT32(gain, CS30BLState),
        VMSTATE_UINT32(current_x1000, CS30BLState),
        VMSTATE_END_OF_LIST()
    }
};

static void cs30bl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = cs30bl_reset;
    dc->vmsd = &vmstate_cs30bl;
    device_class_set_props(dc, cs30bl_properties);

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = cs30bl_process_action;
}
