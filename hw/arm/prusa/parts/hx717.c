/*
    hx717.c - Sim HX717 sensor for
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
#include "qemu/timer.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_HX717 "hx717"

OBJECT_DECLARE_SIMPLE_TYPE(HX717State, HX717)

struct HX717State {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    bool channel; // False = A / T = B
    uint8_t gain;
    uint8_t sck_count;
    int value[2]; // 24bit data to be clocked out.
	bool is_raw[2];// Whether the value is
    int64_t value_gain; // value with gain applied.
    int64_t last_high;

    uint8_t rate;

    QEMUTimer *tick;

    qemu_irq irq;

};

enum {
    ACT_SET,
    ACT_TOGGLE,
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(HX717State, hx717, HX717, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL})


static void hx717_finalize(Object *obj)
{
}

static void hx717_reset(DeviceState *dev)
{
    HX717State *s = HX717(dev);
    s->channel = false;
    s->gain = 128;
    s->sck_count = 0;
    s->rate = 12; // every 100ms. Change to 12 for 80SPS. HX717 can do 10, 20, 80 and 320Hz sample rates.
    s->value[0] = 0;
    //s->value[1] = 1000;
    qemu_set_irq(s->irq,1); // DOUT high when NR
    timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+s->rate);
}

static void hx717_channel_in(void *opaque, int n, int level){
    HX717State *s = HX717(opaque);
    s->value[n] = level;
	s->is_raw[n] = false;
}

static void hx717_channel_in_raw(void *opaque, int n, int level){
    HX717State *s = HX717(opaque);
    s->value[n] = level;
	s->is_raw[n] = true;
	// Tickle the channel if fsensor is pressed...
	// Kind of a hack, but I'm also not sure why this is suddenly necessary.
	timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+s->rate);
}

static void hx717_sck(void *opaque, int n, int level){
    HX717State *s = HX717(opaque);
    int64_t time = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    if (level) {
        s->last_high = time;
        s->sck_count++;
    } else if (time-s->last_high > 600){ // RESET condition... FIXME, at correct 60 this gets hit mid-readount and I don't know why yet.
        return;
        printf("HX717: Reset (%"PRId64")\n", time-s->last_high);
        // s->channel = false;
        // s->gain = 128;
        // s->sck_count = 0;
        qemu_set_irq(s->irq,1); // DOUT high when NR
        timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+s->rate);
    }
    if (!level) {
        return;
    }
    //printf("HX717: L %d SCK: %u\n",level, s->sck_count);
    if (s->sck_count<=24) {
        qemu_set_irq(s->irq, (s->value_gain>>(24-s->sck_count))&1U);
        // if (s->sck_count == 23) {
        //     printf("HX717: Chan %c Gain %u - Clocked out %u\n",s->channel? 'B':'A',s->gain, s->value_gain);
        // }
    } else {
        qemu_irq_raise(s->irq);
        switch (s->sck_count) {
            case 25:
                s->channel = false;
                s->gain = 128;
                break;
            case 26:
                s->channel = true;
                s->gain = 64;
                break;
            case 27:
                s->channel = false;
                s->gain = 64;
                break;
            case 28:
                s->channel = true;
                s->gain = 8;
                break;
            default:
                printf("ERR: Unhandled HX717 channel config! %u\n",s->sck_count);
        }
        timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+s->rate);
    }
}

static void hx717_data_ready(void *opaque) {
    HX717State *s = HX717(opaque);
    qemu_irq_raise(s->irq);
    qemu_irq_lower(s->irq);
    s->sck_count = 0;
    s->value_gain = s->value[s->channel];
	s->value_gain *= s->gain;
	if (!s->is_raw[s->channel])
	{
		s->value_gain/=1000;
	}
    timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+s->rate);
}


static int hx717_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    HX717State *s = HX717(obj);
    switch (action)
    {
        case ACT_SET:
            s->value[0] = scripthost_get_int(args, 0);
            break;
        default:
            return ScriptLS_Unhandled;
    }
    // hx717_update(s);
    return ScriptLS_Finished;
}



static void hx717_init(Object *obj)
{
    HX717State *s = HX717(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1); // DOUT
    qdev_init_gpio_in(DEVICE(obj),hx717_sck,1);

    qdev_init_gpio_in_named(DEVICE(obj),hx717_channel_in,"input_x1000",2);
    qdev_init_gpio_in_named(DEVICE(obj),hx717_channel_in_raw,"input",2);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_HX717);

    script_register_action(pScript, "Set", "Sets the load cell output to the given value", ACT_SET);
    script_add_arg_int(pScript, ACT_SET);

    s->tick = timer_new_ms(QEMU_CLOCK_VIRTUAL,
        (QEMUTimerCB *)hx717_data_ready, s);

    scripthost_register_scriptable(pScript);
}

static const VMStateDescription vmstate_hx717 = {
    .name = TYPE_HX717,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(channel,HX717State),
        VMSTATE_UINT8(gain, HX717State),
        VMSTATE_UINT8(sck_count, HX717State),
        VMSTATE_UINT8(rate, HX717State),
        VMSTATE_INT32_ARRAY(value,HX717State,2), // 24bit data to be clocked out.
        VMSTATE_INT64(value_gain, HX717State),
        VMSTATE_INT64(last_high, HX717State),
        VMSTATE_TIMER_PTR(tick, HX717State),
        VMSTATE_END_OF_LIST()
    }
};

static void hx717_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = hx717_reset;
    dc->vmsd = &vmstate_hx717;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = hx717_process_action;
}
