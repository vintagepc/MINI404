/*
	heater.c - a heater object for MINI404. There's not much to it,
    it just ticks the temperature "up" at a determined rate when active on PWM and down in
    in an exponential curve when off.

	Original (C) 2020 VintagePC <https://github.com/vintagepc/>
    Adapted to QEMU/C in 2021

 	This file is part of MINI404

	MINI404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	MINI404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with MINI404.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include <math.h>
#include "../utility/p404scriptable.h"
#include "../utility/macros.h"
#include "../utility/ScriptHost_C.h"


//#define DBG if(s->chrLabel=='B')

#ifndef DBG
#define DBG if(false)
#endif

#define TYPE_HEATER "heater"
OBJECT_DECLARE_SIMPLE_TYPE(heater_state, HEATER)

struct heater_state {
    SysBusDevice parent;

    float thermalMass;
    float ambientTemp;
    float currentTemp;

    uint8_t chrLabel;
    uint8_t mass10x;

    uint16_t pwm, lastpwm, timeout_level;
    uint16_t custom_pwm;
    int32_t current_x100, ambient_x100;

    int16_t tick_overrun;

    uint64_t last_tick, last_off, last_on;

    bool is_ticking, use_custom_pwm;

    qemu_irq temp_out, pwm_out;
    QEMUTimer *temp_tick, *softpwm_timeout;

	script_handle handle;
};

enum {
    ActNormal,
    ActRunaway,
    ActOpen,
    ActSet,
};


static void heater_softpwm_timeout(void* opaque)
{
    heater_state *s = opaque;
    s->pwm = s->timeout_level;
    // Tickle timer if turned off.
    if (!s->is_ticking) // Start the heater.
    {
        timer_mod(s->temp_tick,qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
        s->is_ticking = true;
    }
}


static void heater_temp_tick_expire(void *opaque)
{
    heater_state *s = opaque;
    static const float updaterate = 0.25;
    uint16_t usedpwmval = s->use_custom_pwm ? s->custom_pwm : s->pwm;

    qemu_set_irq(s->pwm_out, usedpwmval);
    uint64_t tNow = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
    if (usedpwmval || s->lastpwm>0)
    {
        float pwmval = (usedpwmval>s->lastpwm)? usedpwmval : s->lastpwm;
        float fDelta = (s->thermalMass*(pwmval/255.0f))*updaterate;
        s->currentTemp += fDelta;
        DBG printf("Temp: %f %f\n", s->currentTemp, fDelta);
        s->tick_overrun = 4;
        s->lastpwm = usedpwmval;
        s->last_tick = tNow;
     } else {// Cooling - do a little exponential decay
        float dT = (s->currentTemp - s->ambientTemp)*pow(2.7183,-0.005*updaterate);
        s->currentTemp -= s->currentTemp - (s->ambientTemp + dT);
    }

    if (usedpwmval || s->currentTemp>s->ambientTemp+0.3)
	{
        timer_mod(s->temp_tick, tNow+250);
	}
    else
    {
        s->is_ticking = false;
        s->currentTemp = s->ambientTemp;
    }
    qemu_set_irq(s->temp_out, s->currentTemp*256.f);
}

static void heater_pwm_change(void* opaque, int n, int level)
{
    heater_state *s = opaque;
	s->pwm = level;
	// Tickle timer if turned off.
	uint64_t tNow = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
    if (!s->is_ticking) // Start the heater.
    {
        timer_mod(s->temp_tick,tNow + 1);
        s->is_ticking = true;
    }
}

static void heater_soft_pwm_change(void* opaque, int n, int level)
{
    heater_state *s = opaque;
    uint16_t tOn = 0;
    uint64_t tNow = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
    if (level)
    {
        s->last_on = tNow;
        s->timeout_level = 255;
        timer_mod(s->softpwm_timeout, tNow+500);
    } else {
        s->last_off = tNow;
        tOn = tNow - s->last_on;
        s->timeout_level = 0;
        DBG printf("Ontime: %u\n",tOn);
        timer_mod(s->softpwm_timeout, tNow+3000);
        s->pwm = tOn & 0xFF;
    }
    // Tickle timer if turned off.
    if (!s->is_ticking) // Start the heater.
    {
        timer_mod(s->temp_tick,tNow + 1);
        s->is_ticking = true;
    }
}

static void heater_reset(DeviceState *dev)
{
    heater_state *s = HEATER(dev);

    s->thermalMass = ((float)s->mass10x)/10.f;
    s->ambientTemp = 18.f;
    s->currentTemp = s->ambientTemp;
}

static int heater_process_action(P404ScriptIF *obj, unsigned int action, script_args args) {
    heater_state *s = HEATER(obj);
    switch (action){
        case ActNormal:
            s->custom_pwm = 0;
            s->use_custom_pwm = false;
            break;
        case ActRunaway:
            s->custom_pwm = 255;
            s->use_custom_pwm = true;
            if (!s->is_ticking){
                timer_mod(s->softpwm_timeout, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+500);
            }
            break;
        case ActOpen:
            s->custom_pwm = 0;
            s->use_custom_pwm = true;
            break;
        case ActSet:
            s->currentTemp = scripthost_get_float(args, 0);
            qemu_set_irq(s->temp_out, s->currentTemp*256.f);
            break;
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(heater_state, heater, HEATER, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL});

static void heater_finalize(Object *obj)
{

};

static void heater_init(Object *obj)
{
    heater_state *s = HEATER(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->temp_out, "temp_out", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->pwm_out, "pwm-out", 1);


	// TODO - fix these names so soft is explicit and raw is default.
    qdev_init_gpio_in_named(DEVICE(obj),heater_soft_pwm_change, "pwm_in", 1);
    qdev_init_gpio_in_named(DEVICE(obj),heater_pwm_change, "raw-pwm-in", 1);

    s->temp_tick = timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)heater_temp_tick_expire, s);
    s->softpwm_timeout = timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)heater_softpwm_timeout, s);

    s->handle = script_instance_new(P404_SCRIPTABLE(obj), TYPE_HEATER);

    script_register_action(s->handle, "Open","Sets heater as open-circuit", ActOpen);
    script_register_action(s->handle, "Runaway","Sets heater as if in thermal runaway", ActRunaway);
    script_register_action(s->handle, "Restore","Restores normal (non-open or runaway) state", ActNormal);
    script_register_action(s->handle, "SetTemp","Sets the current temperature the heater uses to update the thermistor", ActSet);
    script_add_arg_float(s->handle, ActSet);
    scripthost_register_scriptable(s->handle);

}

static Property heater_properties[] = {
    DEFINE_PROP_UINT8("thermal_mass_x10",heater_state, mass10x, 25),
    DEFINE_PROP_UINT8("label",heater_state, chrLabel, (uint8_t)' '),
    DEFINE_PROP_END_OF_LIST(),
};

static int heater_pre_save(void *opaque) {
    heater_state *s = HEATER(opaque);
    s->ambient_x100 = 100.f * s->ambientTemp;
    s->current_x100 = 100.f * s->currentTemp;
    return 0;
}

static int heater_post_load(void *opaque, int version) {
    heater_state *s = HEATER(opaque);
    s->ambientTemp = (float)s->ambient_x100/100.f;
    s->currentTemp = (float)s->current_x100/100.f;
    s->thermalMass = ((float)s->mass10x)/10.f;
    return 0;
}

static const VMStateDescription vmstate_heater = {
    .name = TYPE_HEATER,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = heater_pre_save,
    .post_load = heater_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(chrLabel,heater_state),
        VMSTATE_UINT8(mass10x,heater_state),
        VMSTATE_UINT16(pwm,heater_state),
        VMSTATE_UINT16(lastpwm,heater_state),
        VMSTATE_UINT16(timeout_level,heater_state),
        VMSTATE_UINT16(custom_pwm,heater_state),
        VMSTATE_INT32(current_x100,heater_state),
        VMSTATE_INT32(ambient_x100,heater_state),
        VMSTATE_INT16(tick_overrun,heater_state),
        VMSTATE_UINT64(last_tick,heater_state),
        VMSTATE_UINT64(last_off,heater_state),
        VMSTATE_UINT64(last_on,heater_state),
        VMSTATE_BOOL(is_ticking,heater_state),
        VMSTATE_BOOL(use_custom_pwm,heater_state),
        VMSTATE_TIMER_PTR(temp_tick,heater_state),
        VMSTATE_TIMER_PTR(softpwm_timeout,heater_state),
        VMSTATE_END_OF_LIST()
    }
};


static void heater_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = heater_reset;
    dc->vmsd = &vmstate_heater;
    device_class_set_props(dc, heater_properties);

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = heater_process_action;

}
