/*
    fan.c - Simple fan tach sim for Mini404

	Original copyright 2020 VintagePC <https://github.com/vintagepc/> as part of MK404
    Ported to C/QEMU in 2020.

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
#include "qom/object.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "../utility/macros.h"
#include "../utility/p404scriptable.h"
#include "../utility/ScriptHost_C.h"
#include "qemu/module.h"
#include "fan.h"
#include "trace.h"

struct  fan_state
{
    SysBusDevice parent;
	bool pulse_state;
	bool is_nonlinear;

    /* Three independent fault types:
         is_stalled    - seized rotor: no tach pulses AND no airflow
         tach_failed   - broken tach wire: fan spins and cools, reports nothing
         airflow_scale - obstructed duct or clogged shroud: cools less than
                         commanded, and the firmware cannot see it at all
       Stall/Resume keep their old both-at-once meaning as a convenience. */
    bool is_stalled;
    bool tach_failed;
    uint8_t airflow_scale;

	uint8_t pwm;
	int last_pwm_level;
	uint32_t max_rpm;
	uint32_t current_rpm;
	uint32_t usec_per_pulse;

    int last_level;

	uint8_t label;

    qemu_irq tach_pulse, pwm_out, rpm_out, airflow_out;

    bool tach_blocked;

	QEMUTimer *tach;
	QEMUTimer *softpwm;
    int64_t tOn, tOff, tLastOn;

	script_handle handle;

};

enum {
    ActResume,
    ActStall,
    ActGetRPM,
    ActTachFail,
    ActTachOK,
    ActSetAirflow,
};

// FIXME/HACK - E fan is nonlinear in the RPM vs PWM.
static uint16_t fan_corrections[] = {1450,2100, 1700, 1050, 0};

#define TYPE_FAN "fan"
OBJECT_DECLARE_SIMPLE_TYPE(fan_state, FAN)


static void fan_tach_expire(void *opaque)
{
    fan_state *s = opaque;
    if (!s->tach_blocked)
    {
        if (s->is_stalled || s->tach_failed) {
            qemu_set_irq(s->tach_pulse, 0);
        } else {
            qemu_set_irq(s->tach_pulse, s->pulse_state^=1);
        }
    }
    timer_mod(s->tach, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL)+s->usec_per_pulse);
}

static void fan_tach_block(void *opaque, int n, int level) {
    fan_state *s = opaque;
    s->tach_blocked = level;
}


/* Air actually moving, as a duty the heater can use. */
static int fan_airflow(fan_state *s, int level)
{
    if (s->is_stalled) {
        return 0;
    }
    return (level * (int)s->airflow_scale) / 255;
}

static void fan_pwm_change(void *opaque, int n, int level) {
    fan_state *s = opaque;

	if (n == FAN_PWM_INPUT_INVERTED)
	{
		level = 255 - level;
	}
    trace_fan_new_pwm_value(s->label, level);
    qemu_set_irq(s->pwm_out, level);
    s->last_pwm_level = level;
    qemu_set_irq(s->airflow_out, fan_airflow(s, level));
    s->current_rpm = (((uint32_t)s->max_rpm)*level)/255;
    if (s->is_nonlinear)
    {
        s->current_rpm += fan_corrections[level/64];
    }
    float fSecPerRev = 60.0f/(float)s->current_rpm;
    float fuSPerRev = 1000000.f*fSecPerRev;
    s->usec_per_pulse = fuSPerRev/4.f; // 4 pulses per rev.
    if (s->current_rpm>0) // Restart the timer if it has expired, otherwise leave it be.
    {
        trace_fan_new_tach_interval(s->label, s->usec_per_pulse);
        timer_mod(s->tach, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL)+s->usec_per_pulse);
    }
    else
    {
        timer_del(s->tach);
    }
    qemu_set_irq(s->rpm_out, s->current_rpm);
}


static void fan_softpwm_timeout(void* opaque)
{
    // printf("timeout\n");
        fan_state *s = opaque;
        fan_pwm_change(opaque, 0,s->last_level*255);
        s->tOn = 0;
}

static void fan_pwm_change_soft(void *opaque, int n, int level)
{
    fan_state *s = opaque;
    int64_t tNow = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
	if (level & !s->last_level) // Was off, start at full, we'll update rate later.
	{
        timer_mod(s->softpwm, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+200);
        s->tLastOn = s->tOn;
		s->tOn = tNow;
	}
	else if (!level && s->last_level)
	{
        s->tOff = tNow;
		uint64_t uiCycleDelta = s->tOff - s->tOn; // This is the on time in us.
        uint64_t tTotal = 50000; // hack, based on TIM1 init config (50 ms period). //s->tOn - s->tLastOn; // Total delta between on pulese (total duty cycle)
        if (uiCycleDelta > tTotal)
        {
            uiCycleDelta %= (tTotal); // HACK: Workaround for the first "off" after the initial 100% "kickstart"
        }
		uint16_t uiSoftPWM = 255.f*((float)uiCycleDelta/(float)tTotal); //62.5 Hz means full on is ~256k cycles.
		fan_pwm_change(s, 0, uiSoftPWM);
        timer_mod(s->softpwm, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+200);

	}
    s->last_level = level;
}

static int fan_process_action(P404ScriptIF *obj, unsigned int action, script_args args) {
    fan_state *s = FAN(obj);
    switch (action) {
        case ActStall:
            s->is_stalled = true;
            s->tach_failed = true;
            qemu_set_irq(s->airflow_out, 0);
            break;
        case ActResume:
            s->is_stalled = false;
            s->tach_failed = false;
            s->airflow_scale = 255;
            qemu_set_irq(s->airflow_out, fan_airflow(s, s->last_pwm_level));
            break;
        case ActTachFail:
            s->tach_failed = true;
            break;
        case ActTachOK:
            s->tach_failed = false;
            break;
        case ActSetAirflow:
            s->airflow_scale = scripthost_get_int(args, 0);
            qemu_set_irq(s->airflow_out, fan_airflow(s, s->last_pwm_level));
            break;
        case ActGetRPM:
            script_print_int((s->is_stalled || s->tach_failed) ? 0 : s->current_rpm);
            break;
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;

}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(fan_state, fan, FAN, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL});

static void fan_reset(DeviceState *dev)
{
    fan_state *s = FAN(dev);
    s->is_stalled = false;
    s->tach_failed = false;
    s->airflow_scale = 255;
   qemu_set_irq(s->rpm_out, s->current_rpm);
}

static void fan_finalize(Object *obj){

}

static void fan_init(Object *obj){

    fan_state *s = FAN(obj);
    s->current_rpm = 0;
    s->airflow_scale = 255;
    s->usec_per_pulse = 0;

    s->tach =timer_new_us(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)fan_tach_expire, s);


    s->softpwm =timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)fan_softpwm_timeout, s);

    qdev_init_gpio_out_named(DEVICE(obj), &s->tach_pulse, "tach-out",1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->pwm_out, "pwm-out",1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->rpm_out, "rpm-out",1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->airflow_out, "airflow-out",1);
    qdev_init_gpio_in_named(DEVICE(obj), fan_tach_block, "tach-disable",1);
    qdev_init_gpio_in_named(DEVICE(obj), fan_pwm_change, "pwm-in",FAN_PWM_INPUT_COUNT);
    qdev_init_gpio_in_named(DEVICE(obj), fan_pwm_change_soft, "pwm-in-soft",1);

    s->handle = script_instance_new(P404_SCRIPTABLE(s), "fan");
    script_register_action(s->handle, "Stall","Stalls the fan: no tach pulses and no airflow",ActStall);
    script_register_action(s->handle, "Resume","Clears every fault: tach reporting on, full airflow.",ActResume);
    script_register_action(s->handle, "TachFail","Breaks the tach wire only, fan still spins",ActTachFail);
    script_register_action(s->handle, "TachOK","Restores tach reporting, (i.e. ~TachFail)",ActTachOK);
    script_register_action(s->handle, "SetAirflow","Scales airflow 0-255 against commanded duty",ActSetAirflow);
    script_add_arg_int(s->handle, ActSetAirflow);
    script_register_action(s->handle, "GetRPM","Reports the current RPM",ActGetRPM);
    scripthost_register_scriptable(s->handle);

}


static const Property fan_properties[] = {
    DEFINE_PROP_UINT8("label", fan_state, label,(uint8_t)' '),
    DEFINE_PROP_UINT32("max_rpm", fan_state, max_rpm,8800),
    DEFINE_PROP_BOOL("is_nonlinear", fan_state, is_nonlinear, 0),
    
};

static const VMStateDescription vmstate_fan = {
    .name = TYPE_FAN,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_BOOL(pulse_state,fan_state),
        VMSTATE_BOOL(is_nonlinear,fan_state),
        VMSTATE_BOOL(is_stalled,fan_state),
        VMSTATE_INT32_V(last_pwm_level,fan_state,2),
        VMSTATE_BOOL_V(tach_failed,fan_state,2),
        VMSTATE_UINT8_V(airflow_scale,fan_state,2),
        VMSTATE_UINT8(pwm,fan_state),
        VMSTATE_UINT8(label,fan_state),
        VMSTATE_UINT32(max_rpm,fan_state),
        VMSTATE_UINT32(current_rpm,fan_state),
        VMSTATE_UINT32(usec_per_pulse,fan_state),
        VMSTATE_INT32(last_level,fan_state),
        VMSTATE_INT64(tOn,fan_state),
        VMSTATE_INT64(tOff,fan_state),
        VMSTATE_INT64(tLastOn,fan_state),
        VMSTATE_TIMER_PTR(tach,fan_state),
        VMSTATE_TIMER_PTR(softpwm,fan_state),
        VMSTATE_END_OF_LIST(),
    }
};


static void fan_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, fan_properties);
    dc->vmsd = &vmstate_fan;
    device_class_set_legacy_reset(dc, fan_reset);
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = fan_process_action;
}
