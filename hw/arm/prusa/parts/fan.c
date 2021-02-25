/*
    fan.c - Simple fan tach sim for Einsy Rambo

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
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "../utility/macros.h"
#include "../utility/p404scriptable.h"
#include "../utility/ScriptHost_C.h"
#include "qemu/module.h"

struct  fan_state //:public SoftPWMable, public Scriptable
{
    SysBusDevice parent;
	bool pulse_state;
	bool is_nonlinear;

    bool is_stalled;

	uint8_t pwm;
	uint32_t max_rpm;
	uint32_t current_rpm;
	uint32_t usec_per_pulse;
	// std::atomic_uint16_t m_uiRot {0};

    int last_level;

	uint8_t label;

    qemu_irq tach_pulse, pwm_out;

	QEMUTimer *tach;
	QEMUTimer *softpwm;
    int64_t tOn, tOff, tLastOn;

};

enum {
    ActResume,
    ActStall,
    ActGetRPM,
};

// FIXME/HACK - E fan is nonlinear in the RPM vs PWM.
static uint16_t fan_corrections[] = {1450,2100, 1700, 1050, 0};

#define TYPE_FAN "fan"
OBJECT_DECLARE_SIMPLE_TYPE(fan_state, FAN)


static void fan_tach_expire(void *opaque)
{
    fan_state *s = opaque;
    if (s->is_stalled) {
        qemu_set_irq(s->tach_pulse, 0);
    } else {
        qemu_set_irq(s->tach_pulse, s->pulse_state^=1);
    }
    timer_mod(s->tach, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL)+s->usec_per_pulse);
}

static void fan_pwm_change(void *opaque, int n, int level) {
    fan_state *s = opaque;
    qemu_set_irq(s->pwm_out, level);
    s->current_rpm = (((uint32_t)s->max_rpm)*level)/255;
    if (s->is_nonlinear)
    {
        s->current_rpm += fan_corrections[level/64];
        // printf( "Corr: %d - ", level/64u);
    }
    float fSecPerRev = 60.0f/(float)s->current_rpm;
    float fuSPerRev = 1000000.f*fSecPerRev;
    s->usec_per_pulse = fuSPerRev/4.f; // 4 pulses per rev.
    // if(s->label == 'E') printf("Fan %c pwm change: %d (RPM %u) uspr: %lu\n", s->label, level, s->current_rpm, s->usec_per_pulse);
    // TRACE(printf("New PWM(%u)/RPM/cyc: %u / %u / %u\n", max_rpm, m_uiPWM.load(), current_rpm, usec_per_pulse));
    if (s->current_rpm>0) // Restart the timer if it has expired, otherwise leave it be.
    {
        timer_mod(s->tach, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL)+s->usec_per_pulse);
    }
    else
    {
        timer_del(s->tach);
    }
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
    //printf("P%d ", level);
    // static const uint32_t timeout = 17000; // fudge factor/trigger timeout... fix this later.
    int64_t tNow = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
	if (level & !s->last_level) // Was off, start at full, we'll update rate later.
	{
        timer_mod(s->softpwm, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+200);
		// if (s->cntTOn>s->cntSoftPWM)
		// {
		// }
        //fan_pwm_change(s, 0, 255);
        s->tLastOn = s->tOn;
		s->tOn = tNow;
        //printf("New Total: %ld, last on %ld\n", s->tOn - s->tLastOn);
	}
	else if (!level && s->last_level)
	{
        s->tOff = tNow;
		uint64_t uiCycleDelta = s->tOff - s->tOn; // This is the on time in us.  
        uint64_t tTotal = 50000; // hack, based on TIM1 init config (50 ms period). //s->tOn - s->tLastOn; // Total delta between on pulese (total duty cycle)
        //printf("New On: %lu/%lu\n",uiCycleDelta, tTotal);
		//TRACE(printf("New soft PWM delta: %d\n",uiCycleDelta/1000));
        if (uiCycleDelta > tTotal)
        {
            uiCycleDelta %= (tTotal); // HACK: Workaround for the first "off" after the initial 100% "kickstart"
        }
		uint16_t uiSoftPWM = 255.f*((float)uiCycleDelta/(float)tTotal); //62.5 Hz means full on is ~256k cycles.
		fan_pwm_change(s, 0, uiSoftPWM);
		// s->cntTOn = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->softpwm, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+200);

	}
    s->last_level = level;
}

static int fan_process_action(P404ScriptIF *obj, unsigned int action, script_args args) {
    fan_state *s = FAN(obj);
    switch (action) {
        case ActStall:
            s->is_stalled = true;
            break;
        case ActResume:
            s->is_stalled = false;
            break;
        case ActGetRPM:
            script_print_int( s->is_stalled? 0 : s->current_rpm);   
            break;
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;

}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(fan_state, fan, FAN, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL});


static void fan_finalize(Object *obj){

}

static void fan_init(Object *obj){

    fan_state *s = FAN(obj);
    s->current_rpm = 0;
    s->usec_per_pulse = 0;

    s->tach =timer_new_us(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)fan_tach_expire, s);


    s->softpwm =timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)fan_softpwm_timeout, s);

    qdev_init_gpio_out_named(DEVICE(obj), &s->tach_pulse, "tach-out",1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->pwm_out, "pwm-out",1);
    qdev_init_gpio_in_named(DEVICE(obj), fan_pwm_change, "pwm-in",1);
    qdev_init_gpio_in_named(DEVICE(obj), fan_pwm_change_soft, "pwm-in-soft",1);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(s), "fan");
    script_register_action(pScript, "Stall","Stalls the fan tachometer",ActStall);
    script_register_action(pScript, "Resume","Resumes a stalled fan.",ActResume);
    script_register_action(pScript, "GetRPM","Reports the current RPM",ActGetRPM);
    scripthost_register_scriptable(pScript);

}


static Property fan_properties[] = {
    DEFINE_PROP_UINT8("label", fan_state, label,(uint8_t)' '),
    DEFINE_PROP_UINT32("max_rpm", fan_state, max_rpm,8800),
    DEFINE_PROP_BOOL("is_nonlinear", fan_state, is_nonlinear, 0),
    DEFINE_PROP_END_OF_LIST()
};


static void fan_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, fan_properties);
   // dc->vmsd = &vmstate_fan;

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = fan_process_action;
}
