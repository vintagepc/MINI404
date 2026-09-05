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
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "fan.h"
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

/* PWM history for the transport delay, one slot per 250 ms tick. */
#define HEATER_TICK_MS 250u
#define HEATER_PWM_HIST 32u

struct heater_state {
    SysBusDevice parent;

    float thermalMass;
    float ambientTemp;
    float currentTemp;

    uint8_t chrLabel;
    uint8_t mass10x;
    uint16_t ambient_x10;      // configured ambient, tenths of degC
    int32_t start_temp_x10;    // configured power-on temperature, tenths of degC; <0 = ambient
    uint16_t cool_tau_s;       // cool-down time constant, seconds, no fans running
    uint16_t cool_tau_pfan_s;  // ditto at full print-fan duty; 0 = fan not characterised
    uint16_t cool_tau_hbr_s;   // ditto at full heatbreak-fan duty
    uint16_t heat_lag_ms;      // block -> sensor lag; 0 = the sensor tracks the block
    uint16_t dead_time_ms;     // heater -> block transport delay

    float sensorTemp;          // what the thermistor sees; == currentTemp with no lag
    uint8_t fan_duty[FAN_COOLING_COUNT];
    uint16_t pwm_hist[HEATER_PWM_HIST];
    uint8_t pwm_hist_idx;
    int32_t sensor_x100;

	uint16_t resistancex100, voltagex100;

    uint16_t pwm, lastpwm, timeout_level;
    uint16_t custom_pwm;
    int32_t current_x100, ambient_x100;

    int16_t tick_overrun;

    uint64_t last_tick, last_off, last_on;

    bool is_ticking, use_custom_pwm;
    bool sock_on;

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

/* Reduction factor applied to thermal_mass_x10 when the silicone heatblock
   sock is fitted. Real with-sock heatup is slower because the sock adds
   thermal mass to the assembly. The MK4 FW compensates with a -20 °C offset
   on the heater selftest range (hotend_type.cpp::hotend_type_heater_selftest_offset
   on HAS_NEXTRUDER); 0.85 lands the 42 s nozzle test reading inside that
   shifted range. */
#define HEATER_SOCK_MASS_FACTOR 0.85f
extern float heater_calculate_current(heater_state *s);

/* Heat transfer under forced convection grows slower than airflow does. The
   exponent is the usual Nusselt-vs-Reynolds form; 0.6 is what the CORE One
   heatbreak fan's two measured duty points give (51/255 and 218/255 -- see
   docs/HotendThermalMeasurement.md 4e). Fan duty is assumed as proportional to
   airflow */
#define HEATER_FAN_FLOW_EXP 0.6f

/* Cool-down constant for the fan state the heater is currently in.
   The two fans do NOT add: on a CORE One the print fan is a shrouded turbine
   blowing directly under the nozzle tip while the heatbreak fan only cools the
   heatsink above the thermal choke, so once the print fan runs the heatbreak
   path stops mattering. Summing the conductances over-predicts cooling by ~25%
   against measurement; taking the dominant fan reproduces all four measured
   states. */
static float heater_cool_tau(heater_state *s)
{
    const float still = 1.f / (float)s->cool_tau_s;
    const uint16_t tau_full[FAN_COOLING_COUNT] = {
        [FAN_COOLING_PRINT] = s->cool_tau_pfan_s,
        [FAN_COOLING_HBR] = s->cool_tau_hbr_s,
    };
    float extra = 0.f;
    for (unsigned i = 0; i < FAN_COOLING_COUNT; i++)
    {
        if (!tau_full[i] || !s->fan_duty[i])
        {
            continue;
        }
        const float duty = (float)s->fan_duty[i] / 255.f;
        /* negative would mean a fan that slows cooling; ignore rather than trust */
        const float c = (1.f / (float)tau_full[i] - still) * powf(duty, HEATER_FAN_FLOW_EXP);
        if (c > extra)
        {
            extra = c;
        }
    }
    return 1.f / (still + extra);
}

/* Record this tick's PWM and return the value the block should feel now. */
static uint16_t heater_delayed_pwm(heater_state *s, uint16_t now)
{
    s->pwm_hist[s->pwm_hist_idx] = now;
    s->pwm_hist_idx = (s->pwm_hist_idx + 1u) % HEATER_PWM_HIST;
    uint32_t slots = s->dead_time_ms / HEATER_TICK_MS;
    if (!slots)
    {
        return now;
    }
    /* realize() rejects a dead time the history cannot hold, so this cannot
       silently truncate; the clamp is only a bounds guard on the index. */
    if (slots > HEATER_PWM_HIST - 1u)
    {
        slots = HEATER_PWM_HIST - 1u;
    }
    return s->pwm_hist[(s->pwm_hist_idx + HEATER_PWM_HIST - 1u - slots) % HEATER_PWM_HIST];
}

static void heater_fan_change(void *opaque, int n, int level)
{
    heater_state *s = opaque;
    if (n >= 0 && n < FAN_COOLING_COUNT)
    {
        s->fan_duty[n] = (level < 0) ? 0 : (level > 255 ? 255 : level);
    }
}

// Calculate current consumption based on given voltage/resistance, and scale by PWM.
extern float heater_calculate_current(heater_state *s)
{
	float currentmA = 0.f;
	if (s->resistancex100)
	{
		float resistance = (float)s->resistancex100*( 1 + (0.0042f * (s->currentTemp - 20.f)) );
		currentmA = ((float)s->voltagex100/resistance)*((float)s->pwm/255.f)*1000.f; // 100x cancelled by division.
		//printf("Current: %u %c %f\n",s->pwm, s->chrLabel, currentmA);
	}
	return currentmA;
}

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
    // PWM is instant, but physics delays thermal effect
    const uint16_t heatpwm = heater_delayed_pwm(s, usedpwmval);
    if (heatpwm || s->lastpwm>0)
    {
        float pwmval = (heatpwm>s->lastpwm)? heatpwm : s->lastpwm;
        float fDelta = (s->thermalMass*(pwmval/255.0f))*updaterate;
        s->currentTemp += fDelta;
        DBG printf("Temp: %f %f\n", s->currentTemp, fDelta);
        s->tick_overrun = 4;
        s->lastpwm = heatpwm;
        s->last_tick = tNow;
     } else {
        /* Newton cooling towards ambient. Measured on a CORE One (see
           sim/docs/HotendThermalMeasurement.md 4e): with both fans off, 324 s
           standard / ~510 s HT. */
        const float decay = expf(-updaterate / heater_cool_tau(s));
        s->currentTemp = s->ambientTemp + (s->currentTemp - s->ambientTemp) * decay;
    }

    // Block -> thermistor lag.
    if (s->heat_lag_ms)
    {
        const float a = 1.f - expf(-updaterate / ((float)s->heat_lag_ms / 1000.f));
        s->sensorTemp += (s->currentTemp - s->sensorTemp) * a;
    }
    else
    {
        s->sensorTemp = s->currentTemp;
    }

    if (usedpwmval || heatpwm || fabsf(s->currentTemp - s->ambientTemp) > 0.3f
        || fabsf(s->sensorTemp - s->currentTemp) > 0.3f)
	{
        timer_mod(s->temp_tick, tNow+HEATER_TICK_MS);
	}
    else
    {
        s->is_ticking = false;
        s->currentTemp = s->ambientTemp;
        s->sensorTemp = s->ambientTemp;
    }
    qemu_set_irq(s->temp_out, s->sensorTemp*256.f);
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
        timer_mod(s->softpwm_timeout, tNow+500);
        s->pwm = tOn & 0xFF;
    }
    // Tickle timer if turned off.
    if (!s->is_ticking) // Start the heater.
    {
        timer_mod(s->temp_tick,tNow + 1);
        s->is_ticking = true;
    }
}

static void heater_apply_sock(heater_state *s)
{
    float base = ((float)s->mass10x) / 10.f;
    s->thermalMass = s->sock_on ? base * HEATER_SOCK_MASS_FACTOR : base;
}

static void heater_reset(DeviceState *dev)
{
    heater_state *s = HEATER(dev);

    heater_apply_sock(s);
    s->ambientTemp = ((float)s->ambient_x10) / 10.f;
    s->currentTemp = (s->start_temp_x10 < 0)
        ? s->ambientTemp
        : ((float)s->start_temp_x10) / 10.f;
    s->sensorTemp = s->currentTemp;
    memset(s->pwm_hist, 0, sizeof(s->pwm_hist));
    s->pwm_hist_idx = 0;
    qemu_set_irq(s->temp_out, s->currentTemp*256.f);
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
            s->sensorTemp = s->currentTemp;
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
    qdev_init_gpio_in_named(DEVICE(obj),heater_fan_change, "cooling-fan-in", FAN_COOLING_COUNT);

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

static const Property heater_properties[] = {
    DEFINE_PROP_UINT8("thermal_mass_x10",heater_state, mass10x, 25),
    DEFINE_PROP_UINT8("label",heater_state, chrLabel, (uint8_t)' '),
	DEFINE_PROP_UINT16("voltage_x100", heater_state, voltagex100, 2400),
	DEFINE_PROP_UINT16("resistance_x100", heater_state, resistancex100, 0),
	DEFINE_PROP_BOOL("has_sock", heater_state, sock_on, false),
	// Ambient is what the block cools *towards*; start_temp is where it powers on.
	DEFINE_PROP_UINT16("ambient_temp_x10", heater_state, ambient_x10, 250),
	DEFINE_PROP_INT32("start_temp_x10", heater_state, start_temp_x10, -1),
	// 200 s is the generic decay for a machine with no measured constant.
	DEFINE_PROP_UINT16("cool_tau_s", heater_state, cool_tau_s, 200),
	// The following default to "not characterized" and are ignored if zero.
	DEFINE_PROP_UINT16("cool_tau_pfan_s", heater_state, cool_tau_pfan_s, 0),
	DEFINE_PROP_UINT16("cool_tau_hbr_s", heater_state, cool_tau_hbr_s, 0),
	DEFINE_PROP_UINT16("heat_lag_ms", heater_state, heat_lag_ms, 0),
	DEFINE_PROP_UINT16("dead_time_ms", heater_state, dead_time_ms, 0),
};

static int heater_pre_save(void *opaque) {
    heater_state *s = HEATER(opaque);
    s->ambient_x100 = 100.f * s->ambientTemp;
    s->current_x100 = 100.f * s->currentTemp;
    s->sensor_x100 = 100.f * s->sensorTemp;
    return 0;
}

static int heater_post_load(void *opaque, int version) {
    heater_state *s = HEATER(opaque);
    s->ambientTemp = (float)s->ambient_x100/100.f;
    s->currentTemp = (float)s->current_x100/100.f;
    // v1 streams predate the sensor node; start it on the block.
    s->sensorTemp = (version < 2) ? s->currentTemp : (float)s->sensor_x100/100.f;
    heater_apply_sock(s);
    return 0;
}

static const VMStateDescription vmstate_heater = {
    .name = TYPE_HEATER,
    .version_id = 2,
    .minimum_version_id = 1,
    .pre_save = heater_pre_save,
    .post_load = heater_post_load,
    .fields = (const VMStateField[]) {
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
        VMSTATE_INT32_V(sensor_x100,heater_state,2),
        VMSTATE_UINT8_ARRAY_V(fan_duty,heater_state,FAN_COOLING_COUNT,2),
        VMSTATE_UINT16_ARRAY_V(pwm_hist,heater_state,HEATER_PWM_HIST,2),
        VMSTATE_UINT8_V(pwm_hist_idx,heater_state,2),
        VMSTATE_END_OF_LIST()
    }
};


static void heater_realize(DeviceState *dev, Error **errp)
{
    heater_state *s = HEATER(dev);
    if (s->cool_tau_s == 0)
    {
        error_setg(errp, "cool_tau_s must be non-zero (it is a divisor)");
        return;
    }
    const uint32_t max_dead_ms = (HEATER_PWM_HIST - 1u) * HEATER_TICK_MS;
    if (s->dead_time_ms > max_dead_ms)
    {
        error_setg(errp, "dead_time_ms %u exceeds the %u ms the PWM history holds",
                   s->dead_time_ms, max_dead_ms);
        return;
    }
    heater_apply_sock(s);
}

static void heater_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, heater_reset);
    dc->realize = heater_realize;
    dc->vmsd = &vmstate_heater;
    device_class_set_props(dc, heater_properties);

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = heater_process_action;

}
