/*
	thermistor.c
	Based on thermistor.c (C) 2008-2012 Michel Pollet <buserror@gmail.com>

    Rewritten for MK404/C++ in 2020 by VintagePC <https://github.com/vintagepc/>
    Backported to C again for QEMU in Mini404 in 2021-3

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
#include "hw/core/irq.h"
#include "qom/object.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "thermistortables.h"
#include "thermistor.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "../utility/macros.h"
#include "../utility/p404scriptable.h"
#include "../utility/ScriptHost_C.h"
#include <math.h>

#define TYPE_THERMISTOR "thermistor"
OBJECT_DECLARE_SIMPLE_TYPE(ThermistorState, THERMISTOR)

struct ThermistorState {
    SysBusDevice parent;

    qemu_irq irq_value, value_x1000, temp_out;

    uint8_t index;
    uint16_t table_index;
    const short int *table;
    int table_length;
    float temperature;
    int8_t oversampling;
    uint16_t start_temp;
    int32_t raw_adc10;   // property: <0 = follow the sensor curve; >=0 = hold this reading (e.g. a fault)
    int32_t override_reading;
    uint8_t noise_adc10; // peak ADC jitter in 10-bit counts; 0 = noiseless (default)
    uint32_t noise_seed;
    uint32_t noise_rng;
    QEMUTimer *noise_tick;

    // Saving state - because there's no VMSTATE_FLOAT
    int32_t temp_256x;

	script_handle handle;
};

enum {
    ActShort,
    ActDisconnect,
    ActNormal,
    ActSet,
    ActGetTemp,
    ActHoldRaw,
    ActReleaseRaw,
};

#define KELVIN_OFFSET 273.15f

static int map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Deterministic ADC noise.
   The real ADC reading jitters; This adds a bounded,
   repeatable perturbation so a classifier's *margin* can be exercised.
   Never applied to a raw reading, only input temperatures. 

   xorshift32 rather than the host RNG so a run is reproducible from the seed. */
static int thermistor_noise(ThermistorState *s)
{
    if (!s->noise_adc10) {
        return 0;
    }
    uint32_t x = s->noise_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s->noise_rng = x;
    const int span = (2 * (int)s->noise_adc10) + 1;
    return ((int)(x % (uint32_t)span) - (int)s->noise_adc10) << 2; /* 10-bit -> 12-bit */
}

/* Convert a temperature to ADC counts.
   Returns <0 if the temperature is off the end of the
   table. */
static int thermistor_reading_for_temp(ThermistorState *s, float temp_c)
{
    if (s->table_index == 0) {
        return s->start_temp;
    }
    if (s->table_index == 65535) { // Modbed conversion map:
        // reversal of how the temperature is calculated by the firmware...
        temp_c += KELVIN_OFFSET;
        temp_c = 1.0f/temp_c;
        temp_c -= (1.0f / (25.0f + KELVIN_OFFSET));
        temp_c *= 4573.f;
        temp_c = exp(temp_c);
        temp_c *= 100000; // should now have resistance.
        return (4095.f * temp_c) / (temp_c + 4700);
    }
    if (s->table_index == 65534) { // ADC val = 10x C table
        return temp_c * 10;
    }
    for (uint16_t i = 0; i < s->table_length; i += 2) {
        if (s->table[i+1] <= temp_c) {
            uint16_t tt = s->table[i];
            /* small linear regression between table samples */
            if (i != 0 && s->table[i+1] < temp_c) {
                int16_t d_adc = s->table[i] - s->table[i-2];
                float d_temp = s->table[i+1] - s->table[i-1];
                float delta = temp_c - s->table[i+1];
                tt = s->table[i] + (d_adc * (delta / d_temp));
            }
            int ivalue = tt / s->oversampling;
            ivalue <<= 2; // Note - ADC takes full 12 bit input, but the tables are only 10-bit
            return ivalue;
        }
    }
    return -1;
}

static void thermistor_update_output(ThermistorState *s) {
    /* temp_out is the block's real temperature. An override models a *sensor*
       fault which does not affect what it's actually doing. */
    qemu_set_irq(s->temp_out, 256U * s->temperature);

    int ivalue = (s->override_reading >= 0)
        ? s->override_reading
        : thermistor_reading_for_temp(s, s->temperature);
    if (ivalue < 0) {
        return;     // off the end of the table; leave the previous reading alone
    }

    // Noise is a sensor property, not an ADC one, so don't noise a raw reading.
    if (s->override_reading < 0 && s->table_index != 0 && s->table_index < 65534) {
        ivalue += thermistor_noise(s);
        if (ivalue < 0) {
            ivalue = 0;
        } else if (ivalue > 0xFFF) {
            ivalue = 0xFFF;
        }
    }
    if (s->table_index == 0 || s->table_index >= 65534) {
        qemu_set_irq(s->irq_value, ivalue);
        return;
    }
    if (s->table_index == 21) {  // Special case for PT100 on HX717 - map ADC value to direct reading.
        ivalue = map(ivalue, 0, 0x3FF, -980000, 2070000) * 125;
    }
    qemu_set_irq(s->irq_value, ivalue);
    qemu_set_irq(s->value_x1000, ivalue);
}

// For efficiency, IRQs only propagate on change. Noise invalidates that assumption
// so we need a timer to have it actually show.
#define THERMISTOR_NOISE_TICK_MS 10

static void thermistor_noise_tick(void *opaque)
{
    ThermistorState *s = opaque;
    thermistor_update_output(s);
    timer_mod(s->noise_tick,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + THERMISTOR_NOISE_TICK_MS);
}

static void thermistor_temp_in(void *opaque, int n, int level)
{
	ThermistorState *s = opaque;
	float fv = (float)(level) / 256.f;
	s->temperature = fv;
	thermistor_update_output(s);
}

static int thermistor_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    ThermistorState *s = THERMISTOR(obj);
    switch (action)
    {
        // Short and D/C are electrical faults
        case ActShort:
            s->override_reading = 0;
            break;
        case ActDisconnect:
            s->override_reading = 0x3FF << 2;
            break;
        case ActNormal:
            s->override_reading = -1;
            break;
        case ActSet:
        {
            const int r = thermistor_reading_for_temp(s, scripthost_get_float(args, 0));
            if (r < 0)
            {
                printf("# thermistor: SetTemp is off the end of table %u; ignored\n",
                       s->table_index);
                return ScriptLS_Finished;
            }
            s->override_reading = r;
            break;
        }
        case ActGetTemp:
            script_print_float(s->temperature);
            break;
        case ActHoldRaw:
            s->override_reading = (scripthost_get_int(args, 0) & 0x3FF) << 2;
            break;
        case ActReleaseRaw:
            s->override_reading = -1;
            break;
        default:
            return ScriptLS_Unhandled;

    }
    thermistor_update_output(s);
    return ScriptLS_Finished;
}

// Table for a given table number, or NULL. len is in shorts (2 per row).
static const short* thermistor_table_for(uint16_t table_no, int *len);

bool thermistor_temp_for_adc10(uint16_t table_no, uint16_t adc10, float *temp_out)
{
    int len = 0;
    const short *t = thermistor_table_for(table_no, &len);
    if (!t || len < 4)
    {
        return false;
    }
    // Rows are ordered descending by temperature; ADC moves monotonically with
    // the row index in whichever direction the sensor happens to run, so bracket
    // on ADC and interpolate the temperature between the two rows.
    const int raw = adc10 * OVERSAMPLENR;
    for (int i = 2; i < len; i += 2)
    {
        const int a0 = t[i-2], a1 = t[i];
        const int lo = a0 < a1 ? a0 : a1, hi = a0 < a1 ? a1 : a0;
        if (raw >= lo && raw <= hi)
        {
            const int d_adc = a1 - a0;
            const float d_temp = t[i+1] - t[i-1];
            *temp_out = d_adc ? (t[i-1] + d_temp * ((float)(raw - a0) / (float)d_adc))
                              : (float)t[i-1];
            return true;
        }
    }
    return false;
}

static const short* thermistor_table_for(uint16_t table_no, int *len)
{
    const short *table = NULL;
    *len = 0;
    switch (table_no)
    {
        case 1:
            *len = 2*BEDTEMPTABLE_LEN;
            table = &temptable_1[0][0];
            break;
        case 5:
            table = &temptable_5[0][0];
            *len = 2*HEATER_0_TEMPTABLE_LEN;
            break;
        case 2000:
            *len = 2*AMBIENTTEMPTABLE_LEN;
            table = &temptable_2000[0][0];
            break;
		case 2004:
            *len = 17*2;
            table = &temptable_2004[0][0];
			break;
        case 2005:
            *len = 22*2;
            table = &temptable_2005[0][0];
            break;
        case 1010: // PT1000, HT hotend. Reversed characteristic: ADC rises with temp.
            *len = 20*2;
            table = &temptable_1010_ht[0][0];
            break;
        case 21:
            *len = 6;
            table = &temptable_21[0][0];
            break;
        case 22:
            *len = 4;
            table = &temptable_22[0][0];
            break;
		case 2006:
			*len = 2U*127;
			table = &temptable_2006[0][0];
			break;
		case 2007:
			*len = 2U*291;
			table = &temptable_2007[0][0];
			break;
		case 2008:
			*len = 2U*205;
			table = &temptable_2008[0][0];
			break;
		case 2009:
			*len = 2U*13;
			table = &temptable_2009[0][0];
			break;
        default:
			printf("# %s WARNING: Unhandled thermistor table %u!\n",__FILE__,table_no);
			/* FALLTHRU */
		case UINT16_MAX-1:
		case UINT16_MAX:
            table = NULL;
            *len = 0;
            break;
    }
    return table;
}

static void thermistor_set_table(ThermistorState *s) {
    s->table = thermistor_table_for(s->table_index, &s->table_length);
    thermistor_update_output(s);
}

static void thermistor_reset(DeviceState *dev)
{
    ThermistorState *s = THERMISTOR(dev);
    s->temperature = s->start_temp;
    s->override_reading = (s->raw_adc10 >= 0) ? (s->raw_adc10 << 2) : -1;
    // Reseeded on reset so a scenario replays identically. xorshift32 stalls on
    // zero, silently disabling noise.
    s->noise_rng = s->noise_seed ? s->noise_seed : 1u;
    if (s->noise_adc10 && s->raw_adc10 < 0) {
        timer_mod(s->noise_tick,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + THERMISTOR_NOISE_TICK_MS);
    } else {
        timer_del(s->noise_tick);
    }
    thermistor_set_table(s);
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(ThermistorState, thermistor, THERMISTOR, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL});

static void thermistor_finalize(Object *obj)
{

}

static void thermistor_init(Object *obj)
{
    ThermistorState *s = THERMISTOR(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_value, "thermistor_value", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->value_x1000, "value_x1000", 1);

    qdev_init_gpio_out_named(DEVICE(obj), &s->temp_out, "temp_out_256x", 1);

    // qdev_init_gpio_in_named(DEVICE(obj),thermistor_read_request, "thermistor_read_request", 1);
    qdev_init_gpio_in_named(DEVICE(obj),thermistor_temp_in, "thermistor_set_temperature", 1);

    s->oversampling = OVERSAMPLENR;

    s->noise_tick = timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)thermistor_noise_tick, s);

    s->handle = script_instance_new(P404_SCRIPTABLE(obj), TYPE_THERMISTOR);
    script_register_action(s->handle, "Short","Shorts the thermistor",ActShort);
    script_register_action(s->handle, "Disconnect","Disconnects the thermistor",ActDisconnect);
    script_register_action(s->handle, "Restore","Restores the thermistor to normal (unshorted, heater-operated) state",ActNormal);
    script_register_action(s->handle, "Set","Sets the temperature to a given value.",ActSet);
    script_add_arg_float(s->handle, ActSet);

    script_register_action(s->handle, "GetTemp","Prints the current temperature",ActGetTemp);

    script_register_action(s->handle, "HoldRaw",
        "Pins the ADC reading (0-1023, 10-bit) regardless of probe temperature",ActHoldRaw);
    script_add_arg_int(s->handle, ActHoldRaw);
    script_register_action(s->handle, "ReleaseRaw",
        "Releases a HoldRaw and returns to the sensor curve",ActReleaseRaw);

    scripthost_register_scriptable(s->handle);
}

static const Property thermistor_properties[] = {
    DEFINE_PROP_UINT16("temp", ThermistorState, start_temp,0),
    DEFINE_PROP_UINT16("table_no", ThermistorState, table_index, 0),
    DEFINE_PROP_UINT8("index", ThermistorState, index, 0),
    DEFINE_PROP_INT32("raw_adc10", ThermistorState, raw_adc10, -1),
    DEFINE_PROP_UINT8("noise_adc10", ThermistorState, noise_adc10, 0),
    DEFINE_PROP_UINT32("noise_seed", ThermistorState, noise_seed, 1),
};


static int thermistor_post_load(void *opaque, int version_id)
{
    ThermistorState *s = THERMISTOR(opaque);

    thermistor_set_table(s);
    if (s->table_index >0 && (s->table_length == 0 || s->table == NULL)) {
        return -EINVAL;
    }

    return 0;
}

static int thermistor_pre_save(void *opaque)
{
    ThermistorState *s = THERMISTOR(opaque);

    s->temp_256x = (s->temperature*256.f);
    return 0;
}

static const VMStateDescription vmstate_thermistor = {
    .name = TYPE_THERMISTOR,
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = thermistor_post_load,
    .pre_save = thermistor_pre_save,
    .fields      = (VMStateField []) {
        VMSTATE_UINT16(table_index,ThermistorState),
        VMSTATE_INT32(temp_256x,ThermistorState),
        VMSTATE_INT32_V(override_reading,ThermistorState,2),
        VMSTATE_INT8(oversampling,ThermistorState),
        VMSTATE_UINT16(start_temp,ThermistorState),
        VMSTATE_END_OF_LIST(),
    }
};

static void thermistor_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, thermistor_reset);
    dc->vmsd = &vmstate_thermistor;
    device_class_set_props(dc, thermistor_properties);

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = thermistor_process_action;
}
