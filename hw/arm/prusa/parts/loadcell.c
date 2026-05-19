/*
    loadcell.c - Sim loadcell sensor for
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
#include "../utility/p404_keyclient.h"
#include "../utility/macros.h"
#include "../utility/ScriptHost_C.h"
#include "../utility/ArgHelper.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "qom/object.h"
#include "hw/core/sysbus.h"
#include "math.h"
#define TYPE_LOADCELL "loadcell"

OBJECT_DECLARE_SIMPLE_TYPE(LoadcellState, LOADCELL)

struct LoadcellState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    qemu_irq irq; //
    bool is_zero;
    int last_pos;
    int32_t pos_um[3];

    bool calpin_present;

	uint8_t tap;
	p404_key_handle key;
    script_handle handle;
	QEMUTimer* timer;

	/* Parameterized tap (loadcell::Tap script action). Independent state so the
	 * legacy KeyCtl::Key('t') path keeps working unchanged. */
	int32_t ptap_peak_raw;
	int32_t ptap_total_ticks;
	int32_t ptap_current_tick;
	uint8_t ptap_state;          /* 0 = idle, 1 = running, 2 = just-finished (awaiting acknowledge) */
	QEMUTimer* ptap_timer;
};

/* Sim-emit raw value → FW grams chain (MK4):
 *   1) loadcell qemu_set_irq(value)      (this file, "input_x1000" output)
 *   2) hx717 input_x1000:  value_gain = value * 128 / 1000   (parts/hx717.c)
 *   3) FW Loadcell::load = (loadcellRaw - tare_offset) * scale,
 *      with scale = 0.0192 g/count  (Buddy/src/common/loadcell.hpp)
 *   4) selftest_loadcell.cpp: load = -1 * Loadcell::load()   (positive = up)
 *
 * So: FW_grams ≈ -value * 128/1000 * 0.0192  ≈  -value / 407
 * Inverting for `loadcell::Tap(force_grams, ...)`:
 *   peak_raw = -force_grams * 407   (negative because positive force = up = -ve raw)
 *
 * Current FW selftest expects load in [500, 2000] g (selftest_MK4.cpp tap_*),
 * so force_grams = 1000 lands mid-range. */
#define LOADCELL_GRAMS_TO_RAW 407

enum {
    ActTap = 1,
};

#define PIN_POSITION_X (180 + 8)
#define PIN_POSITION_Y (180 + 10)
#define PIN_POSITION_Z 4.5
#define PIN_DIAMETER 6

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(LoadcellState, loadcell, LOADCELL, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {TYPE_P404_KEYCLIENT}, {NULL})

static void loadcell_finalize(Object *obj)
{
}

static void loadcell_reset(DeviceState *dev)
{
    LoadcellState *s = LOADCELL(dev);
    qemu_set_irq(s->irq,10);
    s->is_zero = true;
    s->calpin_present = false;
}

static void handle_zpos_buildplate(LoadcellState *s, int level)
{
    // This is just a SWAG... loadcell will start at 0.75mm above "reference"
    // zero.
    bool dir = s->last_pos<level;
    #define START_HEIGHT 750
    if (dir) {
        if (level>START_HEIGHT && !s->is_zero) {
            qemu_set_irq(s->irq,10);
            s->is_zero = true;
        } else if (level <=START_HEIGHT) {
            s->is_zero = false;
            int val_out = START_HEIGHT-(level);
            qemu_set_irq(s->irq, val_out*-250);
            // printf("LC out: %d\n",val_out);
        }
    } else {
        if (level>START_HEIGHT+20 && !s->is_zero) {
            qemu_set_irq(s->irq,10);
            s->is_zero = true;
        } else if (level <=START_HEIGHT-20) {
            s->is_zero = false;
            int val_out = START_HEIGHT-20-(level);
            qemu_set_irq(s->irq, val_out*-250);
            // printf("LC out: %d\n",val_out);
        }
    }
    s->last_pos = level;

}

static void handle_pos_calpin(LoadcellState *s)
{
    // This is just a SWAG... loadcell will start at 0.75mm above "reference"
    // zero.
    double pinPosX = 1000.f * PIN_POSITION_X;
    double pinPosY = 1000.f * PIN_POSITION_Y;
    double pinPosZ = 1000.f * PIN_POSITION_Z;
    double pinDiameter = 1000.f * PIN_DIAMETER;

    double posX = s->pos_um[0];
    double posY = s->pos_um[1];
    double posZ = s->pos_um[2];

    float xy_distance = sqrt(pow(posX - pinPosX, 2) + pow(posY - pinPosY, 2));
    float z_distance = posZ - pinPosZ;
    float final_distance = 0;
    if (z_distance < 0)
    {
        final_distance = xy_distance - (pinDiameter / 2);
    }
    else
    {
        final_distance = z_distance;
    }
    printf("Cal pin distance: %f %f\n",z_distance, final_distance);
    // Sort of a cheat, we re-use the Z-only logic but instead feed it a 
    // distance based off of 3 dimensions rather than just Z.
    handle_zpos_buildplate(s, final_distance);
}

static void loadcell_cal_pin_in(void *opaque, int n, int level){
    LoadcellState *s = LOADCELL(opaque);
    s->calpin_present = level;
    printf("Cal pin %s\n", level? "Installed":"Removed");
}

static void loadcell_pos_in(void *opaque, int n, int level){
    LoadcellState *s = LOADCELL(opaque);
    s->pos_um[n] = level;

    if (s->calpin_present)
    {
        handle_pos_calpin(s);
    }
    else if (n == 2) // Only do buildplate if we get a z update
    {
        // printf("Z pos: %d\n", level);
        handle_zpos_buildplate(s, level);
    }
}

static void loadcell_tap_timer(void *opaque)
{
    LoadcellState *s = LOADCELL(opaque);
	uint32_t value = 0;
	if (s->tap < 25)
	{
		value = s->tap*-20000;
	}
	else
	{
		value = (50-s->tap)*-20000;
	}
	qemu_set_irq(s->irq, value);
	s->tap++;

	if (s->tap > 50)
	{
		s->tap = 0;
		qemu_set_irq(s->irq, 10);

	}

	if (s->tap)
	{
		timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+10);
	}
}

static void loadcell_input_handle_key(P404KeyIF *opaque, Key keycode)
{
    LoadcellState *s = LOADCELL(opaque);
    if (keycode == 't')
	{
		printf("Tapped loadcell\n");
		timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+1);
	}
}

static void loadcell_param_tap_timer(void *opaque)
{
    LoadcellState *s = LOADCELL(opaque);
    int32_t total = s->ptap_total_ticks;
    int32_t t = s->ptap_current_tick;

    if (total < 2 || t > total) {
        /* Pulse complete — drop force back to idle and signal the script-side action. */
        qemu_set_irq(s->irq, 10);
        s->ptap_state = 2;
        return;
    }

    int32_t half = total / 2;
    int32_t value;
    if (half <= 0) {
        value = s->ptap_peak_raw;
    } else if (t <= half) {
        value = (int32_t)(((int64_t)s->ptap_peak_raw * t) / half);
    } else {
        int32_t down = total - t;
        if (down < 0) down = 0;
        value = (int32_t)(((int64_t)s->ptap_peak_raw * down) / half);
    }

    qemu_set_irq(s->irq, value);
    s->ptap_current_tick = t + 1;
    timer_mod(s->ptap_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
}

static int loadcell_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    LoadcellState *s = LOADCELL(obj);
    switch (action)
    {
        case ActTap:
            switch (s->ptap_state) {
                case 1: /* running */
                    return ScriptLS_Waiting;
                case 2: /* just finished */
                    s->ptap_state = 0;
                    s->ptap_total_ticks = 0;
                    s->ptap_current_tick = 0;
                    return ScriptLS_Finished;
                case 0:
                default: {
                    int force_grams = scripthost_get_int(args, 0);
                    int duration_ms = scripthost_get_int(args, 1);
                    if (duration_ms < 20) {
                        duration_ms = 20;
                    }
                    /* FW polarity: positive force_grams = "tap pushing nozzle up" => negative raw. */
                    s->ptap_peak_raw = -force_grams * LOADCELL_GRAMS_TO_RAW;
                    s->ptap_total_ticks = duration_ms / 10;
                    s->ptap_current_tick = 1;
                    s->ptap_state = 1;
                    printf("Loadcell parameterized tap: %d g over %d ms (peak raw %d)\n",
                           force_grams, duration_ms, s->ptap_peak_raw);
                    timer_mod(s->ptap_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
                    return ScriptLS_Waiting;
                }
            }
        default:
            return ScriptLS_Unhandled;
    }
}

static void loadcell_init(Object *obj)
{
    LoadcellState *s = LOADCELL(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in(DEVICE(obj), loadcell_pos_in,3);
    qdev_init_gpio_in_named(DEVICE(obj), loadcell_cal_pin_in,"cal-pin-state", 1);

    s->handle = script_instance_new(P404_SCRIPTABLE(obj), TYPE_LOADCELL);
    script_register_action(s->handle, "Tap",
        "Apply a force pulse of <peak_grams> over <duration_ms> (triangular ramp), synchronously.",
        ActTap);
    script_add_arg_int(s->handle, ActTap);
    script_add_arg_int(s->handle, ActTap);
    scripthost_register_scriptable(s->handle);

	s->key = p404_new_keyhandler(P404_KEYCLIENT(obj));
    p404_register_keyhandler(s->key, 't',"Taps the loadcell");

	s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, loadcell_tap_timer, s);
	s->ptap_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, loadcell_param_tap_timer, s);
	s->ptap_state = 0;
	s->ptap_total_ticks = 0;
	s->ptap_current_tick = 0;
	s->ptap_peak_raw = 0;
}

static const VMStateDescription vmstate_loadcell = {
    .name = TYPE_LOADCELL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(is_zero,LoadcellState),
        VMSTATE_INT32(last_pos, LoadcellState),
        VMSTATE_END_OF_LIST()
    }
};

static void loadcell_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, loadcell_reset);
    dc->vmsd = &vmstate_loadcell;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = loadcell_process_action;

	P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(oc);
	kc->KeyHandler = loadcell_input_handle_key;
}
