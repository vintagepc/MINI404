/*
    buddy_input.c - Knob and reset button input handler for
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
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_BUDDY_INPUT "buddy-input"

OBJECT_DECLARE_SIMPLE_TYPE(InputState, BUDDY_INPUT)

struct InputState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    qemu_irq irq_enc_button;
    qemu_irq irq_enc_a;
    qemu_irq irq_enc_b;
    qemu_irq irq_rst;
    int last_state;
    uint8_t phase;

    int32_t encoder_ticks;
    int8_t encoder_dir;

    QEMUTimer *timer, *release, *scripting;
};

enum {
    ACT_TWIST, 
    ACT_PUSH,
    ACT_RESET
};

static void buddy_input_keyevent(void *opaque, int keycode)
{
    InputState *s = opaque;
    int dir = 0;
    // printf("Key: %04x\n",keycode);
    switch (keycode)
    {
        case 0x50: // down
            dir = 1;
            break;
        case 0x48: // up
            dir = -1;
            break;
        case 0x1c: // enter
            qemu_set_irq(s->irq_enc_button,0);
            timer_mod(s->release, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
            // printf("return\n");
            break;

    }
    if (keycode == QEMU_KEY_UP)
    {
        dir = 1;
    } else if (keycode == QEMU_KEY_DOWN) {
        dir = -1;
    } 
    if (dir) {
        if (s->encoder_dir != dir) // direction change
        {
            s->encoder_ticks = 0;
            s->encoder_dir=dir;
        }
        s->encoder_ticks +=2;
        timer_mod(s->timer,  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
}

static void buddy_script_timer_expire(void *opaque)
{
    InputState *s = opaque;
    scripthost_run(qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    timer_mod(s->scripting, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+10);
}

static void buddy_autorelease_timer_expire(void *opaque)
{
    InputState *s = opaque;
    qemu_set_irq(s->irq_enc_button,1);
}

static void buddy_input_timer_expire(void *opaque)
{
    static const uint8_t buddy_input_phases[4] = {0x00, 0x10, 0x11, 0x01};
    InputState *s = opaque;
    if (s->encoder_dir<0){
        s->phase++;
    } else {
        s->phase+=3;
    }
    s->phase = s->phase%4;
    qemu_set_irq(s->irq_enc_a, (buddy_input_phases[s->phase]&0xF0)>0);
    qemu_set_irq(s->irq_enc_b, (buddy_input_phases[s->phase]&0x0F)>0);
    s->encoder_ticks--;

    if (s->encoder_ticks>0)
    {
        timer_mod(s->timer,  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
    }

}

static void buddy_input_mouseevent(void *opaque, int dx, int dy, int dz, int buttons_state)
{
    InputState *s = opaque;
    int changed = buttons_state^s->last_state;
    if (changed & MOUSE_EVENT_LBUTTON)
    {
        // printf("Button\n");
        qemu_set_irq(s->irq_enc_button,(buttons_state & MOUSE_EVENT_LBUTTON)==0);
    }
    if (dz) // Mouse wheel motion.
    {
        if (s->encoder_dir != dz) // direction change
        {
            s->encoder_ticks = 0;
            s->encoder_dir=dz;
        }
        s->encoder_ticks +=2;
        timer_mod(s->timer,  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
        // printf("phase: %d %d\n",(buddy_input_phases[s->phase]&0xF)>0,(buddy_input_phases[s->phase]&0x0F)>0);
    }
    if (changed)
    {
        s->last_state = buttons_state;
    }

}

// static const VMStateDescription vmstate_buddy_input = {
//     .name = "buddy_input",
//     .version_id = 2,
//     .minimum_version_id = 2,
//     .fields = (VMStateField[]) {
//         VMSTATE_STRUCT(parent_obj, InputState, 0, vmstate_adb_device, ADBDevice),
//         VMSTATE_BUFFER(data, InputState),
//         VMSTATE_INT32(rptr, InputState),
//         VMSTATE_INT32(wptr, InputState),
//         VMSTATE_INT32(count, InputState),
//         VMSTATE_END_OF_LIST()
//     }
// };

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(InputState, buddy_input, BUDDY_INPUT, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {NULL})


static void buddy_input_finalize(Object *obj)
{
    printf("Input_finalize\n");
}

static void buddy_input_reset(DeviceState *dev)
{
    InputState *s = BUDDY_INPUT(dev);
    s->last_state = 0;
    s->phase = 0;
}

int buddy_input_process_action(P404ScriptIF *obj, unsigned int action, const void* args);
int buddy_input_process_action(P404ScriptIF *obj, unsigned int action, const void* args)
{
    InputState *s = BUDDY_INPUT(obj);
    switch (action)
    {
        case ACT_TWIST:
            buddy_input_keyevent(s, QEMU_KEY_UP);
            printf("Pushed!\n");
            break;
            break;
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;
}

static void buddy_input_init(Object *obj)
{
    InputState *s = BUDDY_INPUT(obj);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_button, "buddy-enc-button", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_a, "buddy-enc-a", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_b, "buddy-enc-b", 1);
    qemu_add_mouse_event_handler(&buddy_input_mouseevent,BUDDY_INPUT(obj),false, "buddy-mouse");
    qemu_add_kbd_event_handler(&buddy_input_keyevent,s);

    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                    (QEMUTimerCB *)buddy_input_timer_expire, s);
    s->release = timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)buddy_autorelease_timer_expire, s);

    s->scripting = timer_new_ms(QEMU_CLOCK_VIRTUAL,
        (QEMUTimerCB *)buddy_script_timer_expire, s);

    void *pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_BUDDY_INPUT);

    script_register_action(pScript, "Twist", "Twists the encoder up/down", ACT_TWIST);

    scripthost_register_scriptable(pScript);

    if (scripthost_setup("script.txt")) // TODO- external file
    {
        // Start script timer
        timer_mod(s->scripting,  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
    }
}

static void buddy_input_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = buddy_input_reset;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = buddy_input_process_action;
  //  dc->vmsd = &vmstate_buddy_input;
}
