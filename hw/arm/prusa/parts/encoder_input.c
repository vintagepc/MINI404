/*
    encoder_input.c - Knob and reset button input handler for
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
#include "hw/irq.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "sysemu/runstate.h"
#include "qapi/qapi-commands-run-state.h"
#include "qapi/qapi-events-run-state.h"

#define TYPE_ENCODER_INPUT "encoder-input"

OBJECT_DECLARE_SIMPLE_TYPE(InputState, ENCODER_INPUT)

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

    QEMUTimer *timer, *release;
};

enum {
    ACT_TWIST, 
    ACT_PUSH,
    ACT_RESET
};

static void encoder_input_handle_key(P404KeyIF *opaque, Key keycode)
{
    InputState *s = ENCODER_INPUT(opaque);
    int dir = 0;
    // printf("Key: %04x\n",keycode);
    switch (keycode)
    {
        case 's': // down
            dir = 1;
            break;
        case 'w': // up
            dir = -1;
            break;
        case 13: // enter
            qemu_set_irq(s->irq_enc_button,0);
            timer_mod(s->release, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
            // printf("return\n");
            break;

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

static void buddy_autorelease_timer_expire(void *opaque)
{
    InputState *s = opaque;
    qemu_set_irq(s->irq_enc_button,1);
}

static void encoder_input_timer_expire(void *opaque)
{
    static const uint8_t encoder_input_phases[4] = {0x11, 0x01, 0x00, 0x10};
    InputState *s = opaque;
    if (s->encoder_dir<0){
        s->phase++;
    } else {
        s->phase+=3;
    }
    s->phase = s->phase%4;
    qemu_set_irq(s->irq_enc_a, (encoder_input_phases[s->phase]&0xF0)>0);
    qemu_set_irq(s->irq_enc_b, (encoder_input_phases[s->phase]&0x0F)>0);
    s->encoder_ticks--;

    if (s->encoder_ticks>0)
    {
        timer_mod(s->timer,  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
    }

}

static void encoder_input_mouseevent(void *opaque, int dx, int dy, int dz, int buttons_state)
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
        // printf("phase: %d %d\n",(encoder_input_phases[s->phase]&0xF)>0,(encoder_input_phases[s->phase]&0x0F)>0);
    }
    if (changed)
    {
        s->last_state = buttons_state;
    }

}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(InputState, encoder_input, ENCODER_INPUT, SYS_BUS_DEVICE, {TYPE_P404_SCRIPTABLE}, {TYPE_P404_KEYCLIENT}, {NULL})

static void encoder_input_finalize(Object *obj)
{
    printf("Input_finalize\n");
}

static void encoder_input_reset(DeviceState *dev)
{
    InputState *s = ENCODER_INPUT(dev);
    s->last_state = 0;
    s->phase = 0;
    qemu_irq_lower(s->irq_enc_a);
    qemu_irq_lower(s->irq_enc_b);
}

static int encoder_input_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    switch (action)
    {
        case ACT_TWIST:
        {
            int dir = scripthost_get_int(args, 0);
            Key keycode = dir < 0 ? 's' : 'w';
            for (dir = abs(dir); dir > 0; dir--) {
                encoder_input_handle_key(P404_KEYCLIENT(obj), keycode);
            }
            break;
        }
        case ACT_PUSH:
            encoder_input_handle_key(P404_KEYCLIENT(obj), 0xd);
            break;
        case ACT_RESET:
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            break;
        default:
            return ScriptLS_Unhandled;
    }
    return ScriptLS_Finished;
}

static void encoder_input_init(Object *obj)
{
    InputState *s = ENCODER_INPUT(obj);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_button, "encoder-button", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_a, "encoder-a", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_b, "encoder-b", 1);
    qemu_add_mouse_event_handler(&encoder_input_mouseevent,ENCODER_INPUT(obj),false, "encoder-mouse");
    // qemu_add_kbd_event_handler(&encoder_input_keyevent,s);

    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                    (QEMUTimerCB *)encoder_input_timer_expire, s);
    s->release = timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)buddy_autorelease_timer_expire, s);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(obj), TYPE_ENCODER_INPUT);

    script_register_action(pScript, "Twist", "Twists the encoder up(1)/down(-1)", ACT_TWIST);
    script_add_arg_int(pScript, ACT_TWIST);
    script_register_action(pScript, "Push",  "Presses the encoder", ACT_PUSH);
    script_register_action(pScript, "Reset", "Resets the printer", ACT_RESET);

    scripthost_register_scriptable(pScript);

    p404_key_handle pKey = p404_new_keyhandler(P404_KEYCLIENT(obj));
    p404_register_keyhandler(pKey, 'w',"Twists encoder up");
    p404_register_keyhandler(pKey, 's',"Twists encoder down");
    p404_register_keyhandler(pKey, 0xd,"Presses encoder button");

}

static int encoder_post_load(void *opaque, int version_id)
{
    InputState *s = ENCODER_INPUT(opaque);

    if (s->phase>3) {
        return -EINVAL;
    }
    if (s->encoder_dir <-1 || s->encoder_dir>1)
        return -EINVAL;

    return 0;
}

static const VMStateDescription vmstate_encoder_input = {
    .name = TYPE_ENCODER_INPUT,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = encoder_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_INT32(last_state, InputState),
        VMSTATE_UINT8(phase,InputState),
        VMSTATE_INT32(encoder_ticks,InputState),
        VMSTATE_INT8(encoder_dir,InputState),
        VMSTATE_TIMER_PTR(timer,InputState),
        VMSTATE_TIMER_PTR(release,InputState),
        VMSTATE_END_OF_LIST()
    }
};

static void encoder_input_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = encoder_input_reset;
    // dc->realize = encoder_input_realize;
    // dc->unrealize = encoder_input_unrealize;
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(oc);
    sc->ScriptHandler = encoder_input_process_action;

    P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(oc);
    kc->KeyHandler = encoder_input_handle_key;

    dc->vmsd = &vmstate_encoder_input;
}
