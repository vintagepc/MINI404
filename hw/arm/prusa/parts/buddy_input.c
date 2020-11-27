/*
 * QEMU ADB keyboard support
 *
 * Copyright (c) 2004 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_BUDDY_INPUT "buddy-input"

OBJECT_DECLARE_SIMPLE_TYPE(inputState, BUDDY_INPUT)

struct inputState {
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

    QEMUTimer *timer;
};



static void buddy_input_keyevent(inputState *s, int keycode)
{
    // TODO, keyboard events.
}

static void buddy_input_timer_expire(void *opaque)
{
    static const uint8_t buddy_input_phases[4] = {0x00, 0x10, 0x11, 0x01};
    inputState *s = opaque;
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
    inputState *s = opaque;
    int changed = buttons_state^s->last_state;
    if (changed & MOUSE_EVENT_LBUTTON)
    {
        // printf("Button\n");
        qemu_set_irq(s->irq_enc_button,(buttons_state & MOUSE_EVENT_LBUTTON)>0);
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
//         VMSTATE_STRUCT(parent_obj, inputState, 0, vmstate_adb_device, ADBDevice),
//         VMSTATE_BUFFER(data, inputState),
//         VMSTATE_INT32(rptr, inputState),
//         VMSTATE_INT32(wptr, inputState),
//         VMSTATE_INT32(count, inputState),
//         VMSTATE_END_OF_LIST()
//     }
// };

static void buddy_input_reset(DeviceState *dev)
{
    inputState *s = BUDDY_INPUT(dev);
    s->last_state = 0;
    s->phase = 0;
}

static void buddy_input_initfn(Object *obj)
{
    inputState *s = BUDDY_INPUT(obj);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_button, "buddy-enc-button", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_a, "buddy-enc-a", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_enc_b, "buddy-enc-b", 1);
    qemu_add_mouse_event_handler(&buddy_input_mouseevent,BUDDY_INPUT(obj),false, "buddy-mouse");

    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                    (QEMUTimerCB *)buddy_input_timer_expire, s);
}

static void buddy_input_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
  
    dc->reset = buddy_input_reset;
  //  dc->vmsd = &vmstate_buddy_input;
}

static const TypeInfo buddy_input_type_info = {
    .name = TYPE_BUDDY_INPUT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(inputState),
    .instance_init = buddy_input_initfn,
    .class_init = buddy_input_class_init,
};

static void buddy_input_register_types(void)
{
    type_register_static(&buddy_input_type_info);
}

type_init(buddy_input_register_types)
