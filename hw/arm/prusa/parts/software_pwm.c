/*
 * Software/bitbanged PWM interpreter module.
 *
 * Written for Mini404 in 2022-3 by VintagePC <https://github.com/vintagepc/>
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
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "hw/sysbus.h"
#include "hw/irq.h"

#define TYPE_SOFTWARE_PWM "software-pwm"
#define LINE_COUNT 16

OBJECT_DECLARE_SIMPLE_TYPE(SoftwarePWMState, SOFTWARE_PWM);

typedef struct SoftwarePWMState {
    /* <private> */
    SysBusDevice parent;

    /* <public> */

	uint8_t state_count[LINE_COUNT];
	uint8_t clock_count;
	uint8_t last_pwm[LINE_COUNT];
	uint16_t line_state;

	qemu_irq pwm[LINE_COUNT];

} SoftwarePWMState;


static void software_pwm_reset(DeviceState *dev)
{
    SoftwarePWMState *s = SOFTWARE_PWM(dev);
	for(int i=0; i<LINE_COUNT; i++)
	{
		s->state_count[i] = 0;
		s->line_state = 0;
	}
	s->clock_count = 0;
}


static void software_pwm_tick(void *opaque, int n, int level)
{
    SoftwarePWMState *s = SOFTWARE_PWM(opaque);
	s->clock_count++; // Will overflow and roll back to 0 at 256.
	for(int i=0; i<LINE_COUNT; i++)
	{
		s->state_count[i] += (s->line_state>>i) & 1U;
		if (s->clock_count == 255)
		{
			if (s->last_pwm[i] != s->state_count[i])
			{
				// printf("Software PWM %d: current %u\n",i,  s->state_count[i]);
				qemu_set_irq(s->pwm[i], s->state_count[i]);
			}
			s->last_pwm[i] = s->state_count[i];
			s->state_count[i] = 0;
		}
	}
	if (s->clock_count == 255)
	{
		s->clock_count = 0;
	}

}

static void software_pwm_line(void *opaque, int n, int level)
{
    SoftwarePWMState *s = SOFTWARE_PWM(opaque);
	// printf("MISO: %u\n", level);
	if (level)
	{
		s->line_state |= 1U<<n;
	}
	else
	{
		s->line_state &= ~(1U<<n);
	}
	if (s->last_pwm[n] == 0 && level)
	{
		qemu_set_irq(s->pwm[n], 255);
		printf("SPWM: first-on\n");
	}
}

static const VMStateDescription vmstate_software_pwm= {
    .name = TYPE_SOFTWARE_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
		VMSTATE_UINT8_ARRAY(state_count, SoftwarePWMState, LINE_COUNT),
		VMSTATE_UINT8(clock_count, SoftwarePWMState),
        VMSTATE_END_OF_LIST()
    }
};

static void software_pwm_init(Object *obj)
{
    SoftwarePWMState *s = SOFTWARE_PWM(obj);

    DeviceState *dev = DEVICE(obj);

	qdev_init_gpio_in_named(dev, software_pwm_tick, "tick-in",1);
	qdev_init_gpio_in_named(dev, software_pwm_line, "gpio-in",LINE_COUNT);
	qdev_init_gpio_out(dev, s->pwm, LINE_COUNT);
}

static void softwar_pwm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = software_pwm_reset;
    dc->vmsd = &vmstate_software_pwm;
}

static const TypeInfo SOFTWARE_PWM_info = {
    .name          = TYPE_SOFTWARE_PWM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SoftwarePWMState),
    .instance_init = software_pwm_init,
    .class_init    = softwar_pwm_class_init,
};

static void software_pwm_register_types(void)
{
    type_register_static(&SOFTWARE_PWM_info);
}

type_init(software_pwm_register_types)
