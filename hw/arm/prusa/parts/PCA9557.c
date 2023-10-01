/*
 * PCA9557 I/O expander
 *
 * Written for Mini404 in 2022 by VintagePC <https://github.com/vintagepc/>
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

#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qom/object.h"


#define TYPE_PCA9557 "pca9557"

enum REG {
	RI_INPUT,
	RI_OUTPUT,
	RI_INPUT_POLARITY,
	RI_CONFIG,
	RI_END
};

typedef struct PCA9557State {
    I2CSlave parent_obj;

	uint8_t address;
	uint8_t data;
	qemu_irq outputs[8];
	uint8_t registers[RI_END];
	bool is_data;
} PCA9557State;

DECLARE_INSTANCE_CHECKER(PCA9557State, PCA9557, TYPE_PCA9557)

static int pca9557_event(I2CSlave *ss, enum i2c_event event)
{
    PCA9557State *s = PCA9557(ss);
	if (event == I2C_START_SEND)
	{
		s->is_data = false;
	}
	return 0;
}

static uint8_t pca9557_recv(I2CSlave *ss)
{
    PCA9557State *s = PCA9557(ss);

	if (s->address>=RI_END) return 0;

    uint8_t data = s->registers[s->address];
	if (s->address == RI_INPUT)
	{
		data^=s->registers[RI_INPUT_POLARITY];
	}
	s->address++;

	return data;
}

static int pca9557_send(I2CSlave *ss, uint8_t data)
{
    PCA9557State *s = PCA9557(ss);
	if (!s->is_data)
	{
		s->address=data;
		s->is_data = true;
	}
	else
	{
		// printf("pca9557 write: %02x %02x\n", s->address, data);
		switch(s->address)
		{
			case RI_OUTPUT:
			{
				uint8_t changed = s->registers[RI_OUTPUT] ^ data;
				s->registers[RI_OUTPUT] = data;
				uint8_t pin = 0;
				changed &= ~s->registers[RI_CONFIG]; // Only consider changes on pins set as outputs!
				while(changed)
				{
					if ((changed & 1U))
					{
						qemu_set_irq(s->outputs[pin], data & (1U << pin));
					}
					pin++;
					changed>>=1;
				}
				break;
			}
			case RI_INPUT_POLARITY:
				s->registers[RI_INPUT_POLARITY] = data;
				break;
			case RI_CONFIG:
				s->registers[RI_CONFIG] = data;
				break;
			case RI_INPUT:
				break;
			default:
				return 1; //NAK, out of range.
				break;
		}
		s->address++;
	}
    return 0;
}

static void pca9557_input(void* opaque, int n, int level) {
    PCA9557State *s = PCA9557(opaque);
	if (level)
	{
		s->registers[RI_INPUT] |= (1U << n);
	}
	else
	{
		s->registers[RI_INPUT] &= ~(1U << n);
	}
}

static void pca9557_realize(DeviceState *dev, Error **errp)
{
	PCA9557State *s = PCA9557(dev);
	qdev_init_gpio_out(DEVICE(dev), s->outputs, 8);
	qdev_init_gpio_in(DEVICE(dev), pca9557_input, 8);
}

static void pca9557_reset(DeviceState *state)
{
}

static Property pca9557_props[] = {
    DEFINE_PROP_END_OF_LIST()
};

static
void pca9557_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = &pca9557_realize;
    k->recv = &pca9557_recv;
    k->send = &pca9557_send;
	k->event = &pca9557_event;

    device_class_set_props(dc, pca9557_props);
    dc->reset = pca9557_reset;
}

static
const TypeInfo pca9557_type = {
    .name = TYPE_PCA9557,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(PCA9557State),
    .class_size = sizeof(I2CSlaveClass),
    .class_init = pca9557_class_init,
};

static void pca9557_register(void)
{
    type_register_static(&pca9557_type);
}

type_init(pca9557_register)
