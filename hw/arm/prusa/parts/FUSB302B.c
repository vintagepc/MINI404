/*
	FUSB302B.c - USB stub for Mini404

    Written for Mini404 in 2022-3 by VintagePC <https://github.com/vintagepc/>

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
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qom/object.h"


#define TYPE_FUSB302B "fusb302b"


typedef struct FUSB302BState {
    I2CSlave parent_obj;

	uint8_t data[2];
	uint8_t rx_level;

} FUSB302BState;

DECLARE_INSTANCE_CHECKER(FUSB302BState, FUSB302B,
                         TYPE_FUSB302B)
static
uint8_t fusb302b_recv(I2CSlave *ss)
{
    FUSB302BState *s = FUSB302B(ss);
    uint8_t ret = 0;
	//if (s->rx_level>0)
	{
		switch (s->data[0])
		{
		case 0x40:
			ret = 0x80;
			break;
		default:
			ret = 0;
			break;
		}
	}
	// printf("FUSB tx %02x -> %02x\n", s->data[0],ret);
    return ret;
}

static
int fusb302b_send(I2CSlave *ss, uint8_t data)
{
    FUSB302BState *s = FUSB302B(ss);
	// printf("FUSB rx %02x\n", data);
		s->data[0] = data;
    return 0;
}

static void fusb302b_realize(DeviceState *dev, Error **errp)
{
        //FUSB302BState *s = FUSB302B(s);

}

static
void fusb302b_reset(DeviceState *state)
{
}

static Property fusb302b_props[] = {
    DEFINE_PROP_END_OF_LIST()
};

static
void fusb302b_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = &fusb302b_realize;
    k->recv = &fusb302b_recv;
    k->send = &fusb302b_send;

    device_class_set_props(dc, fusb302b_props);
    dc->reset = fusb302b_reset;
}

static
const TypeInfo fusb302b_type = {
    .name = TYPE_FUSB302B,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(FUSB302BState),
    .class_size = sizeof(I2CSlaveClass),
    .class_init = fusb302b_class_init,
};

static void fusb302b_register(void)
{
    type_register_static(&fusb302b_type);
}

type_init(fusb302b_register)
