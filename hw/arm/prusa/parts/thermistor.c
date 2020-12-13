/*
	thermistor.c
	Based on thermistor.c (C) 2008-2012 Michel Pollet <buserror@gmail.com>
	Rewritten for C++ in 2020 by VintagePC <https://github.com/vintagepc/>
 	This file is part of MK404.
	MK404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	MK404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with MK404.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "thermistortables.h"

#define TYPE_THERMISTOR "thermistor"
OBJECT_DECLARE_SIMPLE_TYPE(ThermistorState, THERMISTOR)

struct ThermistorState {
    SysBusDevice parent;

    qemu_irq irq_value;

    uint16_t table_index;
    const short int *table;    
    int table_length;
    float temperature;
    int8_t oversampling;
    uint16_t start_temp;
};

static void thermistor_read_request(void *opaque, int n, int level) {
    if (!level)
    {
        return;
    }
	ThermistorState *s = opaque;
    if (s->table_index==0) {
        qemu_set_irq(s->irq_value,s->start_temp);
    return;
    }
	for (uint16_t i= 0; i<s->table_length; i+=2) {
		if (s->table[i+1] <= s->temperature) {
			int16_t tt = s->table[i];
			/* small linear regression between table samples */
			if ( i !=0 && s->table[i+1] < s->temperature) {
				int16_t d_adc = s->table[i] - s->table[i-2];
				float d_temp = s->table[i+1] - s->table[i-1];
				float delta = s->temperature - s->table[i+1];
				tt = s->table[i] + (d_adc * (delta / d_temp));
			}
			// if (m_adc_mux_number==-1)
			// 	printf("simAVR ADC out value: %u\n",((tt / m_oversampling) * 5000) / 0x3ff);
			int value = (((tt / s->oversampling)));
			qemu_set_irq(s->irq_value,value);
            return;
		}
	}

}

static void thermistor_temp_in(void *opaque, int n, int level)
{
	ThermistorState *s = opaque;
	float fv = (float)(level) / 256.f;
	s->temperature = fv;
}

static void thermistor_reset(DeviceState *dev)
{
    ThermistorState *s = THERMISTOR(dev);
    s->temperature = s->start_temp;
    switch (s->table_index)
    {
        case 1:
            s->table_length = 2*BEDTEMPTABLE_LEN;
            s->table = &temptable_1[0][0];
            break;
        case 5:
            s->table = &temptable_5[0][0];
            s->table_length = 2*HEATER_0_TEMPTABLE_LEN;
            break;
        case 2000:
            s->table_length = AMBIENTTEMPTABLE_LEN;
            s->table = &temptable_2000[0][0];
            break;
        default:
            s->table = NULL;
    s->table_length = 0;
            break;
    }

}


static void thermistor_init(Object *obj)
{
    ThermistorState *s = THERMISTOR(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_value, "thermistor_value", 1);

    qdev_init_gpio_in_named(DEVICE(obj),thermistor_read_request, "thermistor_read_request", 1);
    qdev_init_gpio_in_named(DEVICE(obj),thermistor_temp_in, "thermistor_set_temperature", 1);

    s->oversampling = OVERSAMPLENR;
}

static Property thermistor_properties[] = {
    DEFINE_PROP_UINT16("temp", ThermistorState, start_temp,0),
    DEFINE_PROP_UINT16("table_no", ThermistorState, table_index, 0),
   // DEFINE_PROP_("cpu-type", STM32F407State, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};


static void thermistor_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = thermistor_reset;
    // dc->props = thermistor_properties;
    device_class_set_props(dc, thermistor_properties);
    // dc->vmsd = &vmstate_thermistor;
}

static const TypeInfo thermistor_info = {
    .name          = TYPE_THERMISTOR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ThermistorState),
    .instance_init = thermistor_init,
    .class_init    = thermistor_class_init,
};

static void thermistor_register_types(void)
{
    type_register_static(&thermistor_info);
}

type_init(thermistor_register_types)