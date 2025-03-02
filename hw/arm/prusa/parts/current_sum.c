/*
	current_sum.c - simple current summation for a series of heaters

    Written for Mini404 in 2023 by VintagePC <https://github.com/vintagepc/>

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
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "../utility/macros.h"
#include "migration/vmstate.h"

#define TYPE_CURRENT_SUM "current-sum"
OBJECT_DECLARE_SIMPLE_TYPE(CurrentSumState, CURRENT_SUM)

// Number of items contributing to the reading...
#define NUM_ITEMS 10

typedef struct heater_state heater_state;

struct CurrentSumState {
    SysBusDevice parent;

	heater_state *heaters[NUM_ITEMS];
	float amps;
    qemu_irq irq;
};

extern float heater_calculate_current(heater_state *s);

static void current_sum_update_irq(CurrentSumState *s) {
    // ADC expects values directly in "ADC units, 12 bit."
	// 90 mV/Amp, scaled to 3.3v AREF
	s->amps = 0;
	for (int i=0; i<NUM_ITEMS; i++)
	{
		if (s->heaters[i] != NULL)
		{
			s->amps += (heater_calculate_current(s->heaters[i])/1000.f);
		}
	}
    int adc_readout = 4095.f * ((s->amps * 0.09f)/3.3f);
	//if (s->amps>0) printf("current out %f A (adc %u)\n",s->amps, adc_readout);
    qemu_set_irq(s->irq,adc_readout);
}

static void current_sum_read_request(void *opaque, int n, int level)
{
    CurrentSumState *s = CURRENT_SUM(opaque);
	current_sum_update_irq(s);
}

// static void current_sum_in(void *opaque, int n, int level)
// {
//     CurrentSumState *s = CURRENT_SUM(opaque);
// 	s->ma_in[n] = level;
// 	float amps = 0.f;
// 	for (int i=0; i<NUM_ITEMS; i++)
// 	{
// 		amps += (float)s->ma_in[i]/1000.f;
// 	}
// 	s->amps = amps;
// 	printf("Current in: %d %d, total %f\n", n, level, amps);
// }

static void current_sum_reset(DeviceState *dev)
{
    CurrentSumState *s = CURRENT_SUM(dev);
	s->amps = 0;
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(CurrentSumState, current_sum, CURRENT_SUM, SYS_BUS_DEVICE, {NULL});

static void current_sum_finalize(Object *obj)
{

}

static void current_sum_init(Object *obj)
{
	CurrentSumState *s = CURRENT_SUM(obj);
    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "adc_out", 1);
    qdev_init_gpio_in_named(DEVICE(obj), current_sum_read_request,"adc_read_request",1);
}

static Property current_sum_properties[] = {
    DEFINE_PROP_LINK("heater[0]",CurrentSumState, heaters[0], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[1]",CurrentSumState, heaters[1], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[2]",CurrentSumState, heaters[2], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[3]",CurrentSumState, heaters[3], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[4]",CurrentSumState, heaters[4], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[5]",CurrentSumState, heaters[5], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[6]",CurrentSumState, heaters[6], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[7]",CurrentSumState, heaters[7], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[8]",CurrentSumState, heaters[8], "heater", heater_state*),
    DEFINE_PROP_LINK("heater[9]",CurrentSumState, heaters[9], "heater", heater_state*),
    DEFINE_PROP_END_OF_LIST(),
};

// static const VMStateDescription vmstate_acs711 = {
//     .name = TYPE_ACS711,
//     .version_id = 1,
//     .minimum_version_id = 1,
//     .fields = (VMStateField[]) {
//         VMSTATE_UINT8(on_count, ACS711State),
//         VMSTATE_UINT8_ARRAY(pwm_on, ACS711State,NUM_ITEMS),
//         VMSTATE_END_OF_LIST()
//     }
// };

static void current_sum_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = current_sum_reset;
 //   dc->vmsd = &vmstate_acs711;
   device_class_set_props(dc, current_sum_properties);
}
