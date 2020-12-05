/*
	Heater.c - a heater object for MINI404. There's not much to it,
    it just ticks the temperature "up" at a determined rate when active on PWM and down in
    in an exponential curve when off.

	Copyright 2020 VintagePC <https://github.com/vintagepc/>

 	This file is part of MINI404 (Adapted from MK404 for QEMU)

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
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include <math.h>


#define TYPE_HEATER "heater"
OBJECT_DECLARE_SIMPLE_TYPE(heater_state, HEATER)

struct heater_state {
    SysBusDevice parent; 

    float thermalMass;
    float ambientTemp;
    float currentTemp;
    // bool m_bIsBed = false;
    char m_chrLabel;
    uint16_t pwm, lastpwm;

    uint8_t mass10x;

    int tick_overrun;

    qemu_irq temp_out;
    QEMUTimer *temp_tick;
};

static void heater_temp_tick_expire(void *opaque)
{
    heater_state *s = opaque;

    if (s->pwm>0)// || (pAVR->cycle-m_cntOff)<(pAVR->frequency/100))
    {
        float fDelta = (s->thermalMass*((float)(s->pwm)/255.0f))*0.3f;
        s->currentTemp += fDelta;
        s->tick_overrun = 3; 
        s->lastpwm = s->pwm;
    } else if (s->tick_overrun>0){
        float fDelta = (s->thermalMass*((float)(s->lastpwm)/255.0f))*0.3f;
        s->currentTemp += fDelta;
        s->tick_overrun--;
    } else {// Cooling - do a little exponential decay
        float dT = (s->currentTemp - s->ambientTemp)*pow(2.7183,-0.005*0.3);
        s->currentTemp -= s->currentTemp - (s->ambientTemp + dT);
    }
	// m_iDrawTemp = s->currentTemp;

    if (s->pwm>0 || s->currentTemp>s->ambientTemp+0.3)
	{
        timer_mod(s->temp_tick,  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+300);
	}
    else
    {
        s->currentTemp = s->ambientTemp;
        //qemu_set_irq(s->temp_out, s->currentTemp*256.f);
    }
    qemu_set_irq(s->temp_out, s->currentTemp*256.f);
    // printf("Temp: %f\n",s->currentTemp);
}


static void heater_pwm_change(void* opaque, int n, int level)
{
    heater_state *s = opaque;
   if (s->pwm == level)
        return;

    //printf("New pwm: %d\n",level);
    s->pwm = level;
    if (s->pwm > 0)
	{
        timer_mod(s->temp_tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+100);
	}
    // if (GetIRQ(ON_OUT)->value != (s->pwm>0))
	// {
    //     RaiseIRQ(ON_OUT,s->pwm>0);
	// }
}

//TCCR0A  _SFR_IO8(0x24)
//#define COM0B0  4

// void Heater::OnDigitalChanged(struct avr_irq_t * irq, uint32_t value)
// {

//     if (m_bIsBed) // The heatbed PWM is based on inverting mode trickery. We can just watch COM0B0 rather than the digital pin output.
//     {
//         avr_regbit_t inv = AVR_IO_REGBIT(0x24 + 32,4);
//         uint8_t COM0B0 = avr_regbit_get(m_pAVR, inv);
//         value = COM0B0^1U;
//     }

//     if (value==1)
// 	{
//         value = 255;
// 	}

//     OnPWMChanged(irq,value);
// }

// void Heater::Set(uint8_t uiPWM)
// {
//     m_bAuto = false;
//     s->pwm = uiPWM;
//     RaiseIRQ(PWM_IN,0XFF);
// }

static void heater_reset(DeviceState *dev)
{
    heater_state *s = HEATER(dev);

    s->thermalMass = ((float)s->mass10x)/10.f;
    s->ambientTemp = 25.f;
    s->currentTemp = s->ambientTemp;
}

static void heater_init(Object *obj)
{
    heater_state *s = HEATER(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->temp_out, "temp_out", 1);

    qdev_init_gpio_in_named(DEVICE(obj),heater_pwm_change, "pwm_in", 1);

    s->temp_tick = timer_new_ms(QEMU_CLOCK_VIRTUAL,
            (QEMUTimerCB *)heater_temp_tick_expire, s);
}

static Property heater_properties[] = {
    DEFINE_PROP_UINT8("thermal_mass_x10",heater_state, mass10x, 40),
 //   DEFINE_PROP_FLOAT("temp", heaterState, start_temp,0),
   // DEFINE_PROP_("cpu-type", STM32F407State, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};


static void heater_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = heater_reset;
    // dc->props = heater_properties;
    device_class_set_props(dc, heater_properties);
    // dc->vmsd = &vmstate_heater;
}

static const TypeInfo heater_info = {
    .name          = TYPE_HEATER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(heater_state),
    .instance_init = heater_init,
    .class_init    = heater_class_init,
};

static void heater_register_types(void)
{
    type_register_static(&heater_info);
}

type_init(heater_register_types)
