/*
    stm32_rcc.c  - Common RCC storage and functions

	Copyright 2022 VintagePC <https://github.com/vintagepc/>
    Ported to Mini404 in 2021

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
#include "qemu-common.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "stm32_rcc.h"
#include "stm32_rcc_if.h"
#include "stm32_clk.h"
#include "../stm32f407/stm32_clktree.h"


extern uint32_t stm32_rcc_if_get_periph_freq(STM32Peripheral* p)
{
	if (p->rcc == NULL)
	{
		printf("ERR: No RCC set when checking clock frequency!\n");
		return 0;
	}
	COM_STRUCT_NAME(Rcc) *s = STM32COM_RCC(p->rcc);
    Clk_t* clk = &s->pclocks[p->periph];

    assert(clk->is_initialized);

    return clktree_get_output_freq(clk);
}

bool stm32_rcc_if_check_periph_clk(STM32Peripheral *p)
{
	if (p->rcc == NULL)
	{
		printf("ERR: No RCC set when checking clock enabled!\n");
		return false;
	}

	COM_STRUCT_NAME(Rcc) *s = STM32COM_RCC(p->rcc);
    Clk_t* clk = &s->pclocks[p->periph];

    assert(clk->is_initialized);

    if(!clktree_is_enabled(clk)) {
        /* I assume writing to a peripheral register while the peripheral clock
         * is disabled is a bug and give a warning to unsuspecting programmers.
         * When I made this mistake on real hardware the write had no effect.
         */
        printf("Warning: You are attempting to use the [%s] peripheral while "
                 "its clock is disabled.\n", clk->name);
        return false;
    }
    return true;
}

void stm32_rcc_if_set_periph_clk_irq(
        STM32Peripheral *p,
        qemu_irq periph_irq)
{
	if (p->rcc == NULL)
	{
		printf("ERR: No RCC set when adding a clock IRQ!\n");
		return;
	}
	COM_STRUCT_NAME(Rcc) *s = STM32COM_RCC(p->rcc);
    Clk_t* clk = &s->pclocks[p->periph];

    assert(clk->is_initialized);

    clktree_adduser(clk, periph_irq);
}

void stm32_common_rcc_reset_write(COM_STRUCT_NAME(Rcc) *s, uint32_t mask, const uint8_t (*vectors)[32]) {
    for (int i=0; i<32; i++) {
        if (mask & 1U<<i && (*vectors)[i] != 0 ) {
			if ((*vectors)[i] == STM32_P_ADC_ALL) // sometimes ADC is shared...
			{
				qemu_irq_pulse(s->reset[STM32_P_ADC2]);
				qemu_irq_pulse(s->reset[STM32_P_ADC3]);
			}
            qemu_irq_pulse(s->reset[(*vectors)[i]]);
        }
    }
}

void stm32_common_rcc_enable_write(COM_STRUCT_NAME(Rcc) *s, uint32_t mask, const uint8_t (*periphs)[32]) {
    for (int i=0; i<32; i++) {
        if ((*periphs)[i] != 0 && s->pclocks[(*periphs)[i]].is_initialized)
		{
			clktree_set_enabled(&s->pclocks[(*periphs)[i]], (mask >> i) & 1);
        }
    }
}

static void stm32_common_rcc_realize(DeviceState *dev, Error **errp)
{
	// Init the four primary clocks:
	COM_STRUCT_NAME(Rcc) *s = STM32COM_RCC(dev);
	clktree_create_src_clk(&s->HSICLK, "HSI", s->hse_freq, false);
    clktree_create_src_clk(&s->LSICLK, "LSI", s->lse_freq, false);
    clktree_create_src_clk(&s->HSECLK, "HSE", s->hsi_freq, false);
    clktree_create_src_clk(&s->LSECLK, "LSE", s->lsi_freq, false);
}

static void stm32_common_rcc_instance_init(Object* obj)
{
	COM_STRUCT_NAME(Rcc) *s = STM32COM_RCC(obj);
    qdev_init_gpio_out_named(DEVICE(obj),s->reset,"reset",ARRAY_SIZE(s->reset));
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(int i = 0; i < STM32_P_COUNT; i++) {
        s->pclocks[i].is_initialized = false;
    }
	s->realize_func = stm32_common_rcc_realize;
}

static Property stm32_common_rcc_properties[] = {
    DEFINE_PROP_UINT32("hse_freq", COM_STRUCT_NAME(Rcc), hse_freq, 0),
    DEFINE_PROP_UINT32("lse_freq", COM_STRUCT_NAME(Rcc), lse_freq, 0),
	DEFINE_PROP_UINT32("hsi_freq", COM_STRUCT_NAME(Rcc), hsi_freq, 0),
    DEFINE_PROP_UINT32("lsi_freq", COM_STRUCT_NAME(Rcc), lsi_freq, 0),
	DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32_common_rcc = {
    .name = TYPE_STM32COM_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(hsi_freq, COM_STRUCT_NAME(Rcc)),
        VMSTATE_UINT32(hse_freq, COM_STRUCT_NAME(Rcc)),
        VMSTATE_UINT32(lsi_freq, COM_STRUCT_NAME(Rcc)),
        VMSTATE_UINT32(lse_freq, COM_STRUCT_NAME(Rcc)),
		VMSTATE_STRUCT(HSICLK,COM_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(HSECLK,COM_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(LSECLK,COM_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT(LSICLK,COM_STRUCT_NAME(Rcc), 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_STRUCT_ARRAY(pclocks, COM_STRUCT_NAME(Rcc), STM32_P_COUNT, 1, vmstate_stm32_common_rcc_clk, Clk_t),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_common_rcc_class_init(ObjectClass* class, void* class_data)
{
	DeviceClass *dc = DEVICE_CLASS(class);
	device_class_set_props(dc, stm32_common_rcc_properties);
	dc->vmsd = &vmstate_stm32_common_rcc;
}


static const TypeInfo stm32_common_rcc_type_info = {
    .name = TYPE_STM32COM_RCC,
    .parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(COM_STRUCT_NAME(Rcc)),
    .class_size = sizeof(COM_CLASS_NAME(Rcc)),
	.class_init = stm32_common_rcc_class_init,
	.instance_init = stm32_common_rcc_instance_init,
	.abstract = true,
};

static void stm32_common_rcc_if_register_types(void)
{
    type_register_static(&stm32_common_rcc_type_info);
}

type_init(stm32_common_rcc_if_register_types)
