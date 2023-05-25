/*
 * STM32G070x SoC
 *
 * Original F405 base (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified and adapted for G070xx 2022 by VintagePC <http://github.com/vintagepc>
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
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/misc/unimp.h"
#include "chardev/char.h"
#include "net/net.h"
#include "hw/arm/armv7m.h"
#include "hw/i2c/smbus_eeprom.h"
#include "exec/ramblock.h"
#include "../stm32_common/stm32_shared.h"
#include "../stm32_common/stm32_chip_macros.h"
#include "../stm32_common/stm32_rcc.h"
#include "../stm32_chips/stm32g070xx.h"
#include "../utility/macros.h"

OBJECT_DECLARE_SIMPLE_TYPE(STM32G070_STRUCT_NAME(), STM32G070XX_BASE)

struct STM32G070_STRUCT_NAME() {
    /*< private >*/
    STM32SOC parent;
    /*< public >*/

    ARMv7MState armv7m;

	MemoryRegion *system_memory;

    MemoryRegion sram;
    MemoryRegion sram_alias;
    MemoryRegion flash;
    MemoryRegion flash_alias;
    MemoryRegion ccmsram;
};

static void stm32g070_soc_initfn(Object *obj)
{
    STM32G070_STRUCT_NAME() *s = STM32G070XX_BASE(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
	s->parent.cpu = DEVICE(&s->armv7m);

}

static void stm32g070_soc_realize(DeviceState *dev_soc, Error **errp)
{
	STM32G070_STRUCT_NAME() *s = STM32G070XX_BASE(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *armv7m;
    Error *err = NULL;

	hwaddr flash_size = stm32_soc_get_flash_size(dev_soc);
	hwaddr sram_size = stm32_soc_get_sram_size(dev_soc);
	hwaddr ccmsram_size = stm32_soc_get_ccmsram_size(dev_soc);

	const stm32_soc_cfg_t* cfg = (STM32_SOC_GET_CLASS(dev_soc))->cfg;

	stm32_soc_setup_flash(dev_soc, &s->flash, &err);

    if (err != NULL) {
        error_propagate(errp, err); // LCOV_EXCL_LINE
        return; // LCOV_EXCL_LINE
    }
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "STM32F030.flash.alias", &s->flash, 0,
                             flash_size);

    memory_region_add_subregion(system_memory, cfg->flash_base, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    memory_region_init_ram(&s->sram, OBJECT(dev_soc), "STM32F030.sram", sram_size,
                           &err);
	memory_region_init_alias(&s->sram_alias, OBJECT(dev_soc),
		"STM32F030.sram.alias", &s->sram, 0,
		sram_size);

	// disabled unless otherwise set such by syscfg.
	memory_region_set_enabled(&s->sram_alias, false);
	memory_region_add_subregion(system_memory, 0, &s->sram_alias);

    if (err != NULL) {
        error_propagate(errp, err); // LCOV_EXCL_LINE
        return;	// LCOV_EXCL_LINE
    }
    memory_region_add_subregion(system_memory, cfg->sram_base, &s->sram);

    memory_region_init_ram(&s->ccmsram, OBJECT(dev_soc), "STM32F030.ccmsram", ccmsram_size,&err);

    memory_region_add_subregion(system_memory, cfg->ccmsram_base, &s->ccmsram);

    armv7m = DEVICE(&s->armv7m);
	stm32_common_rcc_connect_cpu_clocks(stm32_soc_get_periph(dev_soc, STM32_P_RCC), armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", cfg->nvic_irqs);
    qdev_prop_set_string(armv7m, "cpu-type", s->parent.cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return; // LCOV_EXCL_BR_LINE
    }
    /* System configuration controller */
	object_property_set_link(
			OBJECT(stm32_soc_get_periph(dev_soc, STM32_P_SYSCFG)),
			"flash",
			OBJECT(&s->flash_alias),
		&error_fatal);
	object_property_set_link(
			OBJECT(stm32_soc_get_periph(dev_soc, STM32_P_SYSCFG)),
			"sram",
			OBJECT(&s->sram_alias),
		&error_fatal);
    // sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, SYSCFG_IRQ));

    object_property_set_link(OBJECT(stm32_soc_get_periph(dev_soc, STM32_P_FINT)), "flash", OBJECT(&s->flash), errp);

	object_property_set_link(
		OBJECT(stm32_soc_get_periph(dev_soc, STM32_P_ADC1)),
		"adcc",
		OBJECT(stm32_soc_get_periph(dev_soc, STM32_P_ADCC)),
		&error_fatal
		);

    qdev_prop_set_uint32(stm32_soc_get_periph(dev_soc, STM32_P_RCC), "hsi_freq", cfg->rcc_hsi_freq);
    qdev_prop_set_uint32(stm32_soc_get_periph(dev_soc, STM32_P_RCC), "lsi_freq", cfg->rcc_lsi_freq);
    qdev_prop_set_uint32(stm32_soc_get_periph(dev_soc, STM32_P_RCC), "hse_freq", cfg->rcc_hse_freq);
    qdev_prop_set_uint32(stm32_soc_get_periph(dev_soc, STM32_P_RCC), "lse_freq", cfg->rcc_lse_freq);

    struct Chardev;

    char name[]="g070uart0";

    for (int i = STM32_P_USART_BEGIN; i < STM32G070_USART_END; i++) {
        name[8] = '0' + (i - STM32_P_USART_BEGIN);
        if (qemu_chr_find(name)) {
            printf("Found ID %s - assigned to UART %s\n",name, _PERIPHNAMES[i]);
			qdev_prop_set_chr(stm32_soc_get_periph(dev_soc, i), "chardev", qemu_chr_find(name));
        }
    }

	for (int i = STM32_P_DMA_BEGIN; i <= STM32_P_DMA_END; i++)
	{
		object_property_set_link(OBJECT(stm32_soc_get_periph(dev_soc, i)), "system-memory", OBJECT(system_memory), &error_fatal);
	}

	stm32_soc_realize_all_peripherals(dev_soc, &error_fatal);

	for (int i=STM32_P_GPIO_BEGIN; i<STM32G070_GPIO_END; i++)
	{
		DeviceState* gpio = stm32_soc_get_periph(dev_soc, i);
		if (gpio)
		{
			for (int j=0; j<16; j++)
			{
				qdev_connect_gpio_out_named(gpio, "exti", j, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_EXTI), (16*(i-STM32_P_GPIOA))+j));
			}
		}
	}

    create_unimplemented_device("PWR",         0x40007000, 1U * KiB);

}

static void stm32g070_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = stm32g070_soc_realize;

	STM32SOCClass *sc = STM32_SOC_CLASS(klass);
	sc->cfg = (const stm32_soc_cfg_t*)data;
    /* No vmstate or reset required: device has no internal state */
}

static const TypeInfo
stm32_common_g070xx_info = {
    .name          = TYPE_STM32G070XX_BASE,
    .parent        = TYPE_STM32_SOC,
    .instance_size = sizeof(STM32G070_STRUCT_NAME()),
	.abstract	   = true
};

static void
stm32_g070xx_register_types(void)
{
    type_register_static(&stm32_common_g070xx_info);
	for (const stm32_mem_cfg_t* p = stm32g070xx_cfg.flash_variants; p->chip_type != NULL; p++){
		TypeInfo ti = {
			.name       = p->chip_type,
			.parent     = TYPE_STM32G070XX_BASE,
			.instance_init = stm32g070_soc_initfn,
			.class_init    = stm32g070_soc_class_init,
			.class_data = (void *)&stm32g070xx_cfg,
		};
		type_register(&ti);
	}
}

type_init(stm32_g070xx_register_types);

#include "hw/boards.h"

static void stm32_g070xx_class_init(ObjectClass *oc, void *data)
{
	    MachineClass *mc = MACHINE_CLASS(oc);
	    mc->desc = data;
	    mc->family = TYPE_STM32G070xx,
	    mc->init = stm32_soc_machine_init;
	    mc->default_ram_size = 0; // 0 = use default RAM from chip.
	    mc->no_parallel = 1;
		mc->no_serial = 1;

		STM32SocMachineClass* smc = STM32_MACHINE_CLASS(oc);
		smc->soc_type = data;
		smc->cpu_type = ARM_CPU_TYPE_NAME("cortex-m0");
}

static const TypeInfo stm32g030xx_machine_types[] = {
    {
        .name           = MACHINE_TYPE_NAME(TYPE_STM32G070xB),
        .parent         = TYPE_STM32_MACHINE,
		.class_init     = stm32_g070xx_class_init,
		.class_data		= (void*)(TYPE_STM32G070xB_SOC),
    },
};

DEFINE_TYPES(stm32g030xx_machine_types)
