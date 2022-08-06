/*
 * STM32F407 SoC
 *
 * Original F405 base (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified and adapted for Mini404/F407 2020-22 by VintagePC <http://github.com/vintagepc>
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
#include "qemu-common.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "stm32f407_soc.h"
#include "hw/misc/unimp.h"
#include "net/net.h"
#include "hw/i2c/smbus_eeprom.h"
#include "exec/ramblock.h"
#include "hw/qdev-properties.h"
#include "../stm32_common/stm32_common.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_common/stm32_chip_macros.h"
#include "../stm32_common/stm32_rcc_if.h"
#include "../stm32_common/stm32_rcc.h"
#include "../stm32_chips/stm32f407xx.h"

static const stm32_soc_cfg_t* stm32_f4xx_variants[] = {
	&stm32f407xx_cfg,
};


static void stm32f4xx_soc_initfn(Object *obj)
{
    STM32F4XX_STRUCT_NAME() *s = STM32F4XX_BASE(obj);
    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
	s->parent.cpu = DEVICE(&s->armv7m);

}

static void stm32f4xx_soc_unrealize(DeviceState *dev)
{
    printf("soc unrealize;");
}

static void stm32f4xx_soc_finalize(Object *obj)
{
    printf("soc finalize;");
}

static void stm32f4xx_soc_realize(DeviceState *dev_soc, Error **errp)
{
    STM32F4XX_STRUCT_NAME() *s = STM32F4XX_BASE(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *dev, *armv7m;
    Error *err = NULL;
    int i;

	uint64_t flash_size = stm32_soc_get_flash_size(DEVICE(&s->parent));
	uint64_t sram_size = stm32_soc_get_sram_size(DEVICE(&s->parent));
	uint64_t ccmsram_size = stm32_soc_get_ccmsram_size(DEVICE(&s->parent));

	const stm32_soc_cfg_t* cfg = (STM32_SOC_GET_CLASS(dev_soc))->cfg;

    memory_region_init_rom(&s->flash, OBJECT(dev_soc), "STM32F407.flash",
                           flash_size, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "STM32F407.flash.alias", &s->flash, 0,
                             flash_size);

    // Kinda sketchy but needed to bypass the FW check on the Mini...
     s->flash.ram_block->host[MiB  -1] = 0xFF;

    memory_region_add_subregion(system_memory, cfg->flash_base, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    memory_region_init_ram(&s->sram, NULL, "STM32F407.sram", sram_size,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, cfg->sram_base, &s->sram);

    memory_region_init_ram(&s->ccmsram, OBJECT(dev_soc), "STM32F407.ccmsram", ccmsram_size ,&err);

    memory_region_add_subregion(system_memory, cfg->ccmsram_base, &s->ccmsram);

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", cfg->nvic_irqs);
    qdev_prop_set_string(armv7m, "cpu-type", s->parent.cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

	dev = stm32_soc_get_periph(dev_soc, STM32_P_RCC);
	qdev_prop_set_uint32(dev, "hsi_freq", cfg->rcc_hsi_freq);
    qdev_prop_set_uint32(dev, "lsi_freq", cfg->rcc_lsi_freq);
    qdev_prop_set_uint32(dev, "hse_freq", cfg->rcc_hse_freq);
    qdev_prop_set_uint32(dev, "lse_freq", cfg->rcc_lse_freq);


    // TODO - Connect EXTI and WAKEUP to the GPIOs.

    /* Attach UART (uses USART registers) and USART controllers */
    struct Chardev;

    Chardev* chrdev_assigns[STM_NUM_USARTS];

    char name[]="stm32uart0";

    // First assign by ID
    for (i = 0; i< STM_NUM_USARTS; i++) {
        name[9] = '0' + i;
        chrdev_assigns[i] = qemu_chr_find(name);
        if (chrdev_assigns[i]!=NULL) {
            printf("Found ID %s - assigned to UART %d\n",name, i+1);
			qdev_prop_set_chr(stm32_soc_get_periph(dev_soc, STM32_P_UART1 +i ), "chardev", chrdev_assigns[i]);
        }
    }

	for (int i=STM32_P_ADC_BEGIN; i<STM32_P_ADC_END; i++)
	object_property_set_link(
			OBJECT(stm32_soc_get_periph(dev_soc, i)),
			"common",
			OBJECT(stm32_soc_get_periph(dev_soc, STM32_P_ADCC)),
		&error_fatal);


	DeviceState* syscfg = stm32_soc_get_periph(dev_soc, STM32_P_SYSCFG);
    for (i = 0; i < 16; i++) {
       	qdev_connect_gpio_out(syscfg, i, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_EXTI), i));
    }

	for (int i=STM32_P_GPIO_BEGIN; i<STM32_P_GPIO_END; i++)
	{
		DeviceState* gpio = stm32_soc_get_periph(dev_soc, i);
		if (gpio == NULL)
		{
			continue;
		}
		for (int j=0; j<16; j++)
		{
			qdev_connect_gpio_out_named(gpio, "exti", j, qdev_get_gpio_in(syscfg, (16*(i-STM32_P_GPIOA))+j));
		}
	}

    // TODO - RTC wakeup exti
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 0, qdev_get_gpio_in(exti_dev, 17));
    // // Alarm B
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 1, qdev_get_gpio_in(exti_dev, 17));
    // // Wake up timer
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 2, qdev_get_gpio_in(exti_dev, 22));

    object_property_set_link(OBJECT(stm32_soc_get_periph(dev_soc, STM32_P_FINT)), "flash", OBJECT(&s->flash), errp);


    //s->otg_fs.debug = true;
    qdev_prop_set_chr(stm32_soc_get_periph(dev_soc, STM32_P_USB2), "chardev", qemu_chr_find("stm32usbfscdc"));

    // IRQs: FS wakeup: 42 FS Global: 67
	// STM32_RLZ_AND_MAP(s, usb_fs, armv7m, true, NULL);
   // qdev_connect_gpio_out_named(rcc,"reset",STM32_USB,qdev_get_gpio_in_named(DEVICE(&s->otg_hs),"rcc-reset",0));

    // USB IRQs:
    // Global HS: 77. WKUP: 76, EP1 in/out = 75/74.


    qemu_check_nic_model(&nd_table[0], "stm32f4xx-ethernet");
    dev = stm32_soc_get_periph(dev_soc, STM32_P_ETH);
    qdev_set_nic_properties(dev, &nd_table[0]);
    if (qemu_find_netdev("mini-eth")!=NULL){
        qdev_prop_set_string(dev,"netdev","mini-eth");
        qdev_prop_set_bit(dev, "connected", true);
    } else {
        printf("Ethernet disconnected. use -netdev id=mini-eth,[opts...] to connect it.\n");
    }

	qdev_prop_set_chr(stm32_soc_get_periph(dev_soc, STM32_P_ITM), "chardev", qemu_chr_find("stm32_itm"));

	stm32_soc_realize_all_peripherals(dev_soc, errp);

	//ITM is special, we need to overlay it at the top level because otherwise the NVIC
	// intercepts our read/write calls.
	memory_region_add_subregion_overlap(&s->armv7m.container, cfg->perhipherals[STM32_P_ITM].base_addr, sysbus_mmio_get_region(SYS_BUS_DEVICE(stm32_soc_get_periph(dev_soc, STM32_P_ITM)),0) ,10);

    create_unimplemented_device("WWDG",        0x40002C00, 0x400);
    create_unimplemented_device("I2S2ext",     0x40003000, 0x400);
    create_unimplemented_device("I2S3ext",     0x40004000, 0x400);
    create_unimplemented_device("CAN1",        0x40006400, 0x400);
    create_unimplemented_device("CAN2",        0x40006800, 0x400);
    create_unimplemented_device("DAC",         0x40007400, 0x400);
    create_unimplemented_device("SDIO",        0x40012C00, 0x400);
    create_unimplemented_device("BKPSRAM",     0x40024000, 0x400);
    create_unimplemented_device("Ethernet",    0x40028000, 0x1400);
    create_unimplemented_device("USB OTG FS",  0x50000000, 0x31000); // Note - FS is the serial port/micro-usb connector
    create_unimplemented_device("DCMI",        0x50050000, 0x400);
    create_unimplemented_device("SYSRAM/RSVD",         0x1FFF0000, 0x8000);
    create_unimplemented_device("ETM/DBGMCU/TIPU", 0xE0001000, 0xFEFFF);

  //  create_unimplemented_device("EXTERNAL",    0xA0000000, 0x3FFFFFFF)

}

static void stm32f4xx_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = stm32f4xx_soc_realize;
	dc->unrealize = stm32f4xx_soc_unrealize;

	STM32SOCClass *sc = STM32_SOC_CLASS(klass);
	sc->cfg = (const stm32_soc_cfg_t*)data;

    /* No vmstate or reset required: device has no internal state */
}

static const TypeInfo
stm32_common_f4xx_info = {
    .name          = TYPE_STM32F4XX_BASE,
    .parent        = TYPE_STM32_SOC,
    .instance_size = sizeof(STM32F4XX_STRUCT_NAME()),
	.abstract	   = true,
	.instance_finalize = stm32f4xx_soc_finalize,
};

static void
stm32_f4xx_register_types(void)
{
    type_register_static(&stm32_common_f4xx_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_f4xx_variants); ++i) {
		for (const stm32_mem_cfg_t* p = stm32_f4xx_variants[i]->flash_variants; p->chip_type != NULL; p++){
			TypeInfo ti = {
				.name       = p->chip_type,
				.parent     = TYPE_STM32F4XX_BASE,
				.instance_init = stm32f4xx_soc_initfn,
				.class_init    = stm32f4xx_soc_class_init,
				.class_data = (void *)stm32_f4xx_variants[i],
			};
			type_register(&ti);
		}
    }

}

type_init(stm32_f4xx_register_types);
