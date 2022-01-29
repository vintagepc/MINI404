/*
 * STM32F407 SoC
 *
 * Original F405 base (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified and adapted for Mini404/F407 2020 by VintagePC <http://github.com/vintagepc>
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

	const stm32_soc_cfg_t* cfg = (STM32_SOC_GET_CLASS(obj))->cfg;

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

	STM32_INIT_A(s, adcs);
	STM32_INIT(s, adc_common);
    STM32_INIT(s, crc);
    STM32_INIT_A(s, dmas);
	STM32_INIT(s, exti);
	STM32_INIT(s, flash_if);
    STM32_INIT_A(s, gpios);
    STM32_INIT_A(s, i2cs);
	STM32_INIT(s, itm);
    STM32_INIT(s, iwdg);
	STM32_INIT_A(s, spis);
    STM32_INIT(s, otp);
	STM32_INIT(s, pwr);
	STM32_INIT(s, rcc);
    STM32_INIT(s, rng);
	STM32_INIT(s, rtc);
	STM32_INIT(s, syscfg);
	STM32_INIT_A(s, timers);
    STM32_INIT(s, usb_fs);
    STM32_INIT(s, usb_hs);
	STM32_INIT_A(s, usarts);
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
    DeviceState *dev, *armv7m, *rcc;
    Error *err = NULL;
    int i;

	uint64_t flash_size = stm32_soc_get_flash_size(DEVICE(&s->parent));
	uint64_t sram_size = stm32_soc_get_sram_size(DEVICE(&s->parent));

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

    memory_region_add_subregion(system_memory, cfg->flash_memory.data.base_addr, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    memory_region_init_ram(&s->sram, NULL, "STM32F407.sram", sram_size,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, cfg->sram.data.base_addr, &s->sram);

    memory_region_init_ram(&s->ccmsram, OBJECT(dev_soc), "STM32F407.ccmsram", cfg->ccmsram.data.size ,&err);

    memory_region_add_subregion(system_memory, cfg->ccmsram.data.base_addr, &s->ccmsram);

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", cfg->nvic_irqs);
    qdev_prop_set_string(armv7m, "cpu-type", s->parent.cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }
    /* System configuration controller */
	STM32_RLZ_AND_MAP(s, syscfg, armv7m, false, NULL)
	// syscfg has no irq
    // sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, cfg->syscfg.irq));

    rcc = DEVICE(&(s->rcc));
    qdev_prop_set_uint32(rcc, "hsi_freq", cfg->rcc_hsi_freq);
    qdev_prop_set_uint32(rcc, "lsi_freq", cfg->rcc_lsi_freq);
    qdev_prop_set_uint32(rcc, "hse_freq", cfg->rcc_hse_freq);
    qdev_prop_set_uint32(rcc, "lse_freq", cfg->rcc_lse_freq);

    STM32_RLZ_AND_MAP(s, rcc, armv7m, false, NULL);

    STM32_RLZ_MAP_ALL(s, gpios, armv7m, rcc);

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
			qdev_prop_set_chr(DEVICE(&s->usarts[i]), "chardev", chrdev_assigns[i]);
        }
    }
    STM32_RLZ_MAP_ALL(s, usarts, armv7m, rcc);

    STM32_RLZ_MAP_ALL(s, timers, armv7m, rcc);

	STM32_RLZ_AND_MAP(s, adc_common, armv7m, false, NULL);

    s->adcs[0].common = &s->adc_common;
    s->adcs[1].common = &s->adc_common;
    s->adcs[2].common = &s->adc_common;

	STM32_RLZ_MAP_ALL(s, adcs, armv7m, rcc);

	STM32_RLZ_MAP_ALL(s, spis, armv7m, rcc);
	STM32_RLZ_MAP_ALL(s, i2cs, armv7m, rcc);

    STM32_RLZ_MAP_ALL(s, dmas, armv7m, rcc);

	STM32_CONNECT_DMAR_A(s, usarts, dmas, 2);
	STM32_CONNECT_DMAR_A(s, adcs, dmas, 2);

    /* EXTI device */
    dev = DEVICE(&s->exti);
    STM32_RLZ_AND_MAP(s, exti, armv7m, false,NULL);

    for (i = 0; i < 16; i++) {
        qdev_connect_gpio_out(DEVICE(&s->syscfg), i, qdev_get_gpio_in(dev, i));
    }

	STM32_RLZ_AND_MAP(s, rtc, armv7m, false, NULL);
	STM32_RLZ_AND_MAP(s, crc, armv7m, true, rcc);
    // TODO - RTC wakeup exti
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 0, qdev_get_gpio_in(exti_dev, 17));
    // // Alarm B
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 1, qdev_get_gpio_in(exti_dev, 17));
    // // Wake up timer
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 2, qdev_get_gpio_in(exti_dev, 22));

	STM32_RLZ_AND_MAP(s, flash_if, armv7m, false, NULL);
	STM32_RLZ_AND_MAP(s, otp, armv7m, false, NULL);
	STM32_RLZ_AND_MAP(s, iwdg, armv7m, true, rcc);
	STM32_RLZ_AND_MAP(s, rng, armv7m, true, rcc);
	STM32_RLZ_AND_MAP(s, itm, armv7m, false, NULL);
	STM32_RLZ_AND_MAP(s, pwr, armv7m, false, NULL);

    //s->otg_fs.debug = true;
    qdev_prop_set_chr(DEVICE(&s->usb_fs), "chardev", qemu_chr_find("stm32usbfscdc"));

    // IRQs: FS wakeup: 42 FS Global: 67
	STM32_RLZ_AND_MAP(s, usb_fs, armv7m, true, NULL);
   // qdev_connect_gpio_out_named(rcc,"reset",STM32_USB,qdev_get_gpio_in_named(DEVICE(&s->otg_hs),"rcc-reset",0));

    // USB IRQs:
    // Global HS: 77. WKUP: 76, EP1 in/out = 75/74.
    STM32_RLZ_AND_MAP(s, usb_hs, armv7m, true, rcc);

    qemu_check_nic_model(&nd_table[0], "stm32f4xx-ethernet");
    dev = qdev_new(cfg->eth.type);
    qdev_set_nic_properties(dev, &nd_table[0]);
    if (qemu_find_netdev("mini-eth")!=NULL){
        qdev_prop_set_string(dev,"netdev","mini-eth");
        qdev_prop_set_bit(dev, "connected", true);
    } else {
        printf("Ethernet disconnected. use -netdev id=mini-eth,[opts...] to connect it.\n");
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, cfg->eth.data.base_addr);
    // ETH GI = 61, Wakeup Exti = 62
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, qdev_get_gpio_in(armv7m, cfg->eth.data.irq[0]));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1, qdev_get_gpio_in(armv7m, cfg->eth.data.irq[1]));

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
    create_unimplemented_device("ETM/DBGMCU/TIPU", 0xE0000000, 0xFFFFF);

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
