/*
 * Prusa Buddy board machine model
 *
 * Copyright 2020-2023 VintagePC <github.com/vintagepc>
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
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "hw/arm/boot.h"
#include "hw/ssi/ssi.h"
#include "hw/loader.h"
#include "utility/ArgHelper.h"
#include "sysemu/runstate.h"
#include "parts/dashboard_types.h"
#include "stm32_common/stm32_shared.h"
#include "stm32_common/stm32_common.h"
#include "stm32_common/stm32_types.h"
#include "hw/arm/armv7m.h"

#define BOOTLOADER_IMAGE "bootloader.bin"

#define XFLASH_FN  "Prusa_Mini_xflash.bin"
#define EEPROM_FN  "Prusa_Mini_eeprom.bin"
#define EEPROM_SYS_FN  "Prusa_Mini_eeprom_sys.bin"

typedef struct mini_config_t {
    const char* flash_chip;
} mini_config_t;

static const mini_config_t mini_100_cfg = {
    .flash_chip = "w25q64jv"
};

static const mini_config_t mini_014_cfg = {
    .flash_chip = "w25w80d"
};

static void prusa_mini_init(MachineState *machine, const mini_config_t* cfg);

static void prusa_mini_014_init(MachineState *machine)
{
    prusa_mini_init(machine, &mini_014_cfg);
}

static void prusa_mini_100_init(MachineState *machine)
{
    prusa_mini_init(machine, &mini_100_cfg);
}


static void prusa_mini_init(MachineState *machine, const mini_config_t* cfg)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_STM32F407xG_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_uint32(dev,"sram-size", machine->ram_size);

	DeviceState* otp = stm32_soc_get_periph(dev, STM32_P_OTP);
	qdev_prop_set_uint32(otp, "len-otp-data",8);
	qdev_prop_set_uint32(otp, "otp-data[1]",1081065844);
	qdev_prop_set_uint32(otp, "otp-data[2]",0x56207942);
	qdev_prop_set_uint32(otp, "otp-data[3]",0x61746e69);
	qdev_prop_set_uint32(otp, "otp-data[4]",0x43506567);
	qdev_prop_set_uint32(otp, "otp-data[6]",0x04040000);
	qdev_prop_set_uint32(otp, "otp-data[7]",0x04040404);

    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.
    arghelper_setargs(machine->kernel_cmdline);
    int default_flash_size = stm32_soc_get_flash_size(dev);
    if (arghelper_is_arg("4x_flash"))
    {
        default_flash_size <<=2; // quadruple the flash size for debug code.
    }
    qdev_prop_set_uint32(dev,"flash-size", default_flash_size);
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
	DeviceState* dev_soc = dev;
    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.


    if (arghelper_is_arg("appendix")) {
		qdev_prop_set_uint32(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),"idr-mask", 0x2000);
    }

    char* kfn = machine->kernel_filename;
    int kernel_len = kfn ? strlen(kfn) : 0;
    if (kernel_len >3 && strncmp(kfn + (kernel_len-3), "bbf",3) == 0 )
    {
        // TODO... use initrd_image as a bootloader alternative?
        struct stat bootloader;
        if (stat(BOOTLOADER_IMAGE,&bootloader))
        {
            error_setg(&error_fatal, "No %s file found. It is required to use a .bbf file!",BOOTLOADER_IMAGE);
            return;
        }
        // BBF has an extra 64b header we need to prune. Rather than modify it or use a temp file, offset it
        // by -64 bytes and rely on the bootloader clobbering it.
        load_image_targphys(machine->kernel_filename,0x20000-64,get_image_size(machine->kernel_filename));
        armv7m_load_kernel(ARM_CPU(first_cpu),
            BOOTLOADER_IMAGE,
            default_flash_size);
    }
    else // Raw bin or ELF file, load directly.
    {
        armv7m_load_kernel(ARM_CPU(first_cpu),
                        machine->kernel_filename,
                        default_flash_size);
    }

    DeviceState* key_in = qdev_new("p404-key-input");
    sysbus_realize(SYS_BUS_DEVICE(key_in), &error_fatal);

    /* Wire up display */

    void *bus;
    {
        bus = qdev_get_child_bus(stm32_soc_get_periph(dev_soc, STM32_P_SPI2), "ssi");

        DeviceState *lcd_dev = ssi_create_peripheral(bus, "st7789v");
        qemu_irq lcd_cs = qemu_irq_invert(qdev_get_gpio_in_named(lcd_dev, SSI_GPIO_CS, 0));

        /* Make sure the select pin is high.  */
        qemu_irq_raise(lcd_cs);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),9,lcd_cs);

        qemu_irq lcd_cd = qdev_get_gpio_in(lcd_dev,0);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD),11, lcd_cd);

		qemu_irq lcd_reset = qdev_get_gpio_in_named(lcd_dev,"reset",0);
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),8,lcd_reset);

		qdev_connect_gpio_out_named(lcd_dev, "reset-out", 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),8));
    }
    BlockBackend *blk = NULL;
    {
        bus = qdev_get_child_bus(stm32_soc_get_periph(dev_soc, STM32_P_SPI3), "ssi");
        dev = qdev_new(cfg->flash_chip);
        blk = get_or_create_drive(IF_MTD, 0, XFLASH_FN, XFLASH_ID,  8U*MiB, &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize_and_unref(dev, bus, &error_fatal);
        qemu_irq flash_cs = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        qemu_irq_raise(flash_cs);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 7, flash_cs);



    }
    {
        bus = qdev_get_child_bus(stm32_soc_get_periph(dev_soc, STM32_P_I2C1),"i2c");
        dev = qdev_new("at24c-eeprom");
        qdev_prop_set_uint8(dev, "address", 0x53);
        qdev_prop_set_uint32(dev, "rom-size", 64*KiB / 8U);
		blk = get_or_create_drive(IF_PFLASH, 0, EEPROM_FN, EEPROM_ID, 64*KiB / 8U, &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize(dev, bus, &error_fatal);
        // The QEMU I2CBus doesn't support devices with multiple addresses, so fake it
        // with a second instance at the SYSTEM address.
        dev = qdev_new("at24c-eeprom");
        qdev_prop_set_uint8(dev, "address", 0x57);
        qdev_prop_set_uint32(dev, "rom-size", 64*KiB / 8U);
		blk = get_or_create_drive(IF_PFLASH, 1, EEPROM_SYS_FN, EEPROM_SYS_ID,  64*KiB / 8U,  &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize(dev, bus, &error_fatal);
    }

    DeviceState* pinda = qdev_new("pinda");
    sysbus_realize(SYS_BUS_DEVICE(pinda), &error_fatal);

    // DeviceState *vis = qdev_new("mini-visuals");
    // sysbus_realize(SYS_BUS_DEVICE(vis), &error_fatal);
#ifdef BUDDY_HAS_GL
    DeviceState *gl_db = qdev_new("gl-dashboard");
    if (arghelper_is_arg("gfx-full")) {
        qdev_prop_set_uint8(gl_db, "dashboard_type", DB_MINI_FULL);
    } else if (arghelper_is_arg("gfx-lite")) {
        qdev_prop_set_uint8(gl_db, "dashboard_type", DB_MINI_LITE);
    }
    sysbus_realize(SYS_BUS_DEVICE(gl_db), &error_fatal);
#else
    printf("NOTE: GL support is not compiled in, gfx-* options will be ignored.\n");
#endif

    DeviceState *db2 = qdev_new("2d-dashboard");
    qdev_prop_set_uint8(db2, "fans", 2);
    qdev_prop_set_uint8(db2, "thermistors", 5);
    qdev_prop_set_string(db2, "indicators", "ZF");


    {
        static const char names[4] = {'X','Y','Z','E'};
        static const char* links[4] = {"motor[0]","motor[1]","motor[2]","motor[3]"};
        static const uint8_t addresses[4] = {1, 3,0,2};
        static const uint8_t step_pins[4] = {1, 13, 4, 9};
        static const uint8_t dir_pins[4] = {0, 12, 15, 8};
        static const uint8_t en_pins[4] = {3, 14, 2, 10};
        static const uint8_t diag_pins[4] = {2, 1, 3, 15};
        static const uint8_t diag_ports[4] = {STM32_P_GPIOE, STM32_P_GPIOE, STM32_P_GPIOE, STM32_P_GPIOA};
        static const uint8_t is_inverted[4] = {1,1,0,0};
        static const int32_t ends[4] = { 100*16*182, 100*16*183, 400*16*185,0 };
        static const int32_t stepsize[4] = { 100*16, 100*16, 400*16, 320*16 };


        // bus = qdev_get_child_bus(DEVICE(&SOC->usart2),"spi");
        DeviceState* split_out = qdev_new("split-irq");
        qdev_prop_set_uint16(split_out, "num-lines", 4);
        qdev_realize_and_unref(DEVICE(split_out),NULL,  &error_fatal);
        qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_UART2),"uart-byte-out", 0, qdev_get_gpio_in(split_out,0));
        DeviceState* split_zmin = qdev_new("split-irq");
        qdev_prop_set_uint16(split_zmin, "num-lines", 3);
        qdev_realize_and_unref(DEVICE(split_zmin),NULL,  &error_fatal);
        qdev_connect_gpio_out(split_zmin, 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),8));
        qdev_connect_gpio_out(split_zmin, 1, qdev_get_gpio_in_named(db2,"led-digital",0));
#ifdef BUDDY_HAS_GL
        qdev_connect_gpio_out(split_zmin, 2, qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_ZPROBE));
#endif
        qdev_connect_gpio_out(pinda, 0,  qdev_get_gpio_in(split_zmin,0));

        for (int i=0; i<4; i++){
            dev = qdev_new("tmc2209");
            qdev_prop_set_uint8(dev, "axis",names[i]);
            qdev_prop_set_uint8(dev, "address", addresses[i]);
            qdev_prop_set_uint8(dev, "inverted",is_inverted[i]);
            qdev_prop_set_int32(dev, "max_step", ends[i]);
            qdev_prop_set_int32(dev, "fullstepspermm", stepsize[i]);
            sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
            qdev_connect_gpio_out_named(dev,"byte-out", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_UART2),"uart-byte-in",0));
            qdev_connect_gpio_out(split_out,i, qdev_get_gpio_in_named(dev,"byte-in",0));
            qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), step_pins[i], qdev_get_gpio_in_named(dev,"step",0));
            qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), dir_pins[i], qdev_get_gpio_in_named(dev,"dir",0));
            object_property_set_link(OBJECT(db2), links[i], OBJECT(dev), &error_fatal);
            qdev_connect_gpio_out_named(dev,"diag", 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, diag_ports[i]),diag_pins[i]));
            qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), en_pins[i],qdev_get_gpio_in_named(dev,"enable",0));
#ifdef BUDDY_HAS_GL
            if (i<3) {
                qemu_irq split_step = qemu_irq_split(
                    qdev_get_gpio_in_named(pinda,"position_xyz",i),
                    qdev_get_gpio_in_named(gl_db,"motor-step",DB_MOTOR_X+i)
                );
                qdev_connect_gpio_out_named(dev,"step-out", 0,split_step);
            } else {
                qdev_connect_gpio_out_named(dev,"step-out", 0, qdev_get_gpio_in_named(gl_db,"motor-step",DB_MOTOR_X+i));
            }
#else
            if (i<3) {
                qdev_connect_gpio_out_named(dev,"step-out", 0, qdev_get_gpio_in_named(pinda,"position_xyz",i));
            }
#endif
        }

    }

    sysbus_realize(SYS_BUS_DEVICE(db2), &error_fatal);

    uint16_t startvals[] = {18,18, 25, 512, 512};
    uint8_t channels[] = {10,4,3,5,6};
    int tables[] = {5, 1, 2000,0,0};
    DeviceState *bed = NULL, *hotend = NULL;
    for (int i=0; i<5; i++)
    {
        dev = qdev_new("thermistor");
        if (i==0) hotend = dev;
        if (i==1) bed = dev;
        qdev_prop_set_uint16(dev, "temp",startvals[i]);
        qdev_prop_set_uint16(dev, "table_no", tables[i]);
        qdev_prop_set_uint8(dev, "index", i);
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        //qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", channels[i],  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
        qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",channels[i]));
        qdev_connect_gpio_out_named(dev, "temp_out_256x", 0, qdev_get_gpio_in_named(db2,"therm-temp",i));

    }
    // Heaters - bed is B0/ TIM3C3, E is B1/ TIM3C4

    dev = qdev_new("heater");
    qdev_prop_set_uint8(dev, "thermal_mass_x10",30);
    qdev_prop_set_uint8(dev,"label", 'E');
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM3),"pwm_ratio_changed",3,qdev_get_gpio_in_named(dev, "pwm_in",0));
    qdev_connect_gpio_out_named(dev, "temp_out",0, qdev_get_gpio_in_named(hotend, "thermistor_set_temperature",0));
    qdev_connect_gpio_out_named(dev, "pwm-out", 0, qdev_get_gpio_in_named(db2,"therm-pwm",0));
#ifdef BUDDY_HAS_GL
    qemu_irq split_htr = qemu_irq_split(qdev_get_gpio_in_named(db2,"therm-pwm",0),qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_HTR));
    qdev_connect_gpio_out_named(dev, "pwm-out", 0, split_htr);
#else
    qdev_connect_gpio_out_named(dev, "pwm-out", 0, qdev_get_gpio_in_named(db2,"therm-pwm",0));
#endif

    // Bed.
    dev = qdev_new("heater");
    qdev_prop_set_uint8(dev, "thermal_mass_x10",3);
    qdev_prop_set_uint8(dev,"label", 'B');
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM3),"pwm_ratio_changed",2,qdev_get_gpio_in_named(dev, "pwm_in",0));
    qdev_connect_gpio_out_named(dev, "temp_out",0, qdev_get_gpio_in_named(bed, "thermistor_set_temperature",0));
    qdev_connect_gpio_out_named(dev, "pwm-out", 0, qdev_get_gpio_in_named(db2,"therm-pwm",1));
#ifdef BUDDY_HAS_GL
    qemu_irq split_bed = qemu_irq_split(qdev_get_gpio_in_named(db2,"therm-pwm",1),qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_BED));
    qdev_connect_gpio_out_named(dev, "pwm-out", 0, split_bed);
#else
    qdev_connect_gpio_out_named(dev, "pwm-out", 0, qdev_get_gpio_in_named(db2,"therm-pwm",1));
#endif


    dev = qdev_new("ir-sensor");
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qemu_irq split_fsensor = qemu_irq_split( qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOB),4),qemu_irq_invert(qdev_get_gpio_in_named(db2,"led-digital",1)));
    qdev_connect_gpio_out(dev, 0, split_fsensor);

    // hotend = fan1
    // print fan = fan0
    uint16_t fan_max_rpms[] = { 6600, 8000 };
    uint8_t  fan_pwm_pins[] = { 11, 9};
    uint8_t fan_tach_pins[] = { 10, 14};
    uint8_t fan_labels[] = {'P','E'};
    for (int i=0; i<2; i++)
    {
        dev = qdev_new("fan");
        qdev_prop_set_uint8(dev,"label",fan_labels[i]);
        qdev_prop_set_uint32(dev, "max_rpm",fan_max_rpms[i]);
        qdev_prop_set_bit(dev, "is_nonlinear", i); // E is nonlinear.
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        qdev_connect_gpio_out_named(dev, "tach-out",0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOE),fan_tach_pins[i]));
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOE),fan_pwm_pins[i],qdev_get_gpio_in_named(dev, "pwm-in-soft",0));
        qdev_connect_gpio_out_named(dev, "rpm-out", 0, qdev_get_gpio_in_named(db2,"fan-rpm",i));
#ifdef BUDDY_HAS_GL
        qemu_irq split_fan = qemu_irq_split(qdev_get_gpio_in_named(db2,"fan-pwm",i),qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_PFAN+i));
        qdev_connect_gpio_out_named(dev, "pwm-out", 0, split_fan);
#else
        qdev_connect_gpio_out_named(dev, "pwm-out", 0, qdev_get_gpio_in_named(db2,"fan-pwm",i));
#endif
    }

    dev = qdev_new("encoder-input");
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qdev_connect_gpio_out_named(dev, "encoder-button",	0, 	qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOE),12));
    qdev_connect_gpio_out_named(dev, "encoder-a",		0, 	qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOE),15));
    qdev_connect_gpio_out_named(dev, "encoder-b",		0,  qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOE),13));

    // Needs to come last because it has the scripting engine setup.
    dev = qdev_new("p404-scriptcon");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    // Check for high-level non configuration arguments like help outputs and handle them.
    if (!arghelper_parseargs())
    {
        // We processed an arg that wants us to quit after it's done.
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }

};


static void prusa_mini_machine_init(MachineClass *mc)
{
    mc->desc = "Prusa Mini 1.0+";
    mc->family = "Prusa Mini";
    mc->init = prusa_mini_100_init;
    mc->default_ram_size = 0; // 0 = use default RAM from chip.
    mc->no_parallel = 1;
	mc->no_serial = 1;
}

DEFINE_MACHINE("prusa-mini", prusa_mini_machine_init)

static void prusa_mini_014_machine_init(MachineClass *mc)
{
    mc->desc = "Prusa Mini 0.1.4";
    mc->family = "Prusa Mini";
    mc->init = prusa_mini_014_init;
    mc->default_ram_size = 0; // 0 = use default RAM from chip.
    mc->no_parallel = 1;
	mc->no_serial = 1;
}

DEFINE_MACHINE("prusa-mini-014", prusa_mini_014_machine_init)

// Don't enable this for tests, it breaks because it doesn't run.
#ifndef CONFIG_GCOV

static void buddy_machine_init(MachineClass *mc)
{
    mc->desc = "Prusa Mini Board";
    mc->deprecation_reason = "prusabuddy has been deprecated because it's a board, not a machine. Use -machine prusa-mini instead";
}

DEFINE_MACHINE("prusabuddy", buddy_machine_init)
#endif
