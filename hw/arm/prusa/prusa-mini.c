/*
 * Prusa Buddy board machine model
 *
 * Copyright 2020 VintagePC <github.com/vintagepc>
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
#include "stm32f407/stm32f407_soc.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "utility/ArgHelper.h"
#include "sysemu/runstate.h"
#include "parts/dashboard_types.h"

#define BOOTLOADER_IMAGE "bootloader.bin"

static void prusa_mini_init(MachineState *machine)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_STM32F407xG_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_uint32(dev,"sram-size", machine->ram_size);
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    STM32F4xxState *SOC = STM32F4XX_BASE(dev);
	DeviceState* dev_soc = dev;
    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.
    arghelper_setargs(machine->kernel_cmdline);
    int default_flash_size = stm32_soc_get_flash_size(dev);
    if (arghelper_is_arg("4x_flash"))
    {
        default_flash_size <<=2; // quadruple the flash size for debug code.
    }
    if (arghelper_is_arg("appendix")) {
        SOC->gpios[0].idr_mask |= 0x2000;
    }
    int kernel_len = strlen(machine->kernel_filename);
    if (kernel_len >3)
    {
        const char* kernel_ext = machine->kernel_filename+(kernel_len-3);
        if (strncmp(kernel_ext, "bbf",3)==0)
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
    }

    DeviceState* key_in = qdev_new("p404-key-input");
    sysbus_realize(SYS_BUS_DEVICE(key_in), &error_fatal);

    /* Wire up display */

    void *bus;
    {
        bus = qdev_get_child_bus(DEVICE(&SOC->spis[1]), "ssi");

        DeviceState *lcd_dev = ssi_create_peripheral(bus, "st7789v");
        qemu_irq lcd_cs = qdev_get_gpio_in_named(lcd_dev, SSI_GPIO_CS, 0);

        /* Make sure the select pin is high.  */
        qemu_irq_raise(lcd_cs);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),9,lcd_cs);

        qemu_irq lcd_cd = qdev_get_gpio_in(lcd_dev,0);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD),11, lcd_cd);
    }
    DriveInfo *dinfo = NULL;
    {
        bus = qdev_get_child_bus(DEVICE(&SOC->spis[2]), "ssi");
        dev = qdev_new("w25q64jv");
        dinfo = drive_get_next(IF_MTD);
        if (dinfo) {
            qdev_prop_set_drive(dev, "drive",
                                blk_by_legacy_dinfo(dinfo));
        }
        qdev_realize_and_unref(dev, bus, &error_fatal);
        //DeviceState *flash_dev = ssi_create_slave(bus, "w25q64jv");
        qemu_irq flash_cs = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        qemu_irq_raise(flash_cs);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOD), 7, flash_cs);



    }
    {
        bus = qdev_get_child_bus(DEVICE(&SOC->i2cs[0]),"i2c");
        dev = qdev_new("at24c-eeprom");
        qdev_prop_set_uint8(dev, "address", 0x53);
        qdev_prop_set_uint32(dev, "rom-size", 64*KiB);
        dinfo = drive_get_next(IF_PFLASH);
        if (dinfo) {
            qdev_prop_set_drive(dev, "drive",
                                blk_by_legacy_dinfo(dinfo));
        }
        qdev_realize(dev, bus, &error_fatal);
        // The QEMU I2CBus doesn't support devices with multiple addresses, so fake it
        // with a second instance at the SYSTEM address.
        // bus = qdev_get_child_bus(DEVICE(&SOC->i2cs[0]),"i2c");
        dev = qdev_new("at24c-eeprom");
        qdev_prop_set_uint8(dev, "address", 0x57);
        qdev_prop_set_uint32(dev, "rom-size", 64*KiB);
        dinfo = drive_get_next(IF_PFLASH);
        if (dinfo) {
            qdev_prop_set_drive(dev, "drive",
                                blk_by_legacy_dinfo(dinfo));
        }
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
        qdev_connect_gpio_out_named(DEVICE(&SOC->usarts[1]),"uart-byte-out", 0, qdev_get_gpio_in(split_out,0));
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
            qdev_connect_gpio_out_named(dev,"byte-out", 0, qdev_get_gpio_in_named(DEVICE(&SOC->usarts[1]),"uart-byte-in",0));
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
        qdev_connect_gpio_out_named(DEVICE(&SOC->adcs[0]),"adc_read", channels[i],  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
        qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(DEVICE(&SOC->adcs[0]),"adc_data_in",channels[i]));
        qdev_connect_gpio_out_named(dev, "temp_out_256x", 0, qdev_get_gpio_in_named(db2,"therm-temp",i));

    }
    // Heaters - bed is B0/ TIM3C3, E is B1/ TIM3C4

    dev = qdev_new("heater");
    qdev_prop_set_uint8(dev, "thermal_mass_x10",30);
    qdev_prop_set_uint8(dev,"label", 'E');
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(&SOC->timers[2]),"pwm_ratio_changed",3,qdev_get_gpio_in_named(dev, "pwm_in",0));
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
    qdev_connect_gpio_out_named(DEVICE(&SOC->timers[2]),"pwm_ratio_changed",2,qdev_get_gpio_in_named(dev, "pwm_in",0));
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
    mc->desc = "Prusa Mini";
    mc->init = prusa_mini_init;
    mc->default_ram_size = 0; // 0 = use default RAM from chip.
    mc->no_parallel = 1;
	mc->no_serial = 1;
}

DEFINE_MACHINE("prusa-mini", prusa_mini_machine_init)

static void buddy_machine_init(MachineClass *mc)
{
    mc->desc = "Prusa Mini Board";
    mc->deprecation_reason = "prusabuddy has been deprecated because it's a board, not a machine. Use -machine prusa-mini instead";
}

DEFINE_MACHINE("prusabuddy", buddy_machine_init)
