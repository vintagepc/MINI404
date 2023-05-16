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
#include "hw/ssi/ssi.h"
#include "hw/i2c/i2c.h"
#include "hw/core/split-irq.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "utility/ArgHelper.h"
#include "sysemu/runstate.h"
#include "parts/dashboard_types.h"
#include "parts/xl_bridge.h"
#include "stm32_common/stm32_common.h"
#include "hw/arm/armv7m.h"
#include "parts/spi_rgb.h"

#define BOOTLOADER_IMAGE "Prusa_XL_Boot.bin"
#define XFLASH_FN  "Prusa_XL_xflash.bin"
#define EEPROM_FN  "Prusa_XL_eeprom.bin"
#define EEPROM_SYS_FN  "Prusa_XL_eeprom_sys.bin"
// Upper 8 pins are GPIO
typedef uint16_t stm_pin;

static uint8_t BANK(stm_pin pin) {
    return ((pin >> 8U) + STM32_P_GPIOA);
}

static uint8_t PIN(stm_pin pin) {
    return pin & 0xFFU;
}

#define STM_PIN(gpio,num) (((STM32_P_##gpio - STM32_P_GPIOA) & 0xFF)<< 8) | (num & 0xFF)

enum {
    NONE,
    TMC2209,
    TMC2130,
};

enum {
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    AXIS_E,
    AXIS_MAX,
};

typedef struct xl_cfg_t {
    uint8_t lcd_spi;
    stm_pin lcd_cs;
    bool lcd_cs_invert;
    stm_pin lcd_cd;
    uint8_t w25_spi;
    stm_pin w25_cs;
    uint8_t at24_i2c;
	uint8_t usbc_i2c;
    stm_pin hx717_sck;
    stm_pin hx717_data;
    stm_pin enc_a;
    stm_pin enc_b;
    stm_pin enc_btn;
    stm_pin z_min;
    bool has_at21;
    int e_table_index;
    uint8_t motor;
    char m_label[AXIS_MAX];
	bool m_inversion[AXIS_MAX];
    stm_pin m_step[AXIS_MAX];
    stm_pin m_dir[AXIS_MAX];
    stm_pin m_en[AXIS_MAX];
    stm_pin m_diag[AXIS_MAX];
    stm_pin m_select[AXIS_MAX];
    uint8_t m_spi;
	uint8_t m_uart;
	bool is_400step;

} xl_cfg_t;

static const xl_cfg_t xl_cfg = {
    .lcd_spi = STM32_P_SPI6,
    .lcd_cs = STM_PIN(GPIOD,11),
    .lcd_cs_invert = false,
    .lcd_cd = STM_PIN(GPIOD,15),
    .w25_spi = STM32_P_SPI5,
    .w25_cs = STM_PIN(GPIOF,2),
    .at24_i2c = STM32_P_I2C2,
	.usbc_i2c = STM32_P_I2C1,
    .hx717_data = STM_PIN(GPIOE,7),
    .hx717_sck = STM_PIN(GPIOG,1),
    .enc_a = STM_PIN(GPIOD,13),
    .enc_b = STM_PIN(GPIOD,12),
    .enc_btn = STM_PIN(GPIOG,3),
    .z_min = STM_PIN(GPIOB, 8),
    .has_at21 = false, // NOT IMPLEMENTED YET
    .e_table_index = 2005,
    .motor = TMC2130,
    .m_label = {'A','B','Z','E'},
	.m_inversion = {1,1,0,0},
    .m_step = { STM_PIN(GPIOD,5), STM_PIN(GPIOD,7), STM_PIN(GPIOD,3), STM_PIN(GPIOD,1)},
    .m_dir = { STM_PIN(GPIOD,6), STM_PIN(GPIOD,4), STM_PIN(GPIOD,2), STM_PIN(GPIOD,0)},
    .m_en = { STM_PIN(GPIOB,9), STM_PIN(GPIOB,9), STM_PIN(GPIOB,8), STM_PIN(GPIOD,10)},
    .m_diag = { STM_PIN(GPIOG,9), STM_PIN(GPIOE,13), STM_PIN(GPIOB,4), STM_PIN(GPIOD,14)},
    .m_select = {STM_PIN(GPIOG,15), STM_PIN(GPIOB,5), STM_PIN(GPIOG,10), STM_PIN(GPIOF,12)},
    .m_spi = STM32_P_SPI3,
	.m_uart = STM32_P_UART1,
	.is_400step = false,
};

static const xl_cfg_t xl_cfg_050 = {
    .lcd_spi = STM32_P_SPI6,
    .lcd_cs = STM_PIN(GPIOD,11),
    .lcd_cs_invert = false,
    .lcd_cd = STM_PIN(GPIOD,15),
    .w25_spi = STM32_P_SPI5,
    .w25_cs = STM_PIN(GPIOF,2),
    .at24_i2c = STM32_P_I2C2,
	.usbc_i2c = STM32_P_I2C1,
    .hx717_data = STM_PIN(GPIOE,7),
    .hx717_sck = STM_PIN(GPIOG,1),
    .enc_a = STM_PIN(GPIOD,13),
    .enc_b = STM_PIN(GPIOD,12),
    .enc_btn = STM_PIN(GPIOG,3),
    .z_min = STM_PIN(GPIOB, 8),
    .has_at21 = false, // NOT IMPLEMENTED YET
    .e_table_index = 2005,
    .motor = TMC2130,
    .m_label = {'A','B','Z','E'},
	.m_inversion = {1,0,0,0},
    .m_step = { STM_PIN(GPIOA,0), STM_PIN(GPIOA,3), STM_PIN(GPIOD,3), STM_PIN(GPIOD,1)},
    .m_dir = { STM_PIN(GPIOD,6), STM_PIN(GPIOD,4), STM_PIN(GPIOD,2), STM_PIN(GPIOD,0)},
    .m_en = { STM_PIN(GPIOB,9), STM_PIN(GPIOB,9), STM_PIN(GPIOB,8), STM_PIN(GPIOD,10)},
    .m_diag = { STM_PIN(GPIOG,9), STM_PIN(GPIOE,13), STM_PIN(GPIOB,4), STM_PIN(GPIOD,14)},
    .m_select = {STM_PIN(GPIOG,15), STM_PIN(GPIOB,5), STM_PIN(GPIOG,10), STM_PIN(GPIOF,12)},
    .m_spi = STM32_P_SPI3,
	.m_uart = STM32_P_UART1,
	.is_400step = false,
};

#define DWARF_BOOTLOADER_IMAGE "bl_dwarf.elf.bin"

static void xl_init(MachineState *machine, xl_cfg_t cfg)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_STM32F427xI_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_uint32(dev,"sram-size", machine->ram_size);
	//qdev_prop_set_uint32(dev,"flash-size", 0);
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
	DeviceState* dev_soc = dev;
    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.
    arghelper_setargs(machine->kernel_cmdline);
	bool args_continue_running = arghelper_parseargs();

	uint64_t flash_size = stm32_soc_get_flash_size(dev);

    if (arghelper_is_arg("appendix")) {
        qdev_prop_set_uint32(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),"idr-mask", 0x2000);
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
                flash_size);
        }
        else // Raw bin or ELF file, load directly.
        {
            armv7m_load_kernel(ARM_CPU(first_cpu),
                            machine->kernel_filename,
                            flash_size);
        }
    }

	DeviceState* key_in = qdev_new("p404-key-input");
    sysbus_realize(SYS_BUS_DEVICE(key_in), &error_fatal);

    /* Wire up display */
    void *bus;
	DeviceState* npixel[6];
	DeviceState* lcd_dev;
    {
		bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, cfg.lcd_spi),
			"ssi");

        lcd_dev = ssi_create_peripheral(bus, "ili9488");

        for (int i=0; i<4; i++) {
            npixel[i] = qdev_new("spi_rgb");
            if (i==3) {
                qdev_prop_set_uint8(npixel[i],"led-type",SPI_RGB_WS2811);
            }
            ssi_realize_and_unref(npixel[i], bus, &error_fatal);
            if (i>0)
            {
                qdev_connect_gpio_out(npixel[i-1], 0, qdev_get_gpio_in(npixel[i], 0));
                qdev_connect_gpio_out_named(npixel[i-1], "reset-out", 0, qdev_get_gpio_in_named(npixel[i], "reset", 0));
            }
            qdev_connect_gpio_out_named(npixel[i],"colour",0,
                qdev_get_gpio_in_named(lcd_dev,"leds",i)
                );
        }
		npixel[5] = qdev_new("spi_rgb");
		qdev_prop_set_uint8(npixel[5],"led-type",SPI_RGB_WS2811);
		qdev_prop_set_uint8(npixel[5],"flags",SPI_RGB_FLAG_ALT_TIMINGS | SPI_RGB_FLAG_INVERTED);
		ssi_realize_and_unref(npixel[5],
			(SSIBus*) qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, STM32_P_SPI4),
			"ssi"),
		&error_fatal);
		npixel[4] = qdev_new("spi_rgb");
		qdev_prop_set_uint8(npixel[4],"led-type",SPI_RGB_WS2811);
		qdev_prop_set_uint8(npixel[4],"flags", SPI_RGB_FLAG_ALT_TIMINGS | SPI_RGB_FLAG_INVERTED | SPI_RGB_FLAG_NO_CS);
		ssi_realize_and_unref(npixel[4],
			(SSIBus*) qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, STM32_P_SPI4),
			"ssi"),
		&error_fatal);
		qdev_connect_gpio_out(npixel[4], 0, qdev_get_gpio_in(npixel[5], 0));
		qdev_connect_gpio_out_named(npixel[4], "reset-out", 0, qdev_get_gpio_in_named(npixel[5], "reset", 0));

		qemu_irq lcd_cs = qdev_get_gpio_in_named(lcd_dev, SSI_GPIO_CS, 0);
        if (cfg.lcd_cs_invert) {
            lcd_cs = qemu_irq_invert(lcd_cs);
            qemu_irq_lower(lcd_cs);
        } else {
            /* Make sure the select pin is high.  */
            qemu_irq_raise(lcd_cs);
        }
        qemu_irq lcd_cd = qdev_get_gpio_in(lcd_dev,0);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.lcd_cd)),PIN(cfg.lcd_cd), lcd_cd);

		qemu_irq led_cs = qemu_irq_split(lcd_cs, qdev_get_gpio_in_named(npixel[0], SSI_GPIO_CS, 0));
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.lcd_cs)),PIN(cfg.lcd_cs),led_cs);
    }

	{
        bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, cfg.usbc_i2c),
			"i2c");
        dev = qdev_new("fusb302b");
        qdev_prop_set_uint8(dev, "address", 0x22);
        qdev_realize(dev, bus, &error_fatal);
	}

    BlockBackend *blk = NULL;
    {
        bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, cfg.w25_spi),
			 "ssi");
        dev = qdev_new("w25q64jv");
        blk = get_or_create_drive(IF_MTD, 0, XFLASH_FN, XFLASH_ID,  8U*MiB, &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize_and_unref(dev, bus, &error_fatal);
        //DeviceState *flash_dev = ssi_create_slave(bus, "w25q64jv");
        qemu_irq flash_cs = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        qemu_irq_raise(flash_cs);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.w25_cs)), PIN(cfg.w25_cs), flash_cs);
    }
	DeviceState* expander = NULL;
    {
        bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, cfg.at24_i2c),
			"i2c");

		expander = DEVICE(i2c_slave_create_simple(bus, "pca9557", 0x19));

        dev = qdev_new("at24c-eeprom");
        qdev_prop_set_uint8(dev, "address", 0x53);
        qdev_prop_set_uint32(dev, "rom-size", 64*KiB / 8U);
		blk = get_or_create_drive(IF_PFLASH, 0, EEPROM_FN, EEPROM_ID, 64*KiB / 8U, &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize(dev, bus, &error_fatal);
        // The QEMU I2CBus doesn't support devices with multiple addresses, so fake it
        // with a second instance at the SYSTEM address.
        // bus = qdev_get_child_bus(DEVICE(&SOC->i2cs[0]),"i2c");
        dev = qdev_new("at24c-eeprom");
        qdev_prop_set_uint8(dev, "address", 0x57);
        qdev_prop_set_uint32(dev, "rom-size", 64*KiB / 8U);
		blk = get_or_create_drive(IF_PFLASH, 1, EEPROM_SYS_FN, EEPROM_SYS_ID,  64*KiB / 8U,  &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize(dev, bus, &error_fatal);
    }

    DeviceState *motors[4];
#ifdef BUDDY_HAS_GL
    DeviceState *gl_db = qdev_new("gl-dashboard");
    if (arghelper_is_arg("gfx-lite")) {
        qdev_prop_set_uint8(gl_db, "dashboard_type", DB_MK4_LITE);
    }
    sysbus_realize(SYS_BUS_DEVICE(gl_db), &error_fatal);
#endif

	enum {
		IND_Z,
		IND_RGB,
		IND_WHITE,
		IND_CHEESE,
		IND_F_UNUSED,
		IND_FS0,
		IND_FS1,
		IND_FS2,
		IND_FS3,
		IND_FS4,
		IND_FS5
	};

	DeviceState *db2 = qdev_new("2d-dashboard");
	static const char* links[4] = {"motor[0]","motor[1]","motor[4]"};
	qdev_prop_set_uint8(db2, "fans", 0);
	qdev_prop_set_uint8(db2, "thermistors", 0);
	qdev_prop_set_string(db2, "indicators", "ZRWCF123456");

    {
        static int32_t stepsize[4] = { 80*16, 80*16, 800*16, 400*16 };
        static int32_t ends[4] = { 0*80*16*362, 0*80*16*362, 800*16*370,0 };


        // XY enable is split out
        DeviceState *split_en_out = qdev_new(TYPE_SPLIT_IRQ);
#ifdef BUDDY_HAS_GL
        qdev_prop_set_uint16(split_en_out, "num-lines", 4);
#else
        qdev_prop_set_uint16(split_en_out, "num-lines", 2);
#endif // CONFIG_OPENGL

        qdev_realize_and_unref(DEVICE(split_en_out),NULL,  &error_fatal);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_en[0])), PIN(cfg.m_en[0]),qdev_get_gpio_in(split_en_out,0));
        DeviceState *motor_uart = NULL;
        BusState *motor_spi = NULL;
        switch (cfg.motor) {
            case TMC2130:
            {
                 motor_spi = qdev_get_child_bus(
						stm32_soc_get_periph(dev_soc, cfg.m_spi),
					"ssi");
            }
            break;
            case TMC2209:
            {
                motor_uart = qdev_new(TYPE_SPLIT_IRQ);
                qdev_prop_set_uint16(motor_uart, "num-lines", 4);
                qdev_realize_and_unref(DEVICE(motor_uart),NULL,  &error_fatal);
 				qdev_connect_gpio_out_named(
						stm32_soc_get_periph(dev_soc, cfg.m_uart),
					"uart-byte-out", 0, qdev_get_gpio_in(motor_uart,0));
            }
            break;
            default:
            {
                error_setg(&error_fatal, "Unhandled motor type in cfg_t!");
                return;
            }
        }

        for (int i=0; i<AXIS_E; i++){
            if (cfg.m_label[i] == '\0' ) {
                continue;
            }
            switch (cfg.motor) {
                case TMC2130:
                {
                    dev = qdev_new("tmc2130");
                }
                break;
                case TMC2209:
                {
                    dev = qdev_new("tmc2209");
                    qdev_prop_set_uint8(dev, "address", (cfg.m_select[i]));
                }
                break;
            }
            // Common setup code:
            motors[i] = dev;
            qdev_prop_set_uint8(dev, "axis",cfg.m_label[i]);
            qdev_prop_set_uint8(dev, "inverted",cfg.m_inversion[i]);
            qdev_prop_set_int32(dev, "max_step", ends[i]);
            qdev_prop_set_int32(dev, "fullstepspermm", stepsize[i]);

            switch (cfg.motor) {
                case TMC2130:
                {
                    qdev_realize(dev, motor_spi, &error_fatal);
                    qemu_irq driver_cs = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
                    qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_select[i])),  PIN(cfg.m_select[i]), driver_cs);
                }
                break;
                case TMC2209:
                {
                    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
                    qdev_connect_gpio_out_named(dev,"byte-out", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, cfg.m_uart),"uart-byte-in",0));
                    qdev_connect_gpio_out(motor_uart,i, qdev_get_gpio_in_named(dev,"byte-in",0));
                }
                break;
            }
			qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_step[i])), PIN(cfg.m_step[i]), qdev_get_gpio_in_named(dev,"step",0));
			qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_dir[i])),  PIN(cfg.m_dir[i]),  qdev_get_gpio_in_named(dev,"dir",0));
            object_property_set_link(OBJECT(db2), links[i], OBJECT(dev), &error_fatal);
			qdev_connect_gpio_out_named(dev,"diag", 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(cfg.m_diag[i])),PIN(cfg.m_diag[i])));
            if (i<2) {
                qdev_connect_gpio_out(split_en_out, i, qdev_get_gpio_in_named(dev,"enable",0));
            } else {
                qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_en[i])),  PIN(cfg.m_en[i]), qdev_get_gpio_in_named(dev,"enable",0));
            }
#ifdef BUDDY_HAS_GL
            if (i==2) {
                qemu_irq split_zmin = qemu_irq_split( qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(cfg.z_min)),PIN(cfg.z_min)),qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_ZPROBE));
                qdev_connect_gpio_out_named(dev,"hard", 0, split_zmin);
            }
            qdev_connect_gpio_out_named(dev,"step-out", 0, qdev_get_gpio_in_named(gl_db,"motor-step",DB_MOTOR_X+i));
#else

            if (i==2) qdev_connect_gpio_out_named(dev,"hard", 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(cfg.z_min)),PIN(cfg.z_min)));
#endif
        }

    }

	dev = qdev_new("corexy-helper");
	sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
	qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in_named(motors[0],"ext-stall",0));
	qdev_connect_gpio_out(dev, 1, qdev_get_gpio_in_named(motors[1],"ext-stall",0));
    qdev_connect_gpio_out_named(motors[0],"um-out",0,qdev_get_gpio_in(dev,0));
    qdev_connect_gpio_out_named(motors[1],"um-out",0,qdev_get_gpio_in(dev,1));

	// Cheat/hack, the helper will alternate between the status structures it returns when they are polled by the UI.
	object_property_set_link(OBJECT(db2), "motor[2]", OBJECT(dev), &error_fatal);
	object_property_set_link(OBJECT(db2), "motor[3]", OBJECT(dev), &error_fatal);

    sysbus_realize(SYS_BUS_DEVICE(db2), &error_fatal);

	// Sidebar RGB
 	qdev_connect_gpio_out_named(npixel[4],"colour",0,
		qdev_get_gpio_in_named(db2,"led-rgb",IND_RGB)
	);
	// Sidebar W
	qdev_connect_gpio_out_named(npixel[5],"rgb-out",0,
		qdev_get_gpio_in_named(db2,"led-w",IND_WHITE)
	);
	// Cheese LED
	qdev_connect_gpio_out_named(npixel[5],"rgb-out",1,
		qdev_get_gpio_in_named(db2,"led-w",IND_CHEESE)
	);


    uint16_t startvals[] = {18,20, 25, 512, 20};
    uint8_t channels[] = {10,4,3,5,6};
    const int tables[] = {cfg.e_table_index, 1, 2000,0,5};
    for (int i=0; i<5; i++)
    {
        dev = qdev_new("thermistor");
        qdev_prop_set_uint16(dev, "temp",startvals[i]);
        qdev_prop_set_uint16(dev, "table_no", tables[i]);
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", channels[i],  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
        qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",channels[i]));
    }
    // Heaters - bed is B0/ TIM3C3, E is B1/ TIM3C4

    dev = qdev_new("ir-sensor");
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
#ifdef BUDDY_HAS_GL
    qemu_irq split_fsensor = qemu_irq_split( qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOB),4),qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_FSENS));
    qdev_connect_gpio_out(dev, 0, split_fsensor);
#else
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOB),4));
#endif

    DeviceState *lc = qdev_new("loadcell");
    sysbus_realize(SYS_BUS_DEVICE(lc), &error_fatal);
    qdev_connect_gpio_out_named(motors[2],"um-out",0,qdev_get_gpio_in(lc,0));

    dev = qdev_new("hx717");
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
  	qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(cfg.hx717_data)),PIN(cfg.hx717_data))); // EXTR_DATA
    qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.hx717_sck)),PIN(cfg.hx717_sck),qdev_get_gpio_in(dev, 0)); // EXTR_SCK

    qdev_connect_gpio_out(lc,0, qdev_get_gpio_in_named(dev,"input_x1000",0));

	// Hall sensor mux
	dev = qdev_new("hc4052");
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOF),12,qdev_get_gpio_in_named(dev,"select", 1)); // S0
    qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOG),6,qdev_get_gpio_in_named(dev, "select", 0)); // S1
    qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC3),"adc_read", 10,  qdev_get_gpio_in_named(dev, "adc_read_request",0));
    qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC3),"adc_read", 4,  qdev_get_gpio_in_named(dev, "adc_read_request",1));
	qdev_connect_gpio_out(dev,0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC3),"adc_data_in", 10));
    qdev_connect_gpio_out(dev,1, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC3),"adc_data_in", 4));

	#define N_HALL 6
    DeviceState* hall[N_HALL];
	for (int i=0; i<N_HALL; i++)
	{
		hall[i] = qdev_new("hall-sensor");
		qdev_prop_set_uint8(hall[i],"index",1U + i);
		qdev_prop_set_uint32(hall[i],"present-value",4010);
    	sysbus_realize(SYS_BUS_DEVICE(hall[i]), &error_fatal);
		qdev_connect_gpio_out_named(hall[i],"status", 0, qdev_get_gpio_in_named(db2,"led-digital",IND_FS0 + i));
		if (i<4)
		{
			qdev_connect_gpio_out(hall[i], 0, qdev_get_gpio_in_named(dev,"1Y", i));
		}
		else
		{
			qdev_connect_gpio_out(hall[i], 0, qdev_get_gpio_in_named(dev,"2Y", i-4));
		}
	}



    if (cfg.has_at21) {
        dev = qdev_new("at21csxx");
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        // 2-way bitbang
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOF),13,qdev_get_gpio_in(dev, 0));
        qdev_connect_gpio_out(dev,0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOF), 13));
    }

    dev = qdev_new("hc4052");
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),8,qdev_get_gpio_in_named(dev,"select",0)); // S0
    qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),15,qdev_get_gpio_in_named(dev, "select", 1)); // S1
    qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC2),"adc_read", 3,  qdev_get_gpio_in_named(dev, "adc_read_request",0));
    qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC2),"adc_read", 5,  qdev_get_gpio_in_named(dev, "adc_read_request",1));
	qdev_connect_gpio_out(dev,0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC2),"adc_data_in",3));
    qdev_connect_gpio_out(dev,1, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC2),"adc_data_in",5));

    // Heater v - only on if heating.
    DeviceState* vdev = qdev_new("powersource");
    qdev_prop_set_uint32(vdev,"mV",24000);
    sysbus_realize(SYS_BUS_DEVICE(vdev),&error_fatal);
    qdev_connect_gpio_out_named(vdev, "v_sense",0,qdev_get_gpio_in_named(dev,"1Y",1));

    // Heater current
    vdev = qdev_new("cs30bl");
    qdev_prop_set_uint32(vdev,"mA",5);
    sysbus_realize(SYS_BUS_DEVICE(vdev),&error_fatal);
    qdev_connect_gpio_out_named(vdev, "a_sense",0,qdev_get_gpio_in_named(dev,"2Y",0));

    // Bed V, always on.
    vdev = qdev_new("powersource");
    qdev_prop_set_uint32(vdev,"mV",24000);
    sysbus_realize(SYS_BUS_DEVICE(vdev),&error_fatal);
    qdev_connect_gpio_out_named(vdev, "v_sense",0,qdev_get_gpio_in_named(dev,"1Y",0));
    // qdev_connect_gpio_out_named(vdev, "a_sense",0,qdev_get_gpio_in_named(dev,"2Y",1));

    // TODO - THERM2, THERM3

	// Heater, input, MMU, USB HS, USB FS
	uint16_t oc_latches[] = {STM_PIN(GPIOG, 5), STM_PIN(GPIOG,6), STM_PIN(GPIOB, 6), STM_PIN(GPIOD, 9), STM_PIN(GPIOF, 14)};

	DeviceState* split_reset = qdev_new(TYPE_SPLIT_IRQ);
	qdev_prop_set_uint16(split_reset, "num-lines", ARRAY_SIZE(oc_latches));
	qdev_realize_and_unref(split_reset, NULL, &error_fatal);
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOG),7, qdev_get_gpio_in(split_reset,0));
	for(int i=0; i<ARRAY_SIZE(oc_latches); i++) {
		dev = qdev_new("oc-latch");
		sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
		qdev_connect_gpio_out(dev, 0, qemu_irq_invert(qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(oc_latches[i])), PIN(oc_latches[i]))));
		qdev_connect_gpio_out(split_reset,i, qdev_get_gpio_in(dev,0));
	}

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
#ifdef BUDDY_HAS_GL
        qdev_connect_gpio_out_named(dev, "pwm-out", 0, qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_PFAN+i));
#endif
    }

    DeviceState* encoder = qdev_new("encoder-input");
    sysbus_realize(SYS_BUS_DEVICE(encoder), &error_fatal);
    qdev_connect_gpio_out_named(encoder, "encoder-button",  0,  qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(cfg.enc_btn)), PIN(cfg.enc_btn)));
    qdev_connect_gpio_out_named(encoder, "encoder-a",       0,  qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(cfg.enc_a)),   PIN(cfg.enc_a)));
    qdev_connect_gpio_out_named(encoder, "encoder-b",       0,  qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(cfg.enc_b)),   PIN(cfg.enc_b)));

	bus = qdev_get_child_bus(
		stm32_soc_get_periph(dev_soc, STM32_P_I2C3),
	"i2c");
	dev = qdev_new("gt911");
	qdev_prop_set_uint8(dev, "address", 0x5D);
	qdev_realize(dev, bus, &error_fatal);
	qdev_connect_gpio_out(dev,0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),8));

	qemu_irq x_split = qemu_irq_split(qdev_get_gpio_in_named(lcd_dev, "cursor", 0), qdev_get_gpio_in_named(dev, "x_y_touch", 0));
	qemu_irq y_split = qemu_irq_split(qdev_get_gpio_in_named(lcd_dev, "cursor", 1), qdev_get_gpio_in_named(dev, "x_y_touch", 1));
	qemu_irq t_split = qemu_irq_split(qdev_get_gpio_in_named(lcd_dev, "cursor", 2), qdev_get_gpio_in_named(dev, "x_y_touch", 2));
	qdev_connect_gpio_out_named(encoder, "cursor_xy", 0, x_split);
    qdev_connect_gpio_out_named(encoder, "cursor_xy", 1, y_split);
    qdev_connect_gpio_out_named(encoder, "touch",     0, t_split);

    // Needs to come last because it has the scripting engine setup.
    dev = qdev_new("p404-scriptcon");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    // Check for high-level non configuration arguments like help outputs and handle them.
    if (!args_continue_running)
    {
		arghelper_parseargs(); // Reparse to get the help output for everything initialized since the first call.
        // We processed an arg that wants us to quit after it's done.
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
	else
	{
		dev = qdev_new("xl-bridge");
		qdev_prop_set_uint8(dev, "device", XL_DEV_XBUDDY);
		sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOG), 1, qdev_get_gpio_in_named(dev,"tx-assert",0));
		qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_UART3),"uart-byte-out", 0, qdev_get_gpio_in_named(dev, "byte-send",0));
		qdev_connect_gpio_out_named(dev, "byte-receive", 0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_UART3),"uart-byte-in", 0));

		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_step[AXIS_E])), PIN(cfg.m_step[AXIS_E]), qdev_get_gpio_in_named(dev,"gpio-in",XLBRIDGE_PIN_E_STEP));
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_dir[AXIS_E])),  PIN(cfg.m_dir[AXIS_E]),  qdev_get_gpio_in_named(dev,"gpio-in",XLBRIDGE_PIN_E_DIR));

		qdev_connect_gpio_out_named(motors[2],"um-out",0,qdev_get_gpio_in_named(dev,"gpio-in",XLBRIDGE_PIN_Z_UM));

		qdev_connect_gpio_out(expander, 7, qdev_get_gpio_in_named(dev,"reset-in",XL_DEV_BED));
		qdev_connect_gpio_out(expander, 1, qdev_get_gpio_in_named(dev,"reset-in",XL_DEV_T0));
		qdev_connect_gpio_out(expander, 2, qdev_get_gpio_in_named(dev,"reset-in",XL_DEV_T1));
		qdev_connect_gpio_out(expander, 3, qdev_get_gpio_in_named(dev,"reset-in",XL_DEV_T2));
		qdev_connect_gpio_out(expander, 4, qdev_get_gpio_in_named(dev,"reset-in",XL_DEV_T3));
		qdev_connect_gpio_out(expander, 5, qdev_get_gpio_in_named(dev,"reset-in",XL_DEV_T4));
	}
	//qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_dir[AXIS_E])),  PIN(cfg.m_dir[AXIS_E]),  qdev_get_gpio_in_named(dev,"gpio-in",XLBRIDGE_PIN_nAC_FAULT));

};

static void xl_core_init(MachineState *mc)
{
	xl_init(mc, xl_cfg);
}

static void xl_050_init(MachineState *mc)
{
	xl_init(mc, xl_cfg_050);
}


static void xl_machine_init(MachineClass *mc)
{
    mc->desc = "Prusa XL v0.4.0";
    mc->init = xl_core_init;
	mc->default_ram_size = 0; // 0 is "use chip default"
	mc->no_parallel = 1;
	mc->no_serial = 1;
}

static void xl_machine_init_v050(MachineClass *mc)
{
    mc->desc = "Prusa XL v0.5.0";
    mc->init = xl_050_init;
	mc->default_ram_size = 0; // 0 is "use chip default"
	mc->no_parallel = 1;
	mc->no_serial = 1;
}

DEFINE_MACHINE("prusa-xl-040", xl_machine_init)
DEFINE_MACHINE("prusa-xl-050", xl_machine_init_v050)
