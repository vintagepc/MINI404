/*
 * Prusa MK4 xBuddy machine model
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
#include "hw/core/split-irq.h"
#include "hw/ssi/ssi.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "utility/ArgHelper.h"
#include "sysemu/block-backend.h"
#include "sysemu/runstate.h"
#include "parts/dashboard_types.h"
#include "stm32_common/stm32_common.h"
#include "hw/arm/armv7m.h"
#include "parts/spi_rgb.h"
#include "otp.h"

#define TYPE_XBUDDY_MACHINE "xbuddy-machine"

#define BOOTLOADER_IMAGE(x) "Prusa_"#x"_Boot.bin"
#define XFLASH_FN(x)  "Prusa_"#x"_xflash.bin"
#define EEPROM_FN(x)  "Prusa_"#x"_eeprom.bin"
#define EEPROM_SYS_FN(x)  "Prusa_"#x"_eeprom_sys.bin"


// Upper 8 pins are GPIO
typedef uint16_t stm_pin;

static uint8_t BANK(stm_pin pin) {
    return ((pin >> 8U));
}

static uint8_t PIN(stm_pin pin) {
    return pin & 0xFFU;
}

#define STM_PIN(gpio,num) (((STM32_P_##gpio) & 0xFF)<< 8) | (num & 0xFF)

enum {
    NONE,
    TMC2130,
};

enum {
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    AXIS_E,
    AXIS_MAX,
};

enum {
	T_NOZ,
	T_BED,
	T_BRK,
	T_BRD,
	T_CASE,
	T_MAX,
};

typedef struct temp_cfg_t
{
	stm32_periph_t adc[T_MAX];
    uint8_t channel[T_MAX];// = {10,4,3,5,6};
	uint16_t ambient[T_MAX]; // = {18,20, 25, 22 /*512*/, 20};
    int table[T_MAX];// = {cfg.e_table_index, 1, 2000,0,5};
} temp_cfg_t;

typedef struct mk4_cfg_t {
    uint8_t lcd_spi;
    stm_pin lcd_cs;
    bool lcd_cs_invert;
    stm_pin lcd_cd;
    uint8_t w25_spi;
    stm_pin w25_cs;
    uint8_t at24_i2c;
    stm_pin hx717_sck;
    stm_pin hx717_data;
    stm_pin enc_a;
    stm_pin enc_b;
    stm_pin enc_btn;
    stm_pin z_min;
    bool has_at21;
	bool has_loadcell;
	temp_cfg_t temps;
    uint8_t motor;
	uint8_t e_t_mass;
    char m_label[AXIS_MAX];
    stm_pin m_step[AXIS_MAX];
    stm_pin m_dir[AXIS_MAX];
    stm_pin m_en[AXIS_MAX];
    stm_pin m_diag[AXIS_MAX];
    stm_pin m_select[AXIS_MAX];
	bool m_inverted[AXIS_MAX];
    uint8_t m_spi;
	uint8_t m_uart;
	bool is_400step;
	uint8_t dm_ver;
	const char* boot_fn;
	const char* eeprom_fn;
	const char* eeprom_sys_fn;
	const char* xflash_fn;

} mk4_cfg_t;

typedef struct xBuddyMachineClass {
    MachineClass        parent;
    const mk4_cfg_t     *cfg;
    bool                has_mmu;
} xBuddyMachineClass;

typedef struct xBuddyData {
	const mk4_cfg_t* cfg;
	const char* name;
	const char* descr;
    const bool has_mmu;
} xBuddyData;

#define XBUDDY_MACHINE_CLASS(klass)                                    \
    OBJECT_CLASS_CHECK(xBuddyMachineClass, (klass), TYPE_XBUDDY_MACHINE)
#define XBUDDY_MACHINE_GET_CLASS(obj)                                  \
    OBJECT_GET_CLASS(xBuddyMachineClass, (obj), TYPE_XBUDDY_MACHINE)

#define STM32_P_GPIO_NC (STM32_P_GPIOK + 1)

static const mk4_cfg_t mk4_027c_cfg = {
    .lcd_spi = STM32_P_SPI6,
    .lcd_cs = STM_PIN(GPIOD,11),
    .lcd_cs_invert = false,
    .lcd_cd = STM_PIN(GPIOD,15),
    .w25_spi = STM32_P_SPI5,
    .w25_cs = STM_PIN(GPIOF,2),
    .at24_i2c = STM32_P_I2C2,
    .hx717_data = STM_PIN(GPIOE,7),
    .hx717_sck = STM_PIN(GPIOG,1),
    .enc_a = STM_PIN(GPIOD,13),
    .enc_b = STM_PIN(GPIOD,12),
    .enc_btn = STM_PIN(GPIOG,3),
    .z_min = STM_PIN(GPIOB, 8),
    .has_at21 = true,
	.has_loadcell = true,
	.temps =
	{
		.adc = { [T_NOZ] = STM32_P_ADC1, [T_BED] = STM32_P_ADC1, [T_BRK] = STM32_P_ADC1, [T_BRD] = STM32_P_ADC3, [T_CASE] = STM32_P_ADC3 },
		.channel = { [T_NOZ] = 10, [T_BED] = 4, [T_BRK] = 6, [T_BRD] = 8, [T_CASE] = 15 },
		.ambient = {18, 20, 21, 25, 19},
		.table = { [T_NOZ] = 2005, [T_BED] = 2004, [T_BRK] = 5, [T_BRD] = 2000, [T_CASE] = 2000 }
	},

	.e_t_mass = 35,
    .motor = TMC2130,
    .m_label = {'X','Y','Z','E'},
    .m_step = { STM_PIN(GPIOD,7), STM_PIN(GPIOD,5), STM_PIN(GPIOD,3), STM_PIN(GPIOD,1)},
    .m_dir = { STM_PIN(GPIOD,6), STM_PIN(GPIOD,4), STM_PIN(GPIOD,2), STM_PIN(GPIOD,0)},
    .m_en = { STM_PIN(GPIOB,9), STM_PIN(GPIOB,9), STM_PIN(GPIOB,8), STM_PIN(GPIOD,10)},
    .m_diag = { STM_PIN(GPIOG,9), STM_PIN(GPIOE,13), STM_PIN(GPIOB,4), STM_PIN(GPIOD,14)},
    .m_select = {STM_PIN(GPIOG,15), STM_PIN(GPIOB,5), STM_PIN(GPIOF,15), STM_PIN(GPIOF,12)},
	.m_inverted = {1,0,1,1},
    .m_spi = STM32_P_SPI3,
	.is_400step = true,
	.dm_ver = 27,
	.boot_fn = BOOTLOADER_IMAGE(Mk4),
	.eeprom_fn = EEPROM_FN(Mk4),
	.eeprom_sys_fn = EEPROM_SYS_FN(Mk4),
	.xflash_fn = XFLASH_FN(Mk4)
};

static const mk4_cfg_t mk4_034_cfg = {
    .lcd_spi = STM32_P_SPI6,
    .lcd_cs = STM_PIN(GPIOD,11),
    .lcd_cs_invert = false,
    .lcd_cd = STM_PIN(GPIOD,15),
    .w25_spi = STM32_P_SPI5,
    .w25_cs = STM_PIN(GPIOF,2),
    .at24_i2c = STM32_P_I2C2,
    .hx717_data = STM_PIN(GPIOE,7),
    .hx717_sck = STM_PIN(GPIOG,1),
    .enc_a = STM_PIN(GPIOD,13),
    .enc_b = STM_PIN(GPIOD,12),
    .enc_btn = STM_PIN(GPIOG,3),
    .z_min = STM_PIN(GPIOB, 8),
    .has_at21 = true,
	.has_loadcell = true,
	.temps =
	{
		.adc = { [T_NOZ] = STM32_P_ADC1, [T_BED] = STM32_P_ADC1, [T_BRK] = STM32_P_ADC1, [T_BRD] = STM32_P_ADC3, [T_CASE] = STM32_P_ADC3 },
		.channel = { [T_NOZ] = 10, [T_BED] = 4, [T_BRK] = 6, [T_BRD] = 8, [T_CASE] = 15 },
		.ambient = {18, 20, 21, 25, 19},
		.table = { [T_NOZ] = 2005, [T_BED] = 2004, [T_BRK] = 5, [T_BRD] = 2000, [T_CASE] = 2000 }
	},
	.e_t_mass = 45,
    .motor = TMC2130,
    .m_label = {'X','Y','Z','E'},
    .m_step = { STM_PIN(GPIOA,3), STM_PIN(GPIOA,0), STM_PIN(GPIOD,3), STM_PIN(GPIOD,1)},
    .m_dir = { STM_PIN(GPIOD,6), STM_PIN(GPIOD,4), STM_PIN(GPIOD,2), STM_PIN(GPIOD,0)},
    .m_en = { STM_PIN(GPIOB,9), STM_PIN(GPIOB,9), STM_PIN(GPIOB,8), STM_PIN(GPIOD,10)},
    .m_diag = { STM_PIN(GPIOG,9), STM_PIN(GPIOE,13), STM_PIN(GPIOB,4), STM_PIN(GPIOD,14)},
    .m_select = {STM_PIN(GPIOG,15), STM_PIN(GPIOB,5), STM_PIN(GPIOF,15), STM_PIN(GPIOF,12)},
	.m_inverted = {1,0,1,1},
    .m_spi = STM32_P_SPI3,
	.is_400step = true,
	.dm_ver = 34,
	.boot_fn = BOOTLOADER_IMAGE(Mk4),
	.eeprom_fn = EEPROM_FN(Mk4),
	.eeprom_sys_fn = EEPROM_SYS_FN(Mk4),
	.xflash_fn = XFLASH_FN(Mk4)
};

static const mk4_cfg_t mk3v5_cfg = {
    .lcd_spi = STM32_P_SPI6,
    .lcd_cs = STM_PIN(GPIOD,11),
    .lcd_cs_invert = false,
    .lcd_cd = STM_PIN(GPIOD,15),
    .w25_spi = STM32_P_SPI5,
    .w25_cs = STM_PIN(GPIOF,2),
    .at24_i2c = STM32_P_I2C2,
    .hx717_data = STM_PIN(GPIOE,7),
    .hx717_sck = STM_PIN(GPIOG,1),
    .enc_a = STM_PIN(GPIOD,13),
    .enc_b = STM_PIN(GPIOD,12),
    .enc_btn = STM_PIN(GPIOG,3),
    .z_min = STM_PIN(GPIOB, 8),
    .has_at21 = false,
	.temps =
	{
		.adc = { [T_NOZ] = STM32_P_ADC1, [T_BED] = STM32_P_ADC1, [T_BRK] = STM32_P_ADC1, [T_BRD] = STM32_P_ADC3, [T_CASE] = STM32_P_ADC3 },
		.channel = { [T_NOZ] = 10, [T_BED] = 4, [T_BRK] = 6, [T_BRD] = 8, [T_CASE] = 15 },
		.ambient = {18, 20, 21, 25, 19},
		.table = { [T_NOZ] = 2005, [T_BED] = 2004, [T_BRK] = 5, [T_BRD] = 2000, [T_CASE] = 2000 }
	},
	.e_t_mass = 30,
    .motor = TMC2130,
    .m_label = {'X','Y','Z','E'},
    .m_step = { STM_PIN(GPIOD,7), STM_PIN(GPIOD,5), STM_PIN(GPIOD,3), STM_PIN(GPIOD,1)},
    .m_dir = { STM_PIN(GPIOD,6), STM_PIN(GPIOD,4), STM_PIN(GPIOD,2), STM_PIN(GPIOD,0)},
    .m_en = { STM_PIN(GPIOB,9), STM_PIN(GPIOB,9), STM_PIN(GPIOB,8), STM_PIN(GPIOD,10)},
    .m_diag = { STM_PIN(GPIOG,9), STM_PIN(GPIOE,13), STM_PIN(GPIOB,4), STM_PIN(GPIOD,14)},
    .m_select = {STM_PIN(GPIOG,15), STM_PIN(GPIOB,5), STM_PIN(GPIOF,15), STM_PIN(GPIOF,12)},
	.m_inverted = {0,1,0,1},
    .m_spi = STM32_P_SPI3,
	.is_400step = false,
	.dm_ver = 34,
	.boot_fn = BOOTLOADER_IMAGE(Mk3v5),
	.eeprom_fn = EEPROM_FN(Mk3v5),
	.eeprom_sys_fn = EEPROM_SYS_FN(Mk3v5),
	.xflash_fn = XFLASH_FN(Mk3v5)
};

static const mk4_cfg_t mk3v9_cfg = {
    .lcd_spi = STM32_P_SPI6,
    .lcd_cs = STM_PIN(GPIOD,11),
    .lcd_cs_invert = false,
    .lcd_cd = STM_PIN(GPIOD,15),
    .w25_spi = STM32_P_SPI5,
    .w25_cs = STM_PIN(GPIOF,2),
    .at24_i2c = STM32_P_I2C2,
    .hx717_data = STM_PIN(GPIOE,7),
    .hx717_sck = STM_PIN(GPIOG,1),
    .enc_a = STM_PIN(GPIOD,13),
    .enc_b = STM_PIN(GPIOD,12),
    .enc_btn = STM_PIN(GPIOG,3),
    .z_min = STM_PIN(GPIOB, 8),
    .has_at21 = true,
	.has_loadcell = true,
	.temps =
	{
		.adc = { [T_NOZ] = STM32_P_ADC1, [T_BED] = STM32_P_ADC1, [T_BRK] = STM32_P_ADC1, [T_BRD] = STM32_P_ADC3, [T_CASE] = STM32_P_ADC3 },
		.channel = { [T_NOZ] = 10, [T_BED] = 4, [T_BRK] = 6, [T_BRD] = 8, [T_CASE] = 15 },
		.ambient = {18, 20, 21, 25, 19},
		.table = { [T_NOZ] = 2005, [T_BED] = 2004, [T_BRK] = 5, [T_BRD] = 2000, [T_CASE] = 2000 }
	},
	.e_t_mass = 35,
    .motor = TMC2130,
    .m_label = {'X','Y','Z','E'},
    .m_step = { STM_PIN(GPIOD,7), STM_PIN(GPIOD,5), STM_PIN(GPIOD,3), STM_PIN(GPIOD,1)},
    .m_dir = { STM_PIN(GPIOD,6), STM_PIN(GPIOD,4), STM_PIN(GPIOD,2), STM_PIN(GPIOD,0)},
    .m_en = { STM_PIN(GPIOB,9), STM_PIN(GPIOB,9), STM_PIN(GPIOB,8), STM_PIN(GPIOD,10)},
    .m_diag = { STM_PIN(GPIOG,9), STM_PIN(GPIOE,13), STM_PIN(GPIOB,4), STM_PIN(GPIOD,14)},
    .m_select = {STM_PIN(GPIOG,15), STM_PIN(GPIOB,5), STM_PIN(GPIOF,15), STM_PIN(GPIOF,12)},
	.m_inverted = {0, 1, 0,1},
    .m_spi = STM32_P_SPI3,
	.is_400step = false,
	.dm_ver = 34,
	.boot_fn = BOOTLOADER_IMAGE(Mk3v9),
	.eeprom_fn = EEPROM_FN(Mk3v9),
	.eeprom_sys_fn = EEPROM_SYS_FN(Mk3v9),
	.xflash_fn = XFLASH_FN(Mk3v9)
};

static void mk4_init(MachineState *machine)
{

	const xBuddyMachineClass *mc = XBUDDY_MACHINE_GET_CLASS(OBJECT(machine));
    Object* periphs = container_get(OBJECT(machine), "/peripheral");
	const mk4_cfg_t cfg = *mc->cfg;

	OTP_v4 otp_data = { .version = 4, .size = sizeof(OTP_v4),
		.datamatrix = {'4', '5', '5', '8', '-', '2', '7', '0', '0', '0', '0', '1', '9', '0', '0', '5', '2', '5', '9', '9', '9', '9', 0, 0}
	};
	if (cfg.dm_ver == 34)
	{
		otp_data.datamatrix[5]='3';
		otp_data.datamatrix[6]='4';
	}

	uint32_t* otp_raw = (uint32_t*) &otp_data;

    DeviceState *dev;

    dev = qdev_new(TYPE_STM32F427xI_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_uint32(dev,"sram-size", machine->ram_size);
	uint64_t flash_size = stm32_soc_get_flash_size(dev);
    arghelper_setargs(machine->kernel_cmdline);
	bool args_continue_running = arghelper_parseargs();
	if (arghelper_is_arg("4x_flash"))
    {
        flash_size <<=2; // quadruple the flash size for debug code.
		printf("Extended flash size: now %"PRIu64" kB\n", flash_size/1024);
    }
	qdev_prop_set_uint32(dev,"flash-size", flash_size);

	DeviceState* otp = stm32_soc_get_periph(dev, STM32_P_OTP);
	qdev_prop_set_uint32(otp,"len-otp-data", 9);
	qdev_prop_set_uint32(otp,"otp-data[0]", otp_raw[0]);
	qdev_prop_set_uint32(otp,"otp-data[1]", otp_raw[1]);
	qdev_prop_set_uint32(otp,"otp-data[2]", otp_raw[2]);
	qdev_prop_set_uint32(otp,"otp-data[3]", otp_raw[3]);
	qdev_prop_set_uint32(otp,"otp-data[4]", otp_raw[4]);
	qdev_prop_set_uint32(otp,"otp-data[5]", otp_raw[5]);
	qdev_prop_set_uint32(otp,"otp-data[6]", otp_raw[6]);
	qdev_prop_set_uint32(otp,"otp-data[7]", otp_raw[7]);
	qdev_prop_set_uint32(otp,"otp-data[8]", otp_raw[8]);

    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
	DeviceState* dev_soc = dev;
    // We (ab)use the kernel command line to piggyback custom arguments into QEMU.
    // Parse those now.
	// ugly hack... FIXME.
    if (arghelper_is_arg("appendix")) {
		qdev_prop_set_uint32(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),"idr-mask", 0x2000);
    }

    char* kfn = machine->kernel_filename;
    int kernel_len = kfn ? strlen(kfn) : 0;
    if (kernel_len >3 && strncmp(kfn + (kernel_len-3), "bbf",3) == 0 )
    {
        // TODO... use initrd_image as a bootloader alternative?
        struct stat bootloader;
        if (stat(cfg.boot_fn,&bootloader))
        {
            error_setg(&error_fatal, "No %s file found. It is required to use a .bbf file!",cfg.boot_fn);
            return;
        }
        // BBF has an extra 64b header we need to prune. Rather than modify it or use a temp file, offset it
        // by -64 bytes and rely on the bootloader clobbering it.
        load_image_targphys(machine->kernel_filename,0x20000-64,get_image_size(machine->kernel_filename));
        armv7m_load_kernel(ARM_CPU(first_cpu),
            cfg.boot_fn, 0,
            flash_size);
    }
    else // Raw bin or ELF file, load directly.
    {
        armv7m_load_kernel(ARM_CPU(first_cpu),
                        machine->kernel_filename, 0,
                        flash_size);
    }

	DeviceState* key_in = qdev_new("p404-key-input");
    sysbus_realize(SYS_BUS_DEVICE(key_in), &error_fatal);

	DeviceState* lcd_dev = NULL;
    /* Wire up display */
    void *bus;
    {
        bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, cfg.lcd_spi),
			"ssi");

        lcd_dev = ssi_create_peripheral(bus, "ili9488");

     	DeviceState *npixel[4];
		int display_order[4] = {2, 0, 1, 3};
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
                qdev_get_gpio_in_named(lcd_dev,"leds",display_order[i])
                );
        }

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

		qemu_irq led_cs = NULL;
        if (BANK(cfg.lcd_cs) != (STM32_P_GPIO_NC - STM32_P_GPIOA)) {
			// Replace the LED IRQ with a split if it needs to be shared. otherwise just LED is connected below.
			led_cs = qemu_irq_split(lcd_cs, qdev_get_gpio_in_named(npixel[0], SSI_GPIO_CS, 0));
        }
		else
		{
			led_cs = qdev_get_gpio_in_named(npixel[0], SSI_GPIO_CS, 0);
		}
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.lcd_cs)),PIN(cfg.lcd_cs),led_cs);
    }

    BlockBackend *blk = NULL;
    {
        bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, cfg.w25_spi),
			 "ssi");
        dev = qdev_new("w25q64jv");
        blk = get_or_create_drive(IF_MTD, 0, cfg.xflash_fn, XFLASH_ID,  8U*MiB, &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize_and_unref(dev, bus, &error_fatal);
        //DeviceState *flash_dev = ssi_create_slave(bus, "w25q64jv");
        qemu_irq flash_cs = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        qemu_irq_raise(flash_cs);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.w25_cs)), PIN(cfg.w25_cs), flash_cs);



    }
    {
        bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, cfg.at24_i2c),
			"i2c");
        dev = qdev_new("at24c-eeprom");
        qdev_prop_set_uint8(dev, "address", 0x53);
        qdev_prop_set_uint32(dev, "rom-size", 64*KiB / 8U);
		blk = get_or_create_drive(IF_PFLASH, 0, cfg.eeprom_fn, EEPROM_ID, 64*KiB / 8U, &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize(dev, bus, &error_fatal);
        // The QEMU I2CBus doesn't support devices with multiple addresses, so fake it
        // with a second instance at the SYSTEM address.
        dev = qdev_new("at24c-eeprom");
        qdev_prop_set_uint8(dev, "address", 0x57);
        qdev_prop_set_uint32(dev, "rom-size", 64*KiB / 8U);
		blk = get_or_create_drive(IF_PFLASH, 1, cfg.eeprom_sys_fn, EEPROM_SYS_ID,  64*KiB / 8U,  &error_fatal);
		qdev_prop_set_drive(dev, "drive", blk);
        qdev_realize(dev, bus, &error_fatal);
    }
	{
        bus = qdev_get_child_bus(
				stm32_soc_get_periph(dev_soc, cfg.at24_i2c),
			"i2c");
        dev = qdev_new("fusb302b");
        qdev_prop_set_uint8(dev, "address", 0x23);
        qdev_realize(dev, bus, &error_fatal);
	}

	{
		bus = qdev_get_child_bus(stm32_soc_get_periph(dev_soc, cfg.at24_i2c),"i2c");
        dev = qdev_new("fusb302b");
        qdev_prop_set_uint8(dev, "address", 0x23);
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
    DeviceState *db2 = qdev_new("2d-dashboard");
    qdev_prop_set_uint8(db2, "fans", 2);
    qdev_prop_set_uint8(db2, "thermistors", 5);
    qdev_prop_set_string(db2, "indicators", "ZF");

    {

        int32_t ends[4] = { 100*16*255, 100*16*214, 400*16*(cfg.has_loadcell ? 221: 212),0 };
        static int32_t stepsize[4] = { 100*16, 100*16, 400*16, 320*16 };
 		static const char* links[4] = {"motor[0]","motor[1]","motor[2]","motor[3]"};
        if (cfg.is_400step) {
            stepsize[0] <<= 1;
            stepsize[1] <<= 1;
			ends[0] <<= 1;
            ends[1] <<= 1;
        }

        // XY enable is split out
        DeviceState *split_en_out = qdev_new(TYPE_SPLIT_IRQ);
        qdev_prop_set_uint16(split_en_out, "num-lines", 2);
        qdev_realize_and_unref(DEVICE(split_en_out),NULL,  &error_fatal);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_en[0])), PIN(cfg.m_en[0]),qdev_get_gpio_in(split_en_out,0));
        BusState *motor_spi = NULL;
		motor_spi = qdev_get_child_bus(stm32_soc_get_periph(dev_soc, cfg.m_spi),"ssi");


        for (int i=0; i<AXIS_MAX; i++){
            if (cfg.m_label[i] == '\0' ) {
                continue;
            }
			dev = qdev_new("tmc2130");
            motors[i] = dev;
            qdev_prop_set_uint8(dev, "axis",cfg.m_label[i]);
            qdev_prop_set_uint8(dev, "inverted",cfg.m_inverted[i]);
            qdev_prop_set_int32(dev, "max_step", ends[i]);
            qdev_prop_set_int32(dev, "fullstepspermm", stepsize[i]);

			qdev_realize(dev, motor_spi, &error_fatal);
			qemu_irq driver_cs = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
			qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.m_select[i])),  PIN(cfg.m_select[i]), driver_cs);
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
    sysbus_realize(SYS_BUS_DEVICE(db2), &error_fatal);
    DeviceState *bed = NULL, *hotend = NULL;
    for (int i=0; i<T_MAX; i++)
    {
		if (cfg.temps.adc[i] == STM32_P_UNDEFINED)
		{
			continue;
		}
        dev = qdev_new("thermistor");
        qdev_prop_set_uint16(dev, "temp",cfg.temps.ambient[i]);
        qdev_prop_set_uint16(dev, "table_no", cfg.temps.table[i]);
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        //qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, cfg.temps.adc[i]),"adc_read", cfg.temps.channel[i],  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
        qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, cfg.temps.adc[i]),"adc_data_in",cfg.temps.channel[i]));
        if (i==T_NOZ)
		{
            hotend = dev;
        }
        if (i==T_BED)
		{
			bed = dev;
		}
		qdev_connect_gpio_out_named(dev, "temp_out_256x", 0, qdev_get_gpio_in_named(db2,"therm-temp",i));
    }
    // Heaters - bed is B0/ TIM3C3, E is B1/ TIM3C4

    dev = qdev_new("heater");
    qdev_prop_set_uint8(dev, "thermal_mass_x10",cfg.e_t_mass);
    qdev_prop_set_uint8(dev,"label", 'E');
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM3),"pwm_ratio_changed",3,qdev_get_gpio_in_named(dev, "pwm_in",0));
    qdev_connect_gpio_out_named(dev, "temp_out",0, qdev_get_gpio_in_named(hotend, "thermistor_set_temperature",0));
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
#ifdef BUDDY_HAS_GL
    qemu_irq split_bed = qemu_irq_split(qdev_get_gpio_in_named(db2,"therm-pwm",1),qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_BED));
    qdev_connect_gpio_out_named(dev, "pwm-out", 0, split_bed);
#else
    qdev_connect_gpio_out_named(dev, "pwm-out", 0, qdev_get_gpio_in_named(db2,"therm-pwm",1));
#endif

    DeviceState *hs = NULL;

	if (cfg.has_loadcell) {
		DeviceState *lc = qdev_new("loadcell");
        object_property_add_child(OBJECT(periphs), "loadcell", OBJECT(lc));
		sysbus_realize(SYS_BUS_DEVICE(lc), &error_fatal);
		qdev_connect_gpio_out_named(motors[2],"um-out",0,qdev_get_gpio_in(lc,0));

		hs = qdev_new("hall-sensor");
        object_property_add_child(OBJECT(periphs), "hall-sensor", OBJECT(hs));
        qdev_prop_set_bit(hs, "start-state", !mc->has_mmu); // MMU starts unloaded.
		sysbus_realize(SYS_BUS_DEVICE(hs), &error_fatal);
		qdev_connect_gpio_out_named(hs, "status", 0,qdev_get_gpio_in_named(db2,"led-digital",1));

		dev = qdev_new("hx717");
        object_property_add_child(OBJECT(periphs), "hx717", OBJECT(dev));
		sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
		qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(cfg.hx717_data)),PIN(cfg.hx717_data))); // EXTR_DATA
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, BANK(cfg.hx717_sck)),PIN(cfg.hx717_sck),qdev_get_gpio_in(dev, 0)); // EXTR_SCK
		// PT100
		qdev_connect_gpio_out(lc,0, qdev_get_gpio_in_named(dev,"input_x1000",0));
		if (cfg.temps.table[T_BRK] == 22) // PT100
		{
			qdev_connect_gpio_out_named(hotend, "value_x1000",0, qdev_get_gpio_in_named(dev,"input_x1000",1));
		}
		else
		{
			qdev_connect_gpio_out(hs,0, qdev_get_gpio_in_named(dev,"input",1));
		}

	}
	else
	{
		DeviceState* pinda = qdev_new("pinda");
		sysbus_realize(SYS_BUS_DEVICE(pinda), &error_fatal);
        DeviceState* split_zmin = qdev_new("split-irq");
		qdev_prop_set_uint16(split_zmin, "num-lines", 3);
        qdev_realize_and_unref(DEVICE(split_zmin),NULL,  &error_fatal);
        qdev_connect_gpio_out(split_zmin, 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),6));
        qdev_connect_gpio_out(split_zmin, 1, qdev_get_gpio_in_named(db2,"led-digital",0));
// #ifdef BUDDY_HAS_GL
//         qdev_connect_gpio_out(split_zmin, 2, qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_ZPROBE));
// #endif
        qdev_connect_gpio_out(pinda, 0,  qdev_get_gpio_in(split_zmin,0));
		for (int i=0; i<3; i++)
		{
			qdev_connect_gpio_out_named(motors[i],"step-out", 0, qdev_get_gpio_in_named(pinda,"position_xyz",i));
		}

	}

    if (cfg.has_at21) {
        dev = qdev_new("at21csxx");
		qdev_prop_set_drive(dev, "drive", blk_by_name("loveboard-eeprom"));
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        // 2-way bitbang
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOF),13,qdev_get_gpio_in(dev, 0));
        qdev_connect_gpio_out(dev,0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOF), 13));
    }

    // dev = qdev_new("hc4052");
    // sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    // qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOA),8,qdev_get_gpio_in_named(dev,"select",0)); // S0
    // qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOC),15,qdev_get_gpio_in_named(dev, "select", 1)); // S1
    // qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC2),"adc_read", 3,  qdev_get_gpio_in_named(dev, "adc_read_request",0));
    // qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC2),"adc_read", 5,  qdev_get_gpio_in_named(dev, "adc_read_request",1));
    // qdev_connect_gpio_out(dev,0, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC2),"adc_data_in",3));
    // qdev_connect_gpio_out(dev,1, qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC2),"adc_data_in",5));

    // Heater v - only on if heating.
    DeviceState* vdev = qdev_new("powersource");
    qdev_prop_set_uint32(vdev,"mV",23900);
    sysbus_realize(SYS_BUS_DEVICE(vdev),&error_fatal);
	// qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", 3,  qdev_get_gpio_in_named(vdev, "adc_read_request",0));
    qdev_connect_gpio_out_named(vdev, "v_sense",0,qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",3));
    qdev_connect_gpio_out_named(vdev, "panic",0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOG), 0));

    // Currents - heater, mmu, system. All on ADC3
	// TBD -find out the system scaling, it's not the same as the other two.... perhaps not a CS30
	{
		uint16_t currents[] = {105, 300, 100};
		uint8_t channels[] = {9, 4, 14};
		for (int i=0; i<3; i++)
		{
			vdev = qdev_new("cs30bl");
			qdev_prop_set_uint32(vdev,"mA",currents[i]);
			sysbus_realize(SYS_BUS_DEVICE(vdev),&error_fatal);
			qdev_connect_gpio_out_named(vdev, "a_sense",0,qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC3),"adc_data_in",channels[i]));
			//qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC3),"adc_read", channels[i],  qdev_get_gpio_in_named(vdev, "adc_read_request",0));
		}
	}

    // Bed V, always on.
    vdev = qdev_new("powersource");
    qdev_prop_set_uint32(vdev,"mV",24000);
    sysbus_realize(SYS_BUS_DEVICE(vdev),&error_fatal);
    // qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_read", 5,  qdev_get_gpio_in_named(vdev, "adc_read_request",0));
    qdev_connect_gpio_out_named(vdev, "v_sense",0,qdev_get_gpio_in_named(stm32_soc_get_periph(dev_soc, STM32_P_ADC1),"adc_data_in",5));
    //qdev_connect_gpio_out_named(vdev, "v_sense",0,qdev_get_gpio_in_named(dev,"1Y",0));
    // qdev_connect_gpio_out_named(vdev, "a_sense",0,qdev_get_gpio_in_named(dev,"2Y",1));

    // TODO - THERM2, THERM3

	// Heater, input, MMU, USB HS, USB FS
	uint16_t oc_latches[] = {STM_PIN(GPIOG, 5), STM_PIN(GPIOG,6), 	STM_PIN(GPIOB, 6), 	STM_PIN(GPIOD, 9), 	STM_PIN(GPIOF, 14)};
	bool latch_inverted[] = {0,					0,					0,					1, 					1};

	DeviceState* split_reset = qdev_new(TYPE_SPLIT_IRQ);
	qdev_prop_set_uint16(split_reset, "num-lines", ARRAY_SIZE(oc_latches));
	qdev_realize_and_unref(split_reset, NULL, &error_fatal);
	qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOG),7, qdev_get_gpio_in(split_reset,0));
	for(int i=0; i<ARRAY_SIZE(oc_latches); i++) {
		dev = qdev_new("oc-latch");
		sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
		if (latch_inverted[i])
		{
			qdev_connect_gpio_out(dev, 0, qemu_irq_invert(qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(oc_latches[i])), PIN(oc_latches[i]))));
		}
		else
		{
			qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, BANK(oc_latches[i])), PIN(oc_latches[i])));
		}
		qdev_connect_gpio_out(split_reset,i, qdev_get_gpio_in(dev,0));
	}

    // hotend = fan1
    // print fan = fan0
    uint16_t fan_max_rpms[] = { 6600, 7000 };
    uint8_t  fan_pwm_pins[] = { 11, 9};
    uint8_t fan_tach_pins[] = { 10, 14};
    uint8_t fan_labels[] = {'P','E'};
	DeviceState* fanpwm = qdev_new("software-pwm");
    qdev_prop_set_bit(fanpwm, "is_inverted", true);
	sysbus_realize_and_unref(SYS_BUS_DEVICE(fanpwm),&error_fatal);
	qdev_connect_gpio_out_named(stm32_soc_get_periph(dev_soc, STM32_P_TIM14), "timer", 0, qdev_get_gpio_in_named(fanpwm, "tick-in", 0));
    for (int i=0; i<2; i++)
    {
		qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOE), fan_pwm_pins[i],
			qdev_get_gpio_in_named(fanpwm, "gpio-in",i)
		);
        dev = qdev_new("fan");
        qdev_prop_set_uint8(dev,"label",fan_labels[i]);
        qdev_prop_set_uint32(dev, "max_rpm",fan_max_rpms[i]);
        //qdev_prop_set_bit(dev, "is_nonlinear", i); // E is nonlinear.
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        qdev_connect_gpio_out_named(dev, "tach-out",0,qdev_get_gpio_in(stm32_soc_get_periph(dev_soc, STM32_P_GPIOE),fan_tach_pins[i]));
		qemu_irq split_fan = qemu_irq_split( qdev_get_gpio_in_named(dev, "pwm-in",0), qdev_get_gpio_in_named(db2, "fan-pwm",i));
        qdev_connect_gpio_out_named(dev, "rpm-out", 0, qdev_get_gpio_in_named(db2,"fan-rpm",i));
		qdev_connect_gpio_out(fanpwm,i,split_fan);
// #ifdef BUDDY_HAS_GL
//         qemu_irq split_fan = qemu_irq_split(qdev_get_gpio_in_named(db2,"fan-pwm",i),qdev_get_gpio_in_named(gl_db,"indicator-analog",DB_IND_PFAN+i));
//         qdev_connect_gpio_out_named(dev, "pwm-out", 0, split_fan);
// #else
//         qdev_connect_gpio_out_named(dev, "pwm-out", 0, qdev_get_gpio_in_named(db2,"fan-pwm",i));
// #endif
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

    // Do not create the bridge element if no kernel is suppled. Corner case for qtest.
    if (mc->has_mmu && kernel_len > 0)
    {
        dev = qdev_new("mmu-bridge");
        object_property_add_child(OBJECT(periphs), "mmu-bridge", OBJECT(dev));
    	sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        qdev_connect_gpio_out(stm32_soc_get_periph(dev_soc, STM32_P_GPIOG),8,qdev_get_gpio_in_named(dev, "reset-in", 0));
        if (hs != NULL) 
        {
	        qdev_connect_gpio_out_named(dev, "fs-out",0, qdev_get_gpio_in_named(hs, "ext-in", 0));
        }

    }

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

};

static void xbuddy_class_init(ObjectClass *oc, void *data)
{
		const xBuddyData* d = (xBuddyData*)data;
	    MachineClass *mc = MACHINE_CLASS(oc);
	    mc->desc = d->descr;
	    mc->family = TYPE_XBUDDY_MACHINE,
	    mc->init = mk4_init;
	    mc->default_ram_size = 0; // 0 = use default RAM from chip.
	    mc->no_parallel = 1;
		mc->no_serial = 1;

		xBuddyMachineClass* xmc = XBUDDY_MACHINE_CLASS(oc);
		xmc->cfg = d->cfg;
        xmc->has_mmu = d->has_mmu;
}

static const xBuddyData mk4_027c = {
	.cfg = &mk4_027c_cfg,
	.descr = "Prusa Mk4 0.2.7c",
};

static const xBuddyData mk4_027c_mmu = {
	.cfg = &mk4_027c_cfg,
	.descr = "Prusa Mk4 0.2.7c with MMU3",
    .has_mmu = true,
};

static const xBuddyData mk4_034 = {
	.cfg = &mk4_034_cfg,
	.descr = "Prusa Mk4 0.3.4",
};

static const xBuddyData mk4_034_mmu = {
	.cfg = &mk4_034_cfg,
	.descr = "Prusa Mk4 0.3.4 with MMU3",
    .has_mmu = true,
};


static const xBuddyData mk3v5 = {
	.cfg = &mk3v5_cfg,
	.descr = "Prusa Mk3.5",
};

static const xBuddyData mk3v9 = {
	.cfg = &mk3v9_cfg,
	.descr = "Prusa Mk3.9",
};

static const TypeInfo xbuddy_machine_types[] = {
    {
        .name           = TYPE_XBUDDY_MACHINE,
        .parent         = TYPE_MACHINE,
		.class_size		= sizeof(xBuddyMachineClass),
        .abstract       = true,
    }, {
        .name           = MACHINE_TYPE_NAME("prusa-mk4-027c"),
        .parent         = TYPE_XBUDDY_MACHINE,
		.class_init     = xbuddy_class_init,
		.class_data		= (void*)&mk4_027c
    }, {
        .name           = MACHINE_TYPE_NAME("prusa-mk4-027c-mmu"),
        .parent         = TYPE_XBUDDY_MACHINE,
		.class_init     = xbuddy_class_init,
		.class_data		= (void*)&mk4_027c_mmu,
    }, {
        .name           = MACHINE_TYPE_NAME("prusa-mk4-034"),
        .parent         = TYPE_XBUDDY_MACHINE,
		.class_init     = xbuddy_class_init,
		.class_data		= (void*)&mk4_034
    }, {
        .name           = MACHINE_TYPE_NAME("prusa-mk4-034-mmu"),
        .parent         = TYPE_XBUDDY_MACHINE,
		.class_init     = xbuddy_class_init,
		.class_data		= (void*)&mk4_034_mmu
    },{
        .name           = MACHINE_TYPE_NAME("prusa-mk3-35"),
        .parent         = TYPE_XBUDDY_MACHINE,
		.class_init     = xbuddy_class_init,
		.class_data		= (void*)&mk3v5
    },{
        .name           = MACHINE_TYPE_NAME("prusa-mk3-39"),
        .parent         = TYPE_XBUDDY_MACHINE,
		.class_init     = xbuddy_class_init,
		.class_data		= (void*)&mk3v9
    },
};
DEFINE_TYPES(xbuddy_machine_types)
