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

/* Main SYSCLK frequency in Hz (168MHz) */
#define SYSCLK_FRQ 168000000ULL

static void buddy_init(MachineState *machine)
{
    DeviceState *dev;

    /*
     * TODO: ideally we would model the SoC RCC and let it handle
     * system_clock_scale, including its ability to define different
     * possible SYSCLK sources.
     */
    system_clock_scale = NANOSECONDS_PER_SECOND / SYSCLK_FRQ;

    dev = qdev_new(TYPE_STM32F407_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       FLASH_SIZE);

    STM32F407State *SOC = STM32F407_SOC(dev);

    /* Wire up display */

    void *bus;
    {
        bus = qdev_get_child_bus(DEVICE(&SOC->spi[1]), "ssi");

        DeviceState *lcd_dev = ssi_create_slave(bus, "st7789v");
        qemu_irq lcd_cs = qdev_get_gpio_in_named(lcd_dev, SSI_GPIO_CS, 0);

        /* Make sure the select pin is high.  */
        qemu_irq_raise(lcd_cs);
        void *gpio = DEVICE(&SOC->gpio[GPIO_C]);
        qdev_connect_gpio_out(gpio,9,lcd_cs);

        qemu_irq lcd_cd = qdev_get_gpio_in(lcd_dev,0);
        gpio = DEVICE(&SOC->gpio[GPIO_D]);
        qdev_connect_gpio_out(gpio,11, lcd_cd);
    }
    {
        bus = qdev_get_child_bus(DEVICE(&SOC->i2c[0]),"i2c");
        st25dv64k_init_one(bus, 0x53);
        // The QEMU I2CBus doesn't support devices with multiple addresses, so fake it
        // with a second instance at the SYSTEM address.
        bus = qdev_get_child_bus(DEVICE(&SOC->i2c[0]),"i2c");
        st25dv64k_init_one(bus, 0x57);
    }

    dev = qdev_new("buddy-input");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    // dev_get_gpio_out_connector(dev,"buddy-enc-button",0);
    qdev_connect_gpio_out_named(dev, "buddy-enc-button",0,  qdev_get_gpio_in(DEVICE(&SOC->gpio[GPIO_E]),12));
    qdev_connect_gpio_out_named(dev, "buddy-enc-a",0,  qdev_get_gpio_in(DEVICE(&SOC->gpio[GPIO_E]),15));
    qdev_connect_gpio_out_named(dev, "buddy-enc-b",0,  qdev_get_gpio_in(DEVICE(&SOC->gpio[GPIO_E]),13));

    {
        static char names[4] = {'X','Y','Z','E'};
        static uint8_t addresses[4] = {1, 3,0,2};
        // bus = qdev_get_child_bus(DEVICE(&SOC->usart2),"spi");
        for (int i=0; i<4; i++){
            dev = qdev_new("tmc2209");
            qdev_prop_set_uint8(dev, "axis",names[i]);
            qdev_prop_set_uint16(dev, "address", addresses[i]);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            qdev_connect_gpio_out_named(dev,"tmc2209-byte-out", 0, qdev_get_gpio_in_named(DEVICE(&SOC->usart[1]),"uart-byte-in",0));
            qdev_connect_gpio_out_named(DEVICE(&SOC->usart[1]),"uart-byte-out", 0, qdev_get_gpio_in_named(dev,"tmc2209-byte-in",0));
        }

        // qdev_connect_gpio_out_named(DEVICE(&SOC->usart2),"tmc2209_usart_cs",0, qdev_get_gpio_in_named(tmc, SSI_GPIO_CS, 0));
    }

    uint16_t startvals[] = {966, 977, 512, 512, 512};
    uint8_t channels[] = {4,10,3,5,6};
    for (int i=0; i<5; i++)
    {
        dev = qdev_new("thermistor");
        qdev_prop_set_uint16(dev, "temp",startvals[i]);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        qdev_connect_gpio_out_named(DEVICE(&SOC->adc[0]),"adc_read", channels[i],  qdev_get_gpio_in_named(dev, "thermistor_read_request",0));
        qdev_connect_gpio_out_named(dev, "thermistor_value",0, qdev_get_gpio_in_named(DEVICE(&SOC->adc[0]),"adc_data_in",channels[i]));
    }

};


static void buddy_machine_init(MachineClass *mc)
{
    mc->desc = "Prusa Buddy Board";
    mc->init = buddy_init;
}

DEFINE_MACHINE("prusabuddy", buddy_machine_init)
