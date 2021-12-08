/*-
 * QEMU model of the stm32f2xx GPIO module
 * Copyright (c) 2013 https://github.com/pebble/qemu/
 * Adapted for QEMU 5.2 in 2020 by VintagePC <http://github.com/vintagepc>
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

#include "stm32f2xx_gpio.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
//#include "hw/arm/stm32.h"

//#define DEBUG_STM32_GPIO
#ifdef DEBUG_STM32_GPIO
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F2XX_GPIO: " fmt , ## __VA_ARGS__); \
         usleep(100); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


static uint64_t
stm32f2xx_gpio_read(void *arg, hwaddr offset, unsigned int size)
{
    stm32f2xx_gpio *s = arg;
    uint32_t r;

    offset >>= 2;
    r = s->regs[offset];
//printf("GPIO unit %d reg %x return 0x%x\n", s->periph, (int)offset << 2, r);
    return r;
}

static void
f2xx_update_odr(stm32f2xx_gpio *s, uint16_t val)
{
    int i;
    uint16_t changed = s->regs[STM32_GPIO_ODR] ^ val;

    for (i = 0; i < STM32_GPIO_PIN_COUNT; i++)
    {
        if ((changed & 1<<i) == 0)
            continue;

        DPRINTF("%s changing bit %i to %d\n", s->busdev.parent_obj.id, i, !!(val & 1<<i));
        //printf("gpio %u pin %u = %c\n", s->periph, i, val & 1<<i ? 'H' : 'l');
        qemu_set_irq(s->pin[i], !!(val & 1<<i));
    }
    s->regs[STM32_GPIO_ODR] = val;
}

static void
f2xx_update_mode(stm32f2xx_gpio *s, uint32_t val)
{
    int i;
    uint32_t prev_value = s->regs[STM32_GPIO_MODER];

    for (i = 0; i < STM32_GPIO_PIN_COUNT; i++)
    {
        uint32_t setting = (val >> i*2) & 0x03;
        uint32_t prev_setting = (prev_value >> i*2) & 0x03;
        if (setting == prev_setting) {
            continue;
        }

        DPRINTF("%s mode of pin %i changing to %u\n", s->busdev.parent_obj.id, i, setting);
        bool is_alternate_function = (setting == 2);
        qemu_set_irq(s->alternate_function[i], is_alternate_function);
    }
    s->regs[STM32_GPIO_MODER] = val;
}

static void
stm32f2xx_gpio_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    stm32f2xx_gpio *s = arg;
    int offset = addr & 0x03;

    addr >>= 2;
    if (addr > STM32_GPIO_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid GPIO %d write reg 0x%x\n",
          s->periph, (unsigned int)addr << 2);
        return;
    }

    switch(size) {
    case 1:
        data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
        break;
    case 2:
        data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        break;
    default:
        abort();
    }

    switch (addr) {
    case STM32_GPIO_MODER:
        f2xx_update_mode(s, data);
        break;
    case STM32_GPIO_ODR:
        f2xx_update_odr(s, data);
        break;
    case STM32_GPIO_BSRR:
    {
        uint16_t new_val = s->regs[STM32_GPIO_ODR];
        new_val &= ~((data >> 16) & 0xffff); /* BRy */
        new_val |= data & 0xffff; /* BSy */
        f2xx_update_odr(s, new_val);
        break;
    }
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx GPIO %u reg 0x%x:%d write (0x%x) unimplemented\n",
          s->periph,  (int)addr << 2, offset, (int)data);
        s->regs[addr] = data;
        break;
    }
}

static const MemoryRegionOps stm32f2xx_gpio_ops = {
    .read = stm32f2xx_gpio_read,
    .write = stm32f2xx_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32f2xx_gpio_reset(DeviceState *dev)
{
    stm32f2xx_gpio *s = STM32F2XX_GPIO(dev);

    switch (s->periph) {
    case 0:
        s->regs[STM32_GPIO_MODER] = 0xa8000000;
        s->regs[STM32_GPIO_OSPEEDR] = 0x00000000;
        s->regs[STM32_GPIO_PUPDR] = 0x64000000;
        break;
    case 1:
        s->regs[STM32_GPIO_MODER] = 0x00000280;
        s->regs[STM32_GPIO_OSPEEDR] = 0x000000C0;
        s->regs[STM32_GPIO_PUPDR] = 0x00000100;
        break;
    default:
        s->regs[STM32_GPIO_MODER] = 0x00000000;
        s->regs[STM32_GPIO_OSPEEDR] = 0x00000000;
        s->regs[STM32_GPIO_PUPDR] = 0x00000000;
        break;
    }
    /* Mask out the IDR bits as specified */
    s->regs[STM32_GPIO_IDR] = 0x0000ffff & ~(s->idr_mask);
}

static void
f2xx_gpio_set(void *arg, int pin, int level)
{
    stm32f2xx_gpio *s = arg;
    uint32_t bit = 1<<pin;

    if (level)
        s->regs[STM32_GPIO_IDR] |= bit;
    else
        s->regs[STM32_GPIO_IDR] &= ~bit;

    /* Inform EXTI module of pin state */
    qemu_set_irq(s->exti[pin], level);

    // For the stm32, only GPIOA, pin 0 will have a wakeup handler tied to it. It will be
    // tied to a handler callback in the NVIC. 
    qemu_set_irq(s->cpu_wake[pin], level);

    DPRINTF("GPIO %u set pin %d level %d\n", s->periph, pin, level);
}

void
f2xx_gpio_exti_set(stm32f2xx_gpio *s, unsigned pin, qemu_irq irq)
{
    s->exti[pin] = irq;
    DPRINTF("GPIO %u set exti %u irq %p\n", s->periph, pin, irq);
}

void
f2xx_gpio_wake_set(stm32f2xx_gpio *s, unsigned pin, qemu_irq irq)
{
    s->cpu_wake[pin] = irq;
    DPRINTF("GPIO %u set cpu_wake %u irq %p\n", s->periph, pin, irq);
}

static void stm32f2xx_gpio_rcc_reset(void *opaque, int n, int level) {
    if (!level) {
        stm32f2xx_gpio_reset(DEVICE(opaque));
    }
}

static void
stm32f2xx_gpio_init(Object *obj)
{
    stm32f2xx_gpio *s = STM32F2XX_GPIO(obj);

    memory_region_init_io(&s->iomem, obj, &stm32f2xx_gpio_ops, s, "gpio", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_in(DEVICE(obj), f2xx_gpio_set, STM32_GPIO_PIN_COUNT);
    qdev_init_gpio_out(DEVICE(obj), s->pin, STM32_GPIO_PIN_COUNT);
    qdev_init_gpio_out_named(DEVICE(obj), s->alternate_function, "af", STM32_GPIO_PIN_COUNT);
    qdev_init_gpio_in_named(DEVICE(obj),stm32f2xx_gpio_rcc_reset,"rcc-reset",1);


}

static Property stm32f2xx_gpio_properties[] = {
    DEFINE_PROP_UINT32("periph", stm32f2xx_gpio, periph, -1),
    DEFINE_PROP_UINT32("idr-mask", stm32f2xx_gpio, idr_mask, 0),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32f2xx_gpio = {
    .name = TYPE_STM32F2XX_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(periph, stm32f2xx_gpio),
        VMSTATE_UINT32(idr_mask, stm32f2xx_gpio),
        VMSTATE_UINT32_ARRAY(regs, stm32f2xx_gpio,STM32_GPIO_MAX),
        VMSTATE_UINT32(ccr, stm32f2xx_gpio),
        VMSTATE_END_OF_LIST()
    }
};


static void
stm32f2xx_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32f2xx_gpio_reset;
    dc->vmsd = &vmstate_stm32f2xx_gpio;
    device_class_set_props(dc, stm32f2xx_gpio_properties);
}

static const TypeInfo stm32f2xx_gpio_info = {
    .name = TYPE_STM32F2XX_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(stm32f2xx_gpio),
    .instance_init = stm32f2xx_gpio_init,
    .class_init = stm32f2xx_gpio_class_init
};

static void
stm32f2xx_gpio_register_types(void)
{
    type_register_static(&stm32f2xx_gpio_info);
}

type_init(stm32f2xx_gpio_register_types)
