/*-
 * QEMU model of the stm32f2xx GPIO module
 * Copyright (c) 2013 https://github.com/pebble/qemu/
 * Adapted for QEMU 5.2 in 2020 by VintagePC <http://github.com/vintagepc>
 * Refactored significantly in 2022-3 by VintagePC for G070 and F030
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
#include "hw/sysbus.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "stm32_gpio.h"
#include "stm32_gpio_regdata.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"

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


#define STM32_GPIO_PIN_COUNT 16

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Gpio), COM_CLASS_NAME(Gpio), STM32COM_GPIO);

typedef struct COM_STRUCT_NAME(Gpio) {
    STM32Peripheral parent;
    MemoryRegion iomem;

    uint32_t idr_mask;
	uint32_t force_idr;

    qemu_irq pin[STM32_GPIO_PIN_COUNT];
    qemu_irq exti[STM32_GPIO_PIN_COUNT];
    qemu_irq alternate_function[STM32_GPIO_PIN_COUNT];
    qemu_irq cpu_wake[STM32_GPIO_PIN_COUNT];

    uint32_t regs[RI_END];

	const stm32_reginfo_t (* reginfo)[RI_END];

} COM_STRUCT_NAME(Gpio);

typedef struct COM_CLASS_NAME(Gpio) {
	STM32PeripheralClass parent_class;
    stm32_reginfo_t var_reginfo[MAX_GPIO_BANKS][RI_END];
} COM_CLASS_NAME(Gpio);


static const stm32_periph_banked_variant_t stm32_gpio_variants[4] = {
	{TYPE_STM32F030_GPIO, stm32f030_gpio_reginfo},
	{TYPE_STM32G070_GPIO, stm32g070_gpio_reginfo},
	{TYPE_STM32F2xx_GPIO, stm32f2xx_gpio_reginfo},
	{TYPE_STM32F4xx_GPIO, stm32f4xx_gpio_reginfo}
};


static uint64_t
stm32_common_gpio_read(void *arg, hwaddr addr, unsigned int size)
{
    COM_STRUCT_NAME(Gpio) *s = STM32COM_GPIO(arg);
    uint32_t r;

    addr >>= 2;
    r = s->regs[addr];
	CHECK_BOUNDS_R(addr, RI_END, s->reginfo[s->parent.periph - STM32_P_GPIOA], "STM32 GPIO");
//printf("GPIO unit %d reg %x return 0x%x\n", s->periph, (int)offset << 2, r);
    return r;
}

static void
stm32_common_gpio_update_odr(COM_STRUCT_NAME(Gpio) *s, uint16_t val)
{
    int i;
    uint16_t changed = s->regs[RI_ODR] ^ val;

    for (i = 0; i < STM32_GPIO_PIN_COUNT; i++)
    {
        if ((changed & 1<<i) == 0)
            continue;

        //printf("gpio %u pin %u = %c\n", s->periph, i, val & 1<<i ? 'H' : 'l');
        qemu_set_irq(s->pin[i], !!(val & 1<<i));
    }
    s->regs[RI_ODR] = val;
}

static void
stm32_common_gpio_update_mode(COM_STRUCT_NAME(Gpio) *s, uint32_t val)
{
    int i;
    uint32_t prev_value = s->regs[RI_MODER];

    for (i = 0; i < STM32_GPIO_PIN_COUNT; i++)
    {
        uint32_t setting = (val >> i*2) & 0x03;
        uint32_t prev_setting = (prev_value >> i*2) & 0x03;
        if (setting == prev_setting) {
            continue;
        }
        // int pupd =  (s->regs[RI_PUPDR] >> 2*i) & 0x3;
        //bool otype = (s->regs[RI_OTYPER]>>i) & 1;
        bool current_pin_state = (s->regs[RI_ODR] >>i)&1;
       // if(s->parent.periph==STM32_P_GPIOF) printf("Mode of pin %i changing to %u, with pupd %d and otype %u, current: %u\n", i, setting, pupd, otype, current_pin_state);
        qemu_set_irq(s->alternate_function[i], (setting == R_MODE_ALT) );

        if  (setting == R_MODE_INPUT && !current_pin_state)
        {
            stm32_common_gpio_update_odr(s, s->regs[RI_ODR] | (1U << i));
			// if (!(s->force_idr& (1U<<i)))
            // 	s->regs[RI_IDR] |= 1U << i;
        } else if (setting == R_MODE_OUTPUT && current_pin_state) {
            stm32_common_gpio_update_odr(s, s->regs[RI_ODR] & ~(1U << i));
        }
        (void) current_pin_state;
    }
    s->regs[RI_MODER] = val;
}

static void
stm32_common_gpio_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    COM_STRUCT_NAME(Gpio) *s = STM32COM_GPIO(arg);
    int offset = addr & 0x03;


    addr >>= 2;
    //if (s->periph == 4) printf("GPIOD write: %"HWADDR_PRIx" 0x%"PRIx64"\n", addr, data);
 	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo[s->parent.periph - STM32_P_GPIOA], "STM32 GPIO");
	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs[addr],data, size, offset, 0b111);

    // if (s->periph == 6) printf("GPIO: wrote %0"PRIx64" to  %"HWADDR_PRIx"\n", data, addr);
    switch (addr) {
    case RI_MODER:
        stm32_common_gpio_update_mode(s, data);
        break;
    case RI_ODR:
        stm32_common_gpio_update_odr(s, data);
        break;
    case RI_BSRR:
    {
       // if (s->periph==5) printf("BSRR write %08x\n", (uint32_t)data);
        uint16_t new_val = s->regs[RI_ODR];
        uint16_t  br_bits = data>>16;
        uint16_t  bs_bits = data&0xFFFF;
        new_val &= ~br_bits; /* BRy */
        new_val |= bs_bits; /* BSy */
        s->regs[RI_IDR] &=~br_bits;
        s->regs[RI_IDR] |= bs_bits;
        stm32_common_gpio_update_odr(s, new_val);
        break;
    }
	case RI_BRR:
    {
        uint16_t new_val = s->regs[RI_ODR];
        uint16_t  br_bits = data&0xFFFF;
        new_val &= ~br_bits; /* BRy */
        s->regs[RI_IDR] &=~br_bits;
        stm32_common_gpio_update_odr(s, new_val);
        break;
    }
	case RI_IDR:
	{
        qemu_log_mask(LOG_GUEST_ERROR, "Attempted to write read-only IDR register in %s\n", _PERIPHNAMES[s->parent.periph]);
		break;
	}
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx GPIO %u reg 0x%x:%d write (0x%x) unimplemented\n",
          s->parent.periph,  (int)addr << 2, offset, (int)data);
    /* FALLTHRU */
    case RI_AFRH:
    case RI_AFRL:
        s->regs[addr] = data;
        break;
    }
}

static const MemoryRegionOps STM32FXXGPIOState_ops = {
    .read = stm32_common_gpio_read,
    .write = stm32_common_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32_common_gpio_reset(DeviceState *dev)
{
    COM_STRUCT_NAME(Gpio) *s = STM32COM_GPIO(dev);

	for (int i=0;i<RI_END; i++)
	{
		s->regs[i] = s->reginfo[s->parent.periph - STM32_P_GPIOA][i].reset_val;
	}
    /* Mask out the IDR bits as specified */
    s->regs[RI_IDR] = 0x0000ffff & ~(s->idr_mask);
}

static void
stm32_common_gpio_set(void *arg, int pin, int level)
{
    COM_STRUCT_NAME(Gpio) *s = STM32COM_GPIO(arg);
    uint32_t bit = 1<<pin;
	uint32_t old_state = s->regs[RI_IDR] & bit;

    if (level)
        s->regs[RI_IDR] |= bit;
    else
        s->regs[RI_IDR] &= ~bit;

    /* Inform EXTI module of pin state */
	uint32_t transition = 0;
	if (!old_state && level)
	{
		transition = EXTI_RISING;
	}
	else if (old_state && !level)
	{
		transition = EXTI_FALLING;
	}
    if (transition) qemu_set_irq(s->exti[pin], transition);

    // For the stm32, only GPIOA, pin 0 will have a wakeup handler tied to it. It will be
    // tied to a handler callback in the NVIC.
    qemu_set_irq(s->cpu_wake[pin], level);

    DPRINTF("GPIO %u set pin %d level %d\n", s->periph, pin, level);
}


void
stm32_common_gpio_wake_set(COM_STRUCT_NAME(Gpio) *s, unsigned pin, qemu_irq irq)
{
    s->cpu_wake[pin] = irq;
    DPRINTF("GPIO %u set cpu_wake %u irq %p\n", s->parent.periph, pin, irq);
}

static void
stm32_common_gpio_init(Object *obj)
{
    COM_STRUCT_NAME(Gpio) *s = STM32COM_GPIO(obj);

    STM32_MR_IO_INIT(&s->iomem, obj, &STM32FXXGPIOState_ops, s, 1U * KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_in(DEVICE(obj), stm32_common_gpio_set, STM32_GPIO_PIN_COUNT);
    qdev_init_gpio_out(DEVICE(obj), s->pin, STM32_GPIO_PIN_COUNT);
	qdev_init_gpio_out_named(DEVICE(obj), s->exti, "exti", STM32_GPIO_PIN_COUNT);
    qdev_init_gpio_out_named(DEVICE(obj), s->alternate_function, "af", STM32_GPIO_PIN_COUNT);

	COM_CLASS_NAME(Gpio) *k = STM32COM_GPIO_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;
}

static Property stm32_common_gpio_properties[] = {
    DEFINE_PROP_UINT32("idr-mask", COM_STRUCT_NAME(Gpio), idr_mask, 0),
    DEFINE_PROP_UINT32("idr-force", COM_STRUCT_NAME(Gpio), force_idr, 0),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32_common_gpio = {
    .name = TYPE_STM32COM_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(idr_mask, COM_STRUCT_NAME(Gpio)),
        VMSTATE_UINT32_ARRAY(regs, COM_STRUCT_NAME(Gpio),RI_END),
        VMSTATE_END_OF_LIST()
    }
};


static void
stm32_common_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32_common_gpio_reset;
    dc->vmsd = &vmstate_stm32_common_gpio;
    device_class_set_props(dc, stm32_common_gpio_properties);

	COM_CLASS_NAME(Gpio) *k = STM32COM_GPIO_CLASS(klass);
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[MAX_GPIO_BANKS][RI_END]), "Reginfo not sized correctly!");
	stm32_reginfo_t (** ri_data)[MAX_GPIO_BANKS][RI_END] = data;
	for (int i=0; i<MAX_GPIO_BANKS; i++)
	{
		memcpy(k->var_reginfo[i], ri_data[i], sizeof(k->var_reginfo[0]));
	}

}

static const TypeInfo stm32_common_gpio_info = {
    .name = TYPE_STM32COM_GPIO,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Gpio)),
	.class_size = sizeof(COM_CLASS_NAME(Gpio)),
	.abstract = true
};

static void
stm32_common_gpio_register_types(void)
{
    type_register_static(&stm32_common_gpio_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_gpio_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_gpio_variants[i].variant_name,
            .parent     = TYPE_STM32COM_GPIO,
			.instance_init = stm32_common_gpio_init,
    		.class_init    = stm32_common_gpio_class_init,
            .class_data = (void *)stm32_gpio_variants[i].variant_regs,
        };
        type_register(&ti);
    }

}

type_init(stm32_common_gpio_register_types)
