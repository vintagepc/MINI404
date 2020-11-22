/*
 * STM32F2XX ADC
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
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
#include "hw/irq.h"
#include "qemu/typedefs.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "stm32f4xx_adc.h"

#ifndef STM_ADC_ERR_DEBUG
#define STM_ADC_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_ADC_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

#define ADC_SR    0x00
#define ADC_CR1   0x04
#define ADC_CR2   0x08
#define ADC_SMPR1 0x0C
#define ADC_SMPR2 0x10
#define ADC_JOFR1 0x14
#define ADC_JOFR2 0x18
#define ADC_JOFR3 0x1C
#define ADC_JOFR4 0x20
#define ADC_HTR   0x24
#define ADC_LTR   0x28
#define ADC_SQR1  0x2C
#define ADC_SQR2  0x30
#define ADC_SQR3  0x34
#define ADC_JSQR  0x38
#define ADC_JDR1  0x3C
#define ADC_JDR2  0x40
#define ADC_JDR3  0x44
#define ADC_JDR4  0x48
#define ADC_DR    0x4C


#define R_CR1_RES_SHIFT 24
#define R_CR1_RES (3 << R_CR1_RES_SHIFT)

#define ADC_COMMON_ADDRESS 0x100

#define R_SR_EOC       (1<<1)

#define R_SQR1_L_SHIFT  (20)
#define R_SQR1_L_MASK    (0xF<<R_SQR1_L_SHIFT)

#define R_CR2_SWSTART   (1<<30)
#define R_CR2_ALIGN   0x800
#define R_CR2_EOCS      (1<<10)
#define R_CR2_CONT    0x02
#define R_CR2_ADON      (1<<0)


static void stm32f4xx_adc_reset(DeviceState *dev)
{
    STM32F4XXADCState *s = STM32F4XX_ADC(dev);

    s->adc_sr = 0x00000000;
    s->adc_cr1 = 0x00000000;
    s->adc_cr2 = 0x00000000;
    s->adc_smpr1 = 0x00000000;
    s->adc_smpr2 = 0x00000000;
    s->adc_jofr[0] = 0x00000000;
    s->adc_jofr[1] = 0x00000000;
    s->adc_jofr[2] = 0x00000000;
    s->adc_jofr[3] = 0x00000000;
    s->adc_htr = 0x00000FFF;
    s->adc_ltr = 0x00000000;
    s->adc_sqr1 = 0x00000000;
    s->adc_sqr2 = 0x00000000;
    s->adc_sqr3 = 0x00000000;
    s->adc_jsqr = 0x00000000;
    s->adc_jdr[0] = 0x00000000;
    s->adc_jdr[1] = 0x00000000;
    s->adc_jdr[2] = 0x00000000;
    s->adc_jdr[3] = 0x00000000;
    s->adc_dr = 0x00000000;

    memset(&s->adc_sequence,0,ADC_NUM_REG_CHANNELS);
    memset(&s->adc_data,0,ADC_NUM_REG_CHANNELS*sizeof(int));
    s->adc_sequence_position = 0;
}

static uint32_t stm32f4xx_adc_get_value(STM32F4XXADCState *s)
{
    uint8_t channel = s->adc_sequence[s->adc_sequence_position];
    s->adc_dr = s->adc_data[channel] *4; // TODO... *3 ->sample time

    // printf("Channel %d: %d\n", channel, s->adc_dr);

    switch ((s->adc_cr1 & R_CR1_RES) >> R_CR1_RES_SHIFT) {
    case 0:
        /* 12-bit */
        s->adc_dr &= 0xFFF;
        break;
    case 1:
        /* 10-bit */
        s->adc_dr &= 0x3FF;
        break;
    case 2:
        /* 8-bit */
        s->adc_dr &= 0xFF;
        break;
    default:
        /* 6-bit */
        s->adc_dr &= 0x3F;
    }

    if (s->adc_cr2 & R_CR2_ALIGN) {
        return (s->adc_dr << 1) & 0xFFF0;
    } else {
        return s->adc_dr;
    }
}

static void stm32f4xx_adc_data_in(void *opaque, int n, int level){
    STM32F4XXADCState *s = opaque;
    s->adc_data[n] = level;
    // printf("ADC: Ch %d new data: %d\n",n, level);
}

static uint64_t stm32f4xx_adc_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    STM32F4XXADCState *s = opaque;

   // DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);

    if (addr >= ADC_COMMON_ADDRESS) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: ADC Common Register Unsupported\n", __func__);
    }

    switch (addr) {
    case ADC_SR:
        return s->adc_sr;
    case ADC_CR1:
        return s->adc_cr1;
    case ADC_CR2:
        return s->adc_cr2 & 0xFFFFFFF;
    case ADC_SMPR1:
        return s->adc_smpr1;
    case ADC_SMPR2:
        return s->adc_smpr2;
    case ADC_JOFR1:
    case ADC_JOFR2:
    case ADC_JOFR3:
    case ADC_JOFR4:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        return s->adc_jofr[(addr - ADC_JOFR1) / 4];
    case ADC_HTR:
        return s->adc_htr;
    case ADC_LTR:
        return s->adc_ltr;
    case ADC_SQR1:
        return s->adc_sqr1;
    case ADC_SQR2:
        return s->adc_sqr2;
    case ADC_SQR3:
        return s->adc_sqr3;
    case ADC_JSQR:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        return s->adc_jsqr;
    case ADC_JDR1:
    case ADC_JDR2:
    case ADC_JDR3:
    case ADC_JDR4:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        return s->adc_jdr[(addr - ADC_JDR1) / 4] -
               s->adc_jofr[(addr - ADC_JDR1) / 4];
    case ADC_DR:
        if ((s->adc_cr2 & R_CR2_ADON) && (s->adc_sr & R_SR_EOC)) {
            return stm32f4xx_adc_get_value(s);
            s->adc_sr ^= R_SR_EOC;
        } else {
            return 0;
        }
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }

    return 0;
}

static void stm32f4xx_adc_update_sequence(STM32F4XXADCState *s)
{
    uint8_t length =(s->adc_sqr1 & R_SQR1_L_MASK)>>R_SQR1_L_SHIFT;
    DB_PRINT("ADC Sequence length: %d (", length+1);
    uint32_t *src_reg[3] = {&s->adc_sqr3, &s->adc_sqr2, &s->adc_sqr1 };
    for (uint8_t i=0U; i<ADC_NUM_REG_CHANNELS; i++)
    {
        if (i>length)
        {
            s->adc_sequence[i] = 0; 
            continue;
        }
        uint8_t src_index = i/6;
        uint8_t src_shift = (i%6)*5;
        uint32_t src_mask = 0x1F << src_shift;
        s->adc_sequence[i] = (*src_reg[src_index] & src_mask) >> src_shift;
        DB_PRINT("%d, ", s->adc_sequence[i]);
    }
    DB_PRINT(")\n");

}

static void stm32f4xx_adc_convert(STM32F4XXADCState *s)
{
    s->adc_cr2 ^= R_CR2_SWSTART;
    // TODO- sequences >0
    uint8_t channel = s->adc_sequence[s->adc_sequence_position];
    qemu_irq_pulse(s->irq_read[channel]); // Toggle the data read request IRQ. The receiver can opt to send a new value (or do nothing)
    s->adc_sr |= R_SR_EOC;
}

static void stm32f4xx_adc_write(void *opaque, hwaddr addr,
                       uint64_t val64, unsigned int size)
{
    STM32F4XXADCState *s = opaque;
    uint32_t value = (uint32_t) val64;

    DB_PRINT("Address: 0x%" HWADDR_PRIx ", Value: 0x%x\n",
             addr, value);

    if (addr >= 0x100) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: ADC Common Register Unsupported\n", __func__);
    }

    switch (addr) {
    case ADC_SR:
        s->adc_sr &= (value & 0x3F);
        break;
    case ADC_CR1:
        s->adc_cr1 = value;
        break;
    case ADC_CR2:
        s->adc_cr2 = value;
        if (value & R_CR2_SWSTART)
        {
            stm32f4xx_adc_convert(s);
        }
        break;
    case ADC_SMPR1:
        if (value!=0) printf("FIXME: Nonzero sample time\n");
        s->adc_smpr1 = value;
        break;
    case ADC_SMPR2:
        if (value!=0) printf("FIXME: Nonzero sample time\n");
        s->adc_smpr2 = value;
        break;
    case ADC_JOFR1:
    case ADC_JOFR2:
    case ADC_JOFR3:
    case ADC_JOFR4:
        s->adc_jofr[(addr - ADC_JOFR1) / 4] = (value & 0xFFF);
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        break;
    case ADC_HTR:
        s->adc_htr = value;
        break;
    case ADC_LTR:
        s->adc_ltr = value;
        break;
    case ADC_SQR1:
        s->adc_sqr1 = value;
        stm32f4xx_adc_update_sequence(s);
        break;
    case ADC_SQR2:
        s->adc_sqr2 = value;
        stm32f4xx_adc_update_sequence(s);
        break;
    case ADC_SQR3:
        s->adc_sqr3 = value;
        stm32f4xx_adc_update_sequence(s);
        break;
    case ADC_JSQR:
        s->adc_jsqr = value;
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        break;
    case ADC_JDR1:
    case ADC_JDR2:
    case ADC_JDR3:
    case ADC_JDR4:
        s->adc_jdr[(addr - ADC_JDR1) / 4] = value;
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }
}

static const MemoryRegionOps stm32f4xx_adc_ops = {
    .read = stm32f4xx_adc_read,
    .write = stm32f4xx_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_stm32f4xx_adc = {
    .name = TYPE_STM32F4XX_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(adc_sr, STM32F4XXADCState),
        VMSTATE_UINT32(adc_cr1, STM32F4XXADCState),
        VMSTATE_UINT32(adc_cr2, STM32F4XXADCState),
        VMSTATE_UINT32(adc_smpr1, STM32F4XXADCState),
        VMSTATE_UINT32(adc_smpr2, STM32F4XXADCState),
        VMSTATE_UINT32_ARRAY(adc_jofr, STM32F4XXADCState, 4),
        VMSTATE_UINT32(adc_htr, STM32F4XXADCState),
        VMSTATE_UINT32(adc_ltr, STM32F4XXADCState),
        VMSTATE_UINT32(adc_sqr1, STM32F4XXADCState),
        VMSTATE_UINT32(adc_sqr2, STM32F4XXADCState),
        VMSTATE_UINT32(adc_sqr3, STM32F4XXADCState),
        VMSTATE_UINT32(adc_jsqr, STM32F4XXADCState),
        VMSTATE_UINT32_ARRAY(adc_jdr, STM32F4XXADCState, 4),
        VMSTATE_UINT32(adc_dr, STM32F4XXADCState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f4xx_adc_init(Object *obj)
{
    STM32F4XXADCState *s = STM32F4XX_ADC(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    qdev_init_gpio_out_named(DEVICE(obj), s->irq_read, "adc_read", ADC_NUM_REG_CHANNELS);

    qdev_init_gpio_in_named(DEVICE(obj),stm32f4xx_adc_data_in, "adc_data_in", ADC_NUM_REG_CHANNELS);

    memory_region_init_io(&s->mmio, obj, &stm32f4xx_adc_ops, s,
                          TYPE_STM32F4XX_ADC, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void stm32f4xx_adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32f4xx_adc_reset;
    dc->vmsd = &vmstate_stm32f4xx_adc;
}

static const TypeInfo stm32f4xx_adc_info = {
    .name          = TYPE_STM32F4XX_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F4XXADCState),
    .instance_init = stm32f4xx_adc_init,
    .class_init    = stm32f4xx_adc_class_init,
};

static void stm32f4xx_adc_register_types(void)
{
    type_register_static(&stm32f4xx_adc_info);
}

type_init(stm32f4xx_adc_register_types)
