/*
 * STM32F4XX ADC
 *
 * Original Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified/refactored for Mini404 2021-3 by VintagePC <http://github.com/vintagepc>
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
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "../utility/macros.h"
#include "stm32f4xx_adcc.h"
#include "hw/qdev-properties.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_common/stm32_rcc_if.h"

#ifndef STM_ADC_ERR_DEBUG
#define STM_ADC_ERR_DEBUG 0
#endif

enum RegIndex {
	RI_SR =     (0x00/4),
	RI_CR1 =    (0x04/4),
	RI_CR2 =    (0x08/4),
	RI_SMPR1 =  (0x0C/4),
	RI_SMPR2 =  (0x10/4),
	RI_JOFR1 =  (0x14/4),
	RI_JOFR2 =  (0x18/4),
	RI_JOFR3 =  (0x1C/4),
	RI_JOFR4 =  (0x20/4),
	RI_HTR =    (0x24/4),
	RI_LTR =    (0x28/4),
	RI_SQR1 =   (0x2C/4),
	RI_SQR2 =   (0x30/4),
	RI_SQR3 =   (0x34/4),
	RI_JSQR =   (0x38/4),
	RI_JDR1 =   (0x3C/4),
	RI_JDR2 =   (0x40/4),
	RI_JDR3 =   (0x44/4),
	RI_JDR4 =   (0x48/4),
	RI_DR =     (0x4C/4),
	RI_END,
};

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_ADC_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

#define ADC_COMMON_ADDRESS 0x100

typedef struct STM32F4XXADCCState STM32F4XXADCCState;

#define ADC_NUM_REG_CHANNELS 18

OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XXADCState, STM32F4xx_ADC)

struct STM32F4XXADCState {
    /* <private> */
    STM32Peripheral parent;

    /* <public> */
    MemoryRegion mmio;

    union {
        uint32_t regs[RI_END];
        struct {
            struct {
                REG_B32(AWD);
                REG_B32(EOC);
                REG_B32(JEOC);
                REG_B32(JSTRT);
                REG_B32(STRT);
                REG_B32(OVR);
                uint32_t :26; // unused.
            } QEMU_PACKED  SR;
            struct {
                uint32_t AWDCH :5;
                REG_B32(EOCIE);
                REG_B32(AWDIE);
                REG_B32(JEOCIE);
                REG_B32(SCAN);
                REG_B32(AWDSGL);
                REG_B32(JAUTO);
                REG_B32(DISCEN);
                REG_B32(JDISCEN);
                uint32_t DISCNUM :3;
                uint32_t _unused :6;
                REG_B32(JAWDEN);
                REG_B32(AWDEN);
                uint32_t RES :2;
                REG_B32(OVRIE);
                uint32_t :5; // unused.
            } QEMU_PACKED  CR1;
            struct {
                REG_B32(ADON);
                REG_B32(CONT);
                uint32_t _unused :6;
                REG_B32(DMA);
                REG_B32(DDS);
                REG_B32(EOCS);
                REG_B32(ALIGN);
                uint32_t _unused2 :4;
                uint32_t JEXTSEL :4;
                uint32_t JEXTEN :2;
                REG_B32(JSWSTART);
                REG_B32(_unused3);
                uint32_t EXTSEL :4;
                uint32_t EXTEN :2;
                REG_B32(SWSTART);
                REG_B32(_unused4);
            } QEMU_PACKED  CR2;
            uint32_t SMPR1;
            uint32_t SMPR2;
            REG_S32(JOFR1,12);
            REG_S32(JOFR2,12);
            REG_S32(JOFR3,12);
            REG_S32(JOFR4,12);
            REG_S32(HT,12);
            REG_S32(LT,12);
            struct {
                uint32_t SQ13 :5;
                uint32_t SQ14 :5;
                uint32_t SQ15 :5;
                uint32_t SQ16 :5;
                uint32_t L :4;
                uint32_t   :8;
            } QEMU_PACKED SQR1;
            uint32_t SQR[3];
            REG_S32(JDR1,16);
            REG_S32(JDR2,16);
            REG_S32(JDR3,16);
            REG_S32(JDR4,16);
            REG_S32(DR,16);
        } QEMU_PACKED defs;
    };

    uint8_t  adc_smprs[19];

    STM32F4XXADCCState* common;

    qemu_irq irq;

    //qemu_irq irq_read[ADC_NUM_REG_CHANNELS]; // Set when the ADC wants to get a value from the channel.

    int adc_data[ADC_NUM_REG_CHANNELS]; // Store the peripheral data received.
    uint8_t adc_sequence[ADC_NUM_REG_CHANNELS];

    uint8_t adc_sequence_position, adc_next_seq_pos;

    // Pre-computed conversion times
    uint64_t adc_conv_times_ns[ADC_NUM_REG_CHANNELS];

    QEMUTimer* next_eoc;
};

typedef union {
    uint32_t raw;
    struct {
        uint32_t SMP0 :3;
        uint32_t SMP1 :3;
        uint32_t SMP2 :3;
        uint32_t SMP3 :3;
        uint32_t SMP4 :3;
        uint32_t SMP5 :3;
        uint32_t SMP6 :3;
        uint32_t SMP7 :3;
        uint32_t SMP8 :3;
        uint32_t SMP9 :3;
        uint32_t :5;
    } QEMU_PACKED;
} adc_smpr_t;

static const uint16_t adc_smpr_map[] = { 3, 15, 28, 56, 84, 112, 144, 480 };

static void stm32f4xx_adc_reset(DeviceState *dev)
{
    STM32F4XXADCState *s = STM32F4xx_ADC(dev);

    memset(&s->regs,0,sizeof(s->regs));
    s->defs.HT = 0xFFF;

    memset(&s->adc_sequence,0,ADC_NUM_REG_CHANNELS);
    // We can't reset the data here because it might 
    // clear the initial stuff sent by other device resets. 


    //memset(&s->adc_data,0,ADC_NUM_REG_CHANNELS*sizeof(int));
    s->adc_sequence_position = 0;
	s->adc_next_seq_pos = 0;

	if (s->next_eoc)
		timer_del(s->next_eoc);
}


static uint32_t stm32f4xx_adc_get_value(STM32F4XXADCState *s)
{
    uint8_t channel = s->adc_sequence[s->adc_sequence_position];
    s->defs.DR = s->adc_data[channel];
    // I'm not sure why this is yet - some sort of built in oversampling
    // that is enabled in non-DMA mode?
    if (!s->defs.CR2.DMA) {
        assert(s->adc_smprs[channel] < ARRAY_SIZE(adc_smpr_map));
        s->defs.DR*=(adc_smpr_map[s->adc_smprs[channel]]+1);
    }

    // Mask: RES 0..3 == 12..6 bit mask.
    s->defs.DR = (s->defs.DR & 0xFFF)>>(s->defs.CR1.RES<<1);

    if (s->defs.CR2.ALIGN) {
        return (s->defs.DR << 1) & 0xFFF0;
    } else {
        return s->defs.DR;
    }
}

// ADC data in from peripherals
static void stm32f4xx_adc_data_in(void *opaque, int n, int level){
    STM32F4XXADCState *s = opaque;
    s->adc_data[n] = level;
    // printf("%s: Ch %d new data: %d\n", _PERIPHNAMES[s->parent.periph], n, level);
}

static void stm32f4xx_adc_recalc_times(STM32F4XXADCState *s) {
    // Get the clock rate
    uint64_t clock = s->parent.clock_freq;
    clock /= stm32f4xx_adcc_get_adcpre(s->common);
    uint32_t conv_cycles = (12U - (s->defs.CR1.RES<<1U));
    for (int i=0; i<ADC_NUM_REG_CHANNELS; i++)
    {
        assert(s->adc_smprs[i] < ARRAY_SIZE(adc_smpr_map));
        uint32_t ch_conv_cycles = conv_cycles + adc_smpr_map[s->adc_smprs[i]];
        s->adc_conv_times_ns[i] = 1000000000000U / (clock/ch_conv_cycles);
        if (s->parent.periph == STM32_P_ADC3)
        {
		// Yes, this is an ugly-ass hack. The above calc is off by 1000 and I still need to determine
		// how to deal with the other channels bogging down the simulation when they run at the "real" specified rate.
	    	 s->adc_conv_times_ns[i] /= 1000;
		//printf("ADC conversion: %u cycles @ %"PRIu64" Hz (%lu nSec)\n", conv_cycles, clock, delay_ns);
	    }
    }
}

static void stm32f4xx_adc_schedule_next(STM32F4XXADCState *s) {
    if (!s->defs.CR2.ADON)
        return;
    s->defs.CR2.SWSTART = 0;
    uint8_t channel = s->adc_sequence[s->adc_next_seq_pos];
    timer_mod_ns(s->next_eoc, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+s->adc_conv_times_ns[channel]);
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
    if (size!=4) {
        printf("FIXME: handle non-word ADC reads!\n");
    }
    addr>>=2;

    switch (addr) {
    case RI_SR ... RI_SMPR2:
    case RI_HTR ... RI_SQR3:
        return s->regs[addr];
    case RI_JSQR:
    case RI_JOFR1 ... RI_JOFR4:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        return s->regs[addr];
    case RI_JDR1 ... RI_JDR4:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        return s->regs[addr] - s->regs[addr+RI_JOFR1-RI_JDR1];
    case RI_DR:
        if (s->defs.CR2.ADON && s->defs.SR.EOC) {
            s->defs.SR.EOC ^= s->defs.SR.EOC;
            return stm32f4xx_adc_get_value(s);
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
    uint8_t length = s->defs.SQR1.L;
    DB_PRINT("ADC Sequence length: %d (", length+1);
    uint32_t src_reg[3] = {s->regs[RI_SQR3], s->regs[RI_SQR2], s->regs[RI_SQR1]};
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
        s->adc_sequence[i] = (src_reg[src_index] & src_mask) >> src_shift;
        DB_PRINT("%d, ", s->adc_sequence[i]);
    }
    DB_PRINT(")\n");

}

// static void stm32f4xx_adc_convert(STM32F4XXADCState *s)
// {
//     uint8_t channel = s->adc_sequence[s->adc_sequence_position];
//     qemu_irq_raise(s->irq_read[channel]); // Toggle the data read request IRQ. The receiver can opt to send a new value (or do nothing)
// }

static void stm32f4xx_adc_update_irqs(STM32F4XXADCState *s, int level) {

    bool bChanged = level^s->defs.SR.EOC;
    s->defs.SR.EOC = level;

    if (s->defs.CR1.EOCIE && level && bChanged)
    {
        qemu_irq_raise(s->irq);
    }

    if (s->defs.CR2.DMA && level)
    {
        qemu_set_irq(s->parent.dmar[DMAR_P2M], s->mmio.addr + (4U * RI_DR));
    }

}

static void stm32f4xx_adc_eoc_deadline(void *opaque) {

    STM32F4XXADCState *s = STM32F4xx_ADC(opaque);
	s->adc_sequence_position = s->adc_next_seq_pos;
    //stm32f4xx_adc_convert(s);
    if (s->defs.CR2.EOCS || s->adc_sequence_position==s->defs.SQR1.L)
    {
        // Either end of cycle or end-of-sequence.
        stm32f4xx_adc_update_irqs(s, 1);
    }

    if (s->defs.CR2.DMA)
    {
        if (s->defs.CR2.CONT && s->defs.CR1.SCAN)
        {
            s->adc_next_seq_pos = (s->adc_sequence_position+1)%(s->defs.SQR1.L+1);
            // Schedule next. Only loop if DDS is set:
            if (s->defs.CR2.DDS || s->adc_next_seq_pos !=0) {
                stm32f4xx_adc_schedule_next(s);
            }
        }
    }
}

static void stm32f4xx_adc_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    STM32F4XXADCState *s = opaque;

    // printf("ADC_write %d : 0x%" HWADDR_PRIx ", Value: 0x%x\n",
//             s->id, addr, (uint32_t)value);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: ADC write with size != word (32 bits)!\n", __func__);
    }

    addr>>=2; // Get index in array.

    if (addr >= 0x100) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: ADC Common Register Unsupported\n", __func__);
    }

    switch (addr) {
        case RI_CR1:
            {
                uint8_t old_res = s->defs.CR1.RES;
                s->regs[addr] = value;
                if (s->defs.CR1.RES != old_res)
                {
                    stm32f4xx_adc_recalc_times(s);
                }
            }
            break;
        case RI_SR:
        case RI_HTR:
        case RI_LTR:
            s->regs[addr] = value;
            break;
        case RI_CR2:
            s->regs[addr] = value;
            if (s->defs.CR2.SWSTART)
            {
                stm32f4xx_adc_schedule_next(s);
            }
            // Ugly hack alert. This is just because we don't run the ADC scheduling
            // at full throttle and the Mini FW BSODs if the ADC isn't quite ready in time in non_DMA mode.
            if (!s->defs.CR2.DMA)
            {
                s->defs.SR.EOC = 1;
            }
            break;
        case RI_SMPR1:
            // if (value!=0) printf("FIXME: Nonzero sample time\n");
            s->regs[addr] = value;
            for (int i=0; i<9; i++)
            {
                s->adc_smprs[10+i] = (value >> (3*i)) & 0x7;
            }
            stm32f4xx_adc_recalc_times(s);
            break;
        case RI_SMPR2:
            // if (value!=0) printf("FIXME: Nonzero sample time\n");
            s->regs[addr] = value;
            for (int i=0; i<10; i++)
            {
                s->adc_smprs[i] = (value >> (3*i)) & 0x7;
            }
            stm32f4xx_adc_recalc_times(s);
            break;
        case RI_JOFR1 ... RI_JOFR4:
            s->regs[addr] = (value & 0xFFF);
            qemu_log_mask(LOG_UNIMP, "%s: " \
                        "Injection ADC is not implemented, the registers are " \
                        "included for compatibility\n", __func__);
            break;
        case RI_SQR1 ... RI_SQR3:
            s->regs[addr] = value;
            stm32f4xx_adc_update_sequence(s);
            break;
        case RI_JSQR:
            s->regs[addr] = value;
            qemu_log_mask(LOG_UNIMP, "%s: " \
                        "Injection ADC is not implemented, the registers are " \
                        "included for compatibility\n", __func__);
            break;
        case RI_JDR1 ... RI_JDR4:
            s->regs[addr] = value;
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
    .name = TYPE_STM32F4xx_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, STM32F4XXADCState, RI_END),
        VMSTATE_INT32_ARRAY(adc_data,STM32F4XXADCState, ADC_NUM_REG_CHANNELS),
        VMSTATE_UINT8_ARRAY(adc_sequence,STM32F4XXADCState, ADC_NUM_REG_CHANNELS),
        VMSTATE_UINT8(adc_sequence_position,STM32F4XXADCState),
        VMSTATE_UINT8(adc_next_seq_pos,STM32F4XXADCState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f4xx_adc_init(Object *obj)
{
    STM32F4XXADCState *s = STM32F4xx_ADC(obj);

    // Check the register union definitions... This thows compile errors if they are misaligned, so it's ok in regards to not throwing exceptions
    // during object init in QEMU.
    CHECK_ALIGN(sizeof(s->defs),sizeof(uint32_t)*RI_END, "defs union");
    CHECK_ALIGN(sizeof(s->defs),sizeof(s->regs), "Raw array");
    // Check the bitfields. S32s should be fine because
    // the macro handles the padding math and problems are detected by the overall size change above
    CHECK_REG_u32(s->defs.SR);
    CHECK_REG_u32(s->defs.CR1);
    CHECK_REG_u32(s->defs.CR2);
    CHECK_REG_u32(s->defs.SQR1);
    CHECK_REG_u32(s->defs.SMPR1);
    CHECK_REG_u32(s->defs.SMPR2);
    QEMU_BUILD_BUG_MSG(RI_END != 20, "Size of register array has changed. You need to update VMState!");


    s->next_eoc = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32f4xx_adc_eoc_deadline, s);


    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    // qdev_init_gpio_out_named(DEVICE(obj), s->irq_read, "adc_read", ADC_NUM_REG_CHANNELS);

    qdev_init_gpio_in_named(DEVICE(obj),stm32f4xx_adc_data_in, "adc_data_in", ADC_NUM_REG_CHANNELS);

    STM32_MR_IO_INIT(&s->mmio, obj, &stm32f4xx_adc_ops, s, 1U*256);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static Property stm32f4xx_adc_properties[] = {
    DEFINE_PROP_LINK("common", STM32F4XXADCState, common, TYPE_STM32F4XX_ADCC, STM32F4XXADCCState *),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32f4xx_adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32f4xx_adc_reset;
    dc->vmsd = &vmstate_stm32f4xx_adc;
	device_class_set_props(dc, stm32f4xx_adc_properties);
}

static const TypeInfo stm32f4xx_adc_info = {
    .name          = TYPE_STM32F4xx_ADC,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(STM32F4XXADCState),
    .instance_init = stm32f4xx_adc_init,
    .class_init    = stm32f4xx_adc_class_init,
};

static void stm32f4xx_adc_register_types(void)
{
    type_register_static(&stm32f4xx_adc_info);
}

type_init(stm32f4xx_adc_register_types)
