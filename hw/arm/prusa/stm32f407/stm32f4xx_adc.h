/*
 * STM32F4XX ADC
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified/bugfixed for Mini404 2021 by VintagePC <http://github.com/vintagepc>
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

#ifndef HW_STM32F4XX_ADC_H
#define HW_STM32F4XX_ADC_H

#include "qom/object.h"
#include "../utility/macros.h"
#include "../stm32_common/stm32_common.h"
#include "../stm32_common/stm32_rcc_if.h"
#include "stm32.h"

typedef struct STM32F4XXADCCState STM32F4XXADCCState;

#define ADC_NUM_REG_CHANNELS 16

#define TYPE_STM32F4XX_ADC "stm32f4xx-adc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XXADCState, STM32F4XX_ADC)

#define R_ADC_MAX    (0x50/4)

struct STM32F4XXADCState {
    /* <private> */
    STM32Peripheral parent;

    /* <public> */
    MemoryRegion mmio;

    union {
        uint32_t regs[R_ADC_MAX];
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

    qemu_irq irq_read[ADC_NUM_REG_CHANNELS]; // Set when the ADC wants to get a value from the channel.

    int adc_data[ADC_NUM_REG_CHANNELS]; // Store the peripheral data received.
    uint8_t adc_sequence[ADC_NUM_REG_CHANNELS];

    uint8_t adc_sequence_position;

    QEMUTimer* next_eoc;
};

#endif /* HW_STM32F4XX_ADC_H */
