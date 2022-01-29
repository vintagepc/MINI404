/*-
 * Poritons Copyright (c) 2013 https://github.com/pebble/qemu/
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
/*
 * QEMU stm32f2xx TIM emulation
 */

#ifndef STM32F2XX_TIM_H
#define STM32F2XX_TIM_H

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "stm32.h"
#include "../stm32_common/stm32_shared.h"
#include "../stm32_common/stm32_common.h"


#define SHORT_REG_32(name,used) struct{ uint32_t name :used; uint32_t :32-used; } QEMU_PACKED

#define R_TIM_MAX    (0x54 / 4)

#define TYPE_STM32F4XX_TIMER "stm32f4xx-timer"
OBJECT_DECLARE_SIMPLE_TYPE(f2xx_tim, STM32F4XX_TIMER)

struct f2xx_tim {
    STM32Peripheral parent;
    MemoryRegion iomem;

    QEMUTimer *timer, *ccrtimer[4];
    qemu_irq irq;
    // Union-ized for my/code sanity and easier inspection during debugging.
    union {
        uint32_t regs[R_TIM_MAX];
        struct {
            struct {
                uint32_t CEN :1;
                uint32_t UDIS :1;
                uint32_t URS :1;
                uint32_t OPM :1;
                uint32_t DIR :1;
                uint32_t CMS :2;
                uint32_t ARPE :1;
                uint32_t CKD :2;
                uint32_t :22; // unused.
            } QEMU_PACKED  CR1;
            struct {
                uint32_t CCPC :1;
                uint32_t _reserved :1;
                uint32_t CCUS :1;
                uint32_t CCDS :1;
                uint32_t MMS :3;
                uint32_t TI1S :1;
                uint32_t OIS1 :1;
                uint32_t OIS1N :1;
                uint32_t OIS2 :1;
                uint32_t OIS2N :1;
                uint32_t OIS3 :1;
                uint32_t OIS3N :1;
                uint32_t OIS4 :1;
                uint32_t :32-15; // unused.
            } QEMU_PACKED  CR2;
            struct {
                uint32_t SMS :3;
                uint32_t _reserved :1;
                uint32_t TS :3;
                uint32_t MSM :1;
                uint32_t ETF :4;
                uint32_t ETPS :2;
                uint32_t ECE :1;
                uint32_t ETP :1;
                uint32_t :32-16; // unused.
            } QEMU_PACKED  SMCR;
            struct {
                uint32_t UIE :1;
                uint32_t CC1IE:1;
                uint32_t CC2IE:1;
                uint32_t CC3IE:1;
                uint32_t CC4IE:1;
                uint32_t COMIE:1;
                uint32_t TIE:1;
                uint32_t BIE:1;
                uint32_t UDE:1;
                uint32_t CC1DE:1;
                uint32_t CC2DE:1;
                uint32_t CC3DE:1;
                uint32_t CC4DE:1;
                uint32_t COMDE:1;
                uint32_t TDE:1;
                uint32_t :17;
            } QEMU_PACKED DIER;
            struct {
                uint32_t UIF:1;
                uint32_t CC1IF:1;
                uint32_t CC2IF:1;
                uint32_t CC3IF:1;
                uint32_t CC4IF:1;
                uint32_t COMIF:1;
                uint32_t TIF:1;
                uint32_t BIF:1;
                uint32_t _reserved :1;
                uint32_t CC1OF:1;
                uint32_t CC2OF:1;
                uint32_t CC3OF:1;
                uint32_t CC4OF:1;
                uint32_t :32-13;
            } QEMU_PACKED SR;
            struct {
                uint32_t UG:1;
                uint32_t CC1G:1;
                uint32_t CC2G:1;
                uint32_t CC3G:1;
                uint32_t CC4G:1;
                uint32_t COMG:1;
                uint32_t TG:1;
                uint32_t BG:1;
                uint32_t :32-8;
            } QEMU_PACKED EGR;
            struct {
                uint32_t CC1S :2;
                uint32_t OC1FE :1;
                uint32_t OC1PE :1;
                uint32_t OC1M :3;
                uint32_t OC1CE :1;
                uint32_t CC2S :2;
                uint32_t OC2FE :1;
                uint32_t OC2PE :1;
                uint32_t OC2M :3;
                uint32_t OC2CE :1;
                uint32_t :32-16;
            } QEMU_PACKED CCMR1; // N.B this is only the OC mode, not IC
            struct {
                uint32_t CC3S :2;
                uint32_t OC3FE :1;
                uint32_t OC3PE :1;
                uint32_t OC3M :3;
                uint32_t OC3CE :1;
                uint32_t CC4S :2;
                uint32_t OC4FE :1;
                uint32_t OC4PE :1;
                uint32_t OC4M :3;
                uint32_t OC4CE :1;
                uint32_t :32-16;
            } QEMU_PACKED CCMR2;
            struct {
                uint32_t CC1E :1;
                uint32_t CC1P :1;
                uint32_t CC1NE :1;
                uint32_t CC1NP :1;
                uint32_t CC2E :1;
                uint32_t CC2P :1;
                uint32_t CC2NE :1;
                uint32_t CC2NP :1;
                uint32_t CC3E :1;
                uint32_t CC3P :1;
                uint32_t CC3NE :1;
                uint32_t CC3NP :1;
                uint32_t CC4E :1;
                uint32_t CC4P :1;
                uint32_t :32-14;
            } QEMU_PACKED  CCER;
            uint32_t CNT;
            SHORT_REG_32(PSC, 16);
            uint32_t ARR;
            SHORT_REG_32(REP,8) RCR;
            uint32_t CCR1;
            uint32_t CCR2;
            uint32_t CCR3;
            uint32_t CCR4;
            struct {
                uint32_t DT :8;
                uint32_t LOCK :2;
                uint32_t OSSI :1;
                uint32_t OSSR :1;
                uint32_t BKE :1;
                uint32_t BKP :1;
                uint32_t AOE :1;
                uint32_t MOE :1;
                uint32_t :32-16;
            } QEMU_PACKED  BDTR;
            struct {
                uint32_t DBA:5;
                uint32_t _reserved :3;
                uint32_t DBL :5;
                uint32_t :19;
            } QEMU_PACKED DCR;
            uint32_t DMAR;
            SHORT_REG_32(RMP,2) OR;
        } QEMU_PACKED defs;
    };
    qemu_irq pwm_ratio_changed[4];
    qemu_irq pwm_enable[4], pwm_pin[4];

    int64_t count_timebase;

};

#endif // STM32F2XX_TIM_H
