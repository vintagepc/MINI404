/*
 * STM32F407 SoC
 *
 * Original F405 base (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified and adapted for Mini404/F407 2020 by VintagePC <http://github.com/vintagepc>
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

#ifndef HW_ARM_STM32F4XX_SOC_H
#define HW_ARM_STM32F4XX_SOC_H

#include "hw/misc/stm32f4xx_syscfg.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "qemu/units.h"
#include "hw/misc/stm32f4xx_exti.h"
#include "hw/or-irq.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_adcc.h"
#include "stm32f2xx_crc.h"
#include "../stm32_common/stm32_f2xx_f4xx_dma.h"
#include "stm32f2xx_flashint.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_i2c.h"
#include "stm32f4xx_itm.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f2xx_pwr.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_rtc.h"
#include "stm32f4xx_spi.h"
#include "stm32f2xx_tim.h"
#include "stm32f4xx_otp.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_usb.h"
#include "hw/arm/armv7m.h"
#include "stm32_uart.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XX_STRUCT_NAME(), STM32F4XX_BASE)

#define STM_NUM_USARTS 7
#define STM_NUM_TIMERS 14
#define STM_NUM_ADCS 3
#define STM_NUM_SPIS 6
#define STM_NUM_I2CS 3
#define STM_NUM_GPIOS 11
#define STM_NUM_DMAS 2

struct STM32F4XX_STRUCT_NAME() {
    /*< protected >*/
    STM32SOC parent;
    /*< public >*/

    ARMv7MState armv7m;

    STM32F4xxSyscfgState syscfg;
    STM32F4xxExtiState exti;
    // STM32F2XXUsartState usart[STM_NUM_USARTS];
    Stm32Uart usarts[STM_NUM_USARTS];
    qemu_or_irq adc_irqs;
    STM32F4XXADCState adcs[STM_NUM_ADCS];
    STM32F4XXADCCState adc_common;
    STM32F4XXSPIState spis[STM_NUM_SPIS];
    STM32F2XXI2CState i2cs[STM_NUM_I2CS];
    stm32f2xx_gpio gpios[STM_NUM_GPIOS];

    f2xx_rtc rtc;

    Stm32f2xxRcc rcc;

    stm32f2xx_fint flash_if;

    f2xx_pwr pwr;

    STM32F2xxDmaState dmas[STM_NUM_DMAS];

    f2xx_tim timers[STM_NUM_TIMERS];

    stm32f4xx_itm itm;

    f2xx_crc crc;

    STM32F4xxUSBState usb_fs, usb_hs;

    stm32f4xx_iwdg iwdg;
    // TMC2209UsartState usart2;

    Stm32f4xx_OTP otp;

    Stm32f4xxRNGState rng;

    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;
    MemoryRegion ccmsram;
    MemoryRegion temp_usb;

};

#endif
