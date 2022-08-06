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

#include "qemu/units.h"
#include "../stm32_common/stm32_common.h"
#include "hw/arm/armv7m.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XX_STRUCT_NAME(), STM32F4XX_BASE)

#define STM_NUM_USARTS 8
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


    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;
    MemoryRegion ccmsram;
    MemoryRegion temp_usb;

};

#endif
