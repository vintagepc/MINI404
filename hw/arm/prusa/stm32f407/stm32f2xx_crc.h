/*-
 * QEMU crc emulation
 * Portions Copyright (c) 2013 https://github.com/pebble/qemu/
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

#ifndef STM32F2XX_CRC_H
#define STM32F2XX_CRC_H

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "../stm32_common/stm32_common.h"


#define TYPE_STM32F2XX_CRC "stm32f2xx-crc"
OBJECT_DECLARE_SIMPLE_TYPE(f2xx_crc,STM32F2XX_CRC);

typedef struct f2xx_crc {
    STM32Peripheral parent;
    MemoryRegion iomem;

    uint32_t crc;
    uint8_t idr;

} f2xx_crc;

#endif // STM32F2XX_CRC_H
