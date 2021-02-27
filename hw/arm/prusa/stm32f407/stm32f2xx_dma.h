/*-
 * Copyright (c) 2013 <https://github.com/pebble/qemu>
 * Adapted for Mini404 in 2020 VintagePC <github.com/vinagepc>
 * (Significant portions rewritten and updated)
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
 * QEMU DMA controller device model
 */

#ifndef STM32F2XX_DMA_H
#define STM32F2XX_DMA_H

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"

/* Common interrupt status / clear registers. */
#define R_DMA_LISR           (0x00 / 4)
#define R_DMA_HISR           (0x04 / 4)//r
#define R_DMA_LIFCR          (0x08 / 4)
#define R_DMA_HIFCR          (0x0c / 4)//w
#define R_DMA_ISR_FIEF     (1 << 0)
#define R_DMA_ISR_DMEIF    (1 << 2)
#define R_DMA_ISR_TIEF     (1 << 3)
#define R_DMA_ISR_HTIF     (1 << 4)
#define R_DMA_ISR_TCIF     (1 << 5)

/* Per-stream registers. */
#define R_DMA_Sx             (0x10 / 4)
#define R_DMA_Sx_COUNT           8
#define R_DMA_Sx_REGS            6
#define R_DMA_SxCR           (0x00 / 4)
#define R_DMA_SxCR_EN   0x00000001
#define R_DMA_SxCR_HTIE      (1<<3)
#define R_DMA_SxCR_TCIE      (1<<4)
#define R_DMA_SxCR_PFCTL      (1<<5)
#define R_DMA_SxCR_DIR_SHIFT (6)
#define R_DMA_SxCR_DIR       (3<<R_DMA_SxCR_DIR_SHIFT)
#define R_DMA_SxCR_CIRC      (1<<8)
#define R_DMA_SxCR_PINC      (1<<9)
#define R_DMA_SxCR_MINC      (1<<10)

#define R_DMA_SxNDTR         (0x04 / 4)
#define R_DMA_SxNDTR_EN 0x00000001
#define R_DMA_SxPAR          (0x08 / 4)
#define R_DMA_SxM0AR         (0x0c / 4)
#define R_DMA_SxM1AR         (0x10 / 4)
#define R_DMA_SxFCR          (0x14 / 4)

#define R_DMA_MAX            (0xd0 / 4)
 

// Stores the active transfer, absracting the direction.
typedef struct f2xx_dma_current_xfer {
    uint8_t srcsize;
    uint8_t srcinc;
    uint8_t destinc;
    uint8_t destsize;
    uint32_t src; 
    uint32_t dest;
    uint16_t ndtr;
    uint32_t peripheral;
 } f2xx_dma_current_xfer;

typedef struct f2xx_dma_stream {
    qemu_irq irq;
    uint8_t id;
    uint32_t cr;
    uint16_t ndtr;
    uint32_t par;
    uint32_t m0ar;
    uint32_t m1ar;
    uint8_t isr;
    uint8_t fcr;

    // uint16_t ndtr_circ;

    // // Address of the register that indicates if the peripheral has data.
    // uint32_t pfctrlar; 
    // // Mask for the perhipheral "data ready" flag, e.g. RxNE flag
    // uint32_t pfctrlmask;

    f2xx_dma_current_xfer active_transfer;

    //store info for streams attached to USART DMAR.
    int usart_dmar;

    struct QEMUTimer *rx_timer;

} f2xx_dma_stream;


#define TYPE_STM32F2XX_DMA "stm32f2xx-dma"
OBJECT_DECLARE_SIMPLE_TYPE(f2xx_dma, STM32F2XX_DMA)

typedef struct f2xx_dma {
    SysBusDevice busdev;
    MemoryRegion iomem;

    int id;
    uint32_t ifcr[R_DMA_HIFCR - R_DMA_LIFCR + 1];
    f2xx_dma_stream stream[R_DMA_Sx_COUNT]; 



} f2xx_dma;

#endif // STM32F2XX_DMA
