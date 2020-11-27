/*-
 * Copyright (c) 2013
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

#include "stm32f2xx_dma.h"
#include "qemu/log.h"

//#define DEBUG_STM32F2XX_DMA
#ifdef DEBUG_STM32F2XX_DMA

// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32F2XX_DMA: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


/* Pack ISR bits from four streams, for {L,H}ISR. */
static uint32_t
f2xx_dma_pack_isr(struct f2xx_dma *s, int start_stream)
{
    uint32_t r = 0;
    int i;

    for (i = 0; i < 4; i++) {
        r |= s->stream[i + start_stream].isr << (6 * i);
    }
    return r;
}

/* Per-stream read. */
static uint32_t
f2xx_dma_stream_read(f2xx_dma_stream *s, int stream_no, uint32_t reg)
{
    //   if (stream_no==5) DPRINTF("%s: addr: 0x%llx, steam:%d...\n", __func__, reg, stream_no);
    switch (reg) {
    case R_DMA_SxCR:
        DPRINTF("   %s: stream: %d, register CR\n", __func__, stream_no);
        return s->cr;
    case R_DMA_SxNDTR:
        DPRINTF("   %s: stream: %d, register NDTR (UNIMPLEMENTED)\n", __func__, stream_no);
        return s->ndtr;
    case R_DMA_SxPAR:
        DPRINTF("   %s: stream: %d, register PAR (UNIMPLEMENTED)\n", __func__, stream_no);
        return s->par;
    case R_DMA_SxM0AR:
        DPRINTF("   %s: stream: %d, register M0AR (UNIMPLEMENTED)\n", __func__, stream_no);
        return s->m0ar;
    case R_DMA_SxM1AR:
        DPRINTF("   %s: stream: %d, register M1AR (UNIMPLEMENTED)\n", __func__, stream_no);
        return s->m1ar;
    case R_DMA_SxFCR:
        DPRINTF("   %s: stream: %d, register FCR (UNIMPLEMENTED)\n", __func__, stream_no);
        return s->fcr;
    default:
        DPRINTF("   %s: stream: %d, register 0x%02x\n", __func__, stream_no, reg<<2);
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimp read stream reg 0x%02x\n",
          (unsigned int)reg<<2);
    }
    return 0;
}

/* Register read. */
static uint64_t
f2xx_dma_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_dma *s = arg;
    uint64_t result;

    DPRINTF("%s: addr: 0x%llx, size:%d...\n", __func__, addr, size);

    if (size != 4) {
        qemu_log_mask(LOG_UNIMP, "f2xx crc only supports 4-byte reads\n");
        return 0;
    }

    addr >>= 2;
    if (addr >= R_DMA_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f2xx dma register 0x%02x\n",
                      (unsigned int)addr << 2);
        result = 0;
    } else {
        switch(addr) {
        case R_DMA_LISR:
            DPRINTF("   %s: register LISR\n", __func__);
            result = f2xx_dma_pack_isr(s, 0);
            break;
        case R_DMA_HISR:
            DPRINTF("   %s: register HISR\n", __func__);
            result = f2xx_dma_pack_isr(s, 4);
            break;
        case R_DMA_LIFCR:
            DPRINTF("   %s: register LIFCR\n", __func__);
            result = s->ifcr[addr - R_DMA_LIFCR];
            break;
        case R_DMA_HIFCR:
            DPRINTF("   %s: register HIFCR\n", __func__);
            result = s->ifcr[addr - R_DMA_LIFCR];
            break;
        default:
            /* Only per-stream registers remain. */
            addr -= R_DMA_Sx;
            int stream_no = addr / R_DMA_Sx_REGS;
            result = f2xx_dma_stream_read(&s->stream[stream_no], stream_no,
                                          addr % R_DMA_Sx_REGS);
            break;
        }
    }

    DPRINTF("    %s: result:0x%llx\n", __func__, result);
    return result;
}

/* Start a DMA transfer for a given stream. */
static void
f2xx_dma_stream_start(f2xx_dma_stream *s, int stream_no)
{
    uint8_t buf[4];
    int msize = msize_table[(s->cr >> 13) & 0x3];
    int psize = msize_table[(s->cr >> 11) & 0x3];

    DPRINTF("%s: stream: %d\n", __func__, stream_no);

    if (msize == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma: invalid MSIZE\n");
        return;
    }
    /* XXX Skip USART, as pacing control is not yet in place. */
    if (s->par == 0x40011004 || s->par == 0x40011404) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma: skipping USART\n");
        return;
    }

    f2xx_dma_current_xfer *x = &s->active_transfer;
    uint8_t dir = (s->cr & R_DMA_SxCR_DIR) >> R_DMA_SxCR_DIR_SHIFT;
    memset(x,0,sizeof(f2xx_dma_current_xfer));
    x->peripheral = s->par;
    switch (dir & 0x3)
    {
        case 0:
        case 2:
            x->src = s->par;
            x->dest = s->m0ar;
            if (stream_no==5) printf("Dest str 5: %08x\n", x->dest);
            x->srcsize = psize;
            x->destsize = msize;
            if (s->cr & R_DMA_SxCR_PINC)
                x->srcinc = psize;
            if (s->cr & R_DMA_SxCR_MINC)
                x->destinc = msize;
            x->ndtr = s->ndtr;
            break;
        case 1:
            x->src = s->m0ar;
            x->dest = s->par;
            x->srcsize = msize;
            x->destsize = psize;
            if (s->cr & R_DMA_SxCR_PINC)
                x->destinc = psize;
            if (s->cr & R_DMA_SxCR_MINC)
                x->srcinc = msize;
            x->ndtr = s->ndtr;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma: Invalid DMA DIR value 3\n");

    }
    /* XXX hack do the entire transfer here for now. */
    // DPRINTF("%s: transferring %d x %d byte(s) from 0x%08x to 0x%08x\n", __func__, s->ndtr,
     //         msize, s->m0ar, s->par);

    // If the transfer is perhph to memory, then start teh transfer timer. 
    if (dir==0)
    {
        timer_mod(s->rx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000);
        return;
    }
   
    while (s->ndtr--) {
        cpu_physical_memory_read(x->src, buf, x->srcsize);
        cpu_physical_memory_write(x->dest, buf, x->destsize);
        x->src += x->srcinc;
        x->dest += x->destinc;

    }
    /* Transfer complete. */
    s->cr &= ~R_DMA_SxCR_EN;
    s->isr |= R_DMA_ISR_TCIF;
    if (s->cr & R_DMA_SxCR_TCIE)
        qemu_set_irq(s->irq, 1);
}

static void f2xx_dma_lookup_pfctl(hwaddr periph, hwaddr *addr, hwaddr *mask)
{
    //  TODO- abstract this for other boards.
    switch (periph)
    {
        case 0x40004404: // USART2
        case 0x40011404: // USART6
            *addr = periph - 4; // SR
            *mask = (1U <<5 ); // RxNE;
            break;
        default:
            *addr = 0;
            *mask = 0;
            printf("FIXME: Add pfctl lookup for %"HWADDR_PRIX"\n",periph);
    }
}

// Timer for handling peripheral RX streams where the peripheral is the flow controller
static void stm32f2xx_dma_rx_timer_expire(void *opaque)
{
    f2xx_dma_stream *s = opaque;
     // if (!(s->cr & R_DMA_SxCR_PFCTL))
    //      return;
    

    f2xx_dma_current_xfer *x = &s->active_transfer;
    uint8_t pfctrl[4];
    uint32_t pfctrl32;
    hwaddr pfctrlar, pfctrlmask;
    f2xx_dma_lookup_pfctl(x->peripheral,&pfctrlar, &pfctrlmask);
    if (!pfctrlar || !(s->cr & R_DMA_SxCR_EN))
    {
        return;
    }
    uint8_t buf[4];
    cpu_physical_memory_read(pfctrlar, pfctrl,4);
    memcpy(&pfctrl32, pfctrl, 4);

    // If we are active or there is data, check more often. (10 us vs 100)
    timer_mod(s->rx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ((pfctrl32 & pfctrlmask)? 10000 : 100000));

    int transferred = 0;
    while ((pfctrl32 & pfctrlmask) && s->ndtr--)
    {
      //  printf("Transferring from %08x to %08x\n",x->src, x->dest);
        cpu_physical_memory_read(x->src, buf, x->srcsize);
        cpu_physical_memory_write(x->dest, buf, x->destsize);
        x->src += x->srcinc;
        x->dest += x->destinc;
        transferred++;
        cpu_physical_memory_read(pfctrlar, pfctrl,4);
        memcpy(&pfctrl32, pfctrl, 4);
    }
    if (transferred)
    {
    // printf("Transferred %d bytes.\n",transferred);
    
        if (!(s->cr & R_DMA_SxCR_CIRC))
        {
            s->cr &= ~R_DMA_SxCR_EN;
        }
        if (s->ndtr == (x->ndtr>>1))
        {
            // printf("HTIF\n");
            s->isr |= R_DMA_ISR_HTIF;
            if (s->cr & R_DMA_SxCR_HTIE)
                qemu_set_irq(s->irq,1);
        }
        if (s->ndtr == 0)
        {
            // printf("TCIF\n");
            s->isr |= R_DMA_ISR_TCIF;
            if (s->cr & R_DMA_SxCR_TCIE)
                qemu_set_irq(s->irq, 1);
        }
        if (s->cr & R_DMA_SxCR_CIRC && s->ndtr==0) // Reset the buffer pointer if circ mode.
        {
            x->src -= (x->ndtr*x->srcinc);
            x->dest -= (x->ndtr*x->destinc);
            s->ndtr = x->ndtr;
        }
    }
    // TODO - reset the src/dest pointers for circ mode?
}

/* Per-stream register write. */
static void
f2xx_dma_stream_write(f2xx_dma_stream *s, int stream_no, uint32_t addr, uint32_t data)
{
    // if (stream_no==5) DPRINTF("DMA write stream %d, to addr %04x with data %llx\n",stream_no,addr,data);
     switch (addr) {
    case R_DMA_SxCR:
        DPRINTF("%s: stream: %d, register CR, data:0x%x\n", __func__, stream_no, data);
        if ((s->cr & R_DMA_SxCR_EN) == 0 && (data & R_DMA_SxCR_EN) != 0) {
            s->cr = data;
            if (data & R_DMA_SxCR_CIRC)
            {
                printf("Circ mode %d w/ NDTR: %d\n",stream_no, s->ndtr);
            }
            f2xx_dma_stream_start(s, stream_no);
        } else {
            s->cr = data;
        }
        break;
    case R_DMA_SxNDTR:
        DPRINTF("%s: stream: %d, register NDTR, data:0x%x\n", __func__, stream_no, data);
        if (s->cr & R_DMA_SxNDTR_EN) {
            qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma write to NDTR while enabled\n");
            return;
        }
        s->ndtr = data;
        break;
    case R_DMA_SxPAR:
        DPRINTF("%s: stream: %d, register PAR, data:0x%x\n", __func__, stream_no, data);
        s->par = data;
        break;
    case R_DMA_SxM0AR:
        DPRINTF("%s: stream: %d, register M0AR, data:0x%x\n", __func__, stream_no, data);
        s->m0ar = data;
        break;
    case R_DMA_SxM1AR:
        DPRINTF("%s: stream: %d, register M1AR, data:0x%x\n", __func__, stream_no, data);
        s->m1ar = data;
        break;
    case R_DMA_SxFCR:
        DPRINTF("%s: stream: %d, register FCR (UINIMPLEMENTED), data:0x%x\n", __func__,
                        stream_no, data);
                        // printf("SxFCR\n"); // FIFO mode is not used atm.
        qemu_log_mask(LOG_UNIMP, "f2xx dma SxFCR unimplemented\n");
        break;
    }
}

/* Register write. */
static void
f2xx_dma_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    DPRINTF("DMA write size %d, to addr %04x with data %llx\n",size,addr,data);
    f2xx_dma *s = arg;
    int offset = addr & 0x3;

    (void)offset;

    /* XXX Check DMA peripheral clock enable. */
    if (size != 4) {
        qemu_log_mask(LOG_UNIMP, "f2xx dma only supports 4-byte writes\n");
        return;
    }

    addr >>= 2;
    if (addr >= R_DMA_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f2xx dma register 0x%02x\n",
          (unsigned int)addr << 2);
        return;
    }
    if (addr >= R_DMA_Sx && addr <= 0xcc) {
        int num = (addr - R_DMA_Sx) / R_DMA_Sx_REGS;
        f2xx_dma_stream_write(&s->stream[num], num,
          (addr - R_DMA_Sx) % R_DMA_Sx_REGS, data);
        return;
    }
    switch(addr) {
    case R_DMA_LISR:
        DPRINTF("%s: register LISR (READ-ONLY), data: 0x%llx\n", __func__, data);
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma: invalid write to ISR\n");
        break;
    case R_DMA_HISR:
        DPRINTF("%s: register HISR (READ-ONLY), data: 0x%llx\n", __func__, data);
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx dma: invalid write to ISR\n");
        break;
    case R_DMA_LIFCR:
        DPRINTF("%s: register LIFCR, data: 0x%llx\n", __func__, data);
        // Any interrupt clear write to stream x clears all interrupts for that stream
        s->ifcr[addr - R_DMA_LIFCR] &= ~(data);
        if (data & 0x0f400000) {
            s->stream[3].isr = 0;
            qemu_set_irq(s->stream[3].irq, 0);
        }
        if (data & 0x003d0000) {
            s->stream[2].isr = 0;
            qemu_set_irq(s->stream[2].irq, 0);
        }
        if (data & 0x00000f40) {
            s->stream[1].isr = 0;
            qemu_set_irq(s->stream[1].irq, 0);
        }
        if (data & 0x0000003d) {
            s->stream[0].isr = 0;
            qemu_set_irq(s->stream[0].irq, 0);
        }
        break;
    case R_DMA_HIFCR:
        DPRINTF("%s: register HIFCR, data: 0x%llx\n", __func__, data);
        // Any interrupt clear write to stream x clears all interrupts for that stream
        s->ifcr[addr - R_DMA_LIFCR] &= ~(data);
        if (data & 0x0f400000) {
            s->stream[7].isr = 0;
            qemu_set_irq(s->stream[7].irq, 0);
        }
        if (data & 0x003d0000) {
            s->stream[6].isr = 0;
            qemu_set_irq(s->stream[6].irq, 0);
        }
        if (data & 0x00000f40) {
            s->stream[5].isr = 0;
            qemu_set_irq(s->stream[5].irq, 0);
        }
        if (data & 0x0000003d) {
            s->stream[4].isr = 0;
            qemu_set_irq(s->stream[4].irq, 0);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx dma unimpl write reg 0x%02x\n",
          (unsigned int)addr << 2);
    }
}


static const MemoryRegionOps f2xx_dma_ops = {
    .read = f2xx_dma_read,
    .write = f2xx_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void
f2xx_dma_init(Object *obj)
{
    f2xx_dma *s = STM32F2XX_DMA(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &f2xx_dma_ops, s, "dma", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    for (i = 0; i < R_DMA_Sx_COUNT; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->stream[i].irq);
        s->stream[i].rx_timer =
            timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32f2xx_dma_rx_timer_expire, &s->stream[i]);

    }

 
}

static void
f2xx_dma_reset(DeviceState *ds)
{
    f2xx_dma *s = STM32F2XX_DMA(ds);

    memset(&s->ifcr, 0, sizeof(s->ifcr));

    int i;
    for (i=0; i<R_DMA_Sx_COUNT; i++) {
        qemu_irq save = s->stream[i].irq;
        QEMUTimer *timer = s->stream[i].rx_timer;
        memset(&s->stream[i], 0, sizeof(f2xx_dma_stream));
        s->stream[i].irq = save;
        s->stream[i].rx_timer = timer;
    }
}

// static Property f2xx_dma_properties[] = {
//     DEFINE_PROP_END_OF_LIST(),
// };

static void
f2xx_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = f2xx_dma_reset;
    //TODO: fix this: dc->no_user = 1;
  //  dc->props = f2xx_dma_properties;
}

static const TypeInfo
f2xx_dma_info = {
    .name          = TYPE_STM32F2XX_DMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_dma),
    .instance_init = f2xx_dma_init,
    .class_init    = f2xx_dma_class_init,
};

static void
f2xx_dma_register_types(void)
{
    type_register_static(&f2xx_dma_info);
}

type_init(f2xx_dma_register_types)
