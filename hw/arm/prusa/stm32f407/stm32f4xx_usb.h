/*
 * (STM32F4xx) USB host controller state definitions
 *
 * Based on hw/usb/hcd-dwc2.h
 *
 * Original Copyright (c) 2020 Paul Zimmerman <pauldzim@gmail.com>
 * Adapted for STM32F4xx by VintagePC <http://github.com/vintagepc>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef HW_USB_STM32F4xx_H
#define HW_USB_STM32F4xx_H

#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/usb.h"
#include "sysemu/dma.h"
#include "qemu/units.h"
#include "qom/object.h"

#define STM32F4xx_MMIO_SIZE      0xCFFF // CSRs and FIFOs

#define STM32F4xx_NB_CHAN        16       /* Number of host channels */
// N.B - device has max 12, but the LL HAL code resets up to 16 (likely for other device compat)
#define STM32F4xx_MAX_XFER_SIZE  65536   /* Max transfer size expected in HCTSIZ */

typedef struct STM32F4xxPacket STM32F4xxPacket;
typedef struct STM32F4xxUSBState STM32F4xxUSBState;
typedef struct STM32F4xxClass STM32F4xxClass;

enum async_state {
    STM32F4xx_ASYNC_NONE = 0,
    STM32F4xx_ASYNC_INITIALIZED,
    STM32F4xx_ASYNC_INFLIGHT,
    STM32F4xx_ASYNC_FINISHED,
};

struct STM32F4xxPacket {
    USBPacket packet;
    uint32_t devadr;
    uint32_t epnum;
    uint32_t epdir;
    uint32_t mps;
    uint32_t pid;
    uint32_t index;
    uint32_t pcnt;
    uint32_t len;
    int32_t async;
    bool small;
    bool needs_service;
};
typedef union {
    uint32_t raw;
    struct {
        uint32_t chnum      :4;
        uint32_t bcnt       :11;
        uint32_t dpid       :2;
        uint32_t pktsts     :4;
        uint32_t :11; // unused/reserved
    } QEMU_PACKED;
} rxstatus_t;

struct STM32F4xxUSBState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    USBBus bus;
    qemu_irq irq;
    MemoryRegion dma_mr;
    AddressSpace dma_as;
    MemoryRegion container;
    MemoryRegion hsotg;
    MemoryRegion fifos;

    union {
#define STM32F4xx_GLBREG_SIZE    0x70
        uint32_t glbreg[STM32F4xx_GLBREG_SIZE / sizeof(uint32_t)];
        struct {
            uint32_t gotgctl;       /* 00 */
            uint32_t gotgint;       /* 04 */
            uint32_t gahbcfg;       /* 08 */
            uint32_t gusbcfg;       /* 0c */
            uint32_t grstctl;       /* 10 */
            uint32_t gintsts;       /* 14 */
            uint32_t gintmsk;       /* 18 */
            union {
                uint32_t grxstsr;       /* 1c */
                rxstatus_t defs;
            };
            uint32_t grxstsp;       /* 20 */
            uint32_t grxfsiz;       /* 24 */
            union {
                uint32_t gnptxfsiz;     /* 28 */
                struct {
                    uint32_t gnptxfsiz_txfsa  :16;
                    uint32_t gnptxfsiz_txfd   :16;
                } QEMU_PACKED;
            };
            uint32_t gnptxsts;      /* 2c */
            uint32_t gi2cctl;       /* 30 */
            uint32_t gpvndctl;      /* 34 */
            uint32_t ggpio;         /* 38 */
            uint32_t guid;          /* 3c */
            uint32_t gsnpsid;       /* 40 */
            uint32_t ghwcfg1;       /* 44 */
            uint32_t ghwcfg2;       /* 48 */
            uint32_t ghwcfg3;       /* 4c */
            uint32_t ghwcfg4;       /* 50 */
            uint32_t glpmcfg;       /* 54 */
            uint32_t gpwrdn;        /* 58 */
            uint32_t gdfifocfg;     /* 5c */
            uint32_t gadpctl;       /* 60 */
            uint32_t grefclk;       /* 64 */
            uint32_t gintmsk2;      /* 68 */
            uint32_t gintsts2;      /* 6c */
        };
    };

    union {
#define STM32F4xx_FSZREG_SIZE    0x04
        uint32_t fszreg[STM32F4xx_FSZREG_SIZE / sizeof(uint32_t)];
        struct {
            uint32_t hptxfsiz;      /* 100 */
        };
    };

    union {
#define STM32F4xx_HREG0_SIZE     0x44
        uint32_t hreg0[STM32F4xx_HREG0_SIZE / sizeof(uint32_t)];
        struct {
            uint32_t hcfg;          /* 400 */
            uint32_t hfir;          /* 404 */
            uint32_t hfnum;         /* 408 */
            uint32_t rsvd0;         /* 40c */
            uint32_t hptxsts;       /* 410 */
            uint32_t haint;         /* 414 */
            uint32_t haintmsk;      /* 418 */
            uint32_t hflbaddr;      /* 41c */
            uint32_t rsvd1[8];      /* 420-43c */
            uint32_t hprt0;         /* 440 */
        };
    };

#define STM32F4xx_HREG1_SIZE     (0x20 * STM32F4xx_NB_CHAN)
    uint32_t hreg1[STM32F4xx_HREG1_SIZE / sizeof(uint32_t)];

#define hcchar(_ch)     hreg1[((_ch) << 3) + 0] /* 500, 520, ... */
#define hcsplt(_ch)     hreg1[((_ch) << 3) + 1] /* 504, 524, ... */
#define hcint(_ch)      hreg1[((_ch) << 3) + 2] /* 508, 528, ... */
#define hcintmsk(_ch)   hreg1[((_ch) << 3) + 3] /* 50c, 52c, ... */
#define hctsiz(_ch)     hreg1[((_ch) << 3) + 4] /* 510, 530, ... */
#define hcdma(_ch)      hreg1[((_ch) << 3) + 5] /* 514, 534, ... */
#define hcdmab(_ch)     hreg1[((_ch) << 3) + 7] /* 51c, 53c, ... */

    union {
#define STM32F4xx_PCGREG_SIZE    0x08
        uint32_t pcgreg[STM32F4xx_PCGREG_SIZE / sizeof(uint32_t)];
        struct {
            uint32_t pcgctl;        /* e00 */
            uint32_t pcgcctl1;      /* e04 */
        };
    };

    /* TODO - implement FIFO registers for slave mode */
#define STM32F4xx_HFIFO_SIZE     (0x1000 * STM32F4xx_NB_CHAN)

    /*
     *  Internal state
     */
    QEMUTimer *eof_timer;
    QEMUTimer *frame_timer;
    QEMUBH *async_bh;
    int64_t sof_time;
    int64_t usb_frame_time;
    int64_t usb_bit_time;
    uint32_t usb_version;
    uint16_t frame_number;
    uint16_t fi;
    uint16_t next_chan;
    bool working;
    USBPort uport;
    STM32F4xxPacket packet[STM32F4xx_NB_CHAN];                   /* one packet per chan */
    uint8_t usb_buf[STM32F4xx_NB_CHAN][STM32F4xx_MAX_XFER_SIZE]; /* one buffer per chan */
    uint32_t fifo_ram[(128*KiB)/sizeof(uint32_t)]; // 128K FIFO ram (0x20000- 0x3FFFF)
    // TODO - relocate these and use the correct registers instead. 
    uint16_t fifo_head[STM32F4xx_NB_CHAN];
    uint16_t fifo_level[STM32F4xx_NB_CHAN];
    uint16_t fifo_tail[STM32F4xx_NB_CHAN];
    uint32_t rx_fifo_head;
    uint32_t rx_fifo_tail;
    uint32_t rx_fifo_level;
    uint8_t is_ping;

};

struct STM32F4xxClass {
    /*< private >*/
    SysBusDeviceClass parent_class;
    ResettablePhases parent_phases;

    /*< public >*/
};

#define TYPE_STM32F4xx_USB   "STM32F4xx-usb"
OBJECT_DECLARE_TYPE(STM32F4xxUSBState, STM32F4xxClass, STM32F4xx_USB)

#endif
