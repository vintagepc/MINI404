/*
 * (STM32F4xx) USB host controller state definitions
 *
 * Based on hw/usb/hcd-dwc2.h
 *
 * Original Copyright (c) 2020 Paul Zimmerman <pauldzim@gmail.com>
 * Adapted for STM32F4xx in 2021 by VintagePC <http://github.com/vintagepc>
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
#include "chardev/char-fe.h"
#include "chardev/char.h"
#include "hw/sysbus.h"
#include "hw/usb.h"
#include "sysemu/dma.h"
#include "qemu/units.h"
#include "qom/object.h"
#include "hw/qdev-properties-system.h"
#include "../utility/macros.h"

#define STM32F4xx_MMIO_SIZE      0xCFFF // CSRs and FIFOs

#define STM32F4xx_NB_CHAN        16       /* Number of host channels */
// N.B - device has max 12, but the LL HAL code resets up to 16 (likely for other device compat)
#define STM32F4xx_NB_DEVCHAN        4       /* Number of device-mode channels */
#define STM32F4xx_MAX_XFER_SIZE  65536   /* Max transfer size expected in HCTSIZ */

#define STM32F4xx_EP_FIFO_SIZE 4*KiB / sizeof(uint32_t)
#define STM32F4xx_RX_FIFO_SIZE (128*KiB)/sizeof(uint32_t)

typedef struct STM32F4xxPacket STM32F4xxPacket;
typedef struct STM32F4xxUSBState STM32F4xxUSBState;
typedef struct STM32F4xxClass STM32F4xxClass;

#define R_HREG_MAX (0x20/4)

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

typedef struct {
    uint32_t XFRSIZ :19;
    uint32_t PKTCNT :10;
    uint32_t DPID :2;
    uint32_t DOPING :1;
} QEMU_PACKED hctsiz_t;

typedef struct {
    uint32_t MPSIZ :11;
    uint32_t EPNUM :4;
    uint32_t EPDIR :1;
    uint32_t :1;
    uint32_t LSDEV :1;
    uint32_t EPTYP :2;
    uint32_t MCNT :2;
    uint32_t DAD :7;
    uint32_t ODDFRM :1;
    uint32_t CHDIS :1;
    uint32_t CHENA :1;
} QEMU_PACKED hcchar_t;

typedef struct {
    uint32_t MPSIZ :11;
    uint32_t _reserved :4;
    REG_B32(USBAEP);
    REG_B32(EONUMDPID);
    REG_B32(NAKSTS);
    uint32_t EPTYP :2;
    REG_B32(DO_SNPM);
    REG_B32(STALL);
    uint32_t DI_TXFNUM :4;
    REG_B32(CNAK);
    REG_B32(SNAK);
    REG_B32(SD0PID);
    REG_B32(SODDFRM);
    REG_B32(EPDIS);
    REG_B32(EPENA);
} QEMU_PACKED depctl_t; 

typedef struct {
    REG_B32(XFRC);
    REG_B32(EPDISD);
    REG_B32(_reserved);
    REG_B32(TOC);
    REG_B32(ITTXFE);
    REG_B32(INEPNM);
    REG_B32(INEPNE);
    REG_B32(TXFE);
    uint32_t _reserved2 :3;
    REG_B32(PKTDRPSTS);
    REG_B32(_reserved3);
    REG_B32(NAK);
    uint32_t :18;
} QEMU_PACKED diepint_t;

typedef struct {
    REG_B32(XFRC);
    REG_B32(EPDISD);
    REG_B32(_reserved);
    REG_B32(STUP);
    REG_B32(OTEPDIS);
    REG_B32(STSPHSRX);
    uint32_t _reserved2 :2;
    REG_B32(OUTPKTERR);
    uint32_t _reserved3 :3;
    REG_B32(BERR);
    REG_B32(NAK);
    uint32_t :18;
} QEMU_PACKED doepint_t;

typedef struct {
    uint32_t XFRSIZ :19;
    uint32_t PKTCNT :10;
    uint32_t DO_RXPID_STUPCNT :2;
    uint32_t :1;
} QEMU_PACKED deptsiz_t;

typedef union {
    struct {
        depctl_t DIEPCTL; // 0x900
        uint32_t _unused;  // 0x904
        doepint_t DIEPINT; // 0x908
        uint32_t _unused2; // 0x90C
        deptsiz_t DIEPTSIZ; // 0x910
        uint32_t _unused3;// 0x914
        REG_S32(INEPTSFSAV,16); // 0x918
        uint32_t _unused4;// 0x91c
    };
    uint32_t raw[8];
} QEMU_PACKED dreg_di_set_t;

typedef union {
    struct {
        depctl_t DOEPCTL; // 0xB00
        uint32_t _unused;  // 0xB04
        doepint_t DOEPINT; // 0xB08
        uint32_t _unused2;
        deptsiz_t DIEPTSIZ; // 0xB10
        uint32_t _unused4[3];// 0xB14 - 0xB1C
    }; 
    uint32_t raw[8];
} QEMU_PACKED dreg_do_set_t;

typedef union {
    uint32_t raw[R_HREG_MAX];
    struct {
        hcchar_t HCCHAR; // 0x500
        struct {
            uint32_t PRTADDR :7;
            uint32_t HUBADDR :7;
            uint32_t XACTPOS :2;
            REG_B32(COMPLSPLT);
            uint32_t :14;
            REG_B32(SPLTEN);
        } QEMU_PACKED HCSPLT; // 0x504
        struct {
            REG_B32(XFRC);
            REG_B32(CHH);
            REG_B32(_reserved);
            REG_B32(STALL);
            REG_B32(NAK);
            REG_B32(ACK);
            REG_B32(_reserved2);
            REG_B32(TXERR);
            REG_B32(BBERR);
            REG_B32(FRMOR);
            REG_B32(DTERR);
            uint32_t :21;
        } QEMU_PACKED HCINT; // 0x508
        struct {
            REG_B32(XFRCM);
            REG_B32(CHHM);
            REG_B32(_reserved);
            REG_B32(STALLM);
            REG_B32(NAKM);
            REG_B32(ACKM);
            REG_B32(_reserved2);
            REG_B32(TXERRM);
            REG_B32(BBERRM);
            REG_B32(FRMORM);
            REG_B32(DTERRM);
            uint32_t :21;
        } QEMU_PACKED HCINTMSK; // 0x50C
        hctsiz_t HCTSIZ;    // 0x510
        uint32_t HCDMA;     // 0x514
        uint32_t _unused; // 0x518 Padding (HCDMAB on DWC2)
        uint32_t _unused2; // 0x51C padding
    }defs;
} QEMU_PACKED hreg_set_t;

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
            union {
                uint32_t gintsts;       /* 14 */
                struct {
                    REG_B32(CMOD);
                    REG_B32(MMIS);
                    REG_B32(OTGINT);
                    REG_B32(SOF);
                    REG_B32(RXFLVL);
                    REG_B32(NPTXFE);
                    REG_B32(GINAKEFF);
                    REG_B32(GONAKEFF);
                    uint32_t _reserved :2;
                    REG_B32(ESUSP);
                    REG_B32(USBSUSP);
                    REG_B32(USBRST);
                    REG_B32(ENUMDNE);
                    REG_B32(ISOODRP);
                    REG_B32(EOPF);
                    uint32_t _reserved2 :2;
                    REG_B32(IEPINT);
                    REG_B32(OEPINT);
                    REG_B32(ISOIXFR);
                    REG_B32(IPXFR);
                    uint32_t _reserved3 :2;
                    REG_B32(HPRTINT);
                    REG_B32(HCINT);
                    REG_B32(PTXFE);
                    REG_B32(_reserved4);
                    REG_B32(CIDSCHG);
                    REG_B32(DISCINT);
                    REG_B32(SRQINT);
                    REG_B32(WKUINT);
                } QEMU_PACKED GINTSTS;
            };
            uint32_t gintmsk;       /* 18 */   
            union {
                uint32_t grxstsr;       /* 1c */
                rxstatus_t defs;
            } QEMU_PACKED ;
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
            union {
                struct {
                    REG_B32(PCSTS);
                    REG_B32(PCDET);
                    REG_B32(PENA);
                    REG_B32(PENCHNG);
                    REG_B32(POCA);
                    REG_B32(POCCHNG);
                    REG_B32(PRES);
                    REG_B32(PSUSP);
                    REG_B32(PRST);
                    REG_B32(_reserved);
                    uint32_t PLSTS :2;
                    REG_B32(PPWR);
                    uint32_t PTCTL :4;
                    uint32_t PSPD :2;
                    uint32_t :13;
                } QEMU_PACKED HPRT;
                uint32_t hprt0;         /* 440 */
            };
        };
    };

    union {
        uint32_t hreg1[STM32F4xx_NB_CHAN*R_HREG_MAX];
        hreg_set_t hreg_chan[STM32F4xx_NB_CHAN];
    };

#define STM32F4xx_HREG1_SIZE     (0x20 * STM32F4xx_NB_CHAN)
    // uint32_t hreg1[STM32F4xx_HREG1_SIZE / sizeof(uint32_t)];

    union {
#define STM32F4xx_PCGREG_SIZE    0x08
        uint32_t pcgreg[STM32F4xx_PCGREG_SIZE / sizeof(uint32_t)];
        struct {
            uint32_t pcgctl;        /* e00 */
            uint32_t pcgcctl1;      /* e04 */
        };
    };

// Device mode register definitions.
    union {
        uint32_t dreg0[0x84];
        struct {
            struct {
                uint32_t DSPD :2;
                REG_B32(NZLSOHSK);
                REG_B32(_reserved);
                uint32_t DAD :7;
                uint32_t PFLVL :2;
                uint32_t _reserved2 :11;
                uint32_t PERSCHVL :2;
                uint32_t :6;
            } QEMU_PACKED DCFG; // 0x800
            struct {
                REG_B32(RWUSIG);
                REG_B32(SDIS);
                REG_B32(GINSTS);
                REG_B32(GONSTS);
                uint32_t TCTL :3;
                REG_B32(SGINAK);
                REG_B32(CGINAK);
                REG_B32(SGONAK);
                REG_B32(CGONAK);
                REG_B32(POPRGDNE);
                uint32_t :20;
            } QEMU_PACKED DCTL; // 0x804
            struct {
                REG_B32(SUSPSTS);
                uint32_t ENUMSPD :2;
                REG_B32(EERR);
                uint32_t _unused :4;
                uint32_t FNSOF :14;
                uint32_t :10;
            } QEMU_PACKED DSTS; // 0x808
            uint32_t _unusedl; // 0x80C
            struct {
                REG_B32(XFRCM);
                REG_B32(EPDM);
                REG_B32(AHBERRM);
                REG_B32(TOM);
                REG_B32(ITTXFEMSK);
                REG_B32(INEPNMM);
                REG_B32(INEPNEM);
                REG_B32(_reserved);
                REG_B32(TXFURM);
                uint32_t _reserved2 :4;
                REG_B32(NAKM);
                uint32_t :18;
            } QEMU_PACKED DIEPMSK; // 0x810
            struct {
                REG_B32(XFRCM);
                REG_B32(EPDM);
                REG_B32(AHBERRM);
                REG_B32(STUPM);
                REG_B32(OTEPDM);
                REG_B32(STSPHSRXM);
                REG_B32(B2BSTIP);
                REG_B32(_reserved);
                REG_B32(OPEM);
                uint32_t _reserved2 :3;
                REG_B32(BERRM);
                REG_B32(NAKMSK);
                REG_B32(NYETMSK);
                uint32_t :17;
            } QEMU_PACKED DOEPMSK; // 0x814
            struct {
                uint32_t IEPINT :16;
                uint32_t OEPINT :16;
            } QEMU_PACKED DAINT; // 0x818
            struct {
                uint32_t IEPM :16;
                uint32_t OEPM :16;
            } QEMU_PACKED DAINTMSK; // 0x81C
            uint32_t _unused2[2]; // 0x820-4
            REG_S32(VBUSDT,16); // 0x828
            REG_S32(DVBUSP, 12); // 0x82C
            struct {
                REG_B32(NONISOTHREN);
                REG_B32(ISOTHREN);
                uint32_t TXTHRLEN :9;
                uint32_t _reserved :5;
                REG_B32(RXTHREN);
                uint32_t RXTHRLEN :9;
                REG_B32(_reserved2);
                REG_B32(ARPEN);
                uint32_t :4;
            } QEMU_PACKED DTHRCTRL; // 0x830;
            REG_S32(INEPTXFEM, 16); // 0x834;
            struct {
                REG_B32(_reserved);
                REG_B32(IEP1INT);
                uint32_t _reserved2 :15;
                REG_B32(OEP1INT);
                uint32_t :14;
            } QEMU_PACKED DEACHINT; // 0x838
            struct {
                REG_B32(_reserved);
                REG_B32(IEP1INTM);
                uint32_t _reserved2 :15;
                REG_B32(OEP1INTM);
                uint32_t :14;
            } QEMU_PACKED DEACHINTMSK; // 0x83C
            uint32_t _unused3; // 0x840
            struct {
                REG_B32(XFRCM);
                REG_B32(EPDM);
                REG_B32(AHBERRM);
                REG_B32(TOM);
                REG_B32(ITTXFEMSK);
                REG_B32(INEPNMM);
                REG_B32(INEPNEM);
                REG_B32(_reserved);
                REG_B32(TXFURM);
                REG_B32(BIM);
                uint32_t _reserved2 :3;
                REG_B32(NAKM);
                uint32_t :18;
            } QEMU_PACKED DIEPEACHMSK1; // 0x844
            uint32_t _reserved[0x20/4];
            struct {
                REG_B32(XFRCM);
                REG_B32(EPDM);
                REG_B32(AHBERRM);
                REG_B32(TOM);
                 REG_B32(ITTXFEMSK);
                REG_B32(INEPNMM);
                REG_B32(INEPNEM);
                REG_B32(_reserved);
                REG_B32(TXFURM);
                REG_B32(BIM);
                uint32_t _reserved2 :3;
                REG_B32(BERRM);
                REG_B32(NAKMSK);
                REG_B32(NYETM);
                uint32_t :17;
            } QEMU_PACKED DOEPEACHMSK1; // 0x884
        } QEMU_PACKED dreg_defs;
    };

    dreg_di_set_t dregi[STM32F4xx_NB_DEVCHAN];
    dreg_do_set_t drego[STM32F4xx_NB_DEVCHAN];

    /* TODO - implement FIFO registers for slave mode */
#define STM32F4xx_HFIFO_SIZE     (0x1000 * STM32F4xx_NB_CHAN)

    /*
     *  Internal state
     */
    QEMUTimer *eof_timer;
    QEMUTimer *frame_timer;
    QEMUTimer *cdc_timer;
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


    // TODO- rework this into blocks on a per-channel basis
    // so the math is easier on my head...
    uint32_t fifo_ram[STM32F4xx_RX_FIFO_SIZE]; // 128K FIFO ram (0x20000- 0x3FFFF)
    // TODO - relocate these and use the correct registers instead.
    // #channels *4KiB of space.
    uint32_t tx_fifos[STM32F4xx_NB_CHAN][STM32F4xx_EP_FIFO_SIZE];

    uint16_t fifo_head[STM32F4xx_NB_CHAN];
    uint16_t fifo_level[STM32F4xx_NB_CHAN];
    uint16_t fifo_tail[STM32F4xx_NB_CHAN];
    uint32_t rx_fifo_head;
    uint32_t rx_fifo_tail;
    uint32_t rx_fifo_level;
    uint8_t is_ping;

    uint8_t device_state;

    pthread_t usbip_thread;

    bool debug;

    char cdc_in;

    CharBackend cdc;

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
