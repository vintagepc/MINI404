/*
 * OTG (STM32F4xx) USB host controller emulation
 *
 * Original Copyright (c) 2020 Paul Zimmerman <pauldzim@gmail.com>
 * Based on hw/usb/hcd-dwc2.c as shipped with QEMU
 * Adapted for Mini404/STM32F4xx 2021 by VintagePC <http://github.com/vintagepc>
 *
 * Some useful documentation used to develop this emulation can be
 * found online (as of April 2020) at:
 *
 * http://www.capital-micro.com/PDF/CME-M7_Family_User_Guide_EN.pdf
 * which has a pretty complete description of the controller starting
 * on page 370.
 *
 * https://sourceforge.net/p/wive-ng/wive-ng-mt/ci/master/tree/docs/DataSheets/RT3050_5x_V2.0_081408_0902.pdf
 * which has a description of the controller registers starting on
 * page 130.
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

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "stm32f4xx_usb.h"
#include "hw/usb/dwc2-regs.h"

#undef DCTL
#undef DCFG
#undef DAINT
#undef DAINTMSK
#undef GINTSTS

#include "migration/vmstate.h"
#include "trace.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "hw/qdev-properties.h"
#include "usbip.h"

#define USB_HZ_FS       12000000
#define USB_HZ_HS       96000000 // was 96, this might be wrong but STM clocks usb at 48 mhz
#define USB_FRMINTVL    12000

#define R_GINTSTS  (0x014/4)

#define R_HCCHAR    0x00/4
#define R_HCSPLT    0x04/4
#define R_HCINT     0x08/4
#define R_HCINTMSK  0x0C/4 
#define R_HCTSIZ    0x10/4
#define R_HCDMA     0x14/4

#define R_DCFG          (0x800/4)
#define R_DCTL          (0x804/4)
#define R_DIEPMSK       (0x810/4)
#define R_DOEPMSK       (0x814/4)
#define R_DAINT         (0x818/4)
#define R_DAINTMSK      (0x81C/4)
#define R_DIEPEMPMSK    (0x834/4)

#define R_OFF_DEPCTL 0
#define R_OFF_DEPINT 2
#define R_OFF_DTXFSTS 6

#define R_PCGCTL (0xE00/4)
#define R_PCGCCTL1 (0xE04/4)

/* USBIP handling */ 
#define BSIZE 64 
char buffer[BSIZE+1];
int  bsize=0;

#define DEV_ADDR  4U
static const uint32_t DEV_SETADDR[] = {0x000C0040,  (0x00000500 | (DEV_ADDR<<16)), 0x00000000};
static const uint32_t DEV_GETDESC[] = {0x000C0040, 0x03020680, 0x00FF0000};
static const uint32_t DEV_GETDESC2[] = {0x000C0040, 0x03010680, 0x00FF0000};
static const uint32_t DEV_GETDESC3[] = {0x000C0040, 0x03030680, 0x00FF0000};
static const uint32_t DEV_SETCONF[] = {0x000C0040, 0x00010900, 0x00000000};
// static const uint32_t DEV_SETCODING[] = {0x000C0040, 0x00002021, 0x00070000};
// static const uint32_t DEV_SETCODING2[] = {0x00040070, 0x00002500, 0x00080000};
static const uint32_t DEV_SETCTLLINE[] = {0x000C0040, 0x00032221, 0x00000000};
static const uint32_t DEV_OUT_HEADER = 0x40011;
static const uint32_t DEV_OUT_AT[] = {0x00040021, 0x00000a0d};
//void handle_data(int sockfd, USBIP_RET_SUBMIT *usb_req, int bl)

#define DEVSTATE(name) DEV_ST_##name, DEV_ST_##name##_WAIT,


enum {
    DEVSTATE(RESET)
    DEVSTATE(SETUP)
    DEVSTATE(DESCR)
    DEVSTATE(DESCR2)
    DEVSTATE(DESCR3)
    DEVSTATE(SETCFG)
    DEVSTATE(LINECODING)
    DEVSTATE(LINECODING_DATA)
    DEVSTATE(SETCTLLINE)
    DEV_ST_IO_READY,
    DEVSTATE(IO)
};


/* update irq line */
static inline void STM32F4xx_update_irq(STM32F4xxUSBState *s)
{
    static int oldlevel;
    int level = 0;

    if ((s->gintsts & s->gintmsk) && (s->gahbcfg & GAHBCFG_GLBL_INTR_EN)) {
        level = 1;
    }
    if (level != oldlevel) {
        oldlevel = level;
        // trace_usb_stm_update_irq(level);
        qemu_set_irq(s->irq, level);
    }
}

/* flag interrupt condition */
static inline void STM32F4xx_raise_global_irq(STM32F4xxUSBState *s, uint32_t intr)
{
    if (!(s->gintsts & intr)) {
        s->gintsts |= intr;
        // trace_usb_stm_raise_global_irq(intr);
        STM32F4xx_update_irq(s);

    }
}

static inline void STM32F4xx_lower_global_irq(STM32F4xxUSBState *s, uint32_t intr)
{
    if (s->gintsts & intr) {
        s->gintsts &= ~intr;
        // trace_usb_stm_lower_global_irq(intr);
        STM32F4xx_update_irq(s);
    }
}

static inline void STM32F4xx_raise_host_irq(STM32F4xxUSBState *s, uint32_t host_intr)
{
    if (!(s->haint & host_intr)) {
        s->haint |= host_intr;
        s->haint &= 0xffff;
        // trace_usb_stm_raise_host_irq(host_intr);
        if (s->haint & s->haintmsk) {
            STM32F4xx_raise_global_irq(s, GINTSTS_HCHINT);
        }
    }
}

static inline void STM32F4xx_lower_host_irq(STM32F4xxUSBState *s, uint32_t host_intr)
{
    if (s->haint & host_intr) {
        s->haint &= ~host_intr;
        // trace_usb_stm_lower_host_irq(host_intr);
        if (!(s->haint & s->haintmsk)) {
            STM32F4xx_lower_global_irq(s, GINTSTS_HCHINT);
        }
    }
}


static inline void STM32F4xx_update_hc_irq(STM32F4xxUSBState *s, int index)
{
    uint32_t host_intr = 1 << (index >> 3);
    uint32_t chan = index>>3;

    if (s->hreg_chan[chan].raw[R_HCINT] & s->hreg_chan[chan].raw[R_HCINTMSK]) {
        STM32F4xx_raise_host_irq(s, host_intr);
    } else {
        STM32F4xx_lower_host_irq(s, host_intr);
    }
}

// static inline void STM32F4xx_raise_device_common_irq(STM32F4xxUSBState *s, int ep, bool is_out) {
//     uint32_t bitval = BIT( is_out? ep+16 : ep );

//     if (!(s->dreg0[R_DAINT - R_DCFG] & bitval)) {
//         s->dreg0[R_DAINT - R_DCFG] |= bitval;
//         if (is_out && (s->dreg_defs.DAINTMSK.OEPM & s->dreg_defs.DAINT.OEPINT)) 
//         {
//             STM32F4xx_raise_global_irq(s, GINTSTS_OEPINT);
//         }
//         if (!is_out && (s->dreg_defs.DAINTMSK.IEPM & s->dreg_defs.DAINT.IEPINT)) 
//         {
//             STM32F4xx_raise_global_irq(s, GINTSTS_IEPINT);
//         }
//     }
// }

// static inline void STM32F4xx_lower_device_common_irq(STM32F4xxUSBState *s, int ep, bool is_out) {
//     uint32_t bitval = BIT( is_out? ep+16 : ep );

//     if ((s->dreg0[R_DAINT - R_DCFG] & bitval)) {
//         s->dreg0[R_DAINT - R_DCFG] &= ~bitval;
//         if (is_out && (s->dreg_defs.DAINTMSK.OEPM & s->dreg_defs.DAINT.OEPINT)==0) 
//         {
//             STM32F4xx_lower_global_irq(s, GINTSTS_OEPINT);
//         }
//         if (!is_out && (s->dreg_defs.DAINTMSK.IEPM & s->dreg_defs.DAINT.IEPINT)==0) 
//         {
//             STM32F4xx_lower_global_irq(s, GINTSTS_IEPINT);
//         }
        
//     }
// }

static inline void STM32F4xx_update_device_common_irq(STM32F4xxUSBState *s) 
{
    uint32_t common_val = 0;
    bool raise_irq = 0;
    for (int i=0; i< STM32F4xx_NB_DEVCHAN; i++) {
        if (s->drego[i].raw[R_OFF_DEPINT]) 
        {
            common_val |= (1U << i) << 16U;
            raise_irq |= (s->drego[i].raw[R_OFF_DEPINT] & s->dreg0[R_DOEPMSK-R_DCFG])>0; 
        }
        if (s->dregi[i].raw[R_OFF_DEPINT]) 
        {
            common_val |= (1U << i);
            raise_irq |= (s->dregi[i].raw[R_OFF_DEPINT] & s->dreg0[R_DIEPMSK-R_DCFG])>0; 
        }
        if (s->dreg_defs.INEPTXFEM & 1U<<i)
        {
            common_val |= (1U <<i);
            raise_irq |= (s->dregi[i].raw[R_OFF_DEPINT]& DIEPMSK_TXFIFOEMPTY);
        }
    }
    if (common_val^s->dreg0[R_DAINT-R_DCFG]) 
    {
        // There's a change in the common reg. Update global accordingly. 
        s->dreg0[R_DAINT-R_DCFG] = common_val;
        if (!raise_irq)  // Stop here if IRQs are masked.
        {
            return;
        }
        if (s->dreg_defs.DAINT.OEPINT & s->dreg_defs.DAINTMSK.OEPM) 
        {
            STM32F4xx_raise_global_irq(s, GINTSTS_OEPINT);
        }
        else 
        {
            STM32F4xx_lower_global_irq(s, GINTSTS_OEPINT);
        }
        if (s->dreg_defs.DAINT.IEPINT & s->dreg_defs.DAINTMSK.IEPM) 
        {
            STM32F4xx_raise_global_irq(s, GINTSTS_IEPINT);
        }
        else 
        {
            STM32F4xx_lower_global_irq(s, GINTSTS_IEPINT);
        }
    }
}

static inline void STM32F4xx_raise_device_ep_out_irq(STM32F4xxUSBState *s, int ep, uint32_t dev_intr)
{
    if (!(s->drego[ep].raw[R_OFF_DEPINT] & dev_intr)) 
    {
        s->drego[ep].raw[R_OFF_DEPINT] |= dev_intr;
        STM32F4xx_update_device_common_irq(s);
    }
}

static inline void STM32F4xx_lower_device_ep_out_irq(STM32F4xxUSBState *s, int ep, uint32_t dev_intr)
{
    if (s->drego[ep].raw[R_OFF_DEPINT] & dev_intr) 
    {
        s->drego[ep].raw[R_OFF_DEPINT] &= ~dev_intr;
        STM32F4xx_update_device_common_irq(s);
    }
}

static inline void STM32F4xx_raise_device_ep_in_irq(STM32F4xxUSBState *s, int ep, uint32_t dev_intr)
{
    if (!(s->dregi[ep].raw[R_OFF_DEPINT] & dev_intr)) 
    {
        s->dregi[ep].raw[R_OFF_DEPINT] |= dev_intr;
        STM32F4xx_update_device_common_irq(s);
    }
}

static inline void STM32F4xx_lower_device_ep_in_irq(STM32F4xxUSBState *s, int ep, uint32_t dev_intr)
{
    if (s->dregi[ep].raw[R_OFF_DEPINT] & dev_intr) 
    {
        s->dregi[ep].raw[R_OFF_DEPINT] &= ~dev_intr;
        STM32F4xx_update_device_common_irq(s);
    }
}
/* set a timer for EOF */
static void STM32F4xx_eof_timer(STM32F4xxUSBState *s)
{
    timer_mod(s->eof_timer, s->sof_time + s->usb_frame_time);
}

/* Set a timer for EOF and generate SOF event */
static void STM32F4xx_sof(STM32F4xxUSBState *s)
{
    s->sof_time += s->usb_frame_time;
    // trace_usb_stm_sof(s->sof_time);
    STM32F4xx_eof_timer(s);
    STM32F4xx_raise_global_irq(s, GINTSTS_SOF);
}

/* Do frame processing on frame boundary */
static void STM32F4xx_frame_boundary(void *opaque)
{
    STM32F4xxUSBState *s = opaque;
    int64_t now;
    uint16_t frcnt;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* Frame boundary, so do EOF stuff here */

    /* Increment frame number */
    frcnt = (uint16_t)((now - s->sof_time) / s->fi);
    s->frame_number = (s->frame_number + frcnt) & 0xffff;
    s->hfnum = s->frame_number & HFNUM_MAX_FRNUM;

    /* Do SOF stuff here */
    STM32F4xx_sof(s);
}

/* Start sending SOF tokens on the USB bus */
static void STM32F4xx_bus_start(STM32F4xxUSBState *s)
{
    trace_usb_stm_bus_start();
    s->sof_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    STM32F4xx_eof_timer(s);
}

/* Stop sending SOF tokens on the USB bus */
static void STM32F4xx_bus_stop(STM32F4xxUSBState *s)
{
    trace_usb_stm_bus_stop();
    timer_del(s->eof_timer);
}

static USBDevice *STM32F4xx_find_device(STM32F4xxUSBState *s, uint8_t addr)
{
    USBDevice *dev;

    trace_usb_stm_find_device(addr);

    if (!(s->HPRT.PENA)) {
        trace_usb_stm_port_disabled(0);
    } else {
        dev = usb_find_device(&s->uport, addr);
        if (dev != NULL) {
            trace_usb_stm_device_found(0);
            return dev;
        }
    }

    trace_usb_stm_device_not_found();
    return NULL;
}

static const char *pstatus[] = {
    "USB_RET_SUCCESS", "USB_RET_NODEV", "USB_RET_NAK", "USB_RET_STALL",
    "USB_RET_BABBLE", "USB_RET_IOERROR", "USB_RET_ASYNC",
    "USB_RET_ADD_TO_QUEUE", "USB_RET_REMOVE_FROM_QUEUE"
};

static uint32_t pintr[] = {
    HCINTMSK_XFERCOMPL, HCINTMSK_XACTERR, HCINTMSK_NAK, HCINTMSK_STALL,
    HCINTMSK_BBLERR, HCINTMSK_XACTERR, HCINTMSK_XACTERR, HCINTMSK_XACTERR,
    HCINTMSK_XACTERR
};

static const char *types[] = {
    "Ctrl", "Isoc", "Bulk", "Intr"
};

static const char *dirs[] = {
    "Out", "In"
};

// CDC block handlers:

static void STM32F4xx_cdc_schedule(STM32F4xxUSBState *s) {
    timer_mod(s->cdc_timer, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + 1);
}

static int f4xx_usb_cdc_can_receive(void *opaque)
{
    STM32F4xxUSBState *s = STM32F4xx_USB(opaque);
    return s->cdc_in == 0;
}

static void f4xx_usb_cdc_receive(void *opaque, const uint8_t *buf, int size)
{
    STM32F4xxUSBState *s = STM32F4xx_USB(opaque);

    assert(size > 0);
    /* Copy the characters into our buffer first */
    // if (s->periph==19) {
    s->cdc_in = buf[0];
        // printf("CDC RX: ");
        // for (int i=0; i<size;i++)
        //     printf("%c",buf[i]);
        // printf("\n");
    STM32F4xx_cdc_schedule(s);
    // }
    // TODO - forward chars to a packet.
}

static void f4xx_usb_cdc_sendpkt(STM32F4xxUSBState *s, const uint32_t* pBuff, const uint32_t len) 
{
    for (int i=0; i<len; i++) {
        s->fifo_ram[i] = pBuff[i];
        s->rx_fifo_tail = len;
        s->rx_fifo_level = len;
        s->rx_fifo_head = 0;
    }
}

// Responsible for the initial handshake once device mode is enabled. 
static void f4xx_usb_cdc_setup(STM32F4xxUSBState *s) 
{
    if (!qemu_chr_fe_backend_connected(&s->cdc)) {
        return;
    }

    if (s->debug) printf("USB CDC enable state: %u\n", s->device_state);
    switch (s->device_state) {
        case DEV_ST_RESET:
        {
            STM32F4xx_raise_global_irq(s, GINTSTS_CONIDSTSCHNG | GINTSTS_USBRST | GINTSTS_ENUMDONE);
            s->device_state = DEV_ST_RESET_WAIT;
        }
        break;
        case DEV_ST_SETUP:
        {
            s->device_state = DEV_ST_SETUP_WAIT;
            // Construct ctl set address packet:
            f4xx_usb_cdc_sendpkt(s, DEV_SETADDR, sizeof(DEV_SETADDR)/sizeof(uint32_t));
            STM32F4xx_raise_device_ep_out_irq(s, 0, DOEPMSK_SETUPMSK);
            STM32F4xx_raise_global_irq(s, GINTSTS_RXFLVL);
        }
        break;
        case DEV_ST_SETUP_WAIT:
            s->dregi[0].DIEPCTL.EPENA = 0;
            s->device_state = DEV_ST_DESCR;
            STM32F4xx_cdc_schedule(s);
            qemu_chr_fe_write_all(&s->cdc, (const uint8_t*)"STM32F4xx USB CDC emulation started with device:\r\n", 50);
            break;
        case DEV_ST_DESCR:
        case DEV_ST_DESCR2:
        case DEV_ST_DESCR3:
            switch (s->device_state) {
                case DEV_ST_DESCR:
                    f4xx_usb_cdc_sendpkt(s, DEV_GETDESC, sizeof(DEV_GETDESC)/sizeof(uint32_t));
                    break;
                case DEV_ST_DESCR2:
                    f4xx_usb_cdc_sendpkt(s, DEV_GETDESC2, sizeof(DEV_GETDESC2)/sizeof(uint32_t));
                    break;
                case DEV_ST_DESCR3:
                    f4xx_usb_cdc_sendpkt(s, DEV_GETDESC3, sizeof(DEV_GETDESC3)/sizeof(uint32_t));
                    break;
            }            
            s->device_state++;
            STM32F4xx_raise_device_ep_in_irq(s, 0, DIEPMSK_TXFIFOEMPTY);
            STM32F4xx_raise_device_ep_out_irq(s, 0, DOEPMSK_SETUPMSK);
            STM32F4xx_raise_global_irq(s, GINTSTS_RXFLVL);
        break;
        case DEV_ST_DESCR_WAIT:
        case DEV_ST_DESCR2_WAIT:
        case DEV_ST_DESCR3_WAIT:
            if (s->fifo_level[0]==0) // Data not ready yet. EP is enabled before fifo written.
             
                break;
            // Print out descriptor to chardev.
            {
                uint32_t charcount = s->fifo_level[0]*sizeof(uint32_t);
                uint8_t* pBuff = (uint8_t*)&s->tx_fifos[0][s->fifo_head[0]];
                for (int i=2; i<charcount; i+=2)
                    qemu_chr_fe_write(&s->cdc, &pBuff[i],1);
                const uint8_t nline[] = "\r\n";
                qemu_chr_fe_write(&s->cdc, nline,2);
                
            }
            s->fifo_level[0]=0;
            s->fifo_tail[0]=0;
            s->dregi[0].DIEPCTL.EPENA = 0;
            s->device_state++;
            STM32F4xx_cdc_schedule(s);
        break;
        case DEV_ST_SETCFG:
            STM32F4xx_lower_device_ep_in_irq(s, 0, DIEPMSK_TXFIFOEMPTY);
            f4xx_usb_cdc_sendpkt(s, DEV_SETCONF, sizeof(DEV_SETCONF)/sizeof(uint32_t));
            STM32F4xx_raise_device_ep_out_irq(s, 0, DOEPMSK_SETUPMSK);
            STM32F4xx_raise_global_irq(s, GINTSTS_RXFLVL);
            s->device_state++;
            break;
        case DEV_ST_SETCFG_WAIT:
            s->device_state++;
            s->dregi[0].DIEPCTL.EPENA = 0;
            STM32F4xx_cdc_schedule(s);
            break;
        case DEV_ST_LINECODING:
            s->device_state = DEV_ST_SETCTLLINE;
        //     f4xx_usb_cdc_sendpkt(s, DEV_SETCODING, sizeof(DEV_SETCODING)/sizeof(uint32_t));
        //     STM32F4xx_raise_device_ep_out_irq(s, 0, DOEPMSK_SETUPMSK);
        //     STM32F4xx_raise_global_irq(s, GINTSTS_RXFLVL);
        //     break;
        // case DEV_ST_LINECODING_WAIT:
        //     s->device_state++;
        //     STM32F4xx_cdc_schedule(s);
        //     break;
        // case DEV_ST_LINECODING_DATA:
        //     f4xx_usb_cdc_sendpkt(s, DEV_SETCODING2, sizeof(DEV_SETCODING2)/sizeof(uint32_t));
        //     s->drego[0].DOEPCTL.EPENA = 0;
        //     STM32F4xx_raise_global_irq(s, GINTSTS_RXFLVL);
        //     STM32F4xx_raise_device_ep_out_irq(s, 0, DOEPMSK_SETUPMSK | DOEPMSK_XFERCOMPLMSK);
        //     s->device_state++;
        //     break;
        // case DEV_ST_LINECODING_DATA_WAIT:
        //     s->drego[0].DIEPTSIZ.XFRSIZ = 0;
        //     s->device_state++;
        //     STM32F4xx_cdc_schedule(s);
        //     break;
        // FALLTHRU // (because set linecoding doesn't actually do anything)
        case DEV_ST_SETCTLLINE:
            f4xx_usb_cdc_sendpkt(s, DEV_SETCTLLINE, sizeof(DEV_SETCTLLINE)/sizeof(uint32_t));
            STM32F4xx_raise_device_ep_out_irq(s, 0, DOEPMSK_SETUPMSK);
            //printf("DOEPINT: %08x (bit %u)\n", s->drego[0].raw[R_OFF_DEPINT], s->drego[0].DOEPINT.STUP);
            STM32F4xx_raise_global_irq(s, GINTSTS_RXFLVL);
            s->device_state++;
            break;
        case DEV_ST_SETCTLLINE_WAIT:
            qemu_chr_fe_write_all(&s->cdc, (const uint8_t*)"Control line set\r\n",18);
            s->device_state++;
            STM32F4xx_cdc_schedule(s);
            break;
        case DEV_ST_IO_READY:
            STM32F4xx_lower_global_irq(s, GINTSTS_OEPINT);
            qemu_chr_fe_accept_input(&s->cdc);
            s->device_state++;
            // ugly hack, figure out why the hal is locked in HAL_PCD_EP_Open during USBD_CDC_Init...
            s->dreg_defs.DAINTMSK.OEPM |=2;
            s->dreg_defs.DAINTMSK.IEPM |=2;
            break;
        case DEV_ST_IO:
            if (s->cdc_in!=0)
            {
                if (s->cdc_in == '\n') 
                {
                    f4xx_usb_cdc_sendpkt(s, DEV_OUT_AT, sizeof(DEV_OUT_AT)/sizeof(uint32_t));
                }
                else
                {
                    f4xx_usb_cdc_sendpkt(s, &DEV_OUT_HEADER, 1);
                    s->rx_fifo_level++;
                    s->rx_fifo_tail++;
                    s->fifo_ram[1] = 0 | (s->cdc_in);
                }
                s->cdc_in = 0;
                STM32F4xx_raise_device_ep_out_irq(s, 1, DOEPMSK_XFERCOMPLMSK);
                STM32F4xx_raise_global_irq(s, GINTSTS_OEPINT);
                STM32F4xx_raise_global_irq(s, GINTSTS_RXFLVL);
                s->device_state++;
                timer_mod(s->cdc_timer, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + 5000);
            }
            // Check if there's something to send...
            else if (s->fifo_level[1]>0)
            {
                uint8_t* pBuff = (uint8_t*)s->tx_fifos[1];
                qemu_chr_fe_write_all(&s->cdc, pBuff, s->dregi[1].DIEPTSIZ.XFRSIZ-1);
                qemu_chr_fe_write_all(&s->cdc, (const uint8_t*)"\r\n",2);
                s->fifo_tail[1] = 0;
                s->fifo_level[1] = 0;
                // printf("Setting xfercompl\n");
                STM32F4xx_lower_device_ep_in_irq(s, 1, DIEPMSK_TXFIFOEMPTY);
                s->dreg_defs.DAINT.IEPINT &= (~2U); // todo.. figure out why this doesn't clear when the fifo level does.
                STM32F4xx_lower_global_irq(s, GINTSTS_IEPINT);
                STM32F4xx_raise_device_ep_in_irq(s, 1, DIEPMSK_XFERCOMPLMSK);
            } else {

            }
        break;
        case DEV_ST_IO_WAIT:
            // printf("lower\n");
            STM32F4xx_lower_global_irq(s, GINTSTS_OEPINT);
            s->device_state--;
        break;
        // default:            STM32F4xx_raise_device_ep_in_irq(s, 0, DIEPMSK_TXFIFOEMPTY);


    }
    // Set address
    // Get descroptors
    // Set config
    // Set line control
    // Done - free to send/receive serial data.
}

static void STM32F4xx_cdc_helper(void* opaque) {
    f4xx_usb_cdc_setup((STM32F4xxUSBState *)opaque);
}

// end CDC handlers.

static void STM32F4xx_handle_packet(STM32F4xxUSBState *s, uint32_t devadr, USBDevice *dev,
                               USBEndpoint *ep, uint32_t index, bool send)
{
    STM32F4xxPacket *p;
    uint32_t chan = index >> 3;
    hreg_set_t* r_chan = &s->hreg_chan[chan];
    hctsiz_t* hctsiz = &r_chan->defs.HCTSIZ;
    hcchar_t* hcchar = &r_chan->defs.HCCHAR;
    uint32_t pid, tlen, intr = 0;
    uint32_t tpcnt, stsidx, actual = 0;
    bool do_intr = false, done = false;

    pid = hctsiz->DPID;
    assert(hctsiz->XFRSIZ <= STM32F4xx_MAX_XFER_SIZE);
    p = &s->packet[chan];

    if (hcchar->CHDIS)
    {
        printf("Channel %u disabled, ignoring.\n",chan);
        return;
    }

    if (hctsiz->DOPING){
        // Ping packet. just ACK it. 
        if (!s->is_ping) {
            r_chan->defs.HCINT.ACK |= r_chan->defs.HCINTMSK.ACKM;
            // s->hreg1[index + 4] &= ~TSIZ_DOPNG; // clear ping bit, it's done.
            // printf("PING ack'ed (EN: %u), tsiz: %08x\n",(hcchar & HCCHAR_CHDIS)>0, s->hreg1[index+4]);
            // s->hreg1[index] &= (~HCCHAR_CHENA);
            s->is_ping=true;
        } else if (s->is_ping<2){
            // Reenabled post-ping...
            // printf("Ping idling...\n");
        } else {
            s->is_ping = false;
            // printf("Ping complete\n");
            hcchar->CHENA = false;
        }
       // s->is_ping = true;
        STM32F4xx_update_hc_irq(s, index);
        return;
        // printf("Ping send (pid %02x)\n",pid);
    } else if (s->is_ping) {
        // printf("ping clear\n");
        s->is_ping = false;
    }

    trace_usb_stm_handle_packet(chan, dev, &p->packet, 
            hcchar->EPNUM, 
            types[hcchar->EPTYP],
            dirs[hcchar->EPDIR], 
            hcchar->MPSIZ, 
            hctsiz->XFRSIZ,
            hctsiz->PKTCNT);


    if(hctsiz->PKTCNT==0)
    {
        //printf("pcnt=0, returning.\n"); // TODO - figure out why this happens and the halt reenables the channel.
        r_chan->defs.HCCHAR.CHENA = false;
        return;
    }

    if (hcchar->EPTYP == USB_ENDPOINT_XFER_CONTROL && hctsiz->DPID == TSIZ_SC_MC_PID_SETUP) {
        pid = USB_TOKEN_SETUP;
    } else {
        pid = hcchar->EPDIR ? USB_TOKEN_IN : USB_TOKEN_OUT;
    }

    if (send) {
        tlen = hctsiz->XFRSIZ;
        if (p->small) {
            if (tlen > hcchar->MPSIZ) {
                tlen = hcchar->MPSIZ;
            }
        }

        if (pid != USB_TOKEN_IN) {
            trace_usb_stm_memory_read(s->fifo_head[chan], tlen);
            uint32_t words_required = (tlen>>2) + ((tlen%4>0));
            if (s->fifo_level[chan]<words_required)
            {
                //printf("No data to send in FIFO, ignoring...\n");
                return; // No data to send yet...
            }
            uint32_t copy_end = s->fifo_head[chan] + words_required;
            if (copy_end > STM32F4xx_EP_FIFO_SIZE){
                // Split copy because we're looping back around.
                size_t bytes_to_end = ((STM32F4xx_EP_FIFO_SIZE - s->fifo_head[chan])) * sizeof(uint32_t); 
                memcpy(s->usb_buf[chan],&s->tx_fifos[chan][s->fifo_head[chan]],bytes_to_end); // Copy from head to end of ringbuffer
                s->fifo_head[chan] = 0;
                memcpy(&s->usb_buf[chan][bytes_to_end],&s->tx_fifos[chan][0],tlen-bytes_to_end); // Copy remainder.
                s->fifo_head[chan] = copy_end %(STM32F4xx_EP_FIFO_SIZE);
            } else { 
                memcpy(s->usb_buf[chan],&s->tx_fifos[chan][s->fifo_head[chan]],tlen);
                s->fifo_head[chan] += words_required;
            }
            s->fifo_level[chan] -= words_required;
            // printf("USB TX:");
            // for (int i=0; i<tlen; i++)
            //     printf("%02x ",s->usb_buf[chan][i]);
            // printf("\n");
            // if (dma_memory_read(&s->dma_as, hcdma,                                          
            //                     s->usb_buf[chan], tlen) != MEMTX_OK) {
            //     qemu_log_mask(LOG_GUEST_ERROR, "%s: dma_memory_read failed\n",
            //                   __func__);
            // }
        }

        usb_packet_init(&p->packet);
        usb_packet_setup(&p->packet, pid, ep, 0, s->fifo_head[chan],
                         pid != USB_TOKEN_IN, true);
        usb_packet_addbuf(&p->packet, s->usb_buf[chan], hctsiz->XFRSIZ);
        p->async = STM32F4xx_ASYNC_NONE;
        usb_handle_packet(dev, &p->packet);
    } else {
        tlen = p->len;
    }

    stsidx = -p->packet.status;
    assert(stsidx < sizeof(pstatus) / sizeof(*pstatus));
    actual = p->packet.actual_length;
    trace_usb_stm_packet_status(pstatus[stsidx], actual);

babble:
    if (p->packet.status != USB_RET_SUCCESS &&
            p->packet.status != USB_RET_NAK &&
            p->packet.status != USB_RET_STALL &&
            p->packet.status != USB_RET_ASYNC) {
        trace_usb_stm_packet_error(pstatus[stsidx]);
    }

    if (p->packet.status == USB_RET_ASYNC) {
        trace_usb_stm_async_packet(&p->packet, chan, dev, hcchar->EPNUM,
                                    dirs[hcchar->EPDIR], tlen);
        usb_device_flush_ep_queue(dev, ep);
        assert(p->async != STM32F4xx_ASYNC_INFLIGHT);
        p->devadr = devadr;
        p->epnum = hcchar->EPNUM;
        p->epdir = hcchar->EPDIR;
        p->mps = hcchar->MPSIZ;
        p->pid = pid;
        p->index = index;
        p->pcnt = hctsiz->PKTCNT;
        p->len = tlen;
        p->async = STM32F4xx_ASYNC_INFLIGHT;
        p->needs_service = false;
        return;
    }

    if (p->packet.status == USB_RET_SUCCESS) {
        if (actual > tlen) {
            p->packet.status = USB_RET_BABBLE;
            goto babble;
        }

        if (pid == USB_TOKEN_IN) {
            // First constuct the packet header
            if (actual>0) {
                trace_usb_stm_memory_write(s->rx_fifo_tail, actual);
                rxstatus_t header = {.chnum = chan, .bcnt = actual, .dpid = hctsiz->DPID, .pktsts = 0b0010};
                // printf("packet in, size: %u\n",actual);
                // uint32_t orig_head =s->rx_fifo_tail;
                uint32_t words = (actual>>2); // +1 for header.
                if (actual%4 !=0) {
                    words++; 
                }
                if (s->rx_fifo_level + words + 1 > STM32F4xx_RX_FIFO_SIZE)
                {
                    qemu_log_mask(LOG_GUEST_ERROR,"Receive FIFO full, data discarded!");
                }
                s->rx_fifo_tail %= STM32F4xx_RX_FIFO_SIZE;
                s->fifo_ram[s->rx_fifo_tail++] = header.raw;
                s->rx_fifo_tail %= STM32F4xx_RX_FIFO_SIZE;
                s->rx_fifo_level++;
                uint32_t copy_end = s->rx_fifo_tail + words;
                if (copy_end > STM32F4xx_RX_FIFO_SIZE){
                    size_t bytes_to_end = ((STM32F4xx_RX_FIFO_SIZE - s->rx_fifo_tail)) * sizeof(uint32_t); 
                    // Split copy because we're looping back around.
                    memcpy(&s->fifo_ram[s->rx_fifo_tail],s->usb_buf[chan],bytes_to_end);
                    s->rx_fifo_tail = 0; // Loop around.
                    memcpy(&s->fifo_ram[s->rx_fifo_tail],&s->usb_buf[chan][bytes_to_end],actual - bytes_to_end);
                    s->rx_fifo_tail = copy_end % (STM32F4xx_RX_FIFO_SIZE);
                } else { 
                    memcpy(&s->fifo_ram[s->rx_fifo_tail],s->usb_buf[chan],actual);
                    s->rx_fifo_tail += words;
                }
                // printf("USB RX %u: ", actual);
                // for (int i=0; i<actual; i++)
                //     printf("%02x ",s->usb_buf[chan][i]);
                // printf("\n");
                s->rx_fifo_level += words;
                assert(s->rx_fifo_tail <= 32768); // Fix this you lazy lump...
          
                // while (orig_head != s->rx_fifo_tail)
                // {
                //     printf("RxFifo add @ %u: %08x \n", orig_head, s->fifo_ram[orig_head]);
                //     orig_head++;
                // }
                s->gintsts |= GINTSTS_RXFLVL;
            }
            else
            {
                // printf("Incoming size 0, doping: %u\n",s->is_ping);
                hctsiz->PKTCNT--;
                // if (s->is_ping)
                // {
                //     intr |= HCINTMSK_ACK | HCINTMSK_XFERCOMPL;
                //     s->is_ping = 0;
                //     STM32F4xx_update_hc_irq(s, index);
                //     usb_packet_cleanup(&p->packet);
                //     return;
                // }
            }
            // cpu_physical_memory_write(hcdma + 0x1000,s->usb_buf[chan],tlen);
            // if (dma_memory_write(&s->dma_as, hcdma, s->usb_buf[chan],
            //                      actual) != MEMTX_OK) {
            //     qemu_log_mask(LOG_GUEST_ERROR, "%s: dma_memory_write failed\n",
            //                   __func__);
            // }
        } 

        tpcnt = actual / hcchar->MPSIZ;
        if (actual % hcchar->MPSIZ || tlen==0) {
            tpcnt++;
            if (pid == USB_TOKEN_IN) {
                done = true;
                // intr |= HCINTMSK_NYET; // Transfer complete.
            }
        }

        hctsiz->PKTCNT -= MIN(tpcnt, hctsiz->PKTCNT);
        hctsiz->XFRSIZ -= MIN(actual, hctsiz->XFRSIZ);
        // hcdma += actual;
        // s->hreg1[index + 5] = hcdma;

     if (hctsiz->PKTCNT == 0 || hctsiz->XFRSIZ == 0 || actual == 0) {
            done = true;
            // intr |= HCINTMSK_ACK;
        }
    } else {
        intr |= pintr[stsidx];
        if (p->packet.status == USB_RET_NAK &&
            (hcchar->EPTYP == USB_ENDPOINT_XFER_CONTROL ||
             hcchar->EPTYP == USB_ENDPOINT_XFER_BULK)) {
            /*
             * for ctrl/bulk, automatically retry on NAK,
             * but send the interrupt anyway
             */
            intr &= ~HCINTMSK_RESERVED14_31;
            s->hreg_chan[chan].raw[R_HCINT] |= intr;
            do_intr = true;
        } else {
            intr |= HCINTMSK_CHHLTD;
            done = true;
        }
    }

    usb_packet_cleanup(&p->packet);

    if (done) {
        hcchar->CHENA = false;
        if (!(intr & HCINTMSK_CHHLTD)) {
            intr |= HCINTMSK_CHHLTD | HCINTMSK_XFERCOMPL;
        }
        intr &= ~HCINTMSK_RESERVED14_31;
        s->hreg_chan[chan].raw[R_HCINT] |= intr;
        p->needs_service = false;
        trace_usb_stm_packet_done(pstatus[stsidx], actual, hctsiz->XFRSIZ, hctsiz->PKTCNT);
        STM32F4xx_update_hc_irq(s, index);
        return;
    }

    p->devadr = devadr;
    p->epnum = hcchar->EPNUM;
    p->epdir = hcchar->EPDIR;
    p->mps = hcchar->MPSIZ;
    p->pid = pid;
    p->index = index;
    p->pcnt = hctsiz->PKTCNT;
    p->len = hctsiz->XFRSIZ;
    p->needs_service = true;
    trace_usb_stm_packet_next(pstatus[stsidx], hctsiz->XFRSIZ, hctsiz->PKTCNT);
    if (do_intr) {
        STM32F4xx_update_hc_irq(s, index);
    }
}

/* Attach or detach a device on root hub */

static const char *speeds[] = {
    "low", "full", "high"
};

static void STM32F4xx_attach(USBPort *port)
{
    STM32F4xxUSBState *s = port->opaque;
    int hispd = 0;

    trace_usb_stm_attach(port);
    assert(port->index == 0);

    if (!port->dev || !port->dev->attached) {
        return;
    }

    assert(port->dev->speed <= USB_SPEED_HIGH);
    trace_usb_stm_attach_speed(speeds[port->dev->speed]);
    s->hprt0 &= ~HPRT0_SPD_MASK;

    switch (port->dev->speed) {
    case USB_SPEED_LOW:
        s->hprt0 |= HPRT0_SPD_LOW_SPEED << HPRT0_SPD_SHIFT;
        break;
    case USB_SPEED_FULL:
        s->hprt0 |= HPRT0_SPD_FULL_SPEED << HPRT0_SPD_SHIFT;
        break;
    case USB_SPEED_HIGH:
        s->hprt0 |= HPRT0_SPD_HIGH_SPEED << HPRT0_SPD_SHIFT;
        hispd = 1;
        break;
    }

    if (hispd) {
        s->usb_frame_time = NANOSECONDS_PER_SECOND / 8000;        /* 125000 */
        if (NANOSECONDS_PER_SECOND >= USB_HZ_HS) {
            s->usb_bit_time = NANOSECONDS_PER_SECOND / USB_HZ_HS; /* 10.4 */
        } else {
            s->usb_bit_time = 1;
        }
    } else {
        s->usb_frame_time = NANOSECONDS_PER_SECOND / 1000;        /* 1000000 */
        if (NANOSECONDS_PER_SECOND >= USB_HZ_FS) {
            s->usb_bit_time = NANOSECONDS_PER_SECOND / USB_HZ_FS; /* 83.3 */
        } else {
            s->usb_bit_time = 1;
        }
    }

    s->fi = USB_FRMINTVL - 1;
    s->HPRT.PCDET = 1;
    s->HPRT.PCSTS = 1;

    STM32F4xx_bus_start(s);
    STM32F4xx_raise_global_irq(s, GINTSTS_PRTINT);
}

static void STM32F4xx_detach(USBPort *port)
{
    STM32F4xxUSBState *s = port->opaque;

    trace_usb_stm_detach(port);
    assert(port->index == 0);

    STM32F4xx_bus_stop(s);
    s->HPRT.PSPD = 0;
    s->HPRT.PSUSP = 0;
    s->HPRT.PENA = 0;
    s->HPRT.PCSTS = 0;
    s->HPRT.PENCHNG = 1;
    s->HPRT.PCDET = 0;

    STM32F4xx_raise_global_irq(s, GINTSTS_PRTINT);
}

static void STM32F4xx_child_detach(USBPort *port, USBDevice *child)
{
    trace_usb_stm_child_detach(port, child);
    assert(port->index == 0);
}

static void STM32F4xx_wakeup(USBPort *port)
{
    STM32F4xxUSBState *s = port->opaque;

    trace_usb_stm_wakeup(port);
    assert(port->index == 0);

    if (s->HPRT.PSUSP) {
        s->HPRT.PRST = 1;
        STM32F4xx_raise_global_irq(s, GINTSTS_PRTINT);
    }

    qemu_bh_schedule(s->async_bh);
}

static void STM32F4xx_async_packet_complete(USBPort *port, USBPacket *packet)
{
    STM32F4xxUSBState *s = port->opaque;
    STM32F4xxPacket *p;
    USBDevice *dev;
    USBEndpoint *ep;

    assert(port->index == 0);
    p = container_of(packet, STM32F4xxPacket, packet);
    dev = STM32F4xx_find_device(s, p->devadr);
    ep = usb_ep_get(dev, p->pid, p->epnum);
    trace_usb_stm_async_packet_complete(port, packet, p->index >> 3, dev,
                                         p->epnum, dirs[p->epdir], p->len);
    assert(p->async == STM32F4xx_ASYNC_INFLIGHT);

    if (packet->status == USB_RET_REMOVE_FROM_QUEUE) {
        usb_cancel_packet(packet);
        usb_packet_cleanup(packet);
        return;
    }

    STM32F4xx_handle_packet(s, p->devadr, dev, ep, p->index, false);

    p->async = STM32F4xx_ASYNC_FINISHED;
    qemu_bh_schedule(s->async_bh);
}

static USBPortOps STM32F4xx_port_ops = {
    .attach = STM32F4xx_attach,
    .detach = STM32F4xx_detach,
    .child_detach = STM32F4xx_child_detach,
    .wakeup = STM32F4xx_wakeup,
    .complete = STM32F4xx_async_packet_complete,
};

static uint32_t STM32F4xx_get_frame_remaining(STM32F4xxUSBState *s)
{
    uint32_t fr = 0;
    int64_t tks;

    tks = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->sof_time;
    if (tks < 0) {
        tks = 0;
    }

    /* avoid muldiv if possible */
    if (tks >= s->usb_frame_time) {
        goto out;
    }
    if (tks < s->usb_bit_time) {
        fr = s->fi;
        goto out;
    }

    /* tks = number of ns since SOF, divided by 83 (fs) or 10 (hs) */
    tks = tks / s->usb_bit_time;
    if (tks >= (int64_t)s->fi) {
        goto out;
    }

    /* remaining = frame interval minus tks */
    fr = (uint32_t)((int64_t)s->fi - tks);

out:
    return fr;
}

static void STM32F4xx_work_bh(void *opaque)
{
    STM32F4xxUSBState *s = opaque;
    STM32F4xxPacket *p;
    USBDevice *dev;
    USBEndpoint *ep;
    int64_t t_now, expire_time;
    int chan;
    bool found = false;

    trace_usb_stm_work_bh();
    if (s->working) {
        return;
    }
    s->working = true;

    t_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    chan = s->next_chan;

    do {
        p = &s->packet[chan];
        if (p->needs_service) {
            dev = STM32F4xx_find_device(s, p->devadr);
            ep = usb_ep_get(dev, p->pid, p->epnum);
            trace_usb_stm_work_bh_service(s->next_chan, chan, dev, p->epnum);
            STM32F4xx_handle_packet(s, p->devadr, dev, ep, p->index, true);
            found = true;
        }
        if (++chan == STM32F4xx_NB_CHAN) {
            chan = 0;
        }
        if (found) {
            s->next_chan = chan;
            trace_usb_stm_work_bh_next(chan);
        }
    } while (chan != s->next_chan);

    if (found) {
        expire_time = t_now + NANOSECONDS_PER_SECOND / 4000;
        timer_mod(s->frame_timer, expire_time);
    }
    s->working = false;
}

static void STM32F4xx_enable_chan(STM32F4xxUSBState *s,  uint32_t index)
{
    USBDevice *dev;
    USBEndpoint *ep;
    hcchar_t* hcchar;
    hctsiz_t* hctsiz;
    STM32F4xxPacket *p;

    uint32_t chan = index>>3;
    assert(chan < STM32F4xx_NB_CHAN);
    hcchar = &s->hreg_chan[chan].defs.HCCHAR;
    hctsiz = &s->hreg_chan[chan].defs.HCTSIZ;
    p = &s->packet[chan];

    dev = STM32F4xx_find_device(s, hcchar->DAD);

    trace_usb_stm_enable_chan(chan, dev, &p->packet, hcchar->EPNUM);
    if (dev == NULL) {
        hcchar->CHENA = false; // Disable channel if device not found.
        return;
    }
    uint32_t pid;
    if (hcchar->EPTYP == USB_ENDPOINT_XFER_CONTROL && hctsiz->DPID == TSIZ_SC_MC_PID_SETUP) {
        pid = USB_TOKEN_SETUP;
    } else {
        pid = hcchar->EPDIR ? USB_TOKEN_IN : USB_TOKEN_OUT;
    }

    ep = usb_ep_get(dev, pid, hcchar->EPNUM);

    /*
     * Hack: Networking doesn't like us delivering large transfers, it kind
     * of works but the latency is horrible. So if the transfer is <= the mtu
     * size, we take that as a hint that this might be a network transfer,
     * and do the transfer packet-by-packet.
     */
    if (hctsiz->XFRSIZ > 1536) {
        p->small = false;
    } else {
        p->small = true;
    }

    STM32F4xx_handle_packet(s, hcchar->DAD, dev, ep, index, true);
    qemu_bh_schedule(s->async_bh);
}

// static const char *glbregnm[] = {
//     "GOTGCTL  ", "GOTGINT  ", "GAHBCFG  ", "GUSBCFG  ", "GRSTCTL  ",
//     "GINTSTS  ", "GINTMSK  ", "GRXSTSR  ", "GRXSTSP  ", "GRXFSIZ  ",
//     "GNPTXFSIZ", "GNPTXSTS ", "GI2CCTL  ", "GPVNDCTL ", "GGPIO    ",
//     "GUID     ", "GSNPSID  ", "GHWCFG1  ", "GHWCFG2  ", "GHWCFG3  ",
//     "GHWCFG4  ", "GLPMCFG  ", "GPWRDN   ", "GDFIFOCFG", "GADPCTL  ",
//     "GREFCLK  ", "GINTMSK2 ", "GINTSTS2 "
// };

static uint64_t STM32F4xx_glbreg_read(void *ptr, hwaddr addr, int index,
                                 unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t val;

    assert(addr <= GINTSTS2);
    val = s->glbreg[index];

    switch (addr) {
    case GRSTCTL:
        /* clear any self-clearing bits that were set */
        val &= ~(GRSTCTL_TXFFLSH | GRSTCTL_RXFFLSH | GRSTCTL_IN_TKNQ_FLSH |
                 GRSTCTL_FRMCNTRRST | GRSTCTL_HSFTRST | GRSTCTL_CSFTRST);
        s->glbreg[index] = val;
        break;
    case GRXSTSP:
        if (s->debug) printf("RX fifo pop (STSP): %08x @ %u\n", s->fifo_ram[s->rx_fifo_head], s->rx_fifo_head);
        if(s->rx_fifo_level>0) {
            val = s->fifo_ram[s->rx_fifo_head++];
            s->rx_fifo_level--;
        } // else: val will be 0 from s->GRXSTSP;
        break;
    case GRXSTSR:
        // printf("rxfifo read\n");
        if (s->rx_fifo_level>0)
            val = s->fifo_ram[s->rx_fifo_head];
        break;
    default:
        break;
    }

   // trace_usb_stm_glbreg_read(addr, glbregnm[index], val);
    return val;
}


static void STM32F4xx_tx_packet(STM32F4xxUSBState *s, int index) {

    USBDevice *dev;
    USBEndpoint *ep;
    STM32F4xxPacket *p;

    assert((index >> 3) < STM32F4xx_NB_CHAN);
    p = &s->packet[index >> 3];

    hcchar_t* hcchar = &s->hreg_chan[index>>3].defs.HCCHAR;
    hctsiz_t* hctsiz = &s->hreg_chan[index>>3].defs.HCTSIZ;

    dev = STM32F4xx_find_device(s, hcchar->DAD);

    if (dev == NULL) {
        return;
    }

    uint32_t pid;
    if (hcchar->EPTYP == USB_ENDPOINT_XFER_CONTROL && hctsiz->DPID == TSIZ_SC_MC_PID_SETUP) {
        pid = USB_TOKEN_SETUP;
    } else {
        pid = hcchar->EPDIR ? USB_TOKEN_IN : USB_TOKEN_OUT;
    }

    ep = usb_ep_get(dev, pid, hcchar->EPNUM);

    /*
     * Hack: Networking doesn't like us delivering large transfers, it kind
     * of works but the latency is horrible. So if the transfer is <= the mtu
     * size, we take that as a hint that this might be a network transfer,
     * and do the transfer packet-by-packet.
     */
    if (hctsiz->XFRSIZ > 1536) {
        p->small = false;
    } else {
        p->small = true;
    }

    STM32F4xx_handle_packet(s, hcchar->DAD, dev, ep, index, true);
    qemu_bh_schedule(s->async_bh);
}


static void STM32F4xx_flush_tx(void *ptr)
{
    STM32F4xxUSBState *s = ptr;
    uint16_t index = (s->grstctl & GRSTCTL_TXFNUM_MASK) >> GRSTCTL_TXFNUM_SHIFT;

    if (index==0b1000 || index>0)
    {
        printf("FIXME: >0 flush\n");
        return;
    }
    if (s->fifo_level[index] == 0) {
        return; // FIFO already empty. 
    }
    // switch (index) {
    //     case 0b10000: // All
    //     break;
    //     default:

    // }

    printf("FIXME: PACKET FLUSH\n");
    //STM32F4xx_tx_packet(s);
    
}

static void STM32F4xx_glbreg_write(void *ptr, hwaddr addr, int index, uint64_t val,
                              unsigned size)
{
    STM32F4xxUSBState *s = ptr;
   // uint64_t orig = val;
    uint32_t *mmio;
    uint32_t old;
    int iflg = 0;

    assert(addr <= GINTSTS2);
    mmio = &s->glbreg[index];
    old = *mmio;

    switch (addr) {
    case GOTGCTL:
        /* don't allow setting of read-only bits */
        val &= ~(GOTGCTL_MULT_VALID_BC_MASK | GOTGCTL_BSESVLD |
                 GOTGCTL_ASESVLD | GOTGCTL_DBNC_SHORT | GOTGCTL_CONID_B |
                 GOTGCTL_HSTNEGSCS | GOTGCTL_SESREQSCS);
        /* don't allow clearing of read-only bits */
        val |= old & (GOTGCTL_MULT_VALID_BC_MASK | GOTGCTL_BSESVLD |
                      GOTGCTL_ASESVLD | GOTGCTL_DBNC_SHORT | GOTGCTL_CONID_B |
                      GOTGCTL_HSTNEGSCS | GOTGCTL_SESREQSCS);
        break;
    case GAHBCFG:
        if ((val & GAHBCFG_GLBL_INTR_EN) && !(old & GAHBCFG_GLBL_INTR_EN)) {
            iflg = 1;
        }
        if (val & GAHBCFG_DMA_EN)
        {
            printf("FIXME: USB dma not implemented!\n");
        }
        break;
    case GUSBCFG: 
        if (val & GUSBCFG_FORCEDEVMODE) {
            printf("USB OTG: Changing to device mode\n");
            s->GINTSTS.CMOD = 0;
        }
        break;
    case GRSTCTL:
        val |= GRSTCTL_AHBIDLE;
        val &= ~GRSTCTL_DMAREQ;
        if (!(old & GRSTCTL_TXFFLSH) && (val & GRSTCTL_TXFFLSH)) {
                /* TODO - TX fifo flush */
            STM32F4xx_flush_tx(s);
        }
        if (!(old & GRSTCTL_RXFFLSH) && (val & GRSTCTL_RXFFLSH)) {
                /* TODO - RX fifo flush */
            qemu_log_mask(LOG_UNIMP, "Rx FIFO flush not implemented\n");
        }
        if (!(old & GRSTCTL_IN_TKNQ_FLSH) && (val & GRSTCTL_IN_TKNQ_FLSH)) {
                /* TODO - device IN token queue flush */
            qemu_log_mask(LOG_UNIMP, "Token queue flush not implemented\n");
        }
        if (!(old & GRSTCTL_FRMCNTRRST) && (val & GRSTCTL_FRMCNTRRST)) {
                /* TODO - host frame counter reset */
            qemu_log_mask(LOG_UNIMP, "Frame counter reset not implemented\n");
        }
        if (!(old & GRSTCTL_HSFTRST) && (val & GRSTCTL_HSFTRST)) {
                /* TODO - host soft reset */
            qemu_log_mask(LOG_UNIMP, "Host soft reset not implemented\n");
        }
        if (!(old & GRSTCTL_CSFTRST) && (val & GRSTCTL_CSFTRST)) {
                /* TODO - core soft reset */
            qemu_log_mask(LOG_UNIMP, "Core soft reset not implemented\n");
        }
        /* don't allow clearing of self-clearing bits */
        val |= old & (GRSTCTL_TXFFLSH | GRSTCTL_RXFFLSH |
                      GRSTCTL_IN_TKNQ_FLSH | GRSTCTL_FRMCNTRRST |
                      GRSTCTL_HSFTRST | GRSTCTL_CSFTRST);
        break;
    case R_GINTSTS*4:
        /* clear the write-1-to-clear bits */
        val |= ~old;
        val = ~val;
        /* don't allow clearing of read-only bits */
        val |= old & (GINTSTS_PTXFEMP | GINTSTS_HCHINT | GINTSTS_PRTINT |
                      GINTSTS_OEPINT | GINTSTS_IEPINT | GINTSTS_GOUTNAKEFF |
                      GINTSTS_GINNAKEFF | GINTSTS_NPTXFEMP | GINTSTS_RXFLVL |
                      GINTSTS_OTGINT | GINTSTS_CURMODE_HOST);
        iflg = 1;
        if ((s->GINTSTS.USBRST && ((~val)&GINTSTS_USBRST)) && s->device_state==DEV_ST_RESET_WAIT) {
            s->device_state = DEV_ST_SETUP;
            STM32F4xx_cdc_schedule(s);
        }
        break;
    case GINTMSK:
        iflg = 1;
        break;
    case GRXSTSP:
    case GRXSTSR:
        printf("write to sts/siz\n");
        /* FALLTHRU */
    case GRXFSIZ:
        break;
    default:
        break;
    }

    //trace_usb_stm_glbreg_write(addr, glbregnm[index], orig, old, val);
    *mmio = val;

    if (iflg) {
        STM32F4xx_update_irq(s);
    }
}

static uint64_t STM32F4xx_fszreg_read(void *ptr, hwaddr addr, int index,
                                 unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t val;

    assert(addr == HPTXFSIZ);
    val = s->fszreg[index];

    trace_usb_stm_fszreg_read(addr, val);
    return val;
}

static void STM32F4xx_fszreg_write(void *ptr, hwaddr addr, int index, uint64_t val,
                              unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t old;

    assert(addr == HPTXFSIZ);
    mmio = &s->fszreg[index];
    old = *mmio;

    trace_usb_stm_fszreg_write(addr, orig, old, val);
    *mmio = val;
}

static const char *hreg0nm[] = {
    "HCFG     ", "HFIR     ", "HFNUM    ", "<rsvd>   ", "HPTXSTS  ",
    "HAINT    ", "HAINTMSK ", "HFLBADDR ", "<rsvd>   ", "<rsvd>   ",
    "<rsvd>   ", "<rsvd>   ", "<rsvd>   ", "<rsvd>   ", "<rsvd>   ",
    "<rsvd>   ", "HPRT0    "
};

static uint64_t STM32F4xx_hreg0_read(void *ptr, hwaddr addr, int index,
                                unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t val;

    assert(addr >= HCFG && addr <= HPRT0);
    val = s->hreg0[index];

    switch (addr) {
    case HFNUM:
        val = (STM32F4xx_get_frame_remaining(s) << HFNUM_FRREM_SHIFT) |
              (s->hfnum << HFNUM_FRNUM_SHIFT);
        break;
    default:
        break;
    }

    trace_usb_stm_hreg0_read(addr, hreg0nm[index], val);
    return val;
}

static void STM32F4xx_hreg0_write(void *ptr, hwaddr addr, int index, uint64_t val,
                             unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    USBDevice *dev = s->uport.dev;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t tval, told, old;
    int prst = 0;
    int iflg = 0;

    assert(addr >= HCFG && addr <= HPRT0);
    mmio = &s->hreg0[index];
    old = *mmio;

    switch (addr) {
    case HFIR:
        break;
    case HFNUM:
    case HPTXSTS:
    case HAINT:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register\n",
                      __func__);
        return;
    case HAINTMSK:
        val &= 0xffff;
        break;
    case HPRT0:
        /* don't allow clearing of read-only bits */
        val |= old & (HPRT0_SPD_MASK | HPRT0_LNSTS_MASK | HPRT0_OVRCURRACT |
                      HPRT0_CONNSTS);
        /* don't allow clearing of self-clearing bits */
        val |= old & (HPRT0_SUSP | HPRT0_RES);
        /* don't allow setting of self-setting bits */
        if (!(old & HPRT0_ENA) && (val & HPRT0_ENA)) {
            val &= ~HPRT0_ENA;
        }
        /* clear the write-1-to-clear bits */
        tval = val & (HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_ENA |
                      HPRT0_CONNDET);
        told = old & (HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_ENA |
                      HPRT0_CONNDET);
        tval |= ~told;
        tval = ~tval;
        tval &= (HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_ENA |
                 HPRT0_CONNDET);
        val &= ~(HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_ENA |
                 HPRT0_CONNDET);
        val |= tval;
        if (!(val & HPRT0_RST) && (old & HPRT0_RST)) {
            if (dev && dev->attached) {
                val |= HPRT0_ENA | HPRT0_ENACHG;
                prst = 1;
            }
        }
        if (val & (HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_CONNDET)) {
            iflg = 1;
        } else {
            iflg = -1;
        }
        break;
    default:
        break;
    }

    if (prst) {
        trace_usb_stm_hreg0_write(addr, hreg0nm[index], orig, old,
                                   val & ~HPRT0_CONNDET);
        trace_usb_stm_hreg0_action("call usb_port_reset");
        usb_port_reset(&s->uport);
        val &= ~HPRT0_CONNDET;
    } else {
        trace_usb_stm_hreg0_write(addr, hreg0nm[index], orig, old, val);
    }

    *mmio = val;

    if (iflg > 0) {
        trace_usb_stm_hreg0_action("enable PRTINT");
        STM32F4xx_raise_global_irq(s, GINTSTS_PRTINT);
    } else if (iflg < 0) {
        trace_usb_stm_hreg0_action("disable PRTINT");
        STM32F4xx_lower_global_irq(s, GINTSTS_PRTINT);
    }
}

static const char *hreg1nm[] = {
    "HCCHAR  ", "HCSPLT  ", "HCINT   ", "HCINTMSK", "HCTSIZ  ", "HCDMA   ",
    "<rsvd>  ", "HCDMAB  "
};

static uint64_t STM32F4xx_hreg1_read(void *ptr, hwaddr addr, int index,
                                unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t val;

    assert(addr >= HCCHAR(0) && addr <= HCDMAB(STM32F4xx_NB_CHAN - 1));
    uint32_t chan = index >> 3;
    uint32_t offset = index & 0b111;
    val = s->hreg_chan[chan].raw[offset];

    //trace_usb_stm_hreg1_read(addr, hreg1nm[index & 7], addr >> 5, val);
    return val;
}

static void STM32F4xx_hreg1_write(void *ptr, hwaddr addr, int index, uint64_t val,
                             unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t old;
    int iflg = 0;
    int enflg = 0;
    int disflg = 0;

    uint32_t chan = index>>3;

    assert(addr >= HCCHAR(0) && addr <= HCDMAB(STM32F4xx_NB_CHAN - 1));
    mmio = &s->hreg_chan[chan].raw[index&0b111U];
    old = *mmio;
    uint64_t  HALT = HCCHAR_CHDIS | HCCHAR_CHENA;
    switch (HSOTG_REG(0x500) + (addr & 0x1c)) {
    case HCCHAR(0):


        if ((val & HCCHAR_CHDIS) && !(old & HCCHAR_CHDIS)) {
            val &= ~(HCCHAR_CHENA | HCCHAR_CHDIS);
            disflg = 1;
            // printf("p1\n");
        } else {
            val |= old & HCCHAR_CHDIS;
            if ((val & HCCHAR_CHENA) && !(old & HCCHAR_CHENA)) {
                val &= ~HCCHAR_CHDIS;
                enflg = 1;
            } else {
                val |= old & HCCHAR_CHENA;
            }
            // printf("p2\n");
        }
        if ((val & HALT) == HALT)
        {
            //printf("CH HALTED\n");
            val &= ~(HCCHAR_CHENA | HCCHAR_CHDIS);
            disflg = 1;
            enflg = 0;
        }
        break;
    case HCINT(0):
        /* clear the write-1-to-clear bits */
        if (s->is_ping && (val & HCINTMSK_CHHLTD)) {
            s->is_ping++;
            // TODO -  the way ping works this needs to be fired after the CHHLTD flag has been handled.
            // but not for bulk endpoints though, it expects those to remain in NREADY
            if ((index>>3) == USB_ENDPOINT_XFER_BULK) {
                s->is_ping = 0;
                s->hreg_chan[chan].defs.HCCHAR.CHENA = false;
            } else {
                old |= HCINTMSK_XFERCOMPL;
                iflg = 1;
                //printf("Ping XFRC\n");
            }
        }
        val |= ~old;
        val = ~val;
        val &= ~HCINTMSK_RESERVED14_31;
        iflg = 1;
        break;
    case HCINTMSK(0):
        val &= ~HCINTMSK_RESERVED14_31;
        iflg = 1;
        break;
    case HCDMAB(0):
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register\n",
                      __func__);
        return;
    default:
        break;
    }

    trace_usb_stm_hreg1_write(addr, hreg1nm[index & 7], index >> 3, orig,
                               old, val);
    *mmio = val;

    if (disflg) {
        /* set ChHltd in HCINT */
        s->hreg_chan[chan].defs.HCINT.CHH = true;
        iflg = 1;
    }

    if (enflg) {
        STM32F4xx_enable_chan(s, index & ~7);
    }

    if (iflg) {
        STM32F4xx_update_hc_irq(s, index & ~7);
    }
}

static void STM32F4xx_dreg0_write(void *ptr, hwaddr addr, int index, uint64_t val,
                             unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    //uint64_t orig = val;
    //uint32_t *mmio;
    uint32_t old;

    old = s->dreg0[index];

    uint32_t changed = old^val;

    addr>>=2;

    assert(addr >= R_DCFG && addr <= R_DIEPEMPMSK);
    switch (addr){
        case R_DCTL:
            s->dreg0[index] = val;
            if (changed&DCTL_SFTDISCON) {
                //DC connected, start setup. 
                if (s->dreg_defs.DCTL.SDIS) {
                    s->device_state = DEV_ST_RESET;
                } else {
                    f4xx_usb_cdc_setup(s);
                }
            }
        break;
        case R_DOEPMSK:
        case R_DIEPMSK:
        case R_DAINTMSK:
            s->dreg0[index] = val;
            STM32F4xx_update_device_common_irq(s);
            break;
        case R_DIEPEMPMSK:
           // printf("EPEMPMSK Write: %08x -> %"PRIx64" to addr %"HWADDR_PRIx"\n",s->dreg0[index] , val, addr<<2);
            s->dreg0[index] = val;
            if ((s->device_state >= DEV_ST_IO_READY) && (val&2U)) 
            {
                STM32F4xx_raise_device_ep_in_irq(s, 1, DIEPMSK_TXFIFOEMPTY);
            }
            // else if (s->device_state >= DEV_ST_IO_READY && s->fifo_level[1] == 0) 
            // {
            //     s->dreg_defs.DAINT.IEPINT &= (~2U);
            //     STM32F4xx_lower_global_irq(s, GINTSTS_IEPINT);
            // }
            else
            {
                STM32F4xx_update_device_common_irq(s);
            }
            break;
        case R_DAINT:
            qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write read-only USB DAINT register!");
            break;
        default:
            printf("unhandled USBD Write: %"PRIx64" to addr %"HWADDR_PRIx"\n", val, addr<<2);
            break;
    }

}

static uint64_t STM32F4xx_dreg0_read(void *ptr, hwaddr addr, int index,
                                unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t val;
    addr >>=2;
    assert(addr >= R_DCFG && addr <= R_DIEPEMPMSK);

    val = s->dreg0[index];
    //printf("DREG0 read: %"HWADDR_PRIx ": %"PRIx32" \n", addr<<2, val );
    return val;
}

static uint64_t STM32F4xx_dregout_read(void *ptr, hwaddr addr, int index,
                                unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t chan = index>>3;
    uint32_t offset = index & 0b111;
    uint32_t val;
    assert(chan<=4);
    assert(offset <=0x20);

    val = s->drego[chan].raw[offset];

    if(s->debug && offset == R_OFF_DEPINT) printf("DREGout int read: %"HWADDR_PRIx ": %"PRIx32" \n", addr<<2, val );


   // printf("DREGOUT read: %"HWADDR_PRIx ": %"PRIx32" \n", addr, val );
    return val;
}

static void STM32F4xx_dregout_write(void *ptr, hwaddr addr, int index,
                                 uint64_t val, unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t chan = index>>3;
    uint32_t offset = index & 0b111;
    assert(chan<=4);
    assert(offset <=0x20);
    //printf("DREGOUT write: %"HWADDR_PRIx ": %"PRIx64" \n", addr, val );
    switch (offset) {
        case R_OFF_DEPCTL: // DIEPCTL
            s->drego[chan].raw[offset] = val;
            s->drego[chan].DOEPCTL.SNAK = 0;
            s->drego[chan].DOEPCTL.CNAK = 0;
            if (s->drego[chan].DOEPCTL.EPENA && s->device_state == DEV_ST_LINECODING){
                s->device_state++;
                STM32F4xx_cdc_schedule(s);
            }
            break;
        case R_OFF_DEPINT: // DOEPINT
            s->drego[chan].raw[offset] &= ~val;
            STM32F4xx_update_device_common_irq(s);
            break;
        default:
            s->drego[chan].raw[offset]= val;
            break;
    }
}


static uint64_t STM32F4xx_dregin_read(void *ptr, hwaddr addr, int index,
                                unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t chan = index>>3;
    uint32_t offset = index & 0b111;
    uint32_t val;
    assert(chan<=4);
    assert(offset <=0x20);

    val = s->dregi[chan].raw[offset];
    switch (offset) {
        case R_OFF_DTXFSTS:
            val = STM32F4xx_EP_FIFO_SIZE - s->fifo_level[chan];
            break;
    }

    //printf("DREGIN read: %"HWADDR_PRIx ": %"PRIx32" \n", addr, val );
    return val;
}

static void STM32F4xx_dregin_write(void *ptr, hwaddr addr, int index,
                                 uint64_t val, unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t chan = index>>3;
    uint32_t offset = index & 0b111;
    assert(chan<=4);
    assert(offset <=0x20);
    //printf("DREGIN write: %"HWADDR_PRIx ": %"PRIx64" \n", addr, val );
    switch (offset) {
        case R_OFF_DEPCTL: // DIEPCTL
            s->dregi[chan].raw[offset] = val;
            s->dregi[chan].DIEPCTL.SNAK = 0;
            s->dregi[chan].DIEPCTL.CNAK = 0;
            if (s->dregi[chan].DIEPCTL.EPENA)
                STM32F4xx_cdc_schedule(s);
            break;
        case R_OFF_DEPINT: // DIEPINT
            s->dregi[chan].raw[offset] &= ~val;
            STM32F4xx_update_device_common_irq(s);
            if ((val & 1U) && s->device_state >= DEV_ST_IO_READY && s->fifo_level[1] == 0) 
            {
            //   printf("Resetting xfcrcmplin\n");
                s->dreg_defs.DAINT.IEPINT &= (~2U);
                STM32F4xx_lower_global_irq(s, GINTSTS_IEPINT);
            }
            break;
        default:
            s->dregi[chan].raw[offset]= val;
            break;
    }

}

static const char *pcgregnm[] = {
        "PCGCTL   ", "PCGCCTL1 "
};

static uint64_t STM32F4xx_pcgreg_read(void *ptr, hwaddr addr, int index,
                                 unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint32_t val;

    assert(addr>>2 >= R_PCGCTL && addr>>2 <= R_PCGCCTL1);
    val = s->pcgreg[index];

    trace_usb_stm_pcgreg_read(addr, pcgregnm[index], val);
    return val;
}

static void STM32F4xx_pcgreg_write(void *ptr, hwaddr addr, int index,
                              uint64_t val, unsigned size)
{
    STM32F4xxUSBState *s = ptr;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t old;

    assert(addr>>2 >= R_PCGCTL && addr>>2 <= R_PCGCCTL1);
    mmio = &s->pcgreg[index];
    old = *mmio;

    trace_usb_stm_pcgreg_write(addr, pcgregnm[index], orig, old, val);
    *mmio = val;
}

static uint64_t STM32F4xx_hsotg_read(void *ptr, hwaddr addr, unsigned size)
{
    uint64_t val;

    switch (addr) {
    case HSOTG_REG(0x000) ... HSOTG_REG(0x0fc):
        val = STM32F4xx_glbreg_read(ptr, addr, (addr - HSOTG_REG(0x000)) >> 2, size);
        break;
    case HSOTG_REG(0x100):
        val = STM32F4xx_fszreg_read(ptr, addr, (addr - HSOTG_REG(0x100)) >> 2, size);
        break;
    case HSOTG_REG(0x104) ... HSOTG_REG(0x3fc):
        /* Gadget-mode registers, just return 0 for now */
        val = 0;
        break;
    case HSOTG_REG(0x400) ... HSOTG_REG(0x4fc):
        val = STM32F4xx_hreg0_read(ptr, addr, (addr - HSOTG_REG(0x400)) >> 2, size);
        break;
    case HSOTG_REG(0x500) ... HSOTG_REG(0x7fc):
        val = STM32F4xx_hreg1_read(ptr, addr, (addr - HSOTG_REG(0x500)) >> 2, size);
        break;
    case HSOTG_REG(0x800) ... HSOTG_REG(0x834):
        return STM32F4xx_dreg0_read(ptr, addr, (addr - HSOTG_REG(0x800)) >> 2, size);
    case HSOTG_REG(0x900) ... HSOTG_REG(0x978):
        return STM32F4xx_dregin_read(ptr, addr, (addr - HSOTG_REG(0x900)) >> 2, size);
    case HSOTG_REG(0xB00) ... HSOTG_REG(0xB70):
        return STM32F4xx_dregout_read(ptr, addr, (addr - HSOTG_REG(0xB00)) >> 2, size);
    case HSOTG_REG(0x980) ... HSOTG_REG(0xAFC):
    case HSOTG_REG(0xB74) ... HSOTG_REG(0xDFC):
        /* Gadget-mode registers, just return 0 for now */
        printf("USBD READ: addr %"HWADDR_PRIx"\n", addr);
        val = 0;
        break;
    case HSOTG_REG(0xe00) ... HSOTG_REG(0xffc):
        val = STM32F4xx_pcgreg_read(ptr, addr, (addr - HSOTG_REG(0xe00)) >> 2, size);
        break;
    default:
        g_assert_not_reached();
    }

    return val;
}

static void STM32F4xx_hsotg_write(void *ptr, hwaddr addr, uint64_t val,
                             unsigned size)
{
    switch (addr) {
    case HSOTG_REG(0x000) ... HSOTG_REG(0x0fc):
        STM32F4xx_glbreg_write(ptr, addr, (addr - HSOTG_REG(0x000)) >> 2, val, size);
        break;
    case HSOTG_REG(0x100):
        STM32F4xx_fszreg_write(ptr, addr, (addr - HSOTG_REG(0x100)) >> 2, val, size);
        break;
    case HSOTG_REG(0x104) ... HSOTG_REG(0x3fc):
        /* Gadget-mode registers, do nothing for now */
        break;
    case HSOTG_REG(0x400) ... HSOTG_REG(0x4fc):
        STM32F4xx_hreg0_write(ptr, addr, (addr - HSOTG_REG(0x400)) >> 2, val, size);
        break;
    case HSOTG_REG(0x500) ... HSOTG_REG(0x7fc):
        STM32F4xx_hreg1_write(ptr, addr, (addr - HSOTG_REG(0x500)) >> 2, val, size);
        break;
    case HSOTG_REG(0x800) ... HSOTG_REG(0x834):
        STM32F4xx_dreg0_write(ptr, addr, (addr - HSOTG_REG(0x800)) >> 2, val, size);
        break;
    case HSOTG_REG(0x900) ... HSOTG_REG(0x978):
        STM32F4xx_dregin_write(ptr, addr, (addr - HSOTG_REG(0x900)) >> 2,  val, size);
        break;
    case HSOTG_REG(0xB00) ... HSOTG_REG(0xB70):
        STM32F4xx_dregout_write(ptr, addr,  (addr - HSOTG_REG(0xB00)) >> 2, val, size);
        break;
    case HSOTG_REG(0x980) ... HSOTG_REG(0xAFC):
    case HSOTG_REG(0xB74) ... HSOTG_REG(0xdfc):
        printf("USBD Write: %"PRIx64" to addr %"HWADDR_PRIx"\n", val, addr);
        /* Gadget-mode registers, do nothing for now */
        break;
    case HSOTG_REG(0xe00) ... HSOTG_REG(0xffc):
        STM32F4xx_pcgreg_write(ptr, addr, (addr - HSOTG_REG(0xe00)) >> 2, val, size);
        break;
    default:
        g_assert_not_reached();
    }
}

static const MemoryRegionOps STM32F4xx_mmio_hsotg_ops = {
    .read = STM32F4xx_hsotg_read,
    .write = STM32F4xx_hsotg_write,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t STM32F4xx_hreg2_read(void *ptr, hwaddr addr, unsigned size)
{
    /* TODO - implement FIFOs to support slave mode */
    STM32F4xxUSBState *s = ptr;
    uint32_t value = 0;
    if (s->rx_fifo_level > 0) {
        value = s->fifo_ram[s->rx_fifo_head++];
        s->rx_fifo_head %= STM32F4xx_RX_FIFO_SIZE;
        s->rx_fifo_level--;
        if (s->rx_fifo_level == 0) {
            s->GINTSTS.RXFLVL = 0;
            STM32F4xx_update_irq(s);
        }
        if (s->debug) printf("FIFO read: %08x (L: %u H: %u T: %u)\n",value, s->rx_fifo_level, s->rx_fifo_head, s->rx_fifo_tail);
    }
    // trace_usb_stm_hreg2_read(addr, addr >> 12, value);
    // qemu_log_mask(LOG_GUEST_ERROR, "FIFO read not implemented\n");
    // printf("FIXME: Read FIFO\n");

    return value;
}

static void STM32F4xx_hreg2_write(void *ptr, hwaddr addr, uint64_t val,
                             unsigned size)
{
    uint64_t orig = val;
    STM32F4xxUSBState *s = ptr;
    uint8_t index = addr >> 12;
    trace_usb_stm_hreg2_write(addr, addr >> 12, orig, s->fifo_tail[index], val);
    if (s->fifo_level[index]==s->gnptxfsiz_txfd) {
        qemu_log_mask(LOG_GUEST_ERROR, "Wrote to a full FIFO (data discarded)!\n");
    }
    s->tx_fifos[index][s->fifo_tail[index]++] = val;
    // %size ensures we automatically loop back around. 
    s->fifo_tail[index] %= STM32F4xx_EP_FIFO_SIZE;
    s->fifo_level[index]++;
    assert(s->fifo_tail[index]<STM32F4xx_EP_FIFO_SIZE); // laziness ftw
    uint32_t bytes_needed;
    int packets_left;
    if (s->GINTSTS.CMOD) {
        packets_left = s->hreg_chan[index].defs.HCTSIZ.PKTCNT;
        bytes_needed =  s->hreg_chan[index].defs.HCTSIZ.XFRSIZ;
    } else {
        packets_left = s->dregi[index].DIEPTSIZ.PKTCNT;
        bytes_needed = s->dregi[index].DIEPTSIZ.XFRSIZ;
    }

    int words_needed = (bytes_needed >> 2);
    if (bytes_needed % 4 !=0)
    {
        words_needed++;
    }
    if ( (s->fifo_level[index]) >= words_needed && packets_left>0) {
        // Enough data written to do a transfer...
        //printf("Data ready for tx on %u\n", index);
        if (s->device_state == DEV_ST_RESET) {
            // NOT CDC hack
            STM32F4xx_tx_packet(s, index<<3);
        } else {
            STM32F4xx_cdc_schedule(s);
        }
    }
}

static const MemoryRegionOps STM32F4xx_mmio_hreg2_ops = {
    .read = STM32F4xx_hreg2_read,
    .write = STM32F4xx_hreg2_write,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void STM32F4xx_wakeup_endpoint(USBBus *bus, USBEndpoint *ep,
                                 unsigned int stream)
{
    STM32F4xxUSBState *s = container_of(bus, STM32F4xxUSBState, bus);

    trace_usb_stm_wakeup_endpoint(ep, stream);

    /* TODO - do something here? */
    qemu_bh_schedule(s->async_bh);
}

static USBBusOps STM32F4xx_bus_ops = {
    .wakeup_endpoint = STM32F4xx_wakeup_endpoint,
};

static void STM32F4xx_work_timer(void *opaque)
{
    STM32F4xxUSBState *s = opaque;

    trace_usb_stm_work_timer();
    qemu_bh_schedule(s->async_bh);
}

static void STM32F4xx_reset_enter(Object *obj, ResetType type)
{
    STM32F4xxClass *c = STM32F4xx_USB_GET_CLASS(obj);
    STM32F4xxUSBState *s = STM32F4xx_USB(obj);
    int i;

    trace_usb_stm_reset_enter();

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

    timer_del(s->frame_timer);
    qemu_bh_cancel(s->async_bh);

    if (s->uport.dev && s->uport.dev->attached) {
        usb_detach(&s->uport);
    }

    STM32F4xx_bus_stop(s);

    s->gotgctl = GOTGCTL_BSESVLD | GOTGCTL_ASESVLD | GOTGCTL_CONID_B;
    s->gotgint = 0;
    s->gahbcfg = 0;
    s->gusbcfg = 5 << GUSBCFG_USBTRDTIM_SHIFT;
    s->grstctl = GRSTCTL_AHBIDLE;
    s->gintsts = 0;
    s->GINTSTS.NPTXFE = 1;
    s->GINTSTS.PTXFE = 1;
    s->GINTSTS.CIDSCHG = 1;
    s->GINTSTS.CMOD = 1;
    s->gintmsk = 0;
    s->grxstsr = 0;
    s->grxstsp = 0;
    s->grxfsiz = 1024;
    s->gnptxfsiz = 1024 << FIFOSIZE_DEPTH_SHIFT;
    s->gnptxsts = (4 << FIFOSIZE_DEPTH_SHIFT) | 1024;
    s->gi2cctl = GI2CCTL_I2CDATSE0 | GI2CCTL_ACK;
    s->gpvndctl = 0;
    s->ggpio = 0;
    s->guid = 0x00001100;
    s->gsnpsid = 0x4f54294a;
    s->ghwcfg1 = 0;
    s->ghwcfg2 = (8 << GHWCFG2_DEV_TOKEN_Q_DEPTH_SHIFT) |
                 (4 << GHWCFG2_HOST_PERIO_TX_Q_DEPTH_SHIFT) |
                 (4 << GHWCFG2_NONPERIO_TX_Q_DEPTH_SHIFT) |
                 GHWCFG2_DYNAMIC_FIFO |
                 GHWCFG2_PERIO_EP_SUPPORTED |
                 ((STM32F4xx_NB_CHAN - 1) << GHWCFG2_NUM_HOST_CHAN_SHIFT) |
                 (GHWCFG2_INT_DMA_ARCH << GHWCFG2_ARCHITECTURE_SHIFT) |
                 (GHWCFG2_OP_MODE_NO_SRP_CAPABLE_HOST << GHWCFG2_OP_MODE_SHIFT);
    s->ghwcfg3 = (4096 << GHWCFG3_DFIFO_DEPTH_SHIFT) |
                 (4 << GHWCFG3_PACKET_SIZE_CNTR_WIDTH_SHIFT) |
                 (4 << GHWCFG3_XFER_SIZE_CNTR_WIDTH_SHIFT);
    s->ghwcfg4 = 0;
    s->glpmcfg = 0;
    s->gpwrdn = GPWRDN_PWRDNRSTN;
    s->gdfifocfg = 0;
    s->gadpctl = 0;
    s->grefclk = 0;
    s->gintmsk2 = 0;
    s->gintsts2 = 0;

    s->hptxfsiz = 500 << FIFOSIZE_DEPTH_SHIFT;

    s->hcfg = 2 << HCFG_RESVALID_SHIFT;
    s->hfir = 60000;
    s->hfnum = 0x3fff;
    s->hptxsts = (16 << TXSTS_QSPCAVAIL_SHIFT) | 32768;
    s->haint = 0;
    s->haintmsk = 0;
    s->hprt0 = 0;

    memset(s->hreg1, 0, sizeof(s->hreg1));
    memset(s->pcgreg, 0, sizeof(s->pcgreg));

    s->sof_time = 0;
    s->frame_number = 0;
    s->fi = USB_FRMINTVL - 1;
    s->next_chan = 0;
    s->working = false;
    s->is_ping = false;

    for (i = 0; i < STM32F4xx_NB_CHAN; i++) {
        s->packet[i].needs_service = false;

    }
    memset(s->tx_fifos, 0, STM32F4xx_NB_CHAN * 4 *KiB);
    memset(s->fifo_ram, 0, sizeof(s->fifo_ram));
    memset(s->fifo_head, 0, sizeof(s->fifo_head));
    memset(s->fifo_level, 0, sizeof(s->fifo_level));
    memset(s->fifo_tail, 0, sizeof(s->fifo_tail));

    memset(s->dreg0, 0, sizeof(s->dreg0));
    memset(s->dregi[0].raw, 0, sizeof(s->dregi));
    memset(s->drego[0].raw, 0, sizeof(s->drego));
    // RX fifo storage is the 2nd half of the 128k... for now. 
    s->rx_fifo_head = 0;
    s->rx_fifo_tail = 0;
    s->rx_fifo_level = 0;

    
}

static void STM32F4xx_reset_hold(Object *obj)
{
    STM32F4xxClass *c = STM32F4xx_USB_GET_CLASS(obj);
    STM32F4xxUSBState *s = STM32F4xx_USB(obj);

    trace_usb_stm_reset_hold();

    if (c->parent_phases.hold) {
        c->parent_phases.hold(obj);
    }

    STM32F4xx_update_irq(s);
}

static void STM32F4xx_reset_exit(Object *obj)
{
    STM32F4xxClass *c = STM32F4xx_USB_GET_CLASS(obj);
    STM32F4xxUSBState *s = STM32F4xx_USB(obj);

    trace_usb_stm_reset_exit();

    if (c->parent_phases.exit) {
        c->parent_phases.exit(obj);
    }

    s->hprt0 = 0;
    s->HPRT.PPWR = 1;
    if (s->uport.dev && s->uport.dev->attached) {
        usb_attach(&s->uport);
        usb_device_reset(s->uport.dev);
    }
}

static void stm32f4xx_usb_reset(void *opaque, int n, int level) {
    // printf("RCC USB reset signal: %d\n",level);
    STM32F4xxUSBState *s = STM32F4xx_USB(opaque);
    if (level) {
        STM32F4xx_reset_enter(OBJECT(s),RESET_TYPE_COLD);
    } else {
        STM32F4xx_reset_exit(OBJECT(s));
    }
}

static void STM32F4xx_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    STM32F4xxUSBState *s = STM32F4xx_USB(dev);
    //Object *obj;

    // obj = object_property_get_link(OBJECT(dev), "dma-mr", &error_abort);

    // s->dma_mr = MEMORY_REGION(obj);
    // I have no idea if this is right, I can't find any docs on where the dma 
    // region for the USB controller lives. 
    // memory_region_add_subregion(&s->container, 0x20000, &s->dma_mr);
    // address_space_init(&s->dma_as, MEMORY_REGION(&s->dma_mr), "STM32F4xxUSBDMA");


    usb_bus_new(&s->bus, sizeof(s->bus), &STM32F4xx_bus_ops, dev);
    usb_register_port(&s->bus, &s->uport, s, 0, &STM32F4xx_port_ops,
                      USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL |
                      (s->usb_version == 2 ? USB_SPEED_MASK_HIGH : 0));
    s->uport.dev = 0;

    s->usb_frame_time = NANOSECONDS_PER_SECOND / 1000;          /* 1000000 */
    if (NANOSECONDS_PER_SECOND >= USB_HZ_FS) {
        s->usb_bit_time = NANOSECONDS_PER_SECOND / USB_HZ_FS;   /* 83.3 */
    } else {
        s->usb_bit_time = 1;
    }

    s->fi = USB_FRMINTVL - 1;
    s->eof_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, STM32F4xx_frame_boundary, s);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, STM32F4xx_work_timer, s);
    s->cdc_timer = timer_new_us(QEMU_CLOCK_VIRTUAL, STM32F4xx_cdc_helper, s);
    s->async_bh = qemu_bh_new(STM32F4xx_work_bh, s);

    qdev_init_gpio_in_named(dev,stm32f4xx_usb_reset,"rcc-reset",1);

    sysbus_init_irq(sbd, &s->irq);

    qemu_chr_fe_set_handlers(&s->cdc, f4xx_usb_cdc_can_receive, f4xx_usb_cdc_receive, NULL,
            NULL,s,NULL,true);
    qemu_chr_fe_set_echo(&s->cdc, true);
}

static void STM32F4xx_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    STM32F4xxUSBState *s = STM32F4xx_USB(obj);

    CHECK_ALIGN(sizeof(s->hreg1), sizeof(s->hreg_chan), "hreg_raw");
    CHECK_ALIGN(sizeof(((hreg_set_t*)0)->defs), sizeof(((hreg_set_t*)0)->raw), "RAW");
    CHECK_REG_u32(s->GINTSTS);
    //CHECK_ALIGN(sizeof(s->dreg0), sizeof(s->dreg_defs), "Common Dev Regs");
    CHECK_REG_u32(s->dreg_defs.DCFG);
    CHECK_REG_u32(s->dreg_defs.DCTL);
    CHECK_REG_u32(s->dreg_defs.DAINT);


    CHECK_TYPEDEF_u32(hreg_set_t,defs.HCCHAR);
    CHECK_TYPEDEF_u32(hreg_set_t,defs.HCSPLT);
    CHECK_TYPEDEF_u32(hreg_set_t,defs.HCINT);
    CHECK_TYPEDEF_u32(hreg_set_t,defs.HCINTMSK);
    CHECK_TYPEDEF_u32(hreg_set_t,defs.HCTSIZ);
    CHECK_TYPEDEF_u32(hreg_set_t,defs.HCDMA);


    memory_region_init(&s->container, obj, "STM32F4xx", STM32F4xx_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->container);

    memory_region_init_io(&s->hsotg, obj, &STM32F4xx_mmio_hsotg_ops, s,
                          "STM32F4xx-io", 4 * KiB);
    memory_region_add_subregion(&s->container, 0x0000, &s->hsotg);

    memory_region_init_io(&s->fifos, obj, &STM32F4xx_mmio_hreg2_ops, s,
                         "STM32F4xx-fifo", 64 * KiB);
    memory_region_add_subregion(&s->container, 0x1000, &s->fifos);

    // memory_region_init_io(&s->dma_mr, obj, &STM32F4xx_mmio_hreg2_ops, s,
    //                       "STM32F4xx-dbg", 0x1FFFF);
    // memory_region_add_subregion(&s->container, 0x20000, &s->dma_mr);
    //memory_region_init_io(&s->dma_mr, obj, NULL, s, "f4xx-usb.dma", 0x1FFFF);

    // printf("Starting USB IP thread...\n");
    // pthread_create(&s->usbip_thread, NULL, usbip_run, NULL);


}

static const VMStateDescription vmstate_STM32F4xx_state_packet = {
    .name = "STM32F4xx/packet",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(devadr, STM32F4xxPacket),
        VMSTATE_UINT32(epnum, STM32F4xxPacket),
        VMSTATE_UINT32(epdir, STM32F4xxPacket),
        VMSTATE_UINT32(mps, STM32F4xxPacket),
        VMSTATE_UINT32(pid, STM32F4xxPacket),
        VMSTATE_UINT32(index, STM32F4xxPacket),
        VMSTATE_UINT32(pcnt, STM32F4xxPacket),
        VMSTATE_UINT32(len, STM32F4xxPacket),
        VMSTATE_INT32(async, STM32F4xxPacket),
        VMSTATE_BOOL(small, STM32F4xxPacket),
        VMSTATE_BOOL(needs_service, STM32F4xxPacket),
        VMSTATE_END_OF_LIST()
    },
};

const VMStateDescription vmstate_STM32F4xx_state = {
    .name = "STM32F4xx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(glbreg, STM32F4xxUSBState,
                             STM32F4xx_GLBREG_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(fszreg, STM32F4xxUSBState,
                             STM32F4xx_FSZREG_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(hreg0, STM32F4xxUSBState,
                             STM32F4xx_HREG0_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(hreg1, STM32F4xxUSBState,
                             STM32F4xx_NB_CHAN*R_HREG_MAX),
        VMSTATE_UINT32_ARRAY(pcgreg, STM32F4xxUSBState,
                             STM32F4xx_PCGREG_SIZE / sizeof(uint32_t)),

        VMSTATE_TIMER_PTR(eof_timer, STM32F4xxUSBState),
        VMSTATE_TIMER_PTR(frame_timer, STM32F4xxUSBState),
        VMSTATE_INT64(sof_time, STM32F4xxUSBState),
        VMSTATE_INT64(usb_frame_time, STM32F4xxUSBState),
        VMSTATE_INT64(usb_bit_time, STM32F4xxUSBState),
        VMSTATE_UINT32(usb_version, STM32F4xxUSBState),
        VMSTATE_UINT16(frame_number, STM32F4xxUSBState),
        VMSTATE_UINT16(fi, STM32F4xxUSBState),
        VMSTATE_UINT16(next_chan, STM32F4xxUSBState),
        VMSTATE_BOOL(working, STM32F4xxUSBState),

        VMSTATE_STRUCT_ARRAY(packet, STM32F4xxUSBState, STM32F4xx_NB_CHAN, 1,
                             vmstate_STM32F4xx_state_packet, STM32F4xxPacket),
        VMSTATE_UINT8_2DARRAY(usb_buf, STM32F4xxUSBState, STM32F4xx_NB_CHAN,
                              STM32F4xx_MAX_XFER_SIZE),
        VMSTATE_UINT32_ARRAY(fifo_ram,STM32F4xxUSBState, STM32F4xx_RX_FIFO_SIZE),
        VMSTATE_UINT32_2DARRAY(tx_fifos,STM32F4xxUSBState,STM32F4xx_NB_CHAN,STM32F4xx_EP_FIFO_SIZE),
        VMSTATE_UINT16_ARRAY(fifo_head,STM32F4xxUSBState, STM32F4xx_NB_CHAN),
        VMSTATE_UINT16_ARRAY(fifo_level,STM32F4xxUSBState, STM32F4xx_NB_CHAN),
        VMSTATE_UINT16_ARRAY(fifo_tail,STM32F4xxUSBState, STM32F4xx_NB_CHAN),
        VMSTATE_UINT32(rx_fifo_head,STM32F4xxUSBState),
        VMSTATE_UINT32(rx_fifo_tail,STM32F4xxUSBState),
        VMSTATE_UINT32(rx_fifo_level,STM32F4xxUSBState),
        VMSTATE_UINT8(is_ping, STM32F4xxUSBState),
        VMSTATE_END_OF_LIST()
    }
};

static Property STM32F4xx_usb_properties[] = {
    DEFINE_PROP_UINT32("usb_version", STM32F4xxUSBState, usb_version, 2),
    DEFINE_PROP_CHR("chardev", STM32F4xxUSBState, cdc),
    DEFINE_PROP_END_OF_LIST(),
};

static void STM32F4xx_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    STM32F4xxClass *c = STM32F4xx_USB_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = STM32F4xx_realize;
    dc->vmsd = &vmstate_STM32F4xx_state;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    device_class_set_props(dc, STM32F4xx_usb_properties);
    resettable_class_set_parent_phases(rc, STM32F4xx_reset_enter, STM32F4xx_reset_hold,
                                       STM32F4xx_reset_exit, &c->parent_phases);
}

static const TypeInfo STM32F4xx_usb_type_info = {
    .name          = TYPE_STM32F4xx_USB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F4xxUSBState),
    .instance_init = STM32F4xx_init,
    .class_size    = sizeof(STM32F4xxClass),
    .class_init    = STM32F4xx_class_init,
};

static void STM32F4xx_usb_register_types(void)
{
    type_register_static(&STM32F4xx_usb_type_info);
}

type_init(STM32F4xx_usb_register_types)
