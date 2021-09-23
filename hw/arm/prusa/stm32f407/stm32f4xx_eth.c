/*
 * STM32F4 ethernet. 
 *
 * derived from the Xilinx AXI-Ethernet by Edgar E. Iglesias.
 * (Based on hw/net/xgmac.c)
 *
 * Copyright (c) 2011 Calxeda, Inc.
 * 
 * Modified for STM32/Mini404 2021 by VintagePC <http://github.com/vintagepc>
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

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "net/checksum.h"
#include "net/eth.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "net/net.h"
#include "hw/net/mii.h"
#include "qom/object.h"
#include "../utility/macros.h"

#ifdef DEBUG_STM32F4XXETH
#define DEBUGF_BRK(message, args...) do { \
                                         fprintf(stderr, (message), ## args); \
                                     } while (0)
#else
#define DEBUGF_BRK(message, args...) do { } while (0)
#endif

#define R_CONTROL           (0x00/4)   /* MAC Configuration */
#define R_FRAME_FILTER      (0x04/4)   /* MAC Frame Filter */
#define R_MACMIIAR          (0x10/4)   /* MII address */
#define R_MACMIIDR          (0x14/4)   /* MII data */

#define R_FLOW_CTRL         (0x18/4)   /* MAC Flow Control */
#define R_VLAN_TAG          (0x1C/4)   /* VLAN Tags */
#define XGMAC_VERSION           0x00000008   /* Version */
/* VLAN tag for insertion or replacement into tx frames */
#define XGMAC_VLAN_INCL         0x00000009
#define XGMAC_LPI_CTRL          0x0000000a   /* LPI Control and Status */
#define XGMAC_LPI_TIMER         0x0000000b   /* LPI Timers Control */
#define XGMAC_TX_PACE           0x0000000c   /* Transmit Pace and Stretch */
#define XGMAC_VLAN_HASH         0x0000000d   /* VLAN Hash Table */
#define R_REMOTE_WAKE       (0x28/4)   /* Remote Wake-Up Frame Filter */
#define R_PMT               (0x2C/4)   /* PMT Control and Status */
#define R_DEBUG             (0x34/4)   /* Debug */
#define R_INT_STATUS        (0x38/4)   /* Interrupt and Control */

// 0x40-0x5c
#define R_ADDR_HIGH(reg)    (0x00000010+((reg) * 2))
#define R_ADDR_LOW(reg)     (0x00000011+((reg) * 2))


/* HASH table registers */
#define XGMAC_HASH(n)           ((0x00000300/4) + (n))
#define XGMAC_NUM_HASH          16
/* Operation Mode */
#define XGMAC_OPMODE            (0x00000400/4)

#define DMA_BUS_MODE            (0x1000/4)   /* Bus Mode */
#define DMA_XMT_POLL_DEMAND     (0x1004/4)   /* Transmit Poll Demand */
#define DMA_RCV_POLL_DEMAND     (0x1008/4)   /* Received Poll Demand */
#define DMA_RCV_BASE_ADDR       (0x100C/4)   /* Receive List Base */
#define DMA_TX_BASE_ADDR        (0x1010/4)   /* Transmit List Base */
#define DMA_STATUS              (0x1014/4)   /* Status Register */
#define DMA_CONTROL             (0x1018/4)   /* Ctrl (Operational Mode) */
#define DMA_INTR_ENA            (0x101C/4)   /* Interrupt Enable */
#define DMA_MISSED_FRAME_CTR    (0x1020/4)   /* Missed Frame Counter */
/* Receive Interrupt Watchdog Timer */
#define DMA_RI_WATCHDOG_TIMER   (0x1024/4)
// #define DMA_AXI_BUS             (0x1000/4)   /* AXI Bus Mode */
// #define DMA_AXI_STATUS          (0x1000/4)   /* AXI Status */
#define DMA_CUR_TX_DESC_ADDR    (0x1048/4)   /* Current Host Tx Descriptor */
#define DMA_CUR_RX_DESC_ADDR    (0x104C/4)   /* Current Host Rx Descriptor */
#define DMA_CUR_TX_BUF_ADDR     (0x1050/4)   /* Current Host Tx Buffer */
#define DMA_CUR_RX_BUF_ADDR     (0x1015/4)   /* Current Host Rx Buffer */
// #define DMA_HW_FEATURE          (0x1000/4)   /* Enabled Hardware Features */

/* DMA Status register defines */
#define DMA_STATUS_GMI          0x08000000   /* MMC interrupt */
#define DMA_STATUS_GLI          0x04000000   /* GMAC Line interface int */
#define DMA_STATUS_EB_MASK      0x00380000   /* Error Bits Mask */
#define DMA_STATUS_EB_TX_ABORT  0x00080000   /* Error Bits - TX Abort */
#define DMA_STATUS_EB_RX_ABORT  0x00100000   /* Error Bits - RX Abort */
#define DMA_STATUS_TS_MASK      0x00700000   /* Transmit Process State */
#define DMA_STATUS_TS_SHIFT     20
#define DMA_STATUS_RS_MASK      0x000e0000   /* Receive Process State */
#define DMA_STATUS_RS_SHIFT     17
#define DMA_STATUS_NIS          0x00010000   /* Normal Interrupt Summary */
#define DMA_STATUS_AIS          0x00008000   /* Abnormal Interrupt Summary */
#define DMA_STATUS_ERI          0x00004000   /* Early Receive Interrupt */
#define DMA_STATUS_FBI          0x00002000   /* Fatal Bus Error Interrupt */
#define DMA_STATUS_ETI          0x00000400   /* Early Transmit Interrupt */
#define DMA_STATUS_RWT          0x00000200   /* Receive Watchdog Timeout */
#define DMA_STATUS_RPS          0x00000100   /* Receive Process Stopped */
#define DMA_STATUS_RU           0x00000080   /* Receive Buffer Unavailable */
#define DMA_STATUS_RI           0x00000040   /* Receive Interrupt */
#define DMA_STATUS_UNF          0x00000020   /* Transmit Underflow */
#define DMA_STATUS_OVF          0x00000010   /* Receive Overflow */
#define DMA_STATUS_TJT          0x00000008   /* Transmit Jabber Timeout */
#define DMA_STATUS_TU           0x00000004   /* Transmit Buffer Unavailable */
#define DMA_STATUS_TPS          0x00000002   /* Transmit Process Stopped */
#define DMA_STATUS_TI           0x00000001   /* Transmit Interrupt */

/* DMA Control register defines */
#define DMA_CONTROL_ST          0x00002000   /* Start/Stop Transmission */
#define DMA_CONTROL_SR          0x00000002   /* Start/Stop Receive */
#define DMA_CONTROL_DFF         0x01000000   /* Disable flush of rx frames */

struct desc {
    uint32_t ctl_stat;
    uint16_t buffer1_size;
    uint16_t buffer2_size;
    uint32_t buffer1_addr;
    uint32_t buffer2_nextdescaddr;
    uint32_t ext_stat;
    uint32_t res[3];
} QEMU_PACKED;

#define R_MAX 0x1400/4

typedef struct RxTxStats {
    uint64_t rx_bytes;
    uint64_t tx_bytes;

    uint64_t rx;
    uint64_t rx_bcast;
    uint64_t rx_mcast;
} RxTxStats;

#define TYPE_STM32F4XXETH "stm32f4xx-ethernet"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32F4xx_Eth, STM32F4XXETH)

struct Stm32F4xx_Eth {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq sbd_irq;
    qemu_irq pmt_irq;
    qemu_irq mci_irq;
    NICState *nic;
    NICConf conf;

    struct RxTxStats stats;
    union {
        uint32_t regs[R_MAX]; // all registers.
        struct {
            uint32_t MACCR;
            uint32_t MACFR;
            uint32_t MACHTHTR;
            uint32_t MACCTLTR;
            struct {
                uint32_t    MB :1;
                uint32_t    MW :1;
                uint32_t    CR :3;
                REG_B32(_unused);
                uint32_t    MR :5;
                uint32_t    PA :5;
                uint32_t    :16;
            } QEMU_PACKED MACMIIAR;
            struct {
                uint32_t    MD :16;
                uint32_t :16; // unused.
            } QEMU_PACKED MACMIIDR;
        } QEMU_PACKED defs;
    };
    // LAN8742 PHY
    uint16_t mii[32];

    bool is_connected;
};

static void stm32f4xx_eth_read_desc(Stm32F4xx_Eth *s, struct desc *d, int rx)
{
    uint32_t addr = rx ? s->regs[DMA_CUR_RX_DESC_ADDR] :
        s->regs[DMA_CUR_TX_DESC_ADDR];
    cpu_physical_memory_read(addr, d, sizeof(*d));
}

static void stm32f4xx_eth_write_desc(Stm32F4xx_Eth *s, struct desc *d, int rx)
{
    int reg = rx ? DMA_CUR_RX_DESC_ADDR : DMA_CUR_TX_DESC_ADDR;
    uint32_t addr = s->regs[reg];
    // Need to write back to the same place, not the next slot for TX!
    if (!rx) cpu_physical_memory_write(addr, d, sizeof(*d));
    
    s->regs[reg] = d->buffer2_nextdescaddr;

    // STM doesn't use these flags ATM, it is set up in "second address chained" mode. 
    // Leaving this here in case it changes with a HAL upgrade.
    // if (!rx && (d->ctl_stat & 0x00200000)) {
    //     s->regs[reg] = s->regs[DMA_TX_BASE_ADDR];
    // } else if (rx && (d->buffer1_size & 0x8000)) {
    //     s->regs[reg] = s->regs[DMA_RCV_BASE_ADDR];
    // } else {
    //     s->regs[reg] += sizeof(*d);
    // }

    if (rx) cpu_physical_memory_write(addr, d, sizeof(*d));
}

static void stm32f4xx_eth_send(Stm32F4xx_Eth *s)
{
    struct desc bd;
    int frame_size;
    int len;
    uint8_t frame[8192];
    uint8_t *ptr;

    ptr = frame;
    frame_size = 0;
    while (1) {
        stm32f4xx_eth_read_desc(s, &bd, 0);
        if ((bd.ctl_stat & 0x80000000) == 0) {
            /* Run out of descriptors to transmit. Ownership of buffer back to cpu.  */
            s->regs[DMA_STATUS] |= DMA_STATUS_TU;
            break;
        }
        len = (bd.buffer1_size & 0xfff) + (bd.buffer2_size & 0xfff);

        /*
         * FIXME: these cases of malformed tx descriptors (bad sizes)
         * should probably be reported back to the guest somehow
         * rather than simply silently stopping processing, but we
         * don't know what the hardware does in this situation.
         * This will only happen for buggy guests anyway.
         */
        if ((bd.buffer1_size & 0xfff) > 2048) {
            DEBUGF_BRK("qemu:%s:ERROR...ERROR...ERROR... -- "
                        "xgmac buffer 1 len on send > 2048 (0x%x)\n",
                         __func__, bd.buffer1_size & 0xfff);
            break;
        }
        if ((bd.buffer2_size & 0xfff) != 0) {
            DEBUGF_BRK("qemu:%s:ERROR...ERROR...ERROR... -- "
                        "xgmac buffer 2 len on send != 0 (0x%x)\n",
                        __func__, bd.buffer2_size & 0xfff);
            break;
        }
        if (frame_size + len >= sizeof(frame)) {
            DEBUGF_BRK("qemu:%s: buffer overflow %d read into %zu "
                        "buffer\n" , __func__, frame_size + len, sizeof(frame));
            DEBUGF_BRK("qemu:%s: buffer1.size=%d; buffer2.size=%d\n",
                        __func__, bd.buffer1_size, bd.buffer2_size);
            break;
        }

        cpu_physical_memory_read(bd.buffer1_addr, ptr, len);
        ptr += len;
        frame_size += len;

        if (bd.ctl_stat & 0x20000000) {
            /* Last buffer in frame.  */
            if ((bd.ctl_stat & 0x00C00000U) ==0x00C00000U) { // HW is expected to do checksum
               net_checksum_calculate(frame, frame_size, CSUM_TCP);  
               eth_fix_ip4_checksum(frame+14,0x14);            
            }

            // printf("Eth TX len %d\n", frame_size);
            qemu_send_packet(qemu_get_queue(s->nic), frame, len);
            ptr = frame;
            frame_size = 0;
            s->regs[DMA_STATUS] |= DMA_STATUS_TI | DMA_STATUS_NIS;
        }
        bd.ctl_stat &= ~0x80000000;
        /* Write back the modified descriptor.  */
        stm32f4xx_eth_write_desc(s, &bd, 0);
    }
}

static void enet_update_irq(Stm32F4xx_Eth *s)
{
    int stat = s->regs[DMA_STATUS] & s->regs[DMA_INTR_ENA];
    qemu_set_irq(s->sbd_irq, !!stat);
}

static uint64_t enet_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32F4xx_Eth *s = opaque;
    uint64_t r = 0;
    addr >>= 2;

    switch (addr) {
    case XGMAC_VERSION:
        r = 0x1012;
        break;
    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            r = s->regs[addr];
        }
        break;
    }
    return r;
}

static void phy_read(Stm32F4xx_Eth *s) {
    s->defs.MACMIIDR.MD = s->mii[s->defs.MACMIIAR.MR];
    // printf("PHY read %02x : %04x\n", s->defs.MACMIIAR.MR, s->defs.MACMIIDR.MD);
}

static void phy_write(Stm32F4xx_Eth *s) {
    // printf("PHY write %02x : %04x\n", s->defs.MACMIIAR.MR, s->defs.MACMIIDR.MD);
    uint16_t data = s->defs.MACMIIDR.MD;
    switch (s->defs.MACMIIAR.MR) 
    {
        case MII_BMCR:
            if (data & MII_BMCR_AUTOEN) {
                s->mii[MII_BMSR] |= (MII_BMSR_AN_COMP | MII_BMSR_10T_FD);
            }
    }
}



static void enet_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned size)
{
    Stm32F4xx_Eth *s = opaque;

    addr >>= 2;

    switch (addr) {
        case R_MACMIIAR:
            s->regs[addr] = value;
            if (s->defs.MACMIIAR.MB) {
                if (s->defs.MACMIIAR.MW) {
                    phy_write(s);
                } else {
                    phy_read(s);
                }
                s->defs.MACMIIAR.MB = false;
            }
            break;
        case R_ADDR_HIGH(0):
            s->conf.macaddr.a[5] = value>>8;
            s->conf.macaddr.a[4] = value;
            break;
        case R_ADDR_LOW(0):
            s->conf.macaddr.a[3] = value >>24;
            s->conf.macaddr.a[2] = value >>16;
            s->conf.macaddr.a[1] = value >>8;
            s->conf.macaddr.a[0] = value;
            qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
            break;
        case DMA_BUS_MODE:
            s->regs[DMA_BUS_MODE] = value & ~0x1;
            break;
        case DMA_XMT_POLL_DEMAND:
            stm32f4xx_eth_send(s);
            break;
        case DMA_STATUS:
            s->regs[DMA_STATUS] = s->regs[DMA_STATUS] & ~value;
            break;
        case DMA_RCV_BASE_ADDR:
            s->regs[DMA_RCV_BASE_ADDR] = s->regs[DMA_CUR_RX_DESC_ADDR] = value;
            // Set TU as this causes the STM HAL to flip it and trigger a poll.
            s->regs[DMA_STATUS] |= DMA_STATUS_TU;
            break;
        case DMA_TX_BASE_ADDR:
            s->regs[DMA_TX_BASE_ADDR] = s->regs[DMA_CUR_TX_DESC_ADDR] = value;
            break;
        default:
           if (addr < ARRAY_SIZE(s->regs)) {
                s->regs[addr] = value;
            }
            break;
    }
    enet_update_irq(s);
}

static const MemoryRegionOps enet_mem_ops = {
    .read = enet_read,
    .write = enet_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static int eth_can_rx(Stm32F4xx_Eth *s)
{
    /* RX enabled?  */
    return s->regs[DMA_CONTROL] & DMA_CONTROL_SR;
}

static ssize_t eth_rx(NetClientState *nc, const uint8_t *buf, size_t size)
{
    Stm32F4xx_Eth *s = qemu_get_nic_opaque(nc);
    static const unsigned char sa_bcast[6] = {0xff, 0xff, 0xff,
                                              0xff, 0xff, 0xff};
    int unicast, broadcast, multicast;
    struct desc bd;
    ssize_t ret;

    if (!eth_can_rx(s)) {
        return -1;
    }
    // printf("Eth RX len %lu\n", size);

    unicast = ~buf[0] & 0x1;
    broadcast = memcmp(buf, sa_bcast, 6) == 0;
    multicast = !unicast && !broadcast;
    if (size < 12) {
        s->regs[DMA_STATUS] |= DMA_STATUS_RI | DMA_STATUS_NIS;
        ret = -1;
        goto out;
    }

    stm32f4xx_eth_read_desc(s, &bd, 1);
    if ((bd.ctl_stat & 0x80000000) == 0) {
        s->regs[DMA_STATUS] |= DMA_STATUS_RU | DMA_STATUS_AIS;
        ret = size;
        goto out;
    }

    cpu_physical_memory_write(bd.buffer1_addr, buf, size);


    /* Add in the 4 bytes for crc (the real hw returns length incl crc) */
    size += 4;
    bd.ctl_stat = (size << 16) | 0x300;
    stm32f4xx_eth_write_desc(s, &bd, 1);

    s->stats.rx_bytes += size;
    s->stats.rx++;
    if (multicast) {
        s->stats.rx_mcast++;
    } else if (broadcast) {
        s->stats.rx_bcast++;
    }

    s->regs[DMA_STATUS] |= DMA_STATUS_RI | DMA_STATUS_NIS;
    ret = size;

out:
    enet_update_irq(s);
    return ret;
}

static NetClientInfo net_stm32f4xx_eth_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = eth_rx,
};

static void stm32f4xx_eth_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Stm32F4xx_Eth *s = STM32F4XXETH(dev);

    CHECK_REG_u32(s->defs.MACMIIAR);
    CHECK_REG_u32(s->defs.MACMIIDR);

    memory_region_init_io(&s->iomem, OBJECT(s), &enet_mem_ops, s,
                          "stm32-eth", 0x1400);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->sbd_irq);
    sysbus_init_irq(sbd, &s->pmt_irq);
    sysbus_init_irq(sbd, &s->mci_irq);
    
    // FIXME - set based on netdev presence!
    if (s->is_connected) {
        s->mii[MII_BMSR] |= MII_BMSR_LINK_ST; // link up. 
    }

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_stm32f4xx_eth_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    s->regs[R_ADDR_HIGH(0)] = (s->conf.macaddr.a[5] << 8) |
                                   s->conf.macaddr.a[4];
    s->regs[R_ADDR_LOW(0)] = (s->conf.macaddr.a[3] << 24) |
                                 (s->conf.macaddr.a[2] << 16) |
                                 (s->conf.macaddr.a[1] << 8) |
                                  s->conf.macaddr.a[0];
}

static Property stm32f4xx_eth_properties[] = {
    DEFINE_NIC_PROPERTIES(Stm32F4xx_Eth, conf),
    DEFINE_PROP_BOOL("connected", Stm32F4xx_Eth, is_connected, false),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_rxtx_stats = {
    .name = "stm32f4xx_eth_stats",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(rx_bytes, RxTxStats),
        VMSTATE_UINT64(tx_bytes, RxTxStats),
        VMSTATE_UINT64(rx, RxTxStats),
        VMSTATE_UINT64(rx_bcast, RxTxStats),
        VMSTATE_UINT64(rx_mcast, RxTxStats),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_stm32f4xx_eth = {
    .name = TYPE_STM32F4XXETH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(stats, Stm32F4xx_Eth, 0, vmstate_rxtx_stats, RxTxStats),
        VMSTATE_UINT32_ARRAY(regs, Stm32F4xx_Eth, R_MAX),
        VMSTATE_UINT16_ARRAY(mii,Stm32F4xx_Eth, 32),
        VMSTATE_BOOL(is_connected, Stm32F4xx_Eth),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f4xx_eth_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f4xx_eth_realize;
    dc->vmsd = &vmstate_stm32f4xx_eth;
    device_class_set_props(dc, stm32f4xx_eth_properties);
}

static const TypeInfo stm32f4xx_eth_info = {
    .name          = TYPE_STM32F4XXETH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32F4xx_Eth),
    .class_init    = stm32f4xx_eth_class_init,
};

static void stm32f4xx_eth_register_types(void)
{
    type_register_static(&stm32f4xx_eth_info);
}

type_init(stm32f4xx_eth_register_types)
