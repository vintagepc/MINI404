/*
 * STM32 Microcontroller UART module
 *
 * Copyright (C) 2010 Andre Beckus
 * Adapted for QEMU 5.2 in 2020 by VintagePC <http://github.com/vintagepc>
 *
 * Source code based on pl011.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "exec/memory.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "chardev/char-fe.h"
#include "chardev/char.h"
#include "stm32.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "stm32_uart.h"
#include "qemu/bitops.h"
#include "assert.h"
#include "../utility/macros.h"



/* DEFINITIONS*/

/* See the README file for details on these settings. */
//#define DEBUG_STM32_UART
#define STM32_UART_NO_BAUD_DELAY 1
//#define STM32_UART_ENABLE_OVERRUN

#ifdef DEBUG_STM32_UART
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32_UART: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define USART_SR_OFFSET     0x00/4
#define USART_DR_OFFSET     0x04/4 
#define USART_BRR_OFFSET    0x08/4
#define USART_CR1_OFFSET    0x0c/4
#define USART_CR2_OFFSET    0x10/4
#define USART_CR3_OFFSET    0x14/4
#define USART_GTPR_OFFSET   0x18/4
#define USART_R_END         0x1c/4

/* HELPER FUNCTIONS */

/* Update the baud rate based on the USART's peripheral clock frequency. */
static void stm32_uart_baud_update(Stm32Uart *s)
{
    uint32_t clk_freq = 84000000;
    if (s->stm32_rcc)
        clk_freq = stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph);

    uint64_t ns_per_bit;

    if((s->regs[USART_BRR_OFFSET] == 0) || (clk_freq == 0)) {
        s->bits_per_sec = 0;
    } else {
        s->bits_per_sec = clk_freq / s->regs[USART_BRR_OFFSET];
        ns_per_bit = 1000000000LL / s->bits_per_sec;

        /* We assume 10 bits per character.  This may not be exactly
         * accurate depending on settings, but it should be good enough. */
        s->ns_per_char = ns_per_bit * 10;
    }

#ifdef DEBUG_STM32_UART
    const char *periph_name = s->busdev.parent_obj.id;
    DPRINTF("%s clock is set to %lu Hz.\n",
                periph_name,
                (unsigned long)clk_freq);
    DPRINTF("%s BRR set to %lu.\n",
                periph_name,
                (unsigned long)s->USART_BRR);
    DPRINTF("%s Baud is set to %lu bits per sec.\n",
                periph_name,
                (unsigned long)s->bits_per_sec);
#endif
}

/* Handle a change in the peripheral clock. */
static void stm32_uart_clk_irq_handler(void *opaque, int n, int level)
{
    Stm32Uart *s = STM32_UART(opaque);

    assert(n == 0);

    /* Only update the BAUD rate if the IRQ is being set. */
    if(level) {
        stm32_uart_baud_update(s);
    }
}

/* Routine which updates the USART's IRQ.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32_uart_update_irq(Stm32Uart *s) {
    /* Note that we are not checking the ORE flag, but we should be. */
    int new_irq_level =
       (s->defs.CR1.TCIE & s->defs.SR.TC) |
       (s->defs.CR1.IDLEIE & s->defs.SR.IDLE) | // IDLE int check.
       (s->defs.CR1.TXEIE & s->defs.SR.TXE) |
       (s->defs.CR1.RXNEIE &
               (s->defs.SR.ORE | s->defs.SR.RXNE));

    /* Only trigger an interrupt if the IRQ level changes.  We probably could
     * set the level regardless, but we will just check for good measure.
     */
    if(new_irq_level ^ s->curr_irq_level) {
        qemu_set_irq(s->irq, new_irq_level);
        s->curr_irq_level = new_irq_level;
    }
    int new_dmar = s->defs.SR.RXNE && (s->defs.CR3.DMAR);
    if (new_dmar != s->dmar_current_level || new_dmar==1)
    {
        s->dmar_current_level = new_dmar;
        qemu_set_irq(s->dmar,new_dmar);
    }
}


static void stm32_uart_start_tx(Stm32Uart *s, uint32_t value);

static int stm32_uart_can_receive(void *opaque);

static void stm32_uart_receive(void *opaque, const uint8_t *buf, int size);

/* Routine to be called when a transmit is complete. */
static void stm32_uart_tx_complete(Stm32Uart *s)
{
    if(s->defs.SR.TXE == 1) {
        /* If the buffer is empty, there is nothing waiting to be transmitted.
         * Mark the transmit complete. */
        s->defs.SR.TC = 1;
        stm32_uart_update_irq(s);
    } else {
        /* Otherwise, mark the transmit buffer as empty and
         * start transmitting the value stored there.
         */
        s->defs.SR.TXE = 1;
        stm32_uart_update_irq(s);
        stm32_uart_start_tx(s, s->USART_TDR);
  
    }
}

/* Start transmitting a character. */
static void stm32_uart_start_tx(Stm32Uart *s, uint32_t value)
{
    uint8_t ch = value; //This will truncate the ninth bit

    /* Reset the Transmission Complete flag to indicate a transmit is in
     * progress.
     */
    s->defs.SR.TC = 0;
    /* Write the character out. */
    uint8_t chcr = '\r';
    if (ch == '\n') qemu_chr_fe_write_all(&s->chr, &chcr, 1);
    qemu_chr_fe_write_all(&s->chr, &ch, 1);
    qemu_set_irq(s->byte_out, ch);
    // if (s->chr_write_obj) {
        // s->chr_write(s->chr_write_obj, &ch, 1);
    // }
#ifdef STM32_UART_NO_BAUD_DELAY
    /* If BAUD delays are not being simulated, then immediately mark the
     * transmission as complete.
     */
    stm32_uart_tx_complete(s);
#else
    /* Otherwise, start the transmit delay timer. */
    timer_mod(s->tx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
#endif
}


/* Put byte into the receive data register, if we have one and the target is 
 * ready for it. */
static void stm32_uart_fill_receive_data_register(Stm32Uart *s)
{
    bool enabled = (s->defs.CR1.UE && s->defs.CR1.RE);

    /* If we have no more data, or we are emulating baud delay and it's not 
     * time yet for the next byte, return without filling the RDR */
    if (!s->rcv_char_bytes || s->receiving) {
        return;
    }

#ifndef STM32_UART_ENABLE_OVERRUN
    /* If overrun is not enabled, don't overwrite the current byte in the RDR */
    if (enabled && s->defs.SR.RXNE) {
        return;
    }
#endif

    /* Pull the byte out of our buffer */
    uint8_t byte = s->rcv_char_buf[0];
    memmove(&s->rcv_char_buf[0], &s->rcv_char_buf[1], --(s->rcv_char_bytes));

    /* Only handle the received character if the module is enabled, */
    if (enabled) {
        if(s->defs.SR.RXNE) {
            DPRINTF("stm32_uart_receive: overrun error\n");
            s->defs.SR.ORE = 1;
            s->sr_read_since_ore_set = false;
            stm32_uart_update_irq(s);
        }

        /* Receive the character and mark the buffer as not empty. */
        s->defs.DR.DR = byte;
        s->defs.SR.RXNE = 1;
        // Unblock this IRQ
        s->idle_interrupt_blocked = false;
        // Clear DMAR flag so the irq gets re-raised.
        stm32_uart_update_irq(s);
    }

#ifndef STM32_UART_NO_BAUD_DELAY
    /* Indicate the module is receiving and start the delay. */
    s->receiving = true;
    timer_mod(s->rx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
#endif
}



/* TIMER HANDLERS */
/* Once the receive delay is finished, indicate the USART is finished receiving.
 * This will allow it to receive the next character.  The current character was
 * already received before starting the delay.
 */
static void stm32_uart_rx_timer_expire(void *opaque) {
    Stm32Uart *s = (Stm32Uart *)opaque;

    s->receiving = false;

    /* Put next byte into the receive data register, if we have one ready */
    stm32_uart_fill_receive_data_register(s);
}

/* When the transmit delay is complete, mark the transmit as complete
 * (the character was already sent before starting the delay). */
static void stm32_uart_tx_timer_expire(void *opaque) {
    Stm32Uart *s = (Stm32Uart *)opaque;

    stm32_uart_tx_complete(s);
}

static void stm32_uart_idle_timer_expire(void *opaque) {
    Stm32Uart *s = (Stm32Uart *)opaque;

    if (s->idle_interrupt_blocked) return;

    // if(!s->defs.SR.IDLE) printf("IDLE SET\n");
    s->defs.SR.IDLE = 1;
    stm32_uart_update_irq(s);
}

/* CHAR DEVICE HANDLERS */

static int stm32_uart_can_receive(void *opaque)
{
    // Note - while more than 1 byte at a time can work in theory
    // for some reason this results in the string being
    // reversed if the receiving end is configured with DMA (e.g ESP01). 
    // For now, just have 1 char at a time and we'll deal with it
    // when it becomes a bottleneck.
    // Stm32Uart *s = (Stm32Uart *)opaque;

    /* How much space do we have in our buffer? */
    return 1;// (USART_RCV_BUF_LEN - s->rcv_char_bytes);
}

static void stm32_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    assert(size > 0);
    /* Copy the characters into our buffer first */
    // if (s->periph==19) {
    //     printf("UART RX: ");
    //     for (int i=0; i<size;i++)
    //         printf("%c",buf[i]);
    //     printf("\n");
    // }
    assert (size <= USART_RCV_BUF_LEN - s->rcv_char_bytes);
    memmove(s->rcv_char_buf + s->rcv_char_bytes, buf, size);
    s->rcv_char_bytes += size;

    /* Put next byte into RDR if the target is ready for it */
    stm32_uart_fill_receive_data_register(s);
    //  if (s->periph==19) printf("DR: %c\n", s->defs.DR.DR);

    //start no-receive idle timer.
    timer_mod(s->idle_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
}

/* REGISTER IMPLEMENTATION */

static void stm32_uart_USART_DR_read(Stm32Uart *s, uint8_t *data_read)
{
    /* If the Overflow flag is set, then it should be cleared if the software
     * performs an SR read followed by a DR read.
     */

    if(s->defs.SR.ORE) {
        if(s->sr_read_since_ore_set) {
            s->defs.SR.ORE = 0;
        }
    }
    if (s->sr_read_since_idle_set) {
        s->defs.SR.IDLE = 0;
        // if (s->periph == 19) printf("IDLE cleared\n");
        s->sr_read_since_idle_set = false;
    }

    if(!s->defs.CR1.UE) {
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to read from USART_DR while UART was disabled.\n");
    }

    if(!s->defs.CR1.RE) {
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to read from USART_DR while UART receiver was disabled.\n");
    }
    if(s->defs.SR.RXNE) {
        /* If the receive buffer is not empty, return the value. and mark the
         * buffer as empty.
         */
        // printf("DR read: %02x\n", *read_value);

        s->defs.SR.RXNE = 0;

        *data_read = s->defs.DR.DR;
        /* Put next character into the RDR if we have one */
        stm32_uart_fill_receive_data_register(s);
    } else {
        // Not sure if this is a sim bug or actual HW behaviour, DMAR always seems to overread one byte. 
        // Doesn't actually cause any problems I've observed, but leaving a note here for future reference.
        if(!s->defs.CR3.DMAR) printf("STM32_UART WARNING: Read value from USART_DR (%08"HWADDR_PRIx") while it was empty.\n", s->iomem.addr);
        s->defs.DR.DR = 0; // Clear value.
    }

    qemu_chr_fe_accept_input(&s->chr);
    stm32_uart_update_irq(s);
}


static void stm32_uart_USART_DR_write(Stm32Uart *s, uint32_t new_value)
{
    uint32_t write_value = new_value & 0x000001ff;
    //printf("uart %d: wr: %02x\n",s->uart_index, write_value);

    if(!s->defs.CR1.UE) {
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write to USART_DR while UART was disabled.");
    }

    if(!s->defs.CR1.TE) {
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write to USART_DR while UART transmitter "
                 "was disabled.");
    }

    if(s->defs.SR.TC) {
        /* If the Transmission Complete bit is set, it means the USART is not
         * currently transmitting.  This means, a transmission can immediately
         * start.
         */
        stm32_uart_start_tx(s, write_value);
    } else {
        /* Otherwise check to see if the buffer is empty.
         * If it is, then store the new character there and mark it as not empty.
         * If it is not empty, trigger a hardware error.  Software should check
         * to make sure it is empty before writing to the Data Register.
         */
        if(s->defs.SR.TXE) {
            s->USART_TDR = write_value;
            s->defs.SR.TXE = 0;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,"Wrote new value to USART_DR while it was non-empty.\n");
        }
    }

    stm32_uart_update_irq(s);
}

static void stm32_uart_reset(DeviceState *dev)
{
    Stm32Uart *s = STM32_UART(dev);

    /* Initialize the status registers.  These are mostly
     * read-only, so we do not call the "write" routine
     * like normal.
     */
    for (int i=0; i<7; i++) {
        s->regs[i] = 0;
    }
    s->defs.SR.TC = 1;
    s->defs.SR.TXE = 1;
    s->dmar_current_level = -1;

    s->sr_read_since_idle_set = false;
    s->idle_interrupt_blocked = false;

    s->curr_irq_level = 0;

    // Do not initialize USART_DR - it is documented as undefined at reset
    // and does not behave like normal registers.
    //stm32_uart_USART_BRR_write(s, 0x00000000, true);

    stm32_uart_update_irq(s);
}

static uint64_t stm32_uart_read(void *opaque, hwaddr addr,
                          unsigned size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;
    int offset = addr & 0x3;
    addr >>= 2;

    if (addr >= USART_R_END) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read usart register 0x%x\n",
          (unsigned int)addr << 2);
        DPRINTF("  %s: result: 0\n", __func__);
        return 0;
    }
    switch (addr) {
        case USART_SR_OFFSET:
            if(s->defs.SR.ORE) {
                s->sr_read_since_ore_set = true;
            }
            if (s->defs.SR.IDLE) {
                s->sr_read_since_idle_set = true;
            }
            qemu_chr_fe_accept_input(&s->chr);
            break;
        case USART_DR_OFFSET:
            {
                uint8_t value = 0;
                stm32_uart_USART_DR_read(s, &value);
                return value;
            }
            break;
        default:  
            s->sr_read_since_idle_set = false;
    }

    uint32_t value = s->regs[addr];

    value = (value >> offset * 8) & ((1ull << (8 * size)) - 1);

    DPRINTF("%s: addr: 0x%llx, size: %u, value: 0x%x\n", __func__, addr, size, value);
    return value;

}

static void stm32_uart_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;
    int offset = addr & 0x3;
    uint32_t data = 0;
    DPRINTF("%s: addr: 0x%llx, data: 0x%x, size: %u\n", __func__, addr, data, size);

    addr >>= 2;
    if (addr >= USART_R_END) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f2xx usart register 0x%x\n",
            (unsigned int)addr << 2);
        return;
    }

    switch(size) {
    case 1:
        data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
        break;
    case 2:
        data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        data = value;
        break;
    default:
        abort();
    }

    if (s->stm32_rcc)
        stm32_rcc_check_periph_clk(s->stm32_rcc, s->periph);

    switch (addr) {
        case USART_SR_OFFSET:
            // Check write mask to see if guest tried to set a clear-only bit
            if ((value & ~(s->regs[USART_SR_OFFSET])) & 0x360) {
                qemu_log_mask(LOG_GUEST_ERROR, "USART - guest attempted to set a clear-only SR bit with value %0" HWADDR_PRIx "\n", value);
                value |= 0x360;
            }
            s->regs[addr] &= value;
            stm32_uart_update_irq(s);
            break;
        case USART_DR_OFFSET:
            stm32_uart_USART_DR_write(s,value);
            break;
        case USART_BRR_OFFSET:
            s->regs[addr] = data;
            stm32_uart_baud_update(s);
            break;
        case USART_CR1_OFFSET:
            s->regs[addr] = data;
            stm32_uart_update_irq(s);
            break;
        default:
            s->regs[addr] = data;
            break;
    }

}

static const MemoryRegionOps stm32_uart_ops = {
    .read = stm32_uart_read,
    .write = stm32_uart_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_uart_rx_wrapper(void *opaque, int n, int level)
{
    uint8_t buff = level;
    if (stm32_uart_can_receive(opaque))
    {
        stm32_uart_receive(opaque, &buff, 1);
    } else {
        printf("FIXME - uart RX full! cannot receive!\n");
    }
}

/* DEVICE INITIALIZATION */

static void stm32_uart_rcc_reset(void *opaque, int n, int level) {
    if (!level) {
        stm32_uart_reset(DEVICE(opaque));
    }
}


static void stm32_uart_init(Object *obj)
{
    Stm32Uart *s = STM32_UART(obj);

    // s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, obj,  &stm32_uart_ops, s,
                          "uart", 0x03ff);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    qdev_init_gpio_in_named(DEVICE(obj),stm32_uart_rx_wrapper,"uart-byte-in",1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->byte_out,"uart-byte-out",1);
    qdev_init_gpio_out_named(DEVICE(obj),&s->dmar,"uart-dmar",1);

    s->rx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_uart_rx_timer_expire, s);
    s->tx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_uart_tx_timer_expire, s);
    s->idle_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_uart_idle_timer_expire, s);

    /* Register handlers to handle updates to the USART's peripheral clock. */
    s->clk_irq =
          qemu_allocate_irqs(stm32_uart_clk_irq_handler, (void *)s, 1);
    if (s->stm32_rcc)
        stm32_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, s->clk_irq[0]);

    //stm32_uart_connect(s, &s->chr);

    s->rcv_char_bytes = 0;

    qdev_init_gpio_in_named(DEVICE(obj),stm32_uart_rcc_reset,"rcc-reset",1);

    stm32_uart_reset(DEVICE(obj));
}

static void stm32_uart_realize(DeviceState *dev, Error **errp)
{
    Stm32Uart *s = STM32_UART(dev);
    qemu_chr_fe_set_handlers(&s->chr, stm32_uart_can_receive, stm32_uart_receive, NULL,
            NULL,s,NULL,true);
    qemu_chr_fe_set_echo(&s->chr, true);
    // Throw compile errors if alignment is off
    CHECK_ALIGN(sizeof(s->defs), sizeof(s->regs), "USART");
    CHECK_ALIGN(sizeof(uint32_t)*7, sizeof(s->regs), "USART");
    CHECK_REG_u32(s->defs.SR);
    CHECK_REG_u32(s->defs.DR);
    CHECK_REG_u32(s->defs.BRR);
    CHECK_REG_u32(s->defs.CR1);
    CHECK_REG_u32(s->defs.CR2);
    CHECK_REG_u32(s->defs.CR3);
    CHECK_REG_u32(s->defs.GTPR);
}

static Property stm32_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", Stm32Uart, chr),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32_uart = {
    .name = TYPE_STM32_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Stm32Uart,USART_R_END),
        VMSTATE_INT32(periph,Stm32Uart),
        VMSTATE_UINT32(bits_per_sec,Stm32Uart),
        VMSTATE_INT64(ns_per_char,Stm32Uart),
        VMSTATE_UINT32(USART_TDR,Stm32Uart),
        VMSTATE_BOOL(sr_read_since_ore_set,Stm32Uart),
        VMSTATE_BOOL(receiving,Stm32Uart),
        VMSTATE_TIMER_PTR(rx_timer,Stm32Uart),
        VMSTATE_TIMER_PTR(tx_timer,Stm32Uart),
        VMSTATE_INT32(curr_irq_level,Stm32Uart),
        VMSTATE_INT32(dmar_current_level,Stm32Uart),
        VMSTATE_UINT8_ARRAY(rcv_char_buf,Stm32Uart,USART_RCV_BUF_LEN),
        VMSTATE_UINT32(rcv_char_bytes,Stm32Uart),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32_uart_reset;
    device_class_set_props(dc, stm32_uart_properties);
    dc->realize = stm32_uart_realize;
    dc->vmsd = &vmstate_stm32_uart;
}

static TypeInfo stm32_uart_info = {
    .name  = TYPE_STM32_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Uart),
    .class_init = stm32_uart_class_init,
    .instance_init = stm32_uart_init
};

static void stm32_uart_register_types(void)
{
    type_register_static(&stm32_uart_info);
}

type_init(stm32_uart_register_types)
