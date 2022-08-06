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

#ifndef STM32_UART_H
#define STM32_UART_H

#include "../stm32_common/stm32_shared.h"
#include "../stm32_common/stm32_common.h"
#include "../stm32_common/stm32_rcc_if.h"
#include "chardev/char-fe.h"

#define USART_RCV_BUF_LEN 256

#define TYPE_STM32UART "stm32-uart"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32Uart, STM32UART)

struct Stm32Rcc;

struct Stm32Uart {
    /* Inherited */
    STM32Peripheral parent;

    /* Private */
    MemoryRegion iomem;

    union {
        uint32_t regs[7];
        struct {
            struct {
                uint32_t PE     :1;
                uint32_t FE     :1;
                uint32_t NF     :1;
                uint32_t ORE    :1;
                uint32_t IDLE   :1;
                uint32_t RXNE   :1;
                uint32_t TC     :1;
                uint32_t TXE    :1;
                uint32_t LBD    :1;
                uint32_t CTS    :1;
                uint32_t :22;
            } QEMU_PACKED SR;
            struct {
                uint32_t DR :8;
                uint32_t :24;
            } QEMU_PACKED DR; // Note - this is the R DR. TXDR is separate.
            struct {
                uint32_t FRACT  :4;
                uint32_t MANT   :12;
                uint32_t :16;
            } QEMU_PACKED BRR;
            struct {
                uint32_t SBK    :1;
                uint32_t RWU    :1;
                uint32_t RE     :1;
                uint32_t TE     :1;
                uint32_t IDLEIE :1;
                uint32_t RXNEIE :1;
                uint32_t TCIE   :1;
                uint32_t TXEIE   :1;
                uint32_t PEIE   :1;
                uint32_t PS     :1;
                uint32_t PCE    :1;
                uint32_t WAKE   :1;
                uint32_t M      :1;
                uint32_t UE     :1;
                uint32_t _reserved :1;
                uint32_t OVER8  :1;
                uint16_t :16;
            } QEMU_PACKED CR1;
            struct {
                uint32_t ADD    :4;
                uint32_t _res   :1;
                uint32_t LBDL   :1;
                uint32_t LBDIE  :1;
                uint32_t _res1  :1;
                uint32_t LBCL   :1;
                uint32_t CPHA   :1;
                uint32_t CPOL   :1;
                uint32_t CLKEN  :1;
                uint32_t STOP   :2;
                uint32_t LINEN  :1;
                uint32_t :17;
            } QEMU_PACKED CR2;
            struct {
                uint32_t EIE    :1;
                uint32_t IREN   :1;
                uint32_t IRLP   :1;
                uint32_t HDSEL  :1;
                uint32_t NACK   :1;
                uint32_t SCEN   :1;
                uint32_t DMAR   :1;
                uint32_t DMAT   :1;
                uint32_t RTSE   :1;
                uint32_t CTSE   :1;
                uint32_t CTSIE  :1;
                uint32_t CNEBI  :1;
                uint32_t :20;
            } QEMU_PACKED CR3;
            struct {
                uint32_t PSC    :8;
                uint32_t GT     :8;
                uint32_t :16;
            } QEMU_PACKED GTPR;
        } QEMU_PACKED defs;
    };


    uint32_t bits_per_sec;
    int64_t ns_per_char;

    /* Register Values */
    uint32_t USART_TDR;

    bool sr_read_since_ore_set, sr_read_since_idle_set;


    // Used to prevent repeated idles unless a new RXNE happens, per datasheet.
    bool idle_interrupt_blocked;

    /* Indicates whether the USART is currently receiving a byte. */
    bool receiving;

    /* Timers used to simulate a delay corresponding to the baud rate. */
    struct QEMUTimer *rx_timer;
    struct QEMUTimer *tx_timer;
    struct QEMUTimer *idle_timer;

    void *chr_write_obj;
    int (*chr_write)(void *chr_write_obj, const uint8_t *buf, int len);

    qemu_irq irq;
    int curr_irq_level;

    // Byte IRQ for connected peripherals to avoid the charbackend complexity.
    qemu_irq byte_out;

    /* We buffer the characters we receive from our qemu_chr receive handler in here
     * to increase our overall throughput. This allows us to tell the target that
     * another character is ready immediately after it does a read.
     */
    uint8_t rcv_char_buf[USART_RCV_BUF_LEN];
    uint32_t rcv_char_bytes;    /* number of bytes avaialable in rcv_char_buf */

    CharBackend chr;

    qemu_irq *clk_irq;
};

void stm32_uart_connect(Stm32Uart *s, CharBackend *chr);

#endif // STM32_UART_H
