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


#include "chardev/char-fe.h"

#define USART_RCV_BUF_LEN 256

#define TYPE_STM32UART "stm32-uart"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32Uart, STM32UART)

struct Stm32Rcc;

struct Stm32Uart {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    // void *stm32_rcc_prop;

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;

    int uart_index;

    uint8_t reply_count;
    uint32_t reply;
    bool is_read;

    stm32_periph_t periph;

    uint32_t bits_per_sec;
    int64_t ns_per_char;

    /* Register Values */
    uint32_t
        USART_RDR,
        USART_TDR,
        USART_BRR,
        USART_CR1,
        USART_CR2,
        USART_CR3;

    /* Register Field Values */
    uint32_t
        USART_SR_TXE,
        USART_SR_TC,
        USART_SR_RXNE,
        USART_SR_ORE,
        USART_CR1_UE,
        USART_CR1_TXEIE,
        USART_CR1_TCIE,
        USART_CR1_RXNEIE,
        USART_CR1_TE,
        USART_CR1_RE;

    bool sr_read_since_ore_set;

    /* Indicates whether the USART is currently receiving a byte. */
    bool receiving;

    /* Timers used to simulate a delay corresponding to the baud rate. */
    struct QEMUTimer *rx_timer;
    struct QEMUTimer *tx_timer;

    void *chr_write_obj;
    int (*chr_write)(void *chr_write_obj, const uint8_t *buf, int len);

    qemu_irq irq;
    int curr_irq_level;

    // Byte IRQ for connected peripherals to avoid the charbackend complexity. 
    qemu_irq byte_out; 

    qemu_irq dmar;
    int dmar_current_level;

    /* We buffer the characters we receive from our qemu_chr receive handler in here
     * to increase our overall throughput. This allows us to tell the target that
     * another character is ready immediately after it does a read.
     */
    uint8_t rcv_char_buf[USART_RCV_BUF_LEN];
    uint32_t rcv_char_bytes;    /* number of bytes avaialable in rcv_char_buf */

    CharBackend chr;
};

void stm32_uart_connect(Stm32Uart *s, CharBackend *chr);

#endif // STM32_UART_H
