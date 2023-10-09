/*
    stm32_usart.c - USART for STM32.
	So far this USART *layout* is common to the following chips:
	- F030 series,
	- G070 series, (with additional registers)

	Portions based on the original F4xx module by Andre Beckus
	Copyright 2021-3 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "exec/memory.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "chardev/char-fe.h"
#include "chardev/char.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "qemu/bitops.h"
#include "assert.h"
#include "stm32_rcc_if.h"
#include "stm32_usart_regdata.h"

/* DEFINITIONS*/

/* See the README file for details on these settings. */
//#define DEBUG_STM32_UART
//#define STM32_UART_NO_BAUD_DELAY 1
//#define STM32_UART_ENABLE_OVERRUN

#ifdef DEBUG_STM32_UART
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32_UART: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define USART_RCV_BUF_LEN 256

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Usart), COM_CLASS_NAME(Usart), STM32COM_USART)

struct Stm32Rcc;

REGDEF_BLOCK_BEGIN()
	REG_B32(UE);
	REG_B32(UESM);
	REG_B32(RE);
	REG_B32(TE);
	REG_B32(IDLEIE);
	REG_B32(RXNEIE);
	REG_B32(TCIE);
	REG_B32(TXEIE);
	REG_B32(PEIE);
	REG_B32(PS);
	REG_B32(PCE);
	REG_B32(WAKE);
	REG_B32(M0);
	REG_B32(MME);
	REG_B32(CMIE);
	REG_B32(OVER8);
	REG_K32(DEDT,5);
	REG_K32(DEAT,5);
	REG_B32(RTOIE);
	REG_B32(EOBIE);
	REG_B32(M1);
	REG_B32(FIFOEN);
	REG_B32(TXFEIE);
	REG_B32(RXFEIE);
REGDEF_BLOCK_END(usart, cr1);

REGDEF_BLOCK_BEGIN()
	REG_R(4);
	REG_B32(ADDM7);
	REG_R(3);
	REG_B32(LBCL);
	REG_B32(CPHA);
	REG_B32(CPOL);
	REG_B32(CLKEN);
	REG_K32(STOP,2);
	REG_R(1);
	REG_B32(SWAP);
	REG_B32(RXINV);
	REG_B32(TXINV);
	REG_B32(DATAINV);
	REG_B32(MSBFIRST);
	REG_B32(ABREN);
	REG_K32(ABRMOD,2);
	REG_B32(RTOEN);
	REG_K32(ADDL,4);
	REG_K32(ADDH,4);
REGDEF_BLOCK_END(usart, cr2);

REGDEF_BLOCK_BEGIN()
	REG_B32(EIE);
	REG_R(2);
	REG_B32(HDSEL);
	REG_R(2);
	REG_B32(DMAR);
	REG_B32(DMAT);
	REG_B32(RTSE);
	REG_B32(CTSE);
	REG_B32(CTSIE);
	REG_B32(ONEBIT);
	REG_B32(OVRDIS);
	REG_B32(DDRE);
	REG_B32(DEM);
	REG_B32(DEP);
	REG_R(16);
REGDEF_BLOCK_END(usart, cr3);

REGDEF_BLOCK_BEGIN()
	REG_K32(FRACT, 4);
	REG_K32(MANT, 12);
	REG_R(16);
REGDEF_BLOCK_END(usart, brr);

REGDEF_BLOCK_BEGIN()
	REG_K32(PSC, 8);
	REG_K32(GT, 8);
	REG_R(16);
REGDEF_BLOCK_END(usart, gtpr);

REGDEF_BLOCK_BEGIN()
	REG_K32(RTO, 24);
	REG_K32(BLEN, 8);
REGDEF_BLOCK_END(usart, rtor);

REGDEF_BLOCK_BEGIN()
	REG_B32(ABRQ);
	REG_B32(SBKRQ);
	REG_B32(MMRQ);
	REG_B32(RXFRQ);
	REG_B32(TXFRQ);
	REG_R(27);
REGDEF_BLOCK_END(usart, rqr);

REGDEF_BLOCK_BEGIN()
	REG_B32(PE);
	REG_B32(FE);
	REG_B32(NF);
	REG_B32(ORE);
	REG_B32(IDLE);
	REG_B32(RXNE);
	REG_B32(TC);
	REG_B32(TXE);
	REG_B32(LBDF);
	REG_B32(CTSIF);
	REG_B32(CTS);
	REG_B32(RTOF);
	REG_B32(EOBF);
	REG_B32(UDR);
	REG_B32(ABRE);
	REG_B32(ABRF);
	REG_B32(BUSY);
	REG_B32(CMF);
	REG_B32(SBKF);
	REG_B32(RWU);
	REG_B32(WUF);
	REG_B32(TEACK);
	REG_B32(REACK);
	REG_B32(TXFE);
	REG_B32(RXFE);
	REG_B32(TCBGT);
	REG_B32(RXFT);
	REG_B32(TXFT);
	REG_R(4);
REGDEF_BLOCK_END(usart, interrupt);

typedef struct COM_STRUCT_NAME(Usart) {
    /* Inherited */
    STM32Peripheral parent;

    /* Private */
    MemoryRegion iomem;

    union {
        struct {
            REGDEF_NAME(usart, cr1) CR1; // 0x00
			REGDEF_NAME(usart, cr2) CR2; // 0x04
			REGDEF_NAME(usart, cr3) CR3; // 0x08
			REGDEF_NAME(usart, brr) BRR; // 0x0C
			REGDEF_NAME(usart, gtpr) GTPR; // 0x10
			REGDEF_NAME(usart, rtor) RTOR;	// 0x14
			REGDEF_NAME(usart,rqr) RQR; // 0x18
			REGDEF_NAME(usart, interrupt) ISR; // 0x1C
			REGDEF_NAME(usart, interrupt) ICR; // 0x20
			REG_S32(RDR, 9);
			REG_S32(TDR,9); // 0x28
			REG_S32(PRESCALE,4); // 0x2c
        } QEMU_PACKED defs; // 0x28
		uint32_t raw[RI_END];
    } regs;

    uint32_t bits_per_sec;
    int64_t ns_per_char;

    bool sr_read_since_ore_set, sr_read_since_idle_set;

    // Used to prevent repeated idles unless a new RXNE happens, per datasheet.
    bool idle_interrupt_blocked;

    /* Indicates whether the USART is currently receiving a byte. */
    bool receiving, last_rto;
	uint16_t count;

    /* Timers used to simulate a delay corresponding to the baud rate. */
    struct QEMUTimer* rx_timer;
    struct QEMUTimer* tx_timer;
    struct QEMUTimer* idle_timer;
    struct QEMUTimer* rto_timer;

    void *chr_write_obj;
    int (*chr_write)(void *chr_write_obj, const uint8_t *buf, int len);

	bool transmitting;

    qemu_irq irq;
    int curr_irq_level;

	qemu_irq byte_out;

	bool do_rs485;
	bool debug_rs485;

	uint8_t rs485_msg[USART_RCV_BUF_LEN];
	uint8_t rs485_index;
	uint8_t rs485_dest;

	uint8_t rs485_in[USART_RCV_BUF_LEN];
	uint8_t rs485_in_i;

    /* We buffer the characters we receive from our qemu_chr receive handler in here
     * to increase our overall throughput. This allows us to tell the target that
     * another character is ready immediately after it does a read.
     */
    uint8_t rcv_char_buf[USART_RCV_BUF_LEN];
    uint32_t rcv_char_bytes;    /* number of bytes avaialable in rcv_char_buf */

	char* prefix;
	bool shift;

    CharBackend chr;

	stm32_reginfo_t* reginfo;

} COM_STRUCT_NAME(Usart);

typedef struct COM_CLASS_NAME(Usart) {
	STM32PeripheralClass parent_class;
    stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Usart);

static const stm32_reginfo_t stm32f030_usart_reginfo[RI_END] =
{
	[RI_CR1] = {.mask = 0x17FFFFFD, .unimp_mask = 0xFFFFFF00},
	[RI_CR2] = {.mask = 0xFFFFBF10, .unimp_mask = 0xFF7FBF10},
	[RI_CR3] = {.mask = 0xFFC9 },
	[RI_BRR] = {.mask = UINT16_MAX},
	[RI_GTPR] = {.is_reserved = true},
	[RI_RTOR] = {.mask = 0xFFFFFF, .unimp_mask = 0xFF000000 },
	[RI_RQR] = {.mask = 0xF, .unimp_mask = 0xF},
	[RI_ISR] = {.mask = 0xFCEFF, .reset_val = 0xC0 },
	[RI_ICR] = {.mask = 0x20A5F },
	[RI_RDR ... RI_TDR] = {.mask = 0x1FF},
	[RI_PRESC] = {.is_reserved = true},
};

static const stm32_reginfo_t stm32g070_usart_reginfo[RI_END] =
{
	[RI_CR1] = {.mask = 0x17FFFFFD, .unimp_mask = 0xFFFFFF00},
	[RI_CR2] = {.mask = 0xFFFFBF10, .unimp_mask = 0xFF7FBF10},
	[RI_CR3] = {.mask = 0xFFC9 },
	[RI_BRR] = {.mask = UINT16_MAX},
	[RI_GTPR] = {.is_reserved = true},
	[RI_RTOR] = {.mask = 0xFFFFFF, .unimp_mask = 0xFF000000 },
	[RI_RQR] = {.mask = 0xF, .unimp_mask = 0xF},
	[RI_ISR] = {.mask = 0xFCEFF, .reset_val = 0xC0 },
	[RI_ICR] = {.mask = 0x20A5F },
	[RI_RDR ... RI_TDR] = {.mask = 0x1FF},
	[RI_PRESC] = {.mask = 0xF, .unimp_mask = 0xF},
};

static const stm32_periph_variant_t stm32_usart_variants[] = {
	{TYPE_STM32F030_USART, stm32f030_usart_reginfo},
	{TYPE_STM32G070_USART, stm32g070_usart_reginfo},
};

static const uint8_t MAX_PRESCALE = 0b1011;
static const uint16_t PRESCALE_DIV[] = {
	1U, 2U, 4U, 6U, 8U, 10U, 12U, 16U, 32U, 64U, 128U, 256U
};

/* HELPER FUNCTIONS */

/* Update the baud rate based on the USART's peripheral clock frequency. */
static void stm32_common_usart_baud_update(COM_STRUCT_NAME(Usart) *s)
{
    uint32_t clk_freq = s->parent.clock_freq;

    uint64_t ns_per_bit;

	clk_freq /= PRESCALE_DIV[s->regs.defs.PRESCALE];

    if((s->regs.defs.BRR.raw == 0) || (clk_freq == 0)) {
        s->bits_per_sec = 0;
    } else {
		float scale = s->regs.defs.CR1.OVER8 ? 8.f : 16.f;
		float clk_div = scale * (s->regs.defs.BRR.MANT + (float)s->regs.defs.BRR.FRACT/scale);
        s->bits_per_sec = clk_freq / clk_div;
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

// /* Handle a change in the peripheral clock. */
// static void stm32_common_usart_clk_irq_handler(void *opaque, int n, int level)
// {
//     COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);

//     assert(n == 0);

//     /* Only update the BAUD rate if the IRQ is being set. */
//     if(level) {
//         stm32_common_usart_baud_update(s);
//     }
// }

/* Routine which updates the USART's IRQ.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32_common_usart_update_irq(COM_STRUCT_NAME(Usart) *s) {
    /* Note that we are not checking the ORE flag, but we should be. */
    int new_irq_level =
       (s->regs.defs.CR1.TCIE & s->regs.defs.ISR.TC) |
	   (s->regs.defs.CR1.RTOIE & s->regs.defs.ISR.RTOF) |  // RTOF
       (s->regs.defs.CR1.IDLEIE & s->regs.defs.ISR.IDLE) | // IDLE int check.
       (s->regs.defs.CR1.TXEIE & s->regs.defs.ISR.TXE) |
       (s->regs.defs.CR1.RXNEIE &
               (s->regs.defs.ISR.ORE | s->regs.defs.ISR.RXNE));

    /* Only trigger an interrupt if the IRQ level changes.  We probably could
     * set the level regardless, but we will just check for good measure.
     */
    if(new_irq_level ^ s->curr_irq_level) {
        qemu_set_irq(s->irq, new_irq_level);
        s->curr_irq_level = new_irq_level;
    }
    if (s->regs.defs.ISR.RXNE && (s->regs.defs.CR3.DMAR))
    {
        qemu_set_irq(s->parent.dmar[DMAR_P2M],s->iomem.addr + (4U*RI_RDR));
    }
	if (s->regs.defs.ISR.TXE && (s->regs.defs.CR3.DMAT))
    {
        qemu_set_irq(s->parent.dmar[DMAR_M2P],s->iomem.addr + (4U*RI_TDR));
    }
}


static void stm32_common_usart_start_tx(COM_STRUCT_NAME(Usart) *s, uint32_t value);

static int stm32_common_usart_can_receive(void *opaque);

static void stm32_common_usart_receive(void *opaque, const uint8_t *buf, int size);

/* Routine to be called when a transmit is complete. */
static void stm32_common_usart_tx_complete(COM_STRUCT_NAME(Usart) *s)
{
    if(s->regs.defs.ISR.TXE == 1) {
        /* If the buffer is empty, there is nothing waiting to be transmitted.
         * Mark the transmit complete. */
        s->regs.defs.ISR.TC = 1;
		s->transmitting = false;
        stm32_common_usart_update_irq(s);
    } else {
        /* Otherwise, mark the transmit buffer as empty and
         * start transmitting the value stored there.
         */
        s->regs.defs.ISR.TXE = 1;
        stm32_common_usart_update_irq(s);
        stm32_common_usart_start_tx(s, s->regs.defs.TDR);

    }
}

/* Start transmitting a character. */
static void stm32_common_usart_start_tx(COM_STRUCT_NAME(Usart) *s, uint32_t value)
{
    uint8_t ch = value; //This will truncate the ninth bit

	qemu_set_irq(s->byte_out, ch);
    /* Reset the Transmission Complete flag to indicate a transmit is in
     * progress.
     */
    s->regs.defs.ISR.TC = 0;
	s->transmitting = true;
	// uint8_t chcr = '\r';
    /* Write the character out. */
	if (s->do_rs485) // LCOV_EXCL_START
	{
		s->rs485_in[s->rs485_in_i++] = ch;
    	if (s->rs485_in_i>2 && s->rs485_in_i==s->rs485_in[2])
		{
			// done, print message.
			// printf("RS485 Tx: ");
			// for (int i=0; i<s->rs485_in_i; i++)
			// {
			// 	printf("%02x ", s->rs485_in[i]);
			// }
			// printf("\n");
			qemu_chr_fe_write_all(&s->chr, &s->rs485_in[3], s->rs485_in_i-7);
			s->rs485_in_i = 0;
		}
	}
	else // LCOV_EXCL_END
	{
    	//if (ch == '\n') qemu_chr_fe_write_all(&s->chr, &chcr, 1);
		if (s->debug_rs485) // LCOV_EXCL_START
		{
			printf("RS485 TX: %02x (%c)\n", ch, ch);
		} // LCOV_EXCL_STOP
    	qemu_chr_fe_write_all(&s->chr, &ch, 1);
	}
    // if (s->chr_write_obj) {
        // s->chr_write(s->chr_write_obj, &ch, 1);
    // }
#ifdef STM32_UART_NO_BAUD_DELAY
    /* If BAUD delays are not being simulated, then immediately mark the
     * transmission as complete.
     */
    stm32_common_usart_tx_complete(s);
#else
    /* Otherwise, start the transmit delay timer. */
    timer_mod(s->tx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
#endif
}


/* Put byte into the receive data register, if we have one and the target is
 * ready for it. */
static void stm32_common_usart_fill_receive_data_register(COM_STRUCT_NAME(Usart) *s)
{
    bool enabled = (s->regs.defs.CR1.UE && s->regs.defs.CR1.RE);

    /* If we have no more data, or we are emulating baud delay and it's not
     * time yet for the next byte, return without filling the RDR */
    if (!s->rcv_char_bytes || s->receiving) {
        return;
    }

#ifndef STM32_UART_ENABLE_OVERRUN
    /* If overrun is not enabled, don't overwrite the current byte in the RDR */
    if (enabled && s->regs.defs.ISR.RXNE) {
        return;
    }
#endif

    /* Pull the byte out of our buffer */
    uint8_t byte = s->rcv_char_buf[0];
    memmove(&s->rcv_char_buf[0], &s->rcv_char_buf[1], --(s->rcv_char_bytes));

	if (s->regs.defs.CR2.RTOEN && s->rcv_char_bytes == 0) // If no more data, tickle the timeout timers.
	{
    	timer_mod(s->idle_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
		timer_mod(s->rto_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (s->ns_per_char * (s->regs.defs.RTOR.RTO)));
	}

    /* Only handle the received character if the module is enabled, */
    if (enabled) {
        if(s->regs.defs.ISR.RXNE) {
            DPRINTF("stm32_common_usart_receive: overrun error\n");
            s->regs.defs.ISR.ORE = 1;
            s->sr_read_since_ore_set = false;
            stm32_common_usart_update_irq(s);
        }

        /* Receive the character and mark the buffer as not empty. */
        s->regs.defs.RDR = byte;
        s->regs.defs.ISR.RXNE = 1;
        // Unblock this IRQ
        s->idle_interrupt_blocked = false;
        // Clear DMAR flag so the irq gets re-raised.
        stm32_common_usart_update_irq(s);
#ifndef STM32_UART_NO_BAUD_DELAY
		/* Indicate the module is receiving and start the delay. */
		s->receiving = true;
		timer_mod(s->rx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
#endif
    }
}



/* TIMER HANDLERS */
/* Once the receive delay is finished, indicate the USART is finished receiving.
 * This will allow it to receive the next character.  The current character was
 * already received before starting the delay.
 */
static void stm32_common_usart_rx_timer_expire(void *opaque) {
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);

    s->receiving = false;

    /* Put next byte into the receive data register, if we have one ready */
    stm32_common_usart_fill_receive_data_register(s);
}

/* When the transmit delay is complete, mark the transmit as complete
 * (the character was already sent before starting the delay). */
static void stm32_common_usart_tx_timer_expire(void *opaque) {
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);

    stm32_common_usart_tx_complete(s);
}

static void stm32_common_usart_idle_timer_expire(void *opaque) {
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);

    if (s->idle_interrupt_blocked) return;

    // if(!s->regs.defs.ISR.IDLE) printf("IDLE SET\n");
    s->regs.defs.ISR.IDLE = 1;
    stm32_common_usart_update_irq(s);
}

static void stm32_common_usart_rto_timer_expire(void *opaque) {
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);

	if (s->rcv_char_bytes)
	{
		return;
	}
    s->regs.defs.ISR.RTOF = 1;
    stm32_common_usart_update_irq(s);
}

/* CHAR DEVICE HANDLERS */

static int stm32_common_usart_can_receive(void *opaque)
{
    // Note - while more than 1 byte at a time can work in theory
    // for some reason this results in the string being
    // reversed if the receiving end is configured with DMA (e.g ESP01).
    // For now, just have 1 char at a time and we'll deal with it
    // when it becomes a bottleneck.
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);

	// if (!s->regs.defs.CR1.RE)
	// {
	// 	return 0; // Can't receive if disabled
	// }

    /* How much space do we have in our buffer? */
    return (USART_RCV_BUF_LEN - s->rcv_char_bytes);
}

// LCOV_EXCL_START
/* Table of CRC values for high-order byte */
static char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static char auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

static uint32_t stm32_common_usart_calccrc(uint8_t *msg, uint16_t len) {
    uint32_t CRC_hi = 0xFF;
    uint32_t CRC_lo = 0xFF;
    uint32_t index;
    while (len--) {
        index = CRC_lo ^ *msg++;
        CRC_lo = (CRC_hi ^ auchCRCHi[index&0xFF])&0xFF;
        CRC_hi = (auchCRCLo[index&0xFF])&0xFF;
    }
    return (CRC_hi << 8 | CRC_lo);
}

static bool stm32_common_usart_rs485(COM_STRUCT_NAME(Usart) *s, const uint8_t *buf, int size)
{
	if (buf[0]!='\n')
	{
		s->rs485_msg[s->rs485_index++] = *buf;
		return false;
	}
	else // EOL, construct RS485
	{
		s->rcv_char_buf[0] = s->rs485_dest;
		s->rcv_char_buf[1] = 0x00;
		s->rcv_char_bytes = s->rs485_index + 3;
		s->rcv_char_buf[2] = s->rcv_char_bytes+4;
		memmove(s->rcv_char_buf+3, s->rs485_msg, s->rs485_index);
		uint32_t crc = GUINT32_SWAP_LE_BE(stm32_common_usart_calccrc(&s->rcv_char_buf[0], s->rs485_index+3));
		memmove(s->rcv_char_buf+s->rcv_char_bytes, (uint8_t*)&crc, 4);
		s->rcv_char_bytes +=4;
		s->rs485_index = 0;
		printf("RS485 Rx: ");
		for (int i=0; i<s->rcv_char_bytes; i++)
		{
			printf("%02x ", s->rcv_char_buf[i]);
		}
		printf("\n");
		return true;
	}
}
// LCOV_EXCL_STOP


static void stm32_common_usart_receive(void *opaque, const uint8_t *buf, int size)
{
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);
	timer_del(s->rto_timer); // Cancel pending RTO
	timer_del(s->idle_timer);
    assert(size > 0);
    /* Copy the characters into our buffer first */
    assert (size <= USART_RCV_BUF_LEN - s->rcv_char_bytes);
	if (s->debug_rs485) // LCOV_EXCL_START
	{
		printf("RS485 RX: %02x (%c)\n", *buf, *buf);
	}
	if (s->do_rs485)
	{
		if (!stm32_common_usart_rs485(s, buf, size))
		{
			return; // message not complete.
		}
	} // LCOV_EXCL_STOP
	else
	{
    	memmove(s->rcv_char_buf + s->rcv_char_bytes, buf, size);
    	s->rcv_char_bytes += size;
	}

    /* Put next byte into RDR if the target is ready for it */
    stm32_common_usart_fill_receive_data_register(s);
    //  if (s->periph==19) printf("DR: %c\n", s->regs.defs.DR.DR);

}

/* REGISTER IMPLEMENTATION */

static void stm32_common_usart_USART_DR_read(COM_STRUCT_NAME(Usart) *s, uint8_t *data_read)
{
    /* If the Overflow flag is set, then it should be cleared if the software
     * performs an SR read followed by a DR read.
     */

    if(s->regs.defs.ISR.ORE) {
        if(s->sr_read_since_ore_set) {
            s->regs.defs.ISR.ORE = 0;
        }
    }
    if (s->sr_read_since_idle_set) {
        s->regs.defs.ISR.IDLE = 0;
        // if (s->periph == 19) printf("IDLE cleared\n");
        s->sr_read_since_idle_set = false;
    }

    if(!s->regs.defs.CR1.UE) {
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to read from USART_DR while UART was disabled.\n");
    }

    if(!s->regs.defs.CR1.RE) {
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to read from USART_DR while UART receiver was disabled.\n");
    }
    if(s->regs.defs.ISR.RXNE) {
        /* If the receive buffer is not empty, return the value. and mark the
         * buffer as empty.
         */
        // if (s->parent.periph == STM32_P_UART1) printf("DR read: %02x\n", s->regs.defs.RDR);

        s->regs.defs.ISR.RXNE = 0;

        *data_read = s->regs.defs.RDR;
        /* Put next character into the RDR if we have one */
        stm32_common_usart_fill_receive_data_register(s);
    } else {
        // Not sure if this is a sim bug or actual HW behaviour, DMAR always seems to overread one byte.
        // This isn't actually a bug, some of the STM32 driver code will flush DR regardless of whether it has data or not
		// e.g. __HAL_UART_CLEAR_OREFLAG
        // if(!s->regs.defs.CR3.DMAR) printf("STM32_UART WARNING: Read value from USART_DR (%08"HWADDR_PRIx") while it was empty.\n", s->iomem.addr);
        s->regs.defs.RDR = 0; // Clear value.
    }

    qemu_chr_fe_accept_input(&s->chr);
    stm32_common_usart_update_irq(s);
}


static void stm32_common_usart_USART_DR_write(COM_STRUCT_NAME(Usart) *s, uint32_t new_value)
{
    uint32_t write_value = new_value & 0x000001ff;

    if(!s->regs.defs.CR1.UE) {
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write to USART_DR while UART was disabled.");
		return;
    }

    if(!s->regs.defs.CR1.TE) {
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write to USART_DR while UART transmitter "
                 "was disabled.");
		return;
    }

    if(s->regs.defs.ISR.TC || !s->transmitting) {
        /* If the Transmission Complete bit is set, it means the USART is not
         * currently transmitting.  This means, a transmission can immediately
         * start. Note the guest might clear this flag hence the internal state logical OR
         */
        stm32_common_usart_start_tx(s, write_value);
    } else {
        /* Otherwise check to see if the buffer is empty.
         * If it is, then store the new character there and mark it as not empty.
         * If it is not empty, trigger a hardware error.  Software should check
         * to make sure it is empty before writing to the Data Register.
         */
        if(s->regs.defs.ISR.TXE) {
            s->regs.defs.TDR = write_value;
            s->regs.defs.ISR.TXE = 0;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,"Wrote new value to USART_DR while it was non-empty.\n");
        }
    }

    stm32_common_usart_update_irq(s);
}

static void stm32_common_usart_reset(DeviceState *dev)
{
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(dev);

    /* Initialize the status registers.  These are mostly
     * read-only, so we do not call the "write" routine
     * like normal.
     */
	for (int i=0;i<RI_END; i++)
	{
		s->regs.raw[i] = s->reginfo[i].reset_val;
	}

	s->transmitting = false;
    s->sr_read_since_idle_set = false;
    s->idle_interrupt_blocked = false;

    s->curr_irq_level = 0;

	s->rcv_char_bytes = 0; // clear the buffer.

    // Do not initialize USART_DR - it is documented as undefined at reset
    // and does not behave like normal registers.
    //stm32_common_usart_USART_BRR_write(s, 0x00000000, true);

    stm32_common_usart_update_irq(s);
}

static uint64_t stm32_common_usart_read(void *opaque, hwaddr addr,
                          unsigned size)
{
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);
    int offset = addr & 0x3;
    addr >>= 2;

    CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "STM32Common USART");
    switch (addr) {
        case RI_ISR:
            if(s->regs.defs.ISR.ORE) {
                s->sr_read_since_ore_set = true;
            }
            if (s->regs.defs.ISR.IDLE) {
                s->sr_read_since_idle_set = true;
            }
            qemu_chr_fe_accept_input(&s->chr);
            break;
        case RI_RDR:
            {
                uint8_t value = 0;
                stm32_common_usart_USART_DR_read(s, &value);
                return value;
            }
            break;
        default:
            s->sr_read_since_idle_set = false;
    }

    uint32_t value = s->regs.raw[addr];

    ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b111);

    DPRINTF("%s: addr: 0x%llx, size: %u, value: 0x%x\n", __func__, addr, size, value);
    return value;

}

static void stm32_common_usart_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned size)
{
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(opaque);
    int offset = addr & 0x3;
    DPRINTF("%s: addr: 0x%llx, data: 0x%x, size: %u\n", __func__, addr, data, size);

    addr >>= 2;

	CHECK_BOUNDS_W(addr, value, RI_END, s->reginfo, "STM32 COmmon USART");

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], value, size, offset, 0b111);

	CHECK_UNIMP_RESVD(value, s->reginfo, addr);

	stm32_rcc_if_check_periph_clk(&s->parent);

    switch (addr) {
        case RI_ISR:
			qemu_log_mask(LOG_GUEST_ERROR, "USART - guest attempted to write read-only ISR register");
			break;
        case RI_TDR:
            stm32_common_usart_USART_DR_write(s,value);
            break;
        case RI_BRR:
            s->regs.raw[addr] = value;
            stm32_common_usart_baud_update(s);
            break;
        case RI_CR1:
            s->regs.raw[addr] = value;
			s->regs.defs.ISR.TEACK = s->regs.defs.CR1.TE;
			s->regs.defs.ISR.REACK = s->regs.defs.CR1.RE;
            stm32_common_usart_update_irq(s);
            break;
		case RI_ICR:
			s->regs.raw[RI_ISR] &= ~value;
			stm32_common_usart_update_irq(s);
			break;
		case RI_CR3:
			s->regs.raw[addr] = value;
			stm32_common_usart_update_irq(s);
			break;
		case RI_PRESC:
			if (value > MAX_PRESCALE)
			{
				qemu_log_mask(LOG_GUEST_ERROR,"%s PRESCALE value %u out of valid range 0-%u\n",_PERIPHNAMES[s->parent.periph], s->regs.defs.PRESCALE, MAX_PRESCALE);
				value = MAX_PRESCALE;
			}
            s->regs.raw[addr] = value;
			stm32_common_usart_baud_update(s);
			break;
        default:
            s->regs.raw[addr] = value;
            break;
    }

}

static const MemoryRegionOps stm32_common_usart_ops = {
    .read = stm32_common_usart_read,
    .write = stm32_common_usart_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};

/* DEVICE INITIALIZATION */

static void stm32_common_usart_byte_in(void *opaque, int n, int level) {
	uint8_t c = level& 0xFF;
	stm32_common_usart_receive(opaque, &c, 1);
}

static void stm32_common_usart_init(Object *obj)
{
    //qemu_irq *clk_irq;
    COM_STRUCT_NAME(Usart) *s = STM32COM_USART(obj);
	COM_CLASS_NAME(Usart) *k = STM32COM_USART_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;

    // s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    STM32_MR_IO_INIT(&s->iomem, obj,  &stm32_common_usart_ops, s, 1U*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    s->rx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_common_usart_rx_timer_expire, s);
    s->tx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_common_usart_tx_timer_expire, s);
    s->idle_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_common_usart_idle_timer_expire, s);

	s->rto_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_common_usart_rto_timer_expire, s);

    /* Register handlers to handle updates to the USART's peripheral clock. */
    //clk_irq =
      //    qemu_allocate_irqs(stm32_common_usart_clk_irq_handler, (void *)s, 1);
    // if (s->stm32_rcc)
    //     stm32_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, clk_irq[0]);

    //stm32_common_usart_connect(s, &s->chr);

    s->rcv_char_bytes = 0;

	s->rs485_index=0;
	//s->do_rs485 = true;

    qdev_init_gpio_in_named(DEVICE(obj),stm32_common_usart_byte_in,"byte-in",1);
	qdev_init_gpio_out_named(DEVICE(obj),&s->byte_out,"byte-out",1);

    stm32_common_usart_reset(DEVICE(obj));

	// Throw compile errors if alignment is off
    CHECK_ALIGN(sizeof(s->regs.defs), sizeof(s->regs.raw), "USART");
    CHECK_ALIGN(sizeof(uint32_t)*RI_END, sizeof(s->regs.raw), "USART");
    CHECK_REG_u32(s->regs.defs.ISR);
    CHECK_REG_u32(s->regs.defs.CR1);
    CHECK_REG_u32(s->regs.defs.CR2);
    CHECK_REG_u32(s->regs.defs.CR3);
    CHECK_REG_u32(s->regs.defs.RQR);
}

static void stm32_common_usart_realize(DeviceState *dev, Error **errp)
{
	COM_STRUCT_NAME(Usart) *s = STM32COM_USART(dev);

    qemu_chr_fe_set_handlers(&s->chr, stm32_common_usart_can_receive, stm32_common_usart_receive, NULL,
            NULL,s,NULL,true);
    qemu_chr_fe_set_echo(&s->chr, true);
    // No symlink support for these.
#if !defined(__CYGWIN__) && !defined(__MINGW32__) && !defined(__MINGW64__)
    if (CHARDEV_IS_PTY(s->chr.chr)) {
        char link_path[] = "/tmp/stmf0-uart0";
        link_path[15] += s->parent.periph - STM32_P_UART1;
		if (s->shift)
			link_path[15] += 5;
        unlink(link_path);
        if (symlink(s->chr.chr->filename+4, link_path) != 0)
        {
            printf("WARN: Can't create %s (%s)\n",link_path, strerror(errno));
        }
        else
        {
            printf("%s now points to: %s\n",link_path, s->chr.chr->filename);
        }
    }
#endif
}

static Property stm32_common_usart_properties[] = {
    DEFINE_PROP_CHR("chardev", COM_STRUCT_NAME(Usart), chr),
	DEFINE_PROP_STRING("prefix", COM_STRUCT_NAME(Usart), prefix),
	DEFINE_PROP_BOOL("do_rs485", COM_STRUCT_NAME(Usart), do_rs485, false),
	DEFINE_PROP_UINT8("rs485_dest", COM_STRUCT_NAME(Usart), rs485_dest, 0),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32_common_usart = {
    .name = TYPE_STM32COM_USART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw, COM_STRUCT_NAME(Usart),RI_END),
        VMSTATE_UINT32(bits_per_sec,COM_STRUCT_NAME(Usart)),
        VMSTATE_INT64(ns_per_char,COM_STRUCT_NAME(Usart)),
        VMSTATE_BOOL(sr_read_since_ore_set,COM_STRUCT_NAME(Usart)),
        VMSTATE_BOOL(receiving,COM_STRUCT_NAME(Usart)),
        VMSTATE_TIMER_PTR(rx_timer,COM_STRUCT_NAME(Usart)),
        VMSTATE_TIMER_PTR(tx_timer,COM_STRUCT_NAME(Usart)),
        VMSTATE_INT32(curr_irq_level,COM_STRUCT_NAME(Usart)),
        VMSTATE_UINT8_ARRAY(rcv_char_buf,COM_STRUCT_NAME(Usart),USART_RCV_BUF_LEN),
        VMSTATE_UINT32(rcv_char_bytes,COM_STRUCT_NAME(Usart)),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32_common_usart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32_common_usart_reset;
    device_class_set_props(dc, stm32_common_usart_properties);
    dc->realize = stm32_common_usart_realize;
    dc->vmsd = &vmstate_stm32_common_usart;

	COM_CLASS_NAME(Usart) *k = STM32COM_USART_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
}


static TypeInfo stm32_common_usart_info = {
    .name  = TYPE_STM32COM_USART,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size  = sizeof(COM_STRUCT_NAME(Usart)),
	.class_size = sizeof(COM_CLASS_NAME(Usart)),
	.abstract = true,
};

static void stm32_common_usart_register_types(void)
{
    type_register_static(&stm32_common_usart_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_usart_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_usart_variants[i].variant_name,
            .parent     = TYPE_STM32COM_USART,
			.instance_init = stm32_common_usart_init,
    		.class_init    = stm32_common_usart_class_init,
            .class_data = (void *)stm32_usart_variants[i].variant_regs,
        };
        type_register(&ti);
    }
}

type_init(stm32_common_usart_register_types)
