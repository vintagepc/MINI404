/*
 * STM32 Common I2C
 *
 * Copyright (c) 2024-6 VintagePC <github.com/vinagepc>
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "../utility/macros.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_common/stm32_common.h"
#include "stm32_i2c_regdata.h"
#include "trace.h"

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(I2c), COM_CLASS_NAME(I2c), STM32COM_I2C);

// Convenience IRQ bits:
enum IRQ_bits {
	IRQ_TXE = BIT(0),
	IRQ_TXI = BIT(1),
	IRQ_RXNE = BIT(2),
	IRQ_ADDR = BIT(3),
	IRQ_NACKF = BIT(4),
	IRQ_STOPF = BIT(5),
	IRQ_TC = BIT(6),
	IRQ_TCR = BIT(7),
	IRQ_BERR = BIT(8),
	IRQ_ARLO = BIT(9),
	IRQ_OVR = BIT(10),
	IRQ_PECERR = BIT(11),
	IRQ_TIMEOUT = BIT(12),
	IRQ_ALERT = BIT(13),
	IRQ_BUSY = BIT(15),
	IRQ_EVT = IRQ_RXNE | IRQ_TXI | IRQ_STOPF | IRQ_TCR | IRQ_TC | IRQ_ADDR | IRQ_NACKF,
	IRQ_ERR = IRQ_BERR | IRQ_ARLO | IRQ_OVR | IRQ_PECERR | IRQ_TIMEOUT | IRQ_ALERT,
	IRQ_CR1_MSK = IRQ_TXI | IRQ_RXNE | IRQ_ADDR | IRQ_NACKF | IRQ_STOPF | IRQ_TC,
	IRQ_ALL = IRQ_EVT | IRQ_ERR,
};

#include "../stm32_registers/generated/stm32c092/I2C_reginfo.h"
#include "../stm32_registers/generated/stm32f030/I2C_reginfo.h"
#include "../stm32_registers/generated/stm32g070/I2C_reginfo.h"
#include "../stm32_registers/generated/stm32h503/I2C_reginfo.h"


static const stm32_periph_variant_t stm32_i2c_variants[] = {
	{TYPE_STM32C092_I2C, stm32_c092_i2c_reginfo},
	{TYPE_STM32F030_I2C, stm32_f030_i2c_reginfo},
	{TYPE_STM32G070_I2C, stm32_g070_i2c_reginfo},
	{TYPE_STM32H503_I2C, stm32_h503_i2c_reginfo}
};

typedef union {
	struct {
		uint32_t PE            : 1; // /*!< Peripheral enable */
		uint32_t TXIE          : 1; // /*!< TX interrupt enable */
		uint32_t RXIE          : 1; // /*!< RX interrupt enable */
		uint32_t ADDRIE        : 1; // /*!< Address match interrupt enable */
		uint32_t NACKIE        : 1; // /*!< NACK received interrupt enable */
		uint32_t STOPIE        : 1; // /*!< STOP detection interrupt enable */
		uint32_t TCIE          : 1; // /*!< Transfer complete interrupt enable */
		uint32_t ERRIE         : 1; // /*!< Errors interrupt enable */
		uint32_t DNF           : 4; // /*!< Digital noise filter */
		uint32_t ANFOFF        : 1; // /*!< Analog noise filter OFF */
		uint32_t SWRST         : 1; // /*!< Software reset */
		uint32_t TXDMAEN       : 1; // /*!< DMA transmission requests enable */
		uint32_t RXDMAEN       : 1; // /*!< DMA reception requests enable */
		uint32_t SBC           : 1; // /*!< Slave byte control */
		uint32_t NOSTRETCH     : 1; // /*!< Clock stretching disable */
		uint32_t WUPEN         : 1; // /*!< Wakeup from STOP enable */
		uint32_t GCEN          : 1; // /*!< General call enable */
		uint32_t SMBHEN        : 1; // /*!< SMBus host address enable */
		uint32_t SMBDEN        : 1; // /*!< SMBus device default address enable */
		uint32_t ALERTEN       : 1; // /*!< SMBus alert enable */
		uint32_t PECEN         : 1; // /*!< PEC enable */
		uint32_t FMP           : 1; // /*!< Fast-mode Plus 20 mA drive enable */
		uint32_t _reserved25   : 5;
		uint32_t ADDRACLR      : 1; // /*!< ADDRACLR enable */
		uint32_t STOPFACLR     : 1; // /*!< STOPFACLR enable */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,cr1);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,cr1),bits);

typedef union {
	struct {
		uint32_t SADD          :10; // /*!< Slave address (master mode) */
		uint32_t RD_WRN        : 1; // /*!< Transfer direction (master mode) */
		uint32_t ADD10         : 1; // /*!< 10-bit addressing mode (master mode) */
		uint32_t HEAD10R       : 1; // /*!< 10-bit address header only read direction (master mode) */
		uint32_t START         : 1; // /*!< START generation */
		uint32_t STOP          : 1; // /*!< STOP generation (master mode) */
		uint32_t NACK          : 1; // /*!< NACK generation (slave mode) */
		uint32_t NBYTES        : 8; // /*!< Number of bytes */
		uint32_t RELOAD        : 1; // /*!< NBYTES reload mode */
		uint32_t AUTOEND       : 1; // /*!< Automatic end mode (master mode) */
		uint32_t PECBYTE       : 1; // /*!< Packet error checking byte */
		uint32_t _reserved27   : 5;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,cr2);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,cr2),bits);

typedef union {
	struct {
		uint32_t OA1           :10; // /*!< Interface own address 1 */
		uint32_t OA1MODE       : 1; // /*!< Own address 1 10-bit mode */
		uint32_t _reserved11   : 4;
		uint32_t OA1EN         : 1; // /*!< Own address 1 enable */
		uint32_t _reserved16   :16;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,oar1);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,oar1),bits);

typedef union {
	struct {
		uint32_t _reserved0    : 1;
		uint32_t OA2           : 7; // /*!< Interface own address 2 */
		uint32_t OA2MSK        : 3; // /*!< Own address 2 masks */
		uint32_t _reserved11   : 4;
		uint32_t OA2EN         : 1; // /*!< Own address 2 enable */
		uint32_t _reserved16   :16;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,oar2);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,oar2),bits);

typedef union {
	struct {
		uint32_t SCLL          : 8; // /*!< SCL low period (master mode) */
		uint32_t SCLH          : 8; // /*!< SCL high period (master mode) */
		uint32_t SDADEL        : 4; // /*!< Data hold time */
		uint32_t SCLDEL        : 4; // /*!< Data setup time */
		uint32_t _reserved24   : 4;
		uint32_t PRESC         : 4; // /*!< Timings prescaler */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,timingr);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,timingr),bits);

typedef union {
	struct {
		uint32_t TIMEOUTA      :12; // /*!< Bus timeout A */
		uint32_t TIDLE         : 1; // /*!< Idle clock timeout detection */
		uint32_t _reserved13   : 2;
		uint32_t TIMOUTEN      : 1; // /*!< Clock timeout enable */
		uint32_t TIMEOUTB      :12; // /*!< Bus timeout B*/
		uint32_t _reserved28   : 3;
		uint32_t TEXTEN        : 1; // /*!< Extended clock timeout enable */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,timeoutr);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,timeoutr),bits);

typedef union {
	struct {
		uint32_t TXE           : 1; // /*!< Transmit data register empty */
		uint32_t TXIS          : 1; // /*!< Transmit interrupt status */
		uint32_t RXNE          : 1; // /*!< Receive data register not empty */
		uint32_t ADDR          : 1; // /*!< Address matched (slave mode)*/
		uint32_t NACKF         : 1; // /*!< NACK received flag */
		uint32_t STOPF         : 1; // /*!< STOP detection flag */
		uint32_t TC            : 1; // /*!< Transfer complete (master mode) */
		uint32_t TCR           : 1; // /*!< Transfer complete reload */
		uint32_t BERR          : 1; // /*!< Bus error */
		uint32_t ARLO          : 1; // /*!< Arbitration lost */
		uint32_t OVR           : 1; // /*!< Overrun/Underrun */
		uint32_t PECERR        : 1; // /*!< PEC error in reception */
		uint32_t TIMEOUT       : 1; // /*!< Timeout or Tlow detection flag */
		uint32_t ALERT         : 1; // /*!< SMBus alert */
		uint32_t _reserved14   : 1;
		uint32_t BUSY          : 1; // /*!< Bus busy */
		uint32_t DIR           : 1; // /*!< Transfer direction (slave mode) */
		uint32_t ADDCODE       : 7; // /*!< Address match code (slave mode) */
		uint32_t _reserved24   : 8;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,isr);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,isr),bits);

typedef union {
	struct {
		uint32_t _reserved0    : 3;
		uint32_t ADDRCF        : 1; // /*!< Address matched clear flag */
		uint32_t NACKCF        : 1; // /*!< NACK clear flag */
		uint32_t STOPCF        : 1; // /*!< STOP detection clear flag */
		uint32_t _reserved6    : 2;
		uint32_t BERRCF        : 1; // /*!< Bus error clear flag */
		uint32_t ARLOCF        : 1; // /*!< Arbitration lost clear flag */
		uint32_t OVRCF         : 1; // /*!< Overrun/Underrun clear flag */
		uint32_t PECCF         : 1; // /*!< PAC error clear flag */
		uint32_t TIMOUTCF      : 1; // /*!< Timeout clear flag */
		uint32_t ALERTCF       : 1; // /*!< Alert clear flag */
		uint32_t _reserved14   :18;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,icr);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,icr),bits);

typedef union {
	struct {
		uint32_t PEC           : 8; // /*!< PEC register */
		uint32_t _reserved8    :24;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,pecr);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,pecr),bits);

typedef union {
	struct {
		uint32_t RXDATA        : 8; // /*!< 8-bit receive data */
		uint32_t _reserved8    :24;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,rxdr);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,rxdr),bits);

typedef union {
	struct {
		uint32_t TXDATA        : 8; // /*!< 8-bit transmit data */
		uint32_t _reserved8    :24;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(i2c,txdr);
CHECK_TYPEDEF_u32(REGDEF_NAME(i2c,txdr),bits);

typedef union {
	struct {
		REGDEF_NAME(i2c,cr1) CR1;
		REGDEF_NAME(i2c,cr2) CR2;
		REGDEF_NAME(i2c,oar1) OAR1;
		REGDEF_NAME(i2c,oar2) OAR2;
		REGDEF_NAME(i2c,timingr) TIMINGR;
		REGDEF_NAME(i2c,timeoutr) TIMEOUTR;
		REGDEF_NAME(i2c,isr) ISR;
		REGDEF_NAME(i2c,icr) ICR;
		REGDEF_NAME(i2c,pecr) PECR;
		REGDEF_NAME(i2c,rxdr) RXDR;
		REGDEF_NAME(i2c,txdr) TXDR;
	} /*QEMU_PACKED*/;
	uint32_t raw[RI_END];
}  REGDEF_NAME(stm32com,i2c);

QEMU_BUILD_BUG_MSG(sizeof(REGDEF_NAME(stm32com,i2c)) != sizeof(uint32_t)*RI_END , "Structure Size mismatch - expected uint32[RI_END]");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), CR1) != 0, "Offset mismatch for CR1");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), CR2) != 4, "Offset mismatch for CR2");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), OAR1) != 8, "Offset mismatch for OAR1");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), OAR2) != 12, "Offset mismatch for OAR2");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), TIMINGR) != 16, "Offset mismatch for TIMINGR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), TIMEOUTR) != 20, "Offset mismatch for TIMEOUTR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), ISR) != 24, "Offset mismatch for ISR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), ICR) != 28, "Offset mismatch for ICR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), PECR) != 32, "Offset mismatch for PECR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), RXDR) != 36, "Offset mismatch for RXDR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,i2c), TXDR) != 40, "Offset mismatch for TXDR");


typedef struct COM_STRUCT_NAME(I2c) {
    /* <private> */
    STM32Peripheral parent;

	/* <public> */
	/* Memory region */
	MemoryRegion mmio;

	REGDEF_NAME(stm32com,i2c) regs;

	qemu_irq event, error;

	const stm32_reginfo_t* reginfo;

	bool has_split_irq;

	bool dr_unread;
	bool shift_full;
	bool is_start;
	uint8_t shiftreg;
    I2CBus *bus;

} COM_STRUCT_NAME(I2c);

typedef struct COM_CLASS_NAME(I2c) {
	STM32PeripheralClass parent_class;
    const stm32_reginfo_t* var_reginfo;
} COM_CLASS_NAME(I2c);

/* Routine which updates the I2C's IRQs.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32_common_i2c_update_irq(COM_STRUCT_NAME(I2c) *s)
{
	// Note only the H503 has split EVT/ERR IRQs.
	uint32_t irq_level = 0;
	uint32_t en_mask = (s->regs.CR1.raw & IRQ_CR1_MSK);
	/* TCR shares the TCIE enable (CR1 bit 6) but lives at ISR bit 7 */
	if (s->regs.CR1.bits.TCIE) {
		en_mask |= IRQ_TCR;
	}

	if (s->has_split_irq)
	{
		irq_level = s->regs.ISR.raw & IRQ_ERR;
		if (s->regs.CR1.bits.ERRIE)
		{
			qemu_set_irq(s->error, irq_level>0);
		}
		irq_level = (s->regs.ISR.raw & IRQ_EVT) & en_mask;
	}
	else
	{
		irq_level = (s->regs.ISR.raw & IRQ_ALL) & en_mask;
	}

	if (en_mask)
	{
		qemu_set_irq(s->event, irq_level>0);
	}
	else
	{
		qemu_set_irq(s->event, false);
	}
}


static uint64_t
stm32_common_i2c_read(void *dev, hwaddr offset, unsigned size)
{
    COM_STRUCT_NAME(I2c) *s = STM32COM_I2C(dev);

	int index = offset >> 2U;
	int shift = offset & 0x3U;

	CHECK_BOUNDS_R_V2(index, RI_END, s->reginfo);

	switch(index) {
		case RI_RXDR:
		{
			if (s->regs.ISR.bits.RXNE) {
				s->regs.RXDR.bits.RXDATA = s->shiftreg;
				s->shift_full = false;
				if (s->regs.CR2.bits.NBYTES > 0) {
					s->regs.CR2.bits.NBYTES--;
				}
				if (s->regs.CR2.bits.NBYTES > 0 && i2c_bus_busy(s->bus)) {
					s->shiftreg = i2c_recv(s->bus);
					s->shift_full = true;
				} else {
					s->regs.ISR.bits.RXNE = false;
					bool needs_stop = s->regs.CR2.bits.STOP || s->regs.CR2.bits.AUTOEND;
					if (needs_stop && i2c_bus_busy(s->bus)) {
						i2c_end_transfer(s->bus);
						s->regs.CR2.bits.STOP = false;
						s->regs.ISR.bits.STOPF = true;
					}
				}
			}
			s->dr_unread = false;
		}
			break;
	}

	uint32_t r = s->regs.raw[index];

	ADJUST_FOR_OFFSET_AND_SIZE_R(r, size, shift, 0b110);

    return r;
}


static void
stm32_common_i2c_write(void *dev, hwaddr offset, uint64_t data, unsigned size)
{

    COM_STRUCT_NAME(I2c) *s = STM32COM_I2C(dev);

	int index = offset >> 2U;
	offset &= 0x3U;

	CHECK_BOUNDS_W_V2(index, data , RI_END);

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[index], data, size, offset, 0b110);

	CHECK_UNIMP_RESVD_V2(data, s->regs.raw[index], s->reginfo, index);

	s->regs.raw[index] = data;

	switch (index) {
        case RI_CR2:
            if (s->regs.CR2.bits.START) {
                s->regs.CR2.bits.START = false; // and clear START.
				bool is_read = s->regs.CR2.bits.RD_WRN;
				if (i2c_start_transfer(s->bus, s->regs.CR2.bits.SADD, is_read)){
					// Failed.
					s->regs.ISR.bits.NACKF = true;
				}
				else
				{
					s->regs.ISR.bits.NACKF = false;
					trace_stm32_common_i2c_txis(_PERIPHNAMES[s->parent.periph]);
					if (is_read) {
						s->shiftreg = i2c_recv(s->bus);
						s->shift_full = true;
						s->regs.ISR.bits.RXNE = true;
						s->regs.ISR.bits.TXIS = false;
						s->regs.ISR.bits.TXE = false;
					} else {
						s->regs.ISR.bits.TXIS = true;
						s->regs.ISR.bits.TXE = true;
					}
				}
            }
			// Stop has to happen after the next byte is sent.
            break;
		case RI_ICR:
			s->regs.ISR.raw &= ~data;
			s->regs.ICR.raw = 0;
			break;
		case RI_TXDR:
		{
			bool needs_stop = s->regs.CR2.bits.STOP;
            if (s->regs.ISR.bits.TXE && i2c_bus_busy(s->bus)) { // Continuing to transmit.
                i2c_send(s->bus, data& 0xFF); // Send the byte.
				s->regs.CR2.bits.NBYTES--;
				trace_stm32_common_i2c_txdr(_PERIPHNAMES[s->parent.periph], s->regs.TXDR.bits.TXDATA, s->regs.CR2.bits.NBYTES);
				if (s->regs.CR2.bits.NBYTES == 0) {
					s->regs.ISR.bits.TCR = true;
					s->regs.ISR.bits.TC = true;
					needs_stop |= s->regs.CR2.bits.AUTOEND;
				}
            }
			if (needs_stop)	{
				trace_stm32_common_i2c_stop(_PERIPHNAMES[s->parent.periph]);
				i2c_end_transfer(s->bus);
				s->regs.CR2.bits.STOP = false;
				s->regs.ISR.bits.STOPF = true;
			}
		}
            break;
	}
    stm32_common_i2c_update_irq(s);
}

static const MemoryRegionOps stm32_common_i2c_ops = {
    .read = stm32_common_i2c_read,
    .write = stm32_common_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_common_i2c_reset(DeviceState *dev)
{
    COM_STRUCT_NAME(I2c) *s = STM32COM_I2C(dev);

	memset(&s->regs.raw, 0, sizeof(s->regs.raw));

	for (int i=0; i< RI_END; i++)
	{
		if (s->reginfo[i].not_reserved)
			stm32_common_i2c_write(dev, i<<2U, s->reginfo[i].reset_val, 4U);
	}
}

/* DEVICE INITIALIZATION */
static void stm32_common_i2c_realize(DeviceState *dev, Error **errp)
{
    // STM32COM_STRUCT_NAME(Pwr) *s = STM32H503_PWR(dev);
}

static void stm32_common_i2c_init(Object *obj)
{

    COM_STRUCT_NAME(I2c) *s = STM32COM_I2C(obj);
    STM32_MR_IO_INIT(&s->mmio, obj, &stm32_common_i2c_ops, s, 1U * KiB);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

	COM_CLASS_NAME(I2c) *k = STM32COM_I2C_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;

	s->has_split_irq = (s->reginfo == stm32_h503_i2c_reginfo); // Only the H503 has split IRQs.

	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->event);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->error);

    s->bus = i2c_init_bus(DEVICE(obj),"i2c");

}

static const VMStateDescription vmstate_stm32_common_i2c = {
    .name = TYPE_STM32H503_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs.raw, COM_STRUCT_NAME(I2c), RI_END),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_common_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32_common_i2c_reset);
    dc->realize = stm32_common_i2c_realize;
    dc->vmsd = &vmstate_stm32_common_i2c;

	COM_CLASS_NAME(I2c) *k = STM32COM_I2C_CLASS(klass);
	k->var_reginfo = (const stm32_reginfo_t*)data;
	//QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
}

static TypeInfo stm32_common_i2c_info = {
    .name  = TYPE_STM32COM_I2C,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(I2c)),
	.class_size    = sizeof(COM_CLASS_NAME(I2c)),
	.abstract	   = true,
};

static void stm32_common_i2c_register_types(void)
{
    type_register_static(&stm32_common_i2c_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_i2c_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_i2c_variants[i].variant_name,
            .parent     = TYPE_STM32COM_I2C,
			.instance_init = stm32_common_i2c_init,
    		.class_init    = stm32_common_i2c_class_init,
            .class_data = (void *)stm32_i2c_variants[i].variant_regs,
		};
		type_register_static(&ti);
	}
}

type_init(stm32_common_i2c_register_types)
