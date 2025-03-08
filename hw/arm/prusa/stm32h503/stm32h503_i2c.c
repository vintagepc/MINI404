/*
 * STM32FH503 I2C
 *
 * Copyright (c) 2024 VintagePC <github.com/vinagepc>
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
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "../utility/macros.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_common/stm32_common.h"
#include "../stm32_registers/generated/stm32h503/I2C_index.h"
#include "../stm32_registers/generated/stm32h503/I2C_registers.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(STM32H503_STRUCT_NAME(I2c), STM32H503_I2C);

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

typedef struct STM32H503_STRUCT_NAME(I2c) {
    /* <private> */
    STM32Peripheral parent;

	/* <public> */
	/* Memory region */
	MemoryRegion mmio;

	stm32reg_h503_i2c_t regs;

	qemu_irq event, error;

	bool dr_unread;
	bool shift_full;
	bool is_start;
	uint8_t shiftreg;
    I2CBus *bus;

} STM32H503_STRUCT_NAME(I2c);

/* Routine which updates the I2C's IRQs.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32_h503_i2c_update_irq(STM32H503_STRUCT_NAME(I2c) *s)
{
    uint32_t irq_level = s->regs.ISR.raw & IRQ_ERR;
	if (s->regs.CR1.bits.ERRIE)
	{
		qemu_set_irq(s->error, irq_level>0);
	}
	uint32_t en_mask =  (s->regs.CR1.raw & IRQ_CR1_MSK);
	irq_level = (s->regs.ISR.raw & IRQ_EVT) & en_mask;
	irq_level |= s->regs.ISR.bits.TCR;

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
stm32_h503_i2c_read(void *dev, hwaddr offset, unsigned size)
{
    STM32H503_STRUCT_NAME(I2c) *s = STM32H503_I2C(dev);

	int index = offset >> 2U;
	int shift = offset & 0x3U;

	CHECK_BOUNDS_R_V2(index, RI_END, stm32_h503_i2c_reginfo);

	switch(index) {
		case RI_RXDR:
		{
			bool needs_stop = s->regs.CR2.bits.STOP || (s->regs.CR2.bits.NBYTES == 0 && s->regs.CR2.bits.AUTOEND);
			if (s->regs.ISR.bits.RXNE && i2c_bus_busy(s->bus)) {
				if (s->dr_unread) {
					s->dr_unread = false;
				} else {
					s->regs.RXDR.bits.RXDATA = s->shiftreg;
					s->shiftreg = i2c_recv(s->bus);
				}
			} else if (s->regs.ISR.bits.RXNE) {
				if (s->dr_unread) {
					// Special case for 2 bytes left, don't clobber the wanted value currently in DR.
				} else if (s->shift_full) {
					s->regs.RXDR.bits.RXDATA = s->shiftreg;
					s->shift_full = false;
					s->regs.ISR.bits.RXNE = false;
				} else {
					s->regs.RXDR.bits.RXDATA = 0;
					printf("FIXME: READ with no data!\n");
				}
			}
			if (needs_stop)	{
				i2c_end_transfer(s->bus);
				s->regs.CR2.bits.STOP = false;
				s->regs.ISR.bits.STOPF = true;
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
stm32_h503_i2c_write(void *dev, hwaddr offset, uint64_t data, unsigned size)
{

    STM32H503_STRUCT_NAME(I2c) *s = STM32H503_I2C(dev);

	int index = offset >> 2U;
	offset &= 0x3U;

	CHECK_BOUNDS_W_V2(index, data , RI_END);

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[index], data, size, offset, 0b110);

	CHECK_UNIMP_RESVD_V2(data, s->regs.raw[index], stm32_h503_i2c_reginfo, index);

	bool is_read = s->regs.CR2.bits.RD_WRN;

	s->regs.raw[index] = data;

	switch (index) {
        case RI_CR2:
            if (s->regs.CR2.bits.START) {
                s->regs.CR2.bits.START = false; // and clear START.
				if (i2c_start_transfer(s->bus, s->regs.CR2.bits.SADD, is_read)){
					// Failed.
					s->regs.ISR.bits.NACKF = true;
				}
				else
				{
					s->regs.ISR.bits.NACKF = false;
					trace_stm32h503_i2c_txis(_PERIPHNAMES[s->parent.periph]);
					s->regs.ISR.bits.TXIS = true;
					s->regs.ISR.bits.TXE = true;

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
            if (s->regs.ISR.bits.TXE) { // Continuing to transmit.
                i2c_send(s->bus, data& 0xFF); // Send the byte.
				s->regs.CR2.bits.NBYTES--;
				trace_stm32h503_i2c_txdr(_PERIPHNAMES[s->parent.periph], s->regs.TXDR.bits.TXDATA, s->regs.CR2.bits.NBYTES);
				if (s->regs.CR2.bits.NBYTES == 0) {
					s->regs.ISR.bits.TCR = true;
					s->regs.ISR.bits.TC = true;
					needs_stop |= s->regs.CR2.bits.AUTOEND;
				}
            }
			if (needs_stop)	{
				trace_stm32h503_i2c_stop(_PERIPHNAMES[s->parent.periph]);
				i2c_end_transfer(s->bus);
				s->regs.CR2.bits.STOP = false;
				s->regs.ISR.bits.STOPF = true;
			}
		}
            break;
	}
    stm32_h503_i2c_update_irq(s);
}

static const MemoryRegionOps stm32_h503_i2c_ops = {
    .read = stm32_h503_i2c_read,
    .write = stm32_h503_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_h503_i2c_reset(DeviceState *dev)
{
    STM32H503_STRUCT_NAME(I2c) *s = STM32H503_I2C(dev);

	memset(&s->regs.raw, 0, sizeof(s->regs.raw));

	for (int i=0; i< RI_END; i++)
	{
		if (stm32_h503_i2c_reginfo[i].not_reserved)
			stm32_h503_i2c_write(dev, i<<2U, stm32_h503_i2c_reginfo[i].reset_val, 4U);
	}
}

/* DEVICE INITIALIZATION */
static void stm32_h503_i2c_realize(DeviceState *dev, Error **errp)
{
    // STM32H503_STRUCT_NAME(Pwr) *s = STM32H503_PWR(dev);
}

static void stm32_h503_i2c_init(Object *obj)
{

    STM32H503_STRUCT_NAME(I2c) *s = STM32H503_I2C(obj);
    STM32_MR_IO_INIT(&s->mmio, obj, &stm32_h503_i2c_ops, s, 1U * KiB);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);


	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->event);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->error);

    s->bus = i2c_init_bus(DEVICE(obj),"i2c");

}

static const VMStateDescription vmstate_stm32h503_i2c = {
    .name = TYPE_STM32H503_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs.raw, STM32H503_STRUCT_NAME(I2c), RI_END),
        VMSTATE_END_OF_LIST()
    }
};


static void stm32_h503_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, stm32_h503_i2c_reset);
    dc->realize = stm32_h503_i2c_realize;
    dc->vmsd = &vmstate_stm32h503_i2c;
}

static TypeInfo stm32_h503_i2c_info = {
    .name  = TYPE_STM32H503_I2C,
    .parent = TYPE_STM32_PERIPHERAL,
    .instance_size  = sizeof(STM32H503_STRUCT_NAME(I2c)),
    .class_init = stm32_h503_i2c_class_init,
    .instance_init = stm32_h503_i2c_init,
};

static void stm32_h503_i2c_register_types(void)
{
    type_register_static(&stm32_h503_i2c_info);
}

type_init(stm32_h503_i2c_register_types)
