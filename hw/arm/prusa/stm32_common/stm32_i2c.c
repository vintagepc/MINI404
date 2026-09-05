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
#include "trace.h"
#include "../stm32_registers/generated/common/I2C_TYPE_A_registers.h"

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

typedef struct COM_STRUCT_NAME(I2c) {
    /* <private> */
    STM32Peripheral parent;

	/* <public> */
	/* Memory region */
	MemoryRegion mmio;

	REGDEF_NAME(stm32com,i2c_type_a) regs;

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
