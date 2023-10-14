/*
    at21csxx.c - Sim at21csxx eeprom for
    Mini404.

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
#include "qapi/error.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "../utility/macros.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "sysemu/block-backend.h"
#include "sysemu/cpu-timers.h"
#include "migration/vmstate.h"

#define TYPE_AT21CSXX "at21csxx"

#define AT21_OP_EEA  0xA
#define AT21_OP_SRA  0xB
#define AT21_OP_LSR  0x2
#define AT21_OP_RZRA 0x7
#define AT21_OP_FRZS 0x1
#define AT21_OP_MIR  0xC
#define AT21_OP_SSM  0xD
#define AT21_OP_HSM  0xE

#define ICOUNT_PER_US 168

#define DEV_ADDR 0
#define DEV_SIZE 128

OBJECT_DECLARE_SIMPLE_TYPE(AT21CSxxState, AT21CSXX)

enum SIOState {
	SIO_RESET,
	SIO_DISC,
	SIO_IDLE,
	SIO_DIN,
	SIO_DOUT,
};

typedef union {
    struct {
        uint16_t read       :1;
        uint16_t chip_addr  :3;
        uint16_t opcode     :4;
    } QEMU_PACKED def;
    uint8_t raw;
} at21csxx_cmd_t;


struct AT21CSxxState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    qemu_irq irq; //
    uint8_t data[DEV_SIZE];

    at21csxx_cmd_t cmd;

	QEMUTimer *line_release;

	int64_t low_start, high_start;
	uint8_t bit_counter;
	uint8_t byte_in, byte_out;
	uint8_t sio_state;

	uint8_t byte_count;
	uint8_t address_pointer;

	BlockBackend *blk;
};

static const VMStateDescription vmstate_at21csxx = {
    .name = TYPE_AT21CSXX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
		VMSTATE_UINT8(cmd.raw, AT21CSxxState),
		VMSTATE_TIMER_PTR(line_release, AT21CSxxState),
		VMSTATE_INT64(low_start, AT21CSxxState),
		VMSTATE_INT64(high_start, AT21CSxxState),
		VMSTATE_UINT8(bit_counter, AT21CSxxState),
		VMSTATE_UINT8(byte_in, AT21CSxxState),
		VMSTATE_UINT8(byte_out, AT21CSxxState),
		VMSTATE_UINT8(sio_state, AT21CSxxState),
		VMSTATE_UINT8(byte_count, AT21CSxxState),
		VMSTATE_UINT8(address_pointer, AT21CSxxState),
		VMSTATE_UINT8_ARRAY(data, AT21CSxxState, DEV_SIZE),
        VMSTATE_END_OF_LIST()
    }
};

static void at21csxx_line_release(void *opaque)
{
    AT21CSxxState *s = AT21CSXX(opaque);
	qemu_irq_raise(s->irq); // Release the data line.
	// printf("Line released\n");
}

#define NS_SCALE(x) (x * ns_per_inst)

static bool at21csxx_process_byte(AT21CSxxState *s, uint8_t byte)
{
	switch (s->byte_count)
	{
		case 0:
			s->cmd.raw = byte;
			if (s->cmd.def.chip_addr == DEV_ADDR)
			{
				if (s->cmd.def.read)
				{
					s->sio_state = SIO_DOUT;
				}
				switch (s->cmd.def.opcode)
				{
					case AT21_OP_EEA:
						s->byte_count++;
						return true;
					default:
						printf("AT21 Unhandled write opcode %x\n", s->cmd.def.opcode);
						break;
				}
			}
			return false; // NACK
		case 1:
			if (byte < DEV_SIZE)
			{
				s->address_pointer = byte;
				s->byte_count++;
				return true;
			}
			return false;
		default:
			s->data[s->address_pointer++] = byte;
			s->address_pointer %= DEV_SIZE;
			return true;
	}
}

static void at21csxx_sio(void* opaque, int n, int level) {
    AT21CSxxState *s = AT21CSXX(opaque);
	if (!level)
	{
		s->low_start = icount_get_raw();
		int64_t tHigh = s->low_start - s->high_start;
		if (tHigh > 500 * ICOUNT_PER_US)
		{
			// printf("AT21 Start cond\n");
			s->sio_state = SIO_IDLE;
		}
		return;
	}

	s->high_start = icount_get_raw();

	int64_t tLow = s->high_start - s->low_start;

	if (tLow > 150 * ICOUNT_PER_US)
	{
		s->bit_counter = 0;
		s->byte_in = 0;
		s->sio_state = SIO_DISC;
		return;
	}
	bool logic_one = ICOUNT_PER_US < tLow && tLow < (2 * ICOUNT_PER_US); 			// Logic 1 in high speed mode
	bool logic_zero = (6 * ICOUNT_PER_US) < tLow && tLow < (16 * ICOUNT_PER_US);	// Logic 0 in high speed.
	int64_t ns_per_inst = icount_to_ns(1);
	switch (s->sio_state)
	{
		case SIO_DISC:
			if (logic_one)
			{
				qemu_irq_lower(s->irq);
				timer_mod(s->line_release, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NS_SCALE(6000)); // 2-6 us from the datasheet
				// printf("Discovery ack\n");
				s->sio_state = SIO_IDLE;
			}
			break;
		case SIO_IDLE:
			s->byte_count = 0; // Reset byte count for next data in.
			s->bit_counter = 0;
			s->cmd.raw = 0;
			s->byte_in = 0;
			s->byte_out = 0;
			s->sio_state = SIO_DIN;
			/* FALLTHRU */
		case SIO_DIN:
			if (logic_one || logic_zero )
			{
				if (s->bit_counter < 8)
				{
					s->byte_in <<= 1;
					s->byte_in |= logic_one;
					s->bit_counter++;
					return;
				}
			}
			else
			{
				printf("w1 Bus unhandled low duration: %"PRId64" instructions\n", tLow);
			}
			if (s->bit_counter == 8)
			{
				bool send_ack = at21csxx_process_byte(s, s->byte_in);
				// printf("Byte in: %02x - %s\n", s->byte_in, send_ack? "ACK" : "NACK");
				s->byte_in = 0;
				qemu_set_irq(s->irq, !send_ack);
				timer_mod(s->line_release, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NS_SCALE(2000)); // 2-6 us from the datasheet
				s->bit_counter = 0;
			}
			break;
		case SIO_DOUT:
			switch (s->bit_counter)
			{
				case 0:
					switch (s->cmd.def.opcode)
					{
						case AT21_OP_EEA:
							s->byte_out = s->data[s->address_pointer++];
							printf("AT21CSxx: Sending byte@ %02x: %02x\n",s->address_pointer-1, s->byte_out);
							break;
						default:
							printf("AT21 Unhandled read opcode %x\n", s->cmd.def.opcode);
							break;
					}
					/* fallthru */
				case 1 ... 7 :
					qemu_set_irq(s->irq, (s->byte_out & 0x80) > 0 );
					s->bit_counter++;
					s->byte_out <<= 1;
					timer_mod(s->line_release, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NS_SCALE(1000)); // 2-6 us from the datasheet
					break;
				case 8:
					if (logic_one) // NACK. Read is done.
					{
						s->sio_state = SIO_IDLE;
					}
					s->bit_counter = 0;
			}
			break;

	}
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(AT21CSxxState, at21csxx, AT21CSXX, SYS_BUS_DEVICE, {NULL})

static void at21csxx_finalize(Object *obj)
{
}

static void at21csxx_reset(DeviceState *dev)
{
    AT21CSxxState *s = AT21CSXX(dev);
    qemu_set_irq(s->irq,1);
	s->sio_state = SIO_RESET;
}

static void at21csxx_realize(DeviceState *dev, Error **errp)
{
    AT21CSxxState *s = AT21CSXX(dev);
#ifndef CONFIG_GCOV    
	if (icount_enabled() == 0)
	{
		printf("WARNING: icount is disabled. AT21CSxx EEPROM will NOT WORK!\n");
		printf("WARNING: use -icount [number] to enable it.\n");
	}
#endif    
    if (s->blk) {

        int64_t len = blk_getlength(s->blk);

        // Note - some OSes do not allow files under 1k, so as long as the source is larger it's fine.
        if (len <= sizeof(s->data)) {
            error_setg(errp, "%s: Backing file size %" PRId64 " != %" PRIu64,
                       TYPE_AT21CSXX, len, sizeof(s->data));
            return;
        }

        if (blk_set_perm(s->blk, BLK_PERM_CONSISTENT_READ,
                         BLK_PERM_ALL, errp) < 0)
        {
            error_setg(errp, "%s: Backing file incorrect permission",
                       TYPE_AT21CSXX);
            return;
        }
        len = blk_pread(s->blk, 0, &s->data, sizeof(s->data));

        if (len != sizeof(s->data)) {
            error_setg(errp, "%s: failed to read backing file!",
                TYPE_AT21CSXX);;
        }
    }
	else
	{
		// Add some fake data just to avoid 400 re-read attempts during boot... :(
		s->data[0] = 0x00;
		s->data[1] = 0x20;
		s->data[2] = 0x00;
		s->data[3] = 0x01;
		s->data[8] = '0';
		s->data[9] = '0';
	}
}

static void at21csxx_init(Object *obj)
{
    AT21CSxxState *s = AT21CSXX(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_in(DEVICE(s),at21csxx_sio, 1);
	s->line_release = timer_new_ns(QEMU_CLOCK_VIRTUAL, &at21csxx_line_release, s);
	for (int i=0; i< DEV_SIZE; i++)
	{
		s->data[i] = 0xFF;
	}
}

static Property at21csxx_eeprom_props[] = {
    DEFINE_PROP_DRIVE("drive", AT21CSxxState, blk),
    DEFINE_PROP_END_OF_LIST()
};

static void at21csxx_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = at21csxx_reset;
    dc->vmsd = &vmstate_at21csxx;
	dc->realize = at21csxx_realize;
	device_class_set_props(dc, at21csxx_eeprom_props);
}
