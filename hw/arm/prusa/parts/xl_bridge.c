/*
    xl_bridge.h - (unix) socket transport for XL, which needs
	independent processors/instances of QEMU

	Copyright 2022-3 VintagePC <https://github.com/vintagepc/>

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
#include "../utility/macros.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "xl_bridge.h"
#include "qemu/atomic.h"
#include "qemu/option.h"
#include "qemu/config-file.h"
#include "sysemu/runstate.h"
#include "qapi/qapi-commands-run-state.h"
#include "qapi/qapi-events-run-state.h"

static const char* shm_names[XL_BRIDGE_COUNT] =
{
	"PXL_XLBUDDY",
	"PXL_BED",
	"PXL_E0",
	"PXL_E1",
	"PXL_E2",
	"PXL_E3",
	"PXL_E4",
	"PXL_E5",
    "PXL_ESP32"
};

// Messages are composed of some GPIO status bits
// and a serial byte.

#define TYPE_XLBRIDGE "xl-bridge"

OBJECT_DECLARE_SIMPLE_TYPE(XLBridgeState, XLBRIDGE)

typedef union gpio_state_t{
	struct {
		uint8_t e_step :1;
		uint8_t e_dir  :1;
		uint8_t nAC    :1;
		uint8_t reset  :1;
		uint8_t z_um   :1;
		uint8_t _reserved :3;
	} bits;
	uint8_t byte;
} gpio_state_t;


struct XLBridgeState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/

	// Clients (anything not the board) only use their chardev and connect
	// to the appropriate socket. Base XLBuddy creates all sockets and waits for
	// at least one extruder and one bed.
	CharBackend chr[XL_BRIDGE_COUNT];
	CharBackend gpio[XL_BRIDGE_COUNT];

	qemu_irq byte_receive[XLBRIDGE_UART_COUNT];

	qemu_irq gpio_out[XLBRIDGE_PIN_COUNT];

	gpio_state_t gpio_states[XL_BRIDGE_COUNT];

	bool de_pin_used[XL_BRIDGE_COUNT];
	bool de_pin_asserted[XL_BRIDGE_COUNT];

	uint8_t id;

	uint8_t buffer[256];
	uint8_t buffer_level;

	uint8_t data_remaining;

	union {
		uint8_t bytes[4];
		uint32_t u32;
		int32_t i32;
	} data_4b;
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(XLBridgeState, xl_bridge, XLBRIDGE,SYS_BUS_DEVICE,{NULL});

static void xl_bridge_tx_assert(void *opaque, int n, int level)
{
	XLBridgeState *s = XLBRIDGE(opaque);
	s->de_pin_asserted[s->id] = level;
	if (level) // New transmission:
	{
		s->buffer_level = 0;
		s->de_pin_used[s->id] = true;
		// printf("RS 485 TX high\n");
	}
	else // Transmit just finished. Pump it out over the socket...
	{
		// printf("RS 485 TX low\n");
		// If we are the base XLBuddy, forward the data to all the downstream periphs.
		// Due to RTO timing issues we "cheat" and inspect the traffic to see where it should go
		// This prevents rapidfire messages from stomping on one still being received.
		if (s->id == XL_DEV_XBUDDY) // broadcast messages to 0.
		{
			switch (s->buffer[0]) {
				case 0x0A ... 0x10:	// Tool. Route appropriately.
					qemu_chr_fe_write_all(&s->chr[XL_DEV_BED + (s->buffer[0] - 0x0A)],(uint8_t*)s->buffer, sizeof(s->buffer[0]) * s->buffer_level);
					break;
				case 0x1A ... 0x20:	// Tool. Route appropriately.
					qemu_chr_fe_write_all(&s->chr[XL_DEV_BED + (s->buffer[0] - 0x1A)],(uint8_t*)s->buffer, sizeof(s->buffer[0]) * s->buffer_level);
					break;
				default: // catch-all.
					printf("Unexpected message starting byte, %02x\n",s->buffer[0]);
					/* FALLTHRU */
				case 0x00: // Broadcast message (e.g. bootstrap)
					for (int i=XL_DEV_XBUDDY+1; i<XL_BRIDGE_COUNT; i++)
					{
						qemu_chr_fe_write_all(&s->chr[i],(uint8_t*)s->buffer, sizeof(s->buffer[0]) * s->buffer_level);
					}
					break;
			}
		}
		else
		{
			// Just a single transmit, from downstream to base.
			qemu_chr_fe_write_all(&s->chr[s->id],(uint8_t*)s->buffer, sizeof(s->buffer[0]) * s->buffer_level);

		}
		//printf("%s: sent %u bytes to %d\n",shm_names[s->id], s->buffer_level, s->buffer[0]);
		// for (int i=0; i<s->buffer_level; i++)
		// {
		// 	printf("%02x ",s->buffer[i]);
		// }
		// printf("\n");
	}
}

static void xl_bridge_byte_send(void *opaque, int n, int level)
{
	XLBridgeState *s = XLBRIDGE(opaque);

    // ESP uart isn't RS485, just forward on the data (this is from ESP to STM32)...
    if (n == XLBRIDGE_UART_ESP32)
    {
        uint8_t data = level;
        //printf("ESP32: %02x (%c)\n", data, data);
        qemu_chr_fe_write_all(&s->chr[XL_DEV_ESP32], (uint8_t*)&data, 1);
        return;
    }

	//uint16_t data = 0x00FF | (level & 0xFF)<<8; // swap the bytes here so they come in in the right order.
	// Buffer up the data for a single-shot transmit.
	s->buffer[s->buffer_level++] = level & 0xFF;

	// Right now the bootloader doesn't support the TE pin for dwarf :(
	if (!s->de_pin_used[s->id] && s->id >= XL_DEV_T0)
	{
		if (s->buffer_level > 2)
		{
			uint8_t len = s->buffer[2] + 5;
			if (s->buffer_level == len)
			{
				qemu_chr_fe_write_all(&s->chr[s->id],(uint8_t*)s->buffer, sizeof(s->buffer[0]) * s->buffer_level);
				//printf("XL Puppy: sent %u bytes\n",s->buffer_level);
				s->buffer_level = 0;
			}
			//printf("p sz: %u l: %u\n", s->buffer[2], s->buffer_level);
		}
	}
}


static int xl_bridge_can_receive(void *opaque)
{
   	XLBridgeState *s = XLBRIDGE(opaque);
	// if (s->de_pin_asserted[s->id]){
	// 	printf("recieve blocked by DE assert\n");
	// }
    return s->de_pin_asserted[s->id]? 0 : 64;
}

static int xl_bridge_gpio_can_receive(void *opaque)
{
    return 1; // Currently only 1 byte increments.
}

static void _xl_bridge_receive(void *opaque, const uint8_t *buf, int size, int destination)
{
   	XLBridgeState *s = XLBRIDGE(opaque);
	//#define FILTER size < 20 && s->id == XL_DEV_XBUDDY
	#define FILTER false
    // assert(size % 2 == 0);
	if (FILTER) printf(" %u Received: ", s->id);
	for (const uint8_t* p = buf; p<buf+size; p++)
	{
		qemu_set_irq(s->byte_receive[destination], *p);
		if (FILTER) printf("%02x ",*p);
	}
	if (FILTER) printf("\n");
}

static inline void xl_bridge_receive(void *opaque, const uint8_t *buf, int size)
{
    _xl_bridge_receive(opaque, buf, size, XLBRIDGE_UART_PUPPY);
}

static int xl_bridge_esp_can_receive(void *opaque)
{
    return 1; // Currently only 1 byte increments.
}

static inline void xl_bridge_esp_receive(void *opaque, const uint8_t *buf, int size)
{
    // printf("ESP32 said: %02x (%c)\n", buf[0], buf[0]);    
    _xl_bridge_receive(opaque, buf, size, XLBRIDGE_UART_ESP32);
}

#define PROCESS_BIT(pin, field) \
		if (s->gpio_states[s->id].bits.field != state.bits.field) \
		{ \
			qemu_set_irq(s->gpio_out[pin],state.bits.field); \
		}

static void xl_bridge_gpio_receive(void *opaque, const uint8_t *buf, int size)
{
   	XLBridgeState *s = XLBRIDGE(opaque);
	assert(size==1);

	gpio_state_t state = {.byte  = buf[0]};
	bool data_done = false;

	if (s->data_remaining)
	{
		s->data_4b.bytes[4-s->data_remaining] = buf[0];
		s->data_remaining--;
		if (s->data_remaining != 0)
		{
			return;
		}
		data_done = true;
		state = s->gpio_states[s->id]; // restore the state so we retrigger as "complete."
	}
	// printf("GPIO receive: %u - %02x\n", s->id, buf[0]);

	if (s->id != XL_DEV_XBUDDY)
	{
		if (state.bits.z_um)
		{
			if (data_done)
			{
				// printf("z value received: %d %08x\n", s->data_4b.i32, s->data_4b.i32);
				qemu_set_irq(s->gpio_out[XLBRIDGE_PIN_Z_UM], s->data_4b.i32);
				s->data_4b.u32 = 0;
			}
			else
			{
				s->data_remaining = 4; // read 4 more bytes to get actual position.
			}
		}
		PROCESS_BIT(XLBRIDGE_PIN_E_DIR, e_dir);
		PROCESS_BIT(XLBRIDGE_PIN_E_STEP, e_step);
		PROCESS_BIT(XLBRIDGE_PIN_nAC_FAULT, nAC);
		if (state.bits.reset)
		{
			printf("Puppy %s reset pin asserted\n", shm_names[s->id]);
    		qemu_system_reset_request(SHUTDOWN_CAUSE_SUBSYSTEM_RESET);
		}
		s->gpio_states[s->id].byte = state.byte;
	}
	else
	{
		//TODO - Return signals (fsens?) from remote to xbuddy.
	}
}

static void xl_bridge_reset_in(void *opaque, int n, int level)
{
	XLBridgeState *s = XLBRIDGE(opaque);
	if (s->id != XL_DEV_XBUDDY)
	{
		printf("Got GPIO inputs on client peripheral board %s - ignored.\n",shm_names[s->id]);
		return;
	}
	s->gpio_states[n].bits.reset = level>0;
	// Dispatch the new state.
	qemu_chr_fe_write_all(&s->gpio[n],&s->gpio_states[n].byte, 1);
	// if (level) printf("Sent reset to %02x, %02x\n", n, s->gpio_states[n].byte);//, shm_names[target]);
}

static void xl_bridge_gpio_in(void *opaque, int n, int level)
{
	XLBridgeState *s = XLBRIDGE(opaque);
	if (s->id != XL_DEV_XBUDDY)
	{
		printf("Got GPIO inputs on client peripheral board %s - ignored.\n",shm_names[s->id]);
		return;
	}
	uint8_t target = 0; //n/XLBRIDGE_PIN_COUNT;
	uint8_t pin = n%XLBRIDGE_PIN_COUNT;
	switch (pin)
	{
		case XLBRIDGE_PIN_E_STEP:
			s->gpio_states[target].bits.e_step = level;
			break;
		case XLBRIDGE_PIN_E_DIR:
			s->gpio_states[target].bits.e_dir = level;
			break;
		case XLBRIDGE_PIN_nAC_FAULT:
			s->gpio_states[target].bits.nAC = level;
			break;
		case XLBRIDGE_PIN_Z_UM:
			if (level > 1000)
			{
				return;
			}
			s->gpio_states[target].bits.z_um = 1;
			for (int i=XL_DEV_T0; i<XL_BRIDGE_COUNT; i++)
			{
				qemu_chr_fe_write_all(&s->gpio[i], &s->gpio_states[target].byte, 1);
				qemu_chr_fe_write_all(&s->gpio[i], (uint8_t*)&level, 4);
				// printf("Sent zpos %d 0x%08x\n", level, level);
				return;
			}
			break;
		default:
			printf("ERR: Unhanled pin enumeration for %s\n",__func__);
	}
	// Dispatch the new state.
	for (int i=0; i<XL_BRIDGE_COUNT; i++)
	{
		qemu_chr_fe_write_all(&s->gpio[i],&s->gpio_states[target].byte, 1);
	}
	printf("Sent gpio update %02x\n", s->gpio_states[target].byte);//, shm_names[target]);
}

static void xl_bridge_finalize(Object *obj)
{
}

static void xl_bridge_realize(DeviceState *dev, Error **errp)
{
    XLBridgeState *s = XLBRIDGE(dev);
	if (s->id == XL_DEV_XBUDDY)
	{
		for (int i=XL_DEV_ESP32; i>=XL_DEV_BED; i--) // just two tools for now, because of the "server" wait.
		{
			Chardev* d=qemu_chr_find(shm_names[i]);
			gchar* io_name = g_strdup_printf("%s-io",shm_names[i]);
			Chardev* d2=qemu_chr_find(io_name);
			g_free(io_name);
			// TODO - just create the sockets directly with options here rather than expect the user to get it right.
			if (d)
			{
				printf("Found ID %s - assigned!\n", shm_names[i]);
			}
			else
			{
				printf("Socket ID %s - not found, creating it instead.\n", shm_names[i]);
				QemuOpts *opts;
				// Now create the IO (GPIO) channel.
                opts = qemu_opts_create(qemu_find_opts("chardev"), g_strdup_printf("%s-io",shm_names[i]), 1, NULL);
                    qemu_opt_set(opts, "backend","socket", errp);
                    qemu_opt_set(opts, "path", g_strdup_printf("/tmp/%s-io", shm_names[i]), errp);
                    qemu_opt_set(opts, "server", "on", errp);
                    qemu_opt_set_bool(opts, "wait", false, errp);
                    d2 = qemu_chr_new_from_opts(opts, NULL, errp);
                qemu_opts_del(opts);

				opts = qemu_opts_create(qemu_find_opts("chardev"), g_strdup(shm_names[i]), 1, NULL);
					qemu_opt_set(opts, "backend","socket", errp);
					qemu_opt_set(opts, "path", g_strdup_printf("/tmp/%s", shm_names[i]), errp);
					qemu_opt_set(opts, "server", "on", errp);
					if (i > XL_DEV_T0 && i != XL_DEV_ESP32) // Only force wait for required items, namely tool 0 and the bed.
					{
						qemu_opt_set_bool(opts, "wait", false, errp);
					}
					d = qemu_chr_new_from_opts(opts, NULL, errp);
				qemu_opts_del(opts);

			}
			qemu_chr_fe_init(&s->chr[i],d, errp);
            if (i != XL_DEV_ESP32)
            {
			    qemu_chr_fe_set_handlers(&s->chr[i], xl_bridge_can_receive, xl_bridge_receive, NULL,
				    	NULL,s,NULL,true);
            }
            else
            {
                qemu_chr_fe_set_handlers(&s->chr[i], xl_bridge_esp_can_receive, xl_bridge_esp_receive, NULL,
				    	NULL,s,NULL,true);
            }
			qemu_chr_fe_accept_input(&s->chr[i]);

			qemu_chr_fe_init(&s->gpio[i],d2, errp);
			qemu_chr_fe_set_handlers(&s->gpio[i], xl_bridge_gpio_can_receive, xl_bridge_gpio_receive, NULL,
					NULL,s,NULL,true);
			qemu_chr_fe_accept_input(&s->gpio[i]);
		}
	}
	else
	{
		Chardev* d=qemu_chr_find(shm_names[s->id]);
		gchar* io_name = g_strdup_printf("%s-io",shm_names[s->id]);
		Chardev* d2=qemu_chr_find(io_name);
		g_free(io_name);
		// TODO - just create the sockets directly with options here rather than expect the user to get it right.
		if (d)
		{
			printf("Found ID %s - assigned!", shm_names[s->id]);
		}
		else
		{
			QemuOpts *opts;
				printf("Socket ID %s - not found, creating it instead.\n", shm_names[s->id]);
			opts = qemu_opts_create(qemu_find_opts("chardev"), g_strdup_printf("%s-io",shm_names[s->id]), 1, NULL);
				qemu_opt_set(opts, "backend","socket", errp);
				qemu_opt_set(opts, "path", g_strdup_printf("/tmp/%s-io", shm_names[s->id]), errp);
				d2 = qemu_chr_new_from_opts(opts, NULL, errp);
			qemu_opts_del(opts);
            // ESP can't really use this mecahnism right now. Maybe in the future if it's cleaned up...
            if (s->id != XL_DEV_ESP32)
            {
                opts = qemu_opts_create(qemu_find_opts("chardev"), g_strdup(shm_names[s->id]), 1, NULL);
                    qemu_opt_set(opts, "backend","socket", errp);
                    qemu_opt_set(opts, "path", g_strdup_printf("/tmp/%s", shm_names[s->id]), errp);
                    d = qemu_chr_new_from_opts(opts, NULL, errp);
                qemu_opts_del(opts);
            }
		}
		qemu_chr_fe_init(&s->chr[s->id],d, errp);
        if (s->id != XL_DEV_ESP32)
        {
            qemu_chr_fe_set_handlers(&s->chr[s->id], xl_bridge_can_receive, xl_bridge_receive, NULL,
                NULL,s,NULL,true);
        }
        else
        {
            qemu_chr_fe_set_handlers(&s->chr[s->id], xl_bridge_esp_can_receive, xl_bridge_esp_receive, NULL,
                NULL,s,NULL,true);
        }

		qemu_chr_fe_accept_input(&s->chr[s->id]);

		qemu_chr_fe_init(&s->gpio[s->id],d2, errp);
		qemu_chr_fe_set_handlers(&s->gpio[s->id], xl_bridge_gpio_can_receive, xl_bridge_gpio_receive, NULL,
			NULL,s,NULL,true);
		qemu_chr_fe_accept_input(&s->gpio[s->id]);
	}
}

static void xl_bridge_reset(DeviceState *dev)
{
	XLBridgeState *s = XLBRIDGE(dev);
	s->buffer_level = 0;
	for (int i=0; i<XL_BRIDGE_COUNT; i++)
	{
		s->de_pin_asserted[i] = false;
		s->de_pin_used[i] = false;
		if (s->id == XL_DEV_XBUDDY) xl_bridge_reset_in(dev, i, 1);
		s->gpio_states[i].byte = 0;
	}
	s->data_remaining = 0;
	s->data_4b.u32 = 0;
}

static Property xl_bridge_properties[] = {
    DEFINE_PROP_UINT8("device", XLBridgeState, id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void xl_bridge_init(Object *obj)
{
    XLBridgeState *s = XLBRIDGE(obj);
	// Serial I/O IRQs
	DeviceState* dev = DEVICE(obj);
	qdev_init_gpio_in_named(dev, xl_bridge_byte_send, "byte-send", XLBRIDGE_UART_COUNT);
	qdev_init_gpio_in_named(dev, xl_bridge_tx_assert, "tx-assert", 1);
	qdev_init_gpio_out_named(dev, s->byte_receive, "byte-receive", XLBRIDGE_UART_COUNT);

	qdev_init_gpio_in_named(dev, xl_bridge_gpio_in, "gpio-in",XLBRIDGE_PIN_COUNT);
	qdev_init_gpio_in_named(dev, xl_bridge_reset_in, "reset-in", XL_BRIDGE_COUNT);
	qdev_init_gpio_out_named(dev, s->gpio_out, "gpio-out",XLBRIDGE_PIN_COUNT);


}

static const VMStateDescription vmstate_xl_bridge = {
    .name = TYPE_XLBRIDGE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_END_OF_LIST(),
    }
};

static void xl_bridge_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = xl_bridge_reset;
    dc->vmsd = &vmstate_xl_bridge;
	device_class_set_props(dc, xl_bridge_properties);
	dc->realize = xl_bridge_realize;
}
