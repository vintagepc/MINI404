/*
    c1_bridge.h - (unix) socket transport for XL, which needs
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
#include "c1_bridge.h"
#include "qemu/atomic.h"
#include "qemu/option.h"
#include "qemu/config-file.h"
#include "sysemu/runstate.h"
#include "qapi/qapi-commands-run-state.h"
#include "qapi/qapi-events-run-state.h"
#include "trace.h"

static const char* shm_names[C1_BRIDGE_COUNT] =
{
	"PC1_XBUDDY",
	"PC1_EXT",
};

// Messages are composed of some GPIO status bits
// and a serial byte.

#define TYPE_C1BRIDGE "c1-bridge"

OBJECT_DECLARE_SIMPLE_TYPE(C1BridgeState, C1BRIDGE)

typedef union gpio_state_t{
	struct {
		uint8_t reset  :1;
		uint8_t _reserved :7;
	} bits;
	uint8_t byte;
} gpio_state_t;


struct C1BridgeState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/

	// Clients (anything not the board) only use their chardev and connect
	// to the appropriate socket. Base XLBuddy creates all sockets and waits for
	// at least one extruder and one bed.
	CharBackend chr[C1_BRIDGE_COUNT];
	CharBackend gpio[C1_BRIDGE_COUNT];

	qemu_irq byte_receive;

	qemu_irq gpio_out[C1BRIDGE_PIN_COUNT];

	gpio_state_t gpio_states[C1_BRIDGE_COUNT];

	bool de_pin_used[C1_BRIDGE_COUNT];
	bool de_pin_asserted[C1_BRIDGE_COUNT];

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

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(C1BridgeState, c1_bridge, C1BRIDGE,SYS_BUS_DEVICE,{NULL});

static void c1_bridge_tx_assert(void *opaque, int n, int level)
{
	C1BridgeState *s = C1BRIDGE(opaque);
	trace_c1_bridge_tx_assert(shm_names[s->id], level);
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
		if (s->id == C1_DEV_XBUDDY) // broadcast messages to 0.
		{
			switch (s->buffer[0]) {
				case 0x11:	// Tool. Route appropriately.
					qemu_chr_fe_write_all(&s->chr[C1_DEV_EXT + (s->buffer[0] - 0x11)],(uint8_t*)s->buffer, sizeof(s->buffer[0]) * s->buffer_level);
					break;
				case 0x21:
					qemu_chr_fe_write_all(&s->chr[C1_DEV_EXT + (s->buffer[0] - 0x21)],(uint8_t*)s->buffer, sizeof(s->buffer[0]) * s->buffer_level);
					break;
				default: // catch-all.
					printf("Unexpected message starting byte, %02x\n",s->buffer[0]);
					/* FALLTHRU */
				case 0x00: // Broadcast message (e.g. bootstrap)
					for (int i=C1_DEV_XBUDDY+1; i<C1_BRIDGE_COUNT; i++)
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
		trace_c1_bridge_msg_send(shm_names[s->id], s->buffer[0], s->buffer_level);
		// for (int i=0; i<s->buffer_level; i++)
		// {
		// 	printf("%02x ",s->buffer[i]);
		// }
		// printf("\n");
	}
}

static void c1_bridge_byte_send(void *opaque, int n, int level)
{
	// TODO - construct the control byte.
	C1BridgeState *s = C1BRIDGE(opaque);
	//uint16_t data = 0x00FF | (level & 0xFF)<<8; // swap the bytes here so they come in in the right order.
	// Buffer up the data for a single-shot transmit.
	s->buffer[s->buffer_level++] = level & 0xFF;
	//trace_c1_bridge_byte_input(shm_names[s->id], level);

	// Right now the bootloader doesn't support the TE pin for dwarf :(
	if (!s->de_pin_used[s->id] && s->id >= C1_DEV_EXT)
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


static int c1_bridge_can_receive(void *opaque)
{
   	C1BridgeState *s = C1BRIDGE(opaque);
	// if (s->de_pin_asserted[s->id]){
	// 	printf("recieve blocked by DE assert\n");
	// }
    return s->de_pin_asserted[s->id]? 0 : 64;
}

static int c1_bridge_gpio_can_receive(void *opaque)
{
    return 1; // Currently only 1 byte increments.
}

static void c1_bridge_receive(void *opaque, const uint8_t *buf, int size)
{
   	C1BridgeState *s = C1BRIDGE(opaque);
	//#define FILTER size < 20
	#define FILTER false
    // assert(size % 2 == 0);
	trace_c1_bridge_msg_recv(shm_names[s->id], buf[0], size);
	if (FILTER) printf(" %u Received: ", s->id);
	for (const uint8_t* p = buf; p<buf+size; p++)
	{
		qemu_set_irq(s->byte_receive, *p);
		if (FILTER) printf("%02x ",*p);
	}
	if (FILTER) printf("\n");
}

#define PROCESS_BIT(pin, field) \
		if (s->gpio_states[s->id].bits.field != state.bits.field) \
		{ \
			qemu_set_irq(s->gpio_out[pin],state.bits.field); \
		}

static void c1_bridge_gpio_receive(void *opaque, const uint8_t *buf, int size)
{
   	C1BridgeState *s = C1BRIDGE(opaque);
	assert(size==1);

	gpio_state_t state = {.byte  = buf[0]};

	// printf("GPIO receive: %u - %02x\n", s->id, buf[0]);

	if (s->id != C1_DEV_XBUDDY)
	{
		if (state.bits.reset)
		{
			trace_c1_bridge_device_reset(shm_names[s->id]);
    		qemu_system_reset_request(SHUTDOWN_CAUSE_SUBSYSTEM_RESET);
		}
		s->gpio_states[s->id].byte = state.byte;
	}
	else
	{
		//TODO - Return signals (fsens?) from remote to xbuddy.
	}
}

static void c1_bridge_reset_in(void *opaque, int n, int level)
{
	C1BridgeState *s = C1BRIDGE(opaque);
	if (s->id != C1_DEV_XBUDDY)
	{
		printf("Got GPIO inputs on client peripheral board %s - ignored.\n",shm_names[s->id]);
		return;
	}
	s->gpio_states[n].bits.reset = level>0;
	// Dispatch the new state.
	qemu_chr_fe_write_all(&s->gpio[n],&s->gpio_states[n].byte, 1);
	// if (level) printf("Sent reset to %02x, %02x\n", n, s->gpio_states[n].byte);//, shm_names[target]);
}

static void c1_bridge_gpio_in(void *opaque, int n, int level)
{
	C1BridgeState *s = C1BRIDGE(opaque);
	if (s->id != C1_DEV_XBUDDY)
	{
		printf("Got GPIO inputs on client peripheral board %s - ignored.\n",shm_names[s->id]);
		return;
	}
	uint8_t target = 0; //n/C1BRIDGE_PIN_COUNT;
	uint8_t pin = n%C1BRIDGE_PIN_COUNT;
	switch (pin)
	{
		default:
			printf("ERR: Unhanled pin enumeration for %s\n",__func__);
	}
	// Dispatch the new state.
	for (int i=0; i<C1_BRIDGE_COUNT; i++)
	{
		qemu_chr_fe_write_all(&s->gpio[i],&s->gpio_states[target].byte, 1);
	}
	printf("Sent gpio update %02x\n", s->gpio_states[target].byte);//, shm_names[target]);
}

static void c1_bridge_finalize(Object *obj)
{
}

static void c1_bridge_realize(DeviceState *dev, Error **errp)
{
    C1BridgeState *s = C1BRIDGE(dev);
	if (s->id == C1_DEV_XBUDDY)
	{
		for (int i=C1_DEV_EXT; i>=C1_DEV_EXT; i--)
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
				QemuOpts *opts;
				// Now create the IO (GPIO) channel.
				trace_c1_bridge_create_control_socket(shm_names[i]);
				opts = qemu_opts_create(qemu_find_opts("chardev"), g_strdup_printf("%s-io",shm_names[i]), 1, NULL);
					qemu_opt_set(opts, "backend","socket", errp);
					qemu_opt_set(opts, "path", g_strdup_printf("/tmp/%s-io", shm_names[i]), errp);
					qemu_opt_set(opts, "server", "on", errp);
					qemu_opt_set_bool(opts, "wait", false, errp);
					d2 = qemu_chr_new_from_opts(opts, NULL, errp);
				qemu_opts_del(opts);

				trace_c1_bridge_create_socket(shm_names[i]);
				opts = qemu_opts_create(qemu_find_opts("chardev"), g_strdup(shm_names[i]), 1, NULL);
					qemu_opt_set(opts, "backend","socket", errp);
					qemu_opt_set(opts, "path", g_strdup_printf("/tmp/%s", shm_names[i]), errp);
					qemu_opt_set(opts, "server", "on", errp);
					d = qemu_chr_new_from_opts(opts, NULL, errp);
				qemu_opts_del(opts);

			}
			qemu_chr_fe_init(&s->chr[i],d, errp);
			qemu_chr_fe_set_handlers(&s->chr[i], c1_bridge_can_receive, c1_bridge_receive, NULL,
					NULL,s,NULL,true);
			qemu_chr_fe_accept_input(&s->chr[i]);

			qemu_chr_fe_init(&s->gpio[i],d2, errp);
			qemu_chr_fe_set_handlers(&s->gpio[i], c1_bridge_gpio_can_receive, c1_bridge_gpio_receive, NULL,
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
			trace_c1_bridge_open_socket(shm_names[s->id]);
			opts = qemu_opts_create(qemu_find_opts("chardev"), g_strdup(shm_names[s->id]), 1, NULL);
				qemu_opt_set(opts, "backend","socket", errp);
				qemu_opt_set(opts, "path", g_strdup_printf("/tmp/%s", shm_names[s->id]), errp);
				d = qemu_chr_new_from_opts(opts, NULL, errp);
			qemu_opts_del(opts);
		}
		qemu_chr_fe_init(&s->chr[s->id],d, errp);
		qemu_chr_fe_set_handlers(&s->chr[s->id], c1_bridge_can_receive, c1_bridge_receive, NULL,
			NULL,s,NULL,true);
		qemu_chr_fe_accept_input(&s->chr[s->id]);

		qemu_chr_fe_init(&s->gpio[s->id],d2, errp);
		qemu_chr_fe_set_handlers(&s->gpio[s->id], c1_bridge_gpio_can_receive, c1_bridge_gpio_receive, NULL,
			NULL,s,NULL,true);
		qemu_chr_fe_accept_input(&s->gpio[s->id]);
	}
}

static void c1_bridge_reset(DeviceState *dev)
{
	C1BridgeState *s = C1BRIDGE(dev);
	s->buffer_level = 0;
	for (int i=0; i<C1_BRIDGE_COUNT; i++)
	{
		s->de_pin_asserted[i] = false;
		s->de_pin_used[i] = false;
		if (s->id == C1_DEV_XBUDDY) c1_bridge_reset_in(dev, i, 1);
		s->gpio_states[i].byte = 0;
	}
	s->data_remaining = 0;
	s->data_4b.u32 = 0;
}

static Property c1_bridge_properties[] = {
    DEFINE_PROP_UINT8("device", C1BridgeState, id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void c1_bridge_init(Object *obj)
{
    C1BridgeState *s = C1BRIDGE(obj);
	// Serial I/O IRQs
	DeviceState* dev = DEVICE(obj);
	qdev_init_gpio_in_named(dev, c1_bridge_byte_send, "byte-send", 1);
	qdev_init_gpio_in_named(dev, c1_bridge_tx_assert, "tx-assert", 1);
	qdev_init_gpio_out_named(dev, &s->byte_receive, "byte-receive", 1);

	qdev_init_gpio_in_named(dev, c1_bridge_gpio_in, "gpio-in",C1BRIDGE_PIN_COUNT);
	qdev_init_gpio_in_named(dev, c1_bridge_reset_in, "reset-in", C1_BRIDGE_COUNT);
	qdev_init_gpio_out_named(dev, s->gpio_out, "gpio-out",C1BRIDGE_PIN_COUNT);


}

static const VMStateDescription vmstate_c1_bridge = {
    .name = TYPE_C1BRIDGE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField []) {
        VMSTATE_END_OF_LIST(),
    }
};

static void c1_bridge_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_legacy_reset(dc, c1_bridge_reset);
    dc->vmsd = &vmstate_c1_bridge;
	device_class_set_props(dc, c1_bridge_properties);
	dc->realize = c1_bridge_realize;
}
