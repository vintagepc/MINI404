/*
	ws281x.c

    GPIO implementation for WS281x neopixel LEDs.
	Requires icount auto to work to get correct bit-bang timings.

    Written for Mini404 in 2022-3 by VintagePC <https://github.com/vintagepc/>

    Portions referenced from hw/display/ssd0323.c by Paul Brook

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
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "sysemu/cpu-timers.h"
#include "../utility/macros.h"

typedef union {
    uint32_t raw;
    struct {
        uint8_t b;
        uint8_t r;
        uint8_t g;
        uint8_t _unused;
    } QEMU_PACKED ws8212;
    struct {
        uint8_t b;
        uint8_t g;
        uint8_t r;
        uint8_t _unused;
    } QEMU_PACKED ws8211;
} colour_t;

#define TYPE_WS281X "ws281x"

OBJECT_DECLARE_SIMPLE_TYPE(WS281xState, WS281X)

struct WS281xState {
    SysBusDevice dev;

    uint8_t bit_count;

	int64_t last_highc, last_lowc;

    colour_t current_colour;
    colour_t set_colour;

    bool passthrough;
	bool disabled;
    qemu_irq dout;
    qemu_irq reset;
    qemu_irq colour;

};

#define T0H_NS 350
#define T1H_NS 700
#define T0L_NS 800
#define T1L_NS 600
#define THRESH_NS 150
#define RESET_NS 50000

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(WS281xState, ws281x, WS281X, SYS_BUS_DEVICE, {NULL} );

// handler for chaining via din/dout - so only the first LED on the SPI bus needs to decode the bitpattern.
static void ws281x_din(void* opaque, int n, int level) {
    WS281xState *s = WS281X(opaque);
    if (s->passthrough)
    {
        // printf("Passthru: %08x\n", level);
        qemu_set_irq(s->dout, level);
    }
    else
    {
        s->set_colour.raw = level;
        s->passthrough = true;
        uint32_t rgb;
		rgb = (s->set_colour.ws8212.r << 16) | (s->set_colour.ws8212.g << 8) | s->set_colour.ws8212.b;
        qemu_set_irq(s->colour,rgb);
        // printf("Colour received (rgb): %02x %02x %02x\n",s->set_colour.ws8212.r,s->set_colour.ws8212.g,s->set_colour.ws8212.b);
    }
}

static void ws281x_reset(void* opaque, int n, int level) {
    WS281xState *s = WS281X(opaque);
    // printf("RESET\n");
    s->passthrough = false;
    s->bit_count = 0;
    s->current_colour.raw = 0;
    qemu_set_irq(s->reset,0);
}

static void ws281x_gpio(void* opaque, int n, int level)
{
    WS281xState *s = WS281X(opaque);
	if (s->disabled)
	{
		return;
	}
	if (level)
	{
		s->last_highc = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); //icount_get_raw();
		if (s->last_highc - s->last_lowc >= RESET_NS)
		{
			s->bit_count = 0;
			s->current_colour.raw = 0;
			s->passthrough = false;
			// printf("reset\n");
		}
	}
	else
	{
		s->last_lowc =  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); // icount_get_raw();
		if (s->last_lowc == 0)
		{
			// printf("ERR: WS281x detected '-icount ' is not set. LED will not function without accurate timing!\n");
			//s->disabled = true;
		}
		s->current_colour.raw <<= 1;
		s->current_colour.raw |= abs((s->last_lowc - s->last_highc) - T1H_NS) <= THRESH_NS; // This value is highly dependent on the value of icount. Tested with -icount 2 gets ns values close to correct in real ns
		// printf("last high: %ld\n",(s->last_lowc - s->last_highc));
		s->bit_count++;
        if (s->bit_count == 24)
        {
            // Reached the RGB length.
			// printf("data: %06x\n", s->current_colour.raw);
            ws281x_din(opaque, 0,s->current_colour.raw);
            s->bit_count = 0;
			s->current_colour.raw = 0;
        }
    }
}


static void ws281x_finalize(Object *obj)
{
}

static void ws281x_init(Object *obj)
{
	DeviceState *dev = DEVICE(obj);
    WS281xState *s = WS281X(obj);

    ws281x_reset(s, 0,0);

    qdev_init_gpio_in(dev, ws281x_gpio ,1);
    qdev_init_gpio_out(dev, &s->dout, 1);

    qdev_init_gpio_in_named(dev, ws281x_reset, "reset",1);
    qdev_init_gpio_out_named(dev, &s->reset, "reset-out", 1);
    qdev_init_gpio_out_named(dev, &s->colour, "colour", 1);

}


static void ws281x_class_init(ObjectClass *klass, void *data)
{
}
