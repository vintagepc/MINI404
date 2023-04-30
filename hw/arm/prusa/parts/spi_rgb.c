/*
	spi_rgb.c

    SPI-bashing implementation for WS281x neopixel LEDs.

    Written for Mini404 in 2020 by VintagePC <https://github.com/vintagepc/>

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
#include "hw/ssi/ssi.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "../utility/macros.h"
#include "../utility/p404scriptable.h"
#include "../utility/ScriptHost_C.h"
#include "spi_rgb.h"

#define B1_10M5Hz 0b1111111000000
#define B0_10M5Hz 0b111000000000
#define ALT_B1_10M5Hz 0b1110000000
#define ALT_B0_10M5Hz 0b1111111000
#define RST_10M5Hz 535U
#define ALT_RST_10M5Hz 12U


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


struct RGBLedState {
    SSIPeripheral ssidev;

    uint32_t chunks_in;

    uint8_t bit_count;

    uint16_t zero_count;

    colour_t current_colour;
    colour_t set_colour;

    bool passthrough;
    qemu_irq dout;
    qemu_irq reset;
    qemu_irq colour;
    qemu_irq rgb_out[3];
	uint8_t led_type;
	uint8_t flags;
};

#define TYPE_RGB_LED "spi_rgb"
OBJECT_DECLARE_SIMPLE_TYPE(RGBLedState, RGB_LED)

// handler for chaining via din/dout - so only the first LED on the SPI bus needs to decode the bitpattern.
static void rgb_led_din(void* opaque, int n, int level) {
    RGBLedState *s = RGB_LED(opaque);
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
        if (s->led_type == SPI_RGB_WS2811)
        {
            rgb = (s->set_colour.ws8211.r << 16) | (s->set_colour.ws8211.g << 8) | s->set_colour.ws8211.b;
        }
        else
        {
            rgb = (s->set_colour.ws8212.r << 16) | (s->set_colour.ws8212.g << 8) | s->set_colour.ws8212.b;
        }
		if (s->flags & SPI_RGB_FLAG_INVERTED)
		{
			rgb = 0xFFFFFF - rgb;
		}
        qemu_set_irq(s->colour,rgb);
        qemu_set_irq(s->rgb_out[0], rgb >> 16);
        qemu_set_irq(s->rgb_out[1], (rgb >> 8) & 0xFF);
        qemu_set_irq(s->rgb_out[2], rgb & 0xFF);
    }
}

static void rgb_led_reset(void* opaque, int n, int level) {
    RGBLedState *s = RGB_LED(opaque);
    // printf("RESET\n");
    s->passthrough = false;
    s->bit_count = 0;
    s->current_colour.raw = 0;
    s->zero_count = 0;
    s->chunks_in = 0;
    qemu_set_irq(s->reset,0);
}

static uint32_t rgb_led_transfer(SSIPeripheral *dev, uint32_t data)
{
    RGBLedState *s = RGB_LED(dev);
    for (int i=7; i>=0; i--) {
        s->chunks_in <<=1;
        bool new_bit = ((data & (1U<<i))>0);
        s->chunks_in |= new_bit;
        if (new_bit)
        {
            s->zero_count = 0;
        }
        else
        {
            s->zero_count++;
        }
        switch (s->chunks_in) {
            case B1_10M5Hz:
            case B0_10M5Hz:
				if (s->flags & SPI_RGB_FLAG_ALT_TIMINGS) continue;
                s->current_colour.raw <<=1U;
                s->current_colour.raw |= (s->chunks_in == B1_10M5Hz);
                s->bit_count++;
                s->chunks_in = 0;
                break;
			case ALT_B0_10M5Hz:
			case ALT_B1_10M5Hz:
				if (!(s->flags & SPI_RGB_FLAG_ALT_TIMINGS)) continue;
				s->current_colour.raw <<=1U;
                s->current_colour.raw |= (s->chunks_in == ALT_B1_10M5Hz);
                s->bit_count++;
                s->chunks_in = 0;
                break;
        }
        if (s->bit_count == 24)
        {
			// if (s->no_cs) printf("LED data: %"PRIx32"\n", s->current_colour.raw);
            // Reached the RGB length.
            rgb_led_din(s, 0,s->current_colour.raw);
            s->bit_count = 0;
        }
        if (s->zero_count == (s->flags & SPI_RGB_FLAG_ALT_TIMINGS ? ALT_RST_10M5Hz : RST_10M5Hz)) {
            rgb_led_reset(s, 0,0);
        }
    }
    return 0;

}

static int rgb_led_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
	return ScriptLS_Unhandled;
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(RGBLedState, rgb_led, RGB_LED, SSI_PERIPHERAL, {TYPE_P404_SCRIPTABLE}, {NULL})

static void rgb_led_finalize(Object *obj)
{
}


static void rgb_led_init(Object *obj)
{
}


static void rgb_led_realize(SSIPeripheral *d, Error **errp)
{
    DeviceState *dev = DEVICE(d);
    RGBLedState *s = RGB_LED(d);
	if (s->flags & SPI_RGB_FLAG_NO_CS) d->cs = true;

    rgb_led_reset(s, 0,0);

    qdev_init_gpio_in(dev, rgb_led_din ,1);
    qdev_init_gpio_out(dev, &s->dout, 1);

    qdev_init_gpio_in_named(dev, rgb_led_reset, "reset",1);
    qdev_init_gpio_out_named(dev, &s->reset, "reset-out", 1);
    qdev_init_gpio_out_named(dev, &s->colour, "colour", 1);
    qdev_init_gpio_out_named(dev, s->rgb_out, "rgb-out", 3);

}

static Property rgb_led_properties[] = {
    DEFINE_PROP_UINT8("led-type",RGBLedState, led_type, (uint64_t)SPI_RGB_WS2812),
    DEFINE_PROP_UINT8("flags",RGBLedState, flags, SPI_RGB_FLAG_NONE),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_spi_rgb = {
    .name = TYPE_RGB_LED,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(ssidev,RGBLedState),
        VMSTATE_UINT32(chunks_in,RGBLedState),
        VMSTATE_UINT8(bit_count,RGBLedState),
        VMSTATE_UINT16(zero_count,RGBLedState),
        VMSTATE_UINT32(current_colour.raw,RGBLedState),
        VMSTATE_UINT32(set_colour.raw,RGBLedState),
        VMSTATE_BOOL(passthrough,RGBLedState),
        VMSTATE_END_OF_LIST()
    }
};

static void rgb_led_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_spi_rgb;
    device_class_set_props(dc, rgb_led_properties);


    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = rgb_led_process_action;

    k->realize = rgb_led_realize;
    k->transfer = rgb_led_transfer;
    k->cs_polarity = SSI_CS_HIGH;
}
