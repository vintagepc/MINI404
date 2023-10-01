/*
 * Software/bitbanged GPIO SSI interpreter module.
 *
 * Written for Mini404 in 2022 by VintagePC <https://github.com/vintagepc/>
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "hw/irq.h"

#define TYPE_SOFTWARE_SPI "software-spi"

OBJECT_DECLARE_SIMPLE_TYPE(SoftwareSPIState, SOFTWARE_SPI);

typedef struct SoftwareSPIState {
    /* <private> */
    SysBusDevice parent;

    /* <public> */

	uint8_t byte_in;
	uint8_t byte_out;
	uint8_t clock_count;
	bool current_mosi;

	qemu_irq miso;

    SSIBus *ssi;

} SoftwareSPIState;


static void software_spi_reset(DeviceState *dev)
{
    SoftwareSPIState *s = SOFTWARE_SPI(dev);

    s->byte_in = 0;
	s->byte_out = 0;
	s->clock_count = 0;
	s->current_mosi = 0;
}


static void software_spi_mosi(void *opaque, int n, int level)
{
    SoftwareSPIState *s = SOFTWARE_SPI(opaque);
	// printf("MOSI: %u\n", level);
	s->current_mosi = level;
}

static void software_spi_miso(void *opaque, int n, int level)
{
    SoftwareSPIState *s = SOFTWARE_SPI(opaque);
	// printf("MISO: %u\n", level);
	s->byte_in = level;
}

static void software_spi_sck(void *opaque, int n, int level)
{
    SoftwareSPIState *s = SOFTWARE_SPI(opaque);
	if (level)
	{
		// Clock bit on MOSI
		s->byte_out	<<= 1;
		s->byte_out |= s->current_mosi;

		// printf("bit: %02x (%02x)\n", (s->byte_in&0x80)>0, s->byte_in);
		qemu_set_irq(s->miso, (s->byte_in&0x80)>0 );
		s->byte_in <<= 1;
		s->clock_count++;
	}
	if (s->clock_count == 8)
	{
		// We don't care about the return because it's too late -
		// the bits already need to have been clocked out, hence we must use the miso-byte input.
		ssi_transfer(s->ssi, s->byte_out);
		s->clock_count = 0;
	}
}


static const VMStateDescription vmstate_software_spi = {
    .name = TYPE_SOFTWARE_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {

        VMSTATE_END_OF_LIST()
    }
};

static void software_spi_init(Object *obj)
{
    SoftwareSPIState *s = SOFTWARE_SPI(obj);

    DeviceState *dev = DEVICE(obj);

	qdev_init_gpio_in_named(dev, software_spi_mosi, "mosi",1);
	qdev_init_gpio_in_named(dev, software_spi_sck, "sck",1);
	qdev_init_gpio_in_named(dev, software_spi_miso, "miso-byte",1);
	qdev_init_gpio_out_named(dev, &s->miso, "miso", 1);

    s->ssi = ssi_create_bus(dev, "ssi");
}

static void software_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = software_spi_reset;
    dc->vmsd = &vmstate_software_spi;
}

static const TypeInfo software_spi_info = {
    .name          = TYPE_SOFTWARE_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SoftwareSPIState),
    .instance_init = software_spi_init,
    .class_init    = software_spi_class_init,
};

static void software_spi_register_types(void)
{
    type_register_static(&software_spi_info);
}

type_init(software_spi_register_types)
