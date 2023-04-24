/*
    mod_bed_visuals.c - simple text graphics for modular bed:

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

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
#include "qemu/option.h"
#include "qemu/config-file.h"
#include "qapi/error.h"
#include "qom/object.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "../utility/macros.h"
#include "hw/qdev-properties.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "ui/console.h"

struct ModBedVisualsState {
    SysBusDevice parent;

    Chardev *input_source;

    CharBackend be;

	uint8_t rows;
	uint8_t cols;

	uint8_t data[(20*21)+1];
	uint8_t temps[16];
	uint8_t on_pwm[16];
	uint8_t* on_ptrs[16];
	uint8_t* temp_ptrs[16];
};


#define TYPE_MOD_BED_VISUALS "modular-bed-visuals"
OBJECT_DECLARE_SIMPLE_TYPE(ModBedVisualsState, MOD_BED_VISUALS)

static int mod_bed_visuals_can_read(void* opaque) {
	return 0;
}

static void G_GNUC_PRINTF (2, 3) mod_bed_visuals_printf(void *opaque,
                                                       const char *fmt, ...)
{
    ModBedVisualsState *s = MOD_BED_VISUALS(opaque);
    char* buf;
    va_list ap;
    va_start(ap, fmt);
    buf = g_strdup_vprintf(fmt, ap);
    va_end(ap);
    uint8_t cr = '\r';
    // int start = 0;
    for (int i=0; i<strlen(buf); i++)
    {
        // This is not particularly efficient, but it'll do for now.
        if (buf[i]=='\n') {
            qemu_chr_fe_write(&s->be, &cr, 1);
        }
        qemu_chr_fe_write(&s->be, (uint8_t*)buf+i, 1);
    }
    // qemu_chr_fe_write_all(&s->be, (uint8_t*)buf, strlen(buf));
    g_free(buf);
}

extern void mod_bed_visuals_print_out(void* opaque, const char *str);

extern void mod_bed_visuals_print_out(void* opaque, const char *str) {
    mod_bed_visuals_printf(opaque, "%s\n",str);
}

static void mod_bed_visuals_event(void *opaque, QEMUChrEvent event)
{
    ModBedVisualsState *s = MOD_BED_VISUALS(opaque);
    switch (event) {
        case CHR_EVENT_OPENED:
		mod_bed_visuals_printf(s, "%s", s->data);
        break;
        default:
        break;
    }
}

static const unsigned char box[5][6] = {{0xDA, 0xC4, 0xC4, 0xC4, 0xBF, 0},
								{0xB3, 0x20, 0x20, 0x20, 0xB3, 0},
								{0xB3, 0x30, 0x31, 0x32, 0xB3, 0},
								{0xB3, 0x20, 0xB2, 0x20, 0xB3, 0},
								{0xC0, 0xC4, 0xC4, 0xC4, 0xD9, 0}};

static void mod_bed_init_data(ModBedVisualsState *s)
{
	// construct ascii art
	uint8_t rwidth = 5*s->cols;
	int t_index = 0;
	int l_index = 0;
	for (int k=0; k<s->rows; k++)
	{
		for (int j=0; j<5; j++)
		{
			for (int i=0; i<rwidth; i++)
			{
				int d_index = i+(rwidth*j)+(k*rwidth*5);
				if (box[j][i%5] == 0x30)
				{
					s->temp_ptrs[t_index++] = &s->data[d_index];
				}
				if (box[j][i%5] == 0xB2)
				{
					s->on_ptrs[l_index++] = &s->data[d_index];
				}
				// printf("idx: %d %d %d\n",i,j,k);
				s->data[d_index] = box[j][i%5];
			}
		}
	}
	s->data[400] = ' ';

}

static void mod_bed_update_temp(ModBedVisualsState *s, int index)
{
	uint8_t val = s->temps[index];
	uint8_t h = val/100;
	uint8_t t = (val%100)/10;
	uint8_t ones = (val%10);
	uint8_t* ptr = s->temp_ptrs[index];
	if (ptr)
	{
		*ptr++ = 0x30+h;
		*ptr++ = 0x30+t;
		*ptr++ = 0x30+ones;
	}
	mod_bed_visuals_printf(s, "\r\n\n%s", s->data);
}

static void mod_bed_update_onoff(ModBedVisualsState *s, int index)
{
	if (s->on_ptrs[index])
	{
		uint8_t ind_char = 0x20;
		switch (s->on_pwm[index])
		{
			case 1 ... 63 :
				ind_char = 0xB0;
				break;
			case 64 ... 127:
				ind_char = 0xB1;
				break;
			case 128 ... 191:
				ind_char = 0xB2;
				break;
			case 192 ... 255:
				ind_char = 0xDB;
				break;
			default:
				break;
		}
		*s->on_ptrs[index] = ind_char;
	}
	mod_bed_visuals_printf(s, "\r\n\n%s", s->data);
}

static void mod_bed_visuals_temp_in(void* opaque, int n, int level)
{
    ModBedVisualsState *s = MOD_BED_VISUALS(opaque);

	s->temps[n] = level/256U;
	mod_bed_update_temp(s, n);
}

static void mod_bed_visuals_led_in(void* opaque, int n, int level)
{
    ModBedVisualsState *s = MOD_BED_VISUALS(opaque);
	for (int i=1; i<19; i++)
	s->data[400+i] = level? 0xB2 : 0xB0;
	mod_bed_visuals_printf(s, "\r\n\n%s", s->data);
}

static void mod_bed_visuals_heat_in(void* opaque, int n, int level)
{
    ModBedVisualsState *s = MOD_BED_VISUALS(opaque);
	s->on_pwm[n] = level;
	mod_bed_update_onoff(s, n);
}

static void mod_bed_visuals_read(void *opaque, const uint8_t *buf, int size){
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(ModBedVisualsState, mod_bed_visuals, MOD_BED_VISUALS, SYS_BUS_DEVICE, {NULL})

static void mod_bed_visuals_finalize(Object *obj)
{

}

static void mod_bed_visuals_init(Object *obj)
{

}

static void mod_bed_visuals_realize(DeviceState *d, Error **errp)
{
    ModBedVisualsState *s = MOD_BED_VISUALS(d);
	QemuOpts *opts;

	opts = qemu_opts_create(qemu_find_opts("chardev"), "Modular-Bed", 1, NULL);
	qemu_opt_set(opts, "backend","vc", &error_abort);
	qemu_opt_set(opts, "cols", "20", &error_abort);
	qemu_opt_set(opts, "rows", "21", &error_abort);

	s->input_source =  qemu_chr_new_from_opts(opts, NULL, &error_abort);
    qemu_chr_fe_init(&s->be, s->input_source, errp);

    qemu_chr_fe_set_handlers(&s->be, mod_bed_visuals_can_read, mod_bed_visuals_read, mod_bed_visuals_event, NULL, s, NULL, true);

	qemu_opts_del(opts);

	mod_bed_init_data(s);

	for(int i=0; i<16; i++)
	{
		s->temps[i] = 25;
		s->on_pwm[i] = 0;
		mod_bed_update_temp(s, i);
		mod_bed_update_onoff(s,i);
	}

	qdev_init_gpio_in_named(d, mod_bed_visuals_temp_in, "temp-in",s->rows*s->cols);
	qdev_init_gpio_in_named(d, mod_bed_visuals_heat_in, "heat-in",s->rows*s->cols);
	qdev_init_gpio_in_named(d, mod_bed_visuals_led_in, "led-in",1);

}

static Property mod_bed_visuals_properties[] = {
    DEFINE_PROP_UINT8("rows", ModBedVisualsState, rows, 4),
    DEFINE_PROP_UINT8("cols", ModBedVisualsState, cols, 4),
    DEFINE_PROP_END_OF_LIST(),
};

static void mod_bed_visuals_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = mod_bed_visuals_realize;
    device_class_set_props(dc, mod_bed_visuals_properties);
}
