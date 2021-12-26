/*
	dashboard_2d.c - Just LEDs for now...

    Written for Mini404 in 2021 by VintagePC <https://github.com/vintagepc/>

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
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "ui/console.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "../utility/macros.h"
#include "../utility/p404_motor_if.h"
#include "hw/irq.h"
#include "../utility/text_helper.h"
//#define DEBUG_ILI9488 1

// max count
#define N_LEDS 15
#define N_MOTORS 10
#define N_FANS 6
#define N_THERM 6
#define LINE_HEIGHT (FONT_HEIGHT)

// 3 extra lines for fan, therm, indicators.
#define DPY_MAX_ROWS (LINE_HEIGHT *(N_MOTORS + 3U)) 
#define DPY_MAX_COLS 480
#define LED_HT LINE_HEIGHT

#define LED_W DPY_MAX_COLS/N_LEDS

#define DPY_BUFFSIZE sizeof(uint32_t)*(DPY_MAX_ROWS)*DPY_MAX_COLS

#define TYPE_DB2D_DISPLAY "2d-dashboard"

OBJECT_DECLARE_SIMPLE_TYPE(Dashboard2DState, DB2D_DISPLAY)

struct Dashboard2DState {
    SysBusDevice  busdev;

    QemuConsole *con;

	bool redraw;

    uint32_t framebuffer[DPY_MAX_ROWS * DPY_MAX_COLS];

    char* ind_labels;

    pixman_color_t leds[N_LEDS];
    uint32_t led_colours[N_LEDS];

    uint16_t current_rows;

    uint8_t fan_count;
    uint8_t fan_pwms[N_FANS];
    uint16_t fan_rpms[N_FANS];
    char fan_printf[N_FANS][6];

    uint8_t therm_count;
    uint8_t therm_pwms[N_THERM];
    char therm_printf[N_THERM][7];

    P404MotorIF* motors[N_MOTORS];
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(Dashboard2DState, dashboard_2d, DB2D_DISPLAY, SYS_BUS_DEVICE,{NULL});

static pixman_color_t attr_norm[2] = {
    QEMU_RGB(0xFF, 0xFF, 0xFF),
    QEMU_RGB(0,0,0)
};

static pixman_color_t attr_stealth[2] = {
    QEMU_RGB(0,0xFF,0),
    QEMU_RGB(0,0,0),
};

static pixman_color_t attr_stall[2] = {
    QEMU_RGB(0xFF,0,0),
    QEMU_RGB(0,0,0),
};

static pixman_color_t attr_disabled[2] = {
    QEMU_RGB(0,0,0),
    QEMU_RGB(0xAA,0xAA,0xAA),
};

static pixman_color_t* dashboard_2d_get_attr(const p404_motorif_status_t *s)
{
    return s->status.stalled ? attr_stall : 
        (!s->status.enabled? attr_disabled :
            ( s->status.stealth ? attr_stealth : attr_norm )
        );
}

static void dashboard_2d_update_display(void *opaque)
{
    Dashboard2DState *s = DB2D_DISPLAY(opaque);
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *dest = NULL;
 
    const p404_motorif_status_t* m[10] = {NULL};

    int motor_count = 0;

    for (int i=0; i<N_MOTORS; i++)
    {
        if (s->motors[i] != NULL)
        {
            motor_count++;
            m[i] = p404_motor_if_get_status(s->motors[i]);
            if (m[i]->status.changed)
            {
                s->redraw = true;
                if (m[i]->max_pos>0) // negative max pos means no endstop...
                {
                    int row = FONT_HEIGHT*i;
                    int offset = FONT_WIDTH + (FONT_WIDTH/2);
                    int index = (DPY_MAX_COLS*row)+offset;
                    uint16_t scale =  (m[i]->max_pos<=200 ? 2 : 1);
                    // Clear and generate the line.
                    memset(&s->framebuffer[row*DPY_MAX_COLS], 0, DPY_MAX_COLS*sizeof(uint32_t)*FONT_HEIGHT);
                    for (int j=1; j<FONT_HEIGHT-1; j++)
                    {
                        int offset = index + (j*DPY_MAX_COLS);
                        s->framebuffer[offset] = 0xFFFF0000;
                        s->framebuffer[offset + 1] = 0xFFFF0000;
                        s->framebuffer[offset + 2 + MIN(400, scale*(uint16_t)m[i]->current_pos)] = 0xFF00FFFFU;
                        s->framebuffer[offset + 3 + MIN(400, scale*(uint16_t)m[i]->max_pos)] = 0xFFFF0000U;
                        s->framebuffer[offset + 4 + MIN(400, scale*(uint16_t)m[i]->max_pos)] = 0xFFFF0000U;
                    }
                }
            }
            
        }
    }

    if (!s->redraw)
        return;

    dest = surface_data(surface);
    memcpy(dest, s->framebuffer, DPY_MAX_COLS * (FONT_HEIGHT*(motor_count)) * sizeof(uint32_t) );

    for (int i=0; i<N_MOTORS; i++)
    {
        if (m[i] && m[i]->status.changed)
        {
            char pos[9];
            vga_putcharxy(s->con, 0, i, m[i]->label, dashboard_2d_get_attr(m[i]) );
            snprintf(pos, sizeof(pos),"%8.3f", m[i]->current_pos);
            for (int j=0; j<8; j++)
            {
                vga_putcharxy(s->con, (60-8)+j, i, pos[j], attr_norm);
            }
        }
    }

    for (int i=0; i<MIN(s->fan_count, N_FANS); i++)
    {
        bool light = s->fan_pwms[i]<128;
        pixman_color_t fan_col[2] = {
            QEMU_RGB(0xFF*light, 0xFF*light, 0xFF*light),
            QEMU_RGB(0, s->fan_pwms[i], 0),
        };
        vga_putcharxy(s->con, (10*i) + 0, motor_count , 'F', attr_norm);
        vga_putcharxy(s->con, (10*i) + 1, motor_count , '0' + i, attr_norm);
        vga_putcharxy(s->con, (10*i) + 2, motor_count , ':', attr_norm);
        for (int j=0; j<sizeof(s->fan_printf[1])-1; j++)
        {
            vga_putcharxy(s->con, (10*i) + 4 + j, motor_count, s->fan_printf[i][j], fan_col);
        }
    }

    for (int i=0; i<MIN(s->therm_count, N_THERM); i++)
    {
        bool light = s->therm_pwms[i]<128;
        pixman_color_t ht_col[2] = {
            QEMU_RGB(0xFF*light, 0xFF*light, 0xFF*light),
            QEMU_RGB(s->therm_pwms[i], 0,0),
        };
        
        vga_putcharxy(s->con, (10*i) + 0, motor_count + 1 , 'T', attr_norm);
        vga_putcharxy(s->con, (10*i) + 1, motor_count + 1 , '0' + i, attr_norm);
        vga_putcharxy(s->con, (10*i) + 2, motor_count + 1 , ':', attr_norm);
        for (int j=0; j<sizeof(s->therm_printf[1])-2; j++)
        {
            vga_putcharxy(s->con, (10*i) + 4 + j, motor_count + 1, s->therm_printf[i][j], ht_col);
        }
    }

    for (int i=0; i<MIN(strlen(s->ind_labels), N_LEDS); i++)
    {       
        bool light = s->leds[i].red<(128<<8) && s->leds[i].green<(128<<8) && s->leds[i].blue<(128<<8);
        pixman_color_t fg = QEMU_RGB(255*light, 255*light, 255*light);
        vga_putcharxy_s(s->con, (4*i) +1 , motor_count + 2 , s->ind_labels[i], &fg, &s->leds[i]);
        vga_putcharxy_s(s->con, (4*i) +0 , motor_count + 2 , ' ', &fg, &s->leds[i]);
        vga_putcharxy_s(s->con, (4*i) +2 , motor_count + 2 , ' ', &fg, &s->leds[i]);
    }

	s->redraw = false;
    dpy_gfx_update(s->con, 0, 0,  DPY_MAX_COLS ,  DPY_MAX_ROWS + LED_HT);
}

static void dashboard_2d_invalidate_display(void * opaque)
{
    Dashboard2DState *s = DB2D_DISPLAY(opaque);
    s->redraw = true;
}

static void dashboard_2d_wled_in(void *opaque, int n, int level)
{
    Dashboard2DState *s = DB2D_DISPLAY(opaque);
	if (level>255)
	{
		level = 255U;
	}
	level &= 0xFF;
    s->leds[n].red = level<<8;
    s->leds[n].green = level<<8;
    s->leds[n].blue = level<<8;
}

static void dashboard_2d_rled_in(void *opaque, int n, int level)
{
    Dashboard2DState *s = DB2D_DISPLAY(opaque);
	if (level>255)
	{
		level = 255U;
	}
	s->leds[n].red = level<<8;
}

static void dashboard_2d_gled_in(void *opaque, int n, int level)
{
    Dashboard2DState *s = DB2D_DISPLAY(opaque);
	if (level>255)
	{
		level = 255U;
	}
	s->leds[n].green = level<<8;

}

static void dashboard_2d_bled_in(void *opaque, int n, int level)
{
    Dashboard2DState *s = DB2D_DISPLAY(opaque);
	if (level>255)
	{
		level = 255U;
	}
	s->leds[n].blue = level<<8;

}

static void dashboard_2d_digital_led_in(void *opaque, int n, int level)
{
    Dashboard2DState *s = DB2D_DISPLAY(opaque);
	
	s->leds[n].red =   (level*255)<<8;
    s->leds[n].green = (level*255)<<8;
}

static void dashboard_2d_fan_pwm_in(void *opaque, int n, int level)
{
	Dashboard2DState *s = DB2D_DISPLAY(opaque);
	s->fan_pwms[n] = level>255U ? 255U : (level & 0xFF);
}

static void dashboard_2d_fan_rpm_in(void *opaque, int n, int level)
{
	Dashboard2DState *s = DB2D_DISPLAY(opaque);
	s->fan_rpms[n] = level;
    snprintf(s->fan_printf[n], sizeof(s->fan_printf[n]), "%5d", level);
}

static void dashboard_2d_therm_pwm_in(void *opaque, int n, int level)
{
	Dashboard2DState *s = DB2D_DISPLAY(opaque);
	s->therm_pwms[n] = level>255U ? 255U : (level & 0xFF);
}

static void dashboard_2d_therm_temp_in(void *opaque, int n, int level)
{
	Dashboard2DState *s = DB2D_DISPLAY(opaque);
	float temp = ((float)level)/256.F;
	snprintf(s->therm_printf[n],sizeof(s->therm_printf[n]), "%5.1f", temp);
}

static const GraphicHwOps dashboard_2d_ops = {
    .invalidate  = dashboard_2d_invalidate_display,
    .gfx_update  = dashboard_2d_update_display,
};

static void dashboard_2d_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);

    qdev_init_gpio_in_named(dev, dashboard_2d_wled_in, "led-w", N_LEDS);
	qdev_init_gpio_in_named(dev, dashboard_2d_rled_in, "led-r", N_LEDS);
	qdev_init_gpio_in_named(dev, dashboard_2d_gled_in, "led-g", N_LEDS);
	qdev_init_gpio_in_named(dev, dashboard_2d_bled_in, "led-b", N_LEDS);
	qdev_init_gpio_in_named(dev, dashboard_2d_digital_led_in, "led-digital", N_LEDS);

	qdev_init_gpio_in_named(dev, dashboard_2d_fan_pwm_in, "fan-pwm", N_FANS);
	qdev_init_gpio_in_named(dev, dashboard_2d_fan_rpm_in, "fan-rpm", N_FANS);

    qdev_init_gpio_in_named(dev, dashboard_2d_therm_pwm_in, "therm-pwm",  N_THERM);
	qdev_init_gpio_in_named(dev, dashboard_2d_therm_temp_in, "therm-temp", N_THERM);

}

static void dashboard_2d_realize(DeviceState *dev, Error **errp)
{
    Dashboard2DState *s = DB2D_DISPLAY(dev);

    // Calculate total rows:
    int i = 0;
    for (i=0; i<N_MOTORS; i++)
    {
        if (s->motors[i] == NULL)
            break;
    }

    s->current_rows = LINE_HEIGHT * i;

    s->current_rows += (3 * LINE_HEIGHT);
    s->con = graphic_console_init(dev, 0, &dashboard_2d_ops, s);
    qemu_console_resize(s->con, DPY_MAX_COLS, s->current_rows);
}


static void dashboard_2d_finalize(Object *obj)
{
}


static int dashboard_2d_post_load(void *opaque, int version) {
    dashboard_2d_invalidate_display(opaque);
    return 0;
}

static Property dashboard_2d_properties[] = {
    DEFINE_PROP_LINK("motor[0]",Dashboard2DState, motors[0], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[1]",Dashboard2DState, motors[1], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[2]",Dashboard2DState, motors[2], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[3]",Dashboard2DState, motors[3], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[4]",Dashboard2DState, motors[4], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[5]",Dashboard2DState, motors[5], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[6]",Dashboard2DState, motors[6], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[7]",Dashboard2DState, motors[7], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[8]",Dashboard2DState, motors[8], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_LINK("motor[9]",Dashboard2DState, motors[9], TYPE_P404_MOTOR_IF, P404MotorIF*),
    DEFINE_PROP_UINT8("fans", Dashboard2DState, fan_count, 0),
    DEFINE_PROP_UINT8("thermistors", Dashboard2DState, therm_count, 0),
    DEFINE_PROP_STRING("indicators", Dashboard2DState, ind_labels),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_dashboard_2d = {
    .name = TYPE_DB2D_DISPLAY,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = dashboard_2d_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(framebuffer,Dashboard2DState,DPY_MAX_ROWS*DPY_MAX_COLS),
        VMSTATE_END_OF_LIST()
    }
};

static void dashboard_2d_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_dashboard_2d;
    dc->realize = dashboard_2d_realize;
    device_class_set_props(dc, dashboard_2d_properties);

}

