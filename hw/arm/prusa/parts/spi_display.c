/*
	spi_display.c - Generic SPI display sim.


	Supports the following parts:
    ILITEK ILI9488 320x480 LCD controller.
    Sitronix ST7789v 320x240 LCD controller.

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
#include "migration/vmstate.h"
#include "hw/irq.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "qom/object.h"
#include "../utility/macros.h"
#include "../utility/p404scriptable.h"
#include "../utility/p404_keyclient.h"
#include "../utility/ScriptHost_C.h"

#include "png.h"

//#define DEBUG_ILI9488 1

#ifdef DEBUG_ILI9488
#define DPRINTF(fmt, ...) \
do { printf("spi_display: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { \
    fprintf(stderr, "spi_display: error: " fmt , ## __VA_ARGS__); abort(); \
} while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "spi_display: error: " fmt , ## __VA_ARGS__);} while (0)
#endif

#define REMAP_SWAP_COLUMN 0x01
#define REMAP_SWAP_NYBBLE 0x02
#define REMAP_VERTICAL    0x04
#define REMAP_SWAP_COM    0x10
#define REMAP_SPLIT_COM   0x40

#define CMD_NOP 0x00
#define CMD_MADCTLR 0x0B
#define CMD_SLPOUT 0x11
#define CMD_DINVOFF 0x20
#define CMD_DINVON 0x21
#define CMD_DISPON 0x29
#define CMD_CASET 0x2A
#define CMD_RASET 0x2B
#define CMD_RAMWR 0x2C
#define CMD_MADCTL 0x36
#define CMD_COLMOD 0x3A
// #define CMD_ 0x
// #define CMD_ 0x

#define DPY_MAX_ROWS 320
#define DPY_MAX_COLS 480
#define LED_HT 20
#define N_LEDS 4
#define LED_W DPY_MAX_COLS/N_LEDS
#define TOTAL_HT (DPY_MAX_ROWS + LED_HT)
#define DPY_BUFFSIZE sizeof(uint32_t)*(TOTAL_HT)*DPY_MAX_COLS


enum spi_display_mode
{
    DISPLAY_CMD,
    DISPLAY_DATA
};

typedef struct DisplayInfo {
    const char* name;
    uint16_t rows;
    uint16_t cols;
	int cs_polarity;
} DisplayInfo;

struct SPIDisplayState {
    SSIPeripheral ssidev;
    QemuConsole *con;

    uint32_t cmd_len;
    int32_t cmd;
    bool inversion; // Unused.
    int32_t cmd_data[8];
    int32_t row;
    int32_t row_start;
    int32_t row_end;
    int32_t col;
    int32_t col_start;
    int32_t col_end;
    int32_t redraw;
    int32_t remap;
    uint32_t mode;
    uint8_t bpp_mode;
    uint32_t framebuffer[TOTAL_HT * DPY_MAX_COLS];
    uint32_t leds[N_LEDS];

	uint32_t madctl;

	bool is_reading;

	qemu_irq reset;

    const DisplayInfo* dpy_info;

	int16_t cursor[3];

	script_handle handle;
};



struct SPIDisplayClass {
    SSIPeripheralClass parent_class;
    DisplayInfo *di;
};

#define DPY_ENTRY(_name, _rows, _cols, _pol) \
{ .name = _name, .rows = _rows, .cols = _cols, .cs_polarity=_pol }

#define TYPE_ILI9488 "ili9488"
#define TYPE_ST7789V "st7789v"
#define TYPE_SPI_DISPLAY "generic-spi-display"


static const DisplayInfo spi_display_models[] = {
    DPY_ENTRY(TYPE_ST7789V, 320, 240, SSI_CS_HIGH),
    DPY_ENTRY(TYPE_ILI9488, 320, 480, SSI_CS_LOW),
};


// OBJECT_DECLARE_SIMPLE_TYPE(SPIDisplayState, ILI9488)
OBJECT_DECLARE_TYPE(SPIDisplayState, SPIDisplayClass, SPI_DISPLAY)

static uint32_t spi_display_transfer(SSIPeripheral *dev, uint32_t data)
{
    SPIDisplayState *s = SPI_DISPLAY(dev);
    union color{
        uint32_t full;
        struct{
            uint8_t b;
            uint8_t g;
            uint8_t r;
            uint8_t a;
        };
    }  QEMU_PACKED color;
    color.full = 0;
    uint16_t word = 0;
	// If this is a read, pipe out the data appropriately.
		if (s->is_reading) {
			data = s->cmd_data[--s->cmd_len];
			if (s->cmd_len==0) {
				s->is_reading = false; // Out of data to clock out...
			}
			return data;
		} else if (s->mode == DISPLAY_CMD) {
            s->cmd = data;
            s->cmd_len=0;
        } else {
            s->cmd_data[s->cmd_len] = data;
            s->cmd_len++;
        }
        switch (s->cmd) {
#define DATA(x) if (s->cmd_len < (x)) return 0
        case CMD_NOP:
            break;
        case CMD_SLPOUT:
            break;
		case CMD_MADCTLR:
			if (s->dpy_info == &spi_display_models[0])
			{
				// This is a horrible hack but it fixes the Mini's blinking display for now.
				//TODO - figure out why the readback doesn't work for it (but does for the larger display)
				s->cmd_data[0] = 0xF0;
			}
			else
			{
				s->cmd_data[0] = s->madctl;
			}
			s->cmd_len = 1;
			s->is_reading = true;
			return 0;
        case CMD_DINVOFF:
        case CMD_DINVON:
        // Disabled for now because it seems to be flipped and this inverts the display
            //s->inversion = (s->cmd == CMD_DINVON);
            s->redraw = 1;
            break;
        case CMD_CASET: /* Set column.  */
            DATA(4);
            s->col = s->col_start = (s->cmd_data[0]<<8|s->cmd_data[1]) % s->dpy_info->cols;
            s->col_end = (s->cmd_data[2]<<8|s->cmd_data[3]) % s->dpy_info->cols;
            // printf("CASET %d -> %d (%d -> %d) \n", s->col, s->col_end, s->cmd_data[0], s->cmd_data[1]);
            break;
        case CMD_RASET: /* Set row.  */
            DATA(4);
            s->row = s->row_start = (s->cmd_data[0]<<8|s->cmd_data[1]) % s->dpy_info->rows;
            s->row_end = (s->cmd_data[2]<<8|s->cmd_data[3]) % s->dpy_info->rows;
            // printf("RASET %d -> %d (%d -> %d) \n", s->row, s->row_end, s->cmd_data[0], s->cmd_data[1]);
            break;
        case CMD_MADCTL:
            DATA(1);
            printf("TODO: MADCTL: %02x\n",s->cmd_data[0]);
			s->madctl = s->cmd_data[0];
            break;
        case CMD_COLMOD:
            DATA(1);
            uint32_t value = s->cmd_data[0];
            switch (value&0x07)
            {
                case 0x5: // 5-6-5, 16bpp
                    s->bpp_mode = 16;
                    break;
                case 0x6: // 6-6-6, 18bpp
                    s->bpp_mode = 18;
                    break;
                default:
                    printf("spi_display: COLMOD %02x not implemented.\n", value);
                    abort();
            }
            break;
        case CMD_RAMWR:
            if (s->cmd_len==0) // First one reset index.
            {
                s->row = s->row_start;
                s->col = s->col_start;
                // printf("RAWR: %d %d\n", sr->row, s->col);
                DATA(s->bpp_mode == 16 ? 2 : 3);
            } else {// One of an unknown number of 16-bit words.
                switch (s->bpp_mode) {
                    case 16:
                        DATA(2);
                        word = (s->cmd_data[0]<<8|s->cmd_data[1]);
                        color.r = (word & 0xF800)>>8;
                        color.g = (word & 0x7E0)>> 3;
                        color.b = (word & 0x1F) << 3;
                        break;
                    case 18:
                        DATA(3);
                        color.b = s->cmd_data[0];
                        color.g = s->cmd_data[1];
                        color.r = s->cmd_data[2];
                        break;
                    default:
                        printf("FIXME: unhandled bpp mode in RAMWR: %u\n", s->bpp_mode);
                }
                color.a = 0xFF;
                s->framebuffer[(s->col) + (s->row*s->dpy_info->cols)] = color.full;
                s->col++;
                if (s->col>s->col_end)
                {
                    s->row++;
                    s->col = s->col_start;
                }
                if (s->row>s->row_end)
                {
                    s->row = s->row_start;
                }
                s->cmd_len=0; // "remove" the data from the queue. We'll get more,
                s->redraw = 1;

            }
            break;
        case CMD_DISPON: /* Display on.  */
            /* TODO: Implement power control.  */
            break;
        default:
            BADF("Unknown command: 0x%x\n", data);
        }
        s->cmd_len = 0;
        return 0;

}

static void spi_display_update_display(void *opaque)
{
    SPIDisplayState *s = SPI_DISPLAY(opaque);
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *dest;

    if (!s->redraw)
        return;

    dest = surface_data(surface);
    memcpy(dest, s->framebuffer, s->dpy_info->cols * (s->dpy_info->rows + LED_HT) * sizeof(uint32_t) );
    if (s->inversion) {
        for (int i=0; i<DPY_BUFFSIZE; i++) {
            dest[i] = ~dest[i];
        }
    }

	dest+=4*((s->cursor[0]) + (s->cursor[1]*s->dpy_info->cols));
	// red if clicked, white if not.
	uint32_t cursor_data[2] = {(s->cursor[2]? 0xFF0000 : 0xFFFFFF), (s->cursor[2]? 0xFF0000 : 0xFFFFFF)};
	memcpy(dest, &cursor_data, sizeof(cursor_data));
	dest += sizeof(uint32_t)*(s->dpy_info->cols);
	memcpy(dest, &cursor_data, sizeof(cursor_data));
    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0,  s->dpy_info->cols ,  s->dpy_info->rows + LED_HT);

}

static void spi_display_invalidate_display(void * opaque)
{
    SPIDisplayState *s = SPI_DISPLAY(opaque);
    s->redraw = 1;
}

static void spi_display_reset(void *opaque, int n, int level)
{
    SPIDisplayState *s = SPI_DISPLAY(opaque);
	if (s->dpy_info == &spi_display_models[0])
	{
		// Kinda hacky but it's a workaround for the fw.
		// Someday we'll have a real reset here... ;)
		qemu_irq_raise(s->reset);
	}
}

static void spi_display_cd(void *opaque, int n, int level)
{
    SPIDisplayState *s = (SPIDisplayState *)opaque;
    s->mode = level ? DISPLAY_DATA : DISPLAY_CMD;
    // s->byte_msb = false;
}

static void spi_display_cursor(void *opaque, int n, int level)
{
    SPIDisplayState *s = (SPIDisplayState *)opaque;
	switch (n)
	{
		case 0:
			s->cursor[0] +=level;
			s->cursor[0] = MIN(MAX(s->cursor[0],0),s->dpy_info->cols-1U);
			break;
		case 1:
			s->cursor[1] += level;
			s->cursor[1] = MIN(MAX(s->cursor[1],0), s->dpy_info->rows);
			break;
		case 2:
			s->cursor[2] = level;
			break;
	}
	s->redraw = 1;
}

static void spi_display_led(void *opaque, int n, int level)
{
    SPIDisplayState *s = SPI_DISPLAY(opaque);
	if (n==3)
	{
		// backlight LED
		uint8_t r = level >> 16;
		level = r << 16 | r << 8 | r;
	}
    s->leds[n] = level;
    for (int row =  s->dpy_info->rows; row<  s->dpy_info->rows + LED_HT; row++) {
        for (int col = n*LED_W; col< ((n+1)*LED_W); col++) {
            s->framebuffer[col + (row* s->dpy_info->cols)] = level;
        }
    }
	spi_display_invalidate_display(s);
}


static void spi_display_write_png(SPIDisplayState *s, const char* file)
{
    FILE *handle = fopen(file, "wb");
    if (!handle)
		printf("Screenshot failed - could not open file %s (%s)\n",file, strerror(errno));

    png_structrp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    png_inforp info_p = png_create_info_struct(png_ptr);

    png_init_io(png_ptr, handle);

    png_set_IHDR(png_ptr, info_p,  s->dpy_info->cols,  s->dpy_info->rows, 8, PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    char text1[] = "Mini404 Screenshot";
    char text2[] = "Created using Mini404";
    char text3[] = "http://www.github.com/vintagepc/Mini404/";
    char key1[] = "Title";
    char key2[] = "Description";
    char key3[] = "URL";

    png_text s_title[3] = {
        {
            .compression = PNG_TEXT_COMPRESSION_NONE,
            .key = key1,
            .text = text1,
        },
        {
            .compression = PNG_TEXT_COMPRESSION_NONE,
            .key = key2,
            .text = text2,
        },
        {
            .compression = PNG_TEXT_COMPRESSION_NONE,
            .key = key3,
            .text = text3
        }
    };


    png_set_text(png_ptr, info_p, s_title, 3);

    png_write_info(png_ptr, info_p);
    png_set_bgr(png_ptr);

    png_byte row[s->dpy_info->cols*4];


    for (int i=0; i< s->dpy_info->rows; i++){
        memcpy(&row, &s->framebuffer[i*s->dpy_info->cols], sizeof(row));
        png_write_row(png_ptr, row);
    }

    png_write_end(png_ptr, NULL);

    if (info_p) png_free_data(png_ptr, info_p, PNG_FREE_ALL, -1);

    fclose(handle);
}

static int spi_display_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    if (action == 0) {
        const char* file = scripthost_get_string(args, 0);
        printf("Saving screenshot to: %s\n",file);
        spi_display_write_png(SPI_DISPLAY(obj), file);
        return ScriptLS_Finished;
    } else {
        return ScriptLS_Unhandled;
    }
}

static void spi_display_handle_key(P404KeyIF *obj, Key keycode)
{
    if (keycode == 'S') // S release
    {
        char file[100] = {0};
        time_t now = time(NULL);
	    struct tm *t = localtime(&now);
        strftime(file, sizeof(file)-1, "Snap %d %m %Y %H:%M:%S.png", t);
        printf("Saving screenshot to: %s\n",file);
        spi_display_write_png(SPI_DISPLAY(obj), file);
    }
}

static const GraphicHwOps spi_display_ops = {
    .invalidate  = spi_display_invalidate_display,
    .gfx_update  = spi_display_update_display,
};

// OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(SPIDisplayState, spi_display, ILI9488, SSI_PERIPHERAL, {TYPE_P404_SCRIPTABLE}, {NULL})

static void spi_display_realize(SSIPeripheral *d, Error **errp)
{
    DeviceState *dev = DEVICE(d);
    SPIDisplayState *s = SPI_DISPLAY(d);

    SPIDisplayClass *k = SPI_DISPLAY_GET_CLASS(d);

    s->dpy_info = k->di;

    s->col_end = s->dpy_info->cols-1;
    s->row_end = s->dpy_info->rows-1;
    s->con = graphic_console_init(dev, 0, &spi_display_ops, s);
    qemu_console_resize(s->con, s->dpy_info->cols, s->dpy_info->rows + LED_HT);

    qdev_init_gpio_in(dev, spi_display_cd, 1);
	qdev_init_gpio_in_named(dev, spi_display_cursor, "cursor", 3);
    qdev_init_gpio_in_named(dev, spi_display_led, "leds", N_LEDS);
	qdev_init_gpio_in_named(dev, spi_display_reset, "reset", 1);
	qdev_init_gpio_out_named(dev, &s->reset, "reset-out", 1);

    s->handle = script_instance_new(P404_SCRIPTABLE(s), TYPE_SPI_DISPLAY);

    script_register_action(s->handle, "Screenshot", "Takes a screenshot to the specified file.", 0);
    script_add_arg_string(s->handle, 0);
    scripthost_register_scriptable(s->handle);

	p404_key_handle pKey = p404_new_keyhandler(P404_KEYCLIENT(d));
    p404_register_keyhandler(pKey, 'S', "Takes a screenshot of the LCD with the current time as the filename.");

}

static int spi_display_post_load(void *opaque, int version) {
    spi_display_invalidate_display(opaque);
    return 0;
}

static const VMStateDescription vmstate_spi_display = {
    .name = TYPE_SPI_DISPLAY,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = spi_display_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(ssidev,SPIDisplayState),
        VMSTATE_UINT32(cmd_len,SPIDisplayState),
        VMSTATE_INT32(cmd,SPIDisplayState),
        VMSTATE_BOOL(inversion,SPIDisplayState),
        VMSTATE_INT32_ARRAY(cmd_data,SPIDisplayState,8),
        VMSTATE_INT32(row,SPIDisplayState),
        VMSTATE_INT32(row_start,SPIDisplayState),
        VMSTATE_INT32(row_end,SPIDisplayState),
        VMSTATE_INT32(col,SPIDisplayState),
        VMSTATE_INT32(col_start,SPIDisplayState),
        VMSTATE_INT32(col_end,SPIDisplayState),
        VMSTATE_INT32(redraw,SPIDisplayState),
        VMSTATE_INT32(remap,SPIDisplayState),
        VMSTATE_UINT32(mode,SPIDisplayState),
        VMSTATE_UINT8(bpp_mode, SPIDisplayState),
        VMSTATE_UINT32_ARRAY(framebuffer,SPIDisplayState,TOTAL_HT*DPY_MAX_COLS),
		VMSTATE_BOOL(is_reading,SPIDisplayState),
        VMSTATE_END_OF_LIST()
    }
};

static void spi_display_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_spi_display;

    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = spi_display_process_action;

	P404KeyIFClass *kc = P404_KEYCLIENT_CLASS(klass);
    kc->KeyHandler = spi_display_handle_key;

    k->realize = spi_display_realize;
    k->transfer = spi_display_transfer;

	DisplayInfo* di = (DisplayInfo*) data;

    k->cs_polarity = di->cs_polarity;

    SPIDisplayClass* sdc = SPI_DISPLAY_CLASS(klass);
    sdc->di = data;
}

static const TypeInfo spi_display_info = {
    .name           = TYPE_SPI_DISPLAY,
    .parent         = TYPE_SSI_PERIPHERAL,
    .instance_size  = sizeof(SPIDisplayState),
    .class_size     = sizeof(SPIDisplayClass),
    .abstract       = true,
    .interfaces     = (InterfaceInfo[]) {
        { TYPE_P404_SCRIPTABLE },
		{ TYPE_P404_KEYCLIENT },
        { }
    }
};

static void spi_display_register_types(void)
{
    type_register_static(&spi_display_info);
    for (int i = 0; i < ARRAY_SIZE(spi_display_models); ++i) {
        TypeInfo ti = {
            .name       = spi_display_models[i].name,
            .parent     = TYPE_SPI_DISPLAY,
            .class_init = spi_display_class_init,
            .class_data = (void *)&spi_display_models[i],
        };
        type_register(&ti);
    }

}

type_init(spi_display_register_types)
