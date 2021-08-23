/*
	st7789v.c
	
    Sitronix 7789V 240x320 LCD controller.
    
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
#include "qemu/module.h"
#include "ui/console.h"
#include "qom/object.h"
#include "../utility/macros.h"
#include "../utility/p404scriptable.h"
#include "../utility/ScriptHost_C.h"

#include "png.h"

//#define DEBUG_ST7789V 1

#ifdef DEBUG_ST7789V
#define DPRINTF(fmt, ...) \
do { printf("st7789v: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { \
    fprintf(stderr, "st7789v: error: " fmt , ## __VA_ARGS__); abort(); \
} while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "st7789v: error: " fmt , ## __VA_ARGS__);} while (0)
#endif

/* Scaling factor for pixels.  */
#define MAGNIFY 1

#define REMAP_SWAP_COLUMN 0x01
#define REMAP_SWAP_NYBBLE 0x02
#define REMAP_VERTICAL    0x04
#define REMAP_SWAP_COM    0x10
#define REMAP_SPLIT_COM   0x40

#define CMD_NOP 0x00
#define CMD_SLPOUT 0x11
#define CMD_DISPON 0x29
#define CMD_CASET 0x2A
#define CMD_RASET 0x2B
#define CMD_RAMWR 0x2C
#define CMD_MADCTL 0x36
#define CMD_COLMOD 0x3A
// #define CMD_ 0x
// #define CMD_ 0x

#define DPY_ROWS 320
#define DPY_COLS 240

enum st7789v_mode
{
    ST7789V_CMD,
    ST7789V_DATA
};

struct st7789v_state {
    SSIPeripheral ssidev;
    QemuConsole *con;

    uint32_t cmd_len;
    int32_t cmd;
    bool byte_msb;
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
    uint32_t framebuffer[DPY_ROWS * DPY_COLS];
};

#define TYPE_ST7789V "st7789v"
OBJECT_DECLARE_SIMPLE_TYPE(st7789v_state, ST7789V)

static uint32_t st7789v_transfer(SSIPeripheral *dev, uint32_t data)
{
    st7789v_state *s = ST7789V(dev);
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
        if (s->mode == ST7789V_CMD) {
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
        case CMD_CASET: /* Set column.  */
            DATA(4);
            s->col = s->col_start = (s->cmd_data[0]<<8|s->cmd_data[1]) % DPY_COLS;
            s->col_end = (s->cmd_data[2]<<8|s->cmd_data[3]) % DPY_COLS;
            // printf("CASET %d -> %d (%d -> %d) \n", s->col, s->col_end, s->cmd_data[0], s->cmd_data[1]);
            break;
        case CMD_RASET: /* Set row.  */
            DATA(4);
            s->row = s->row_start = (s->cmd_data[0]<<8|s->cmd_data[1]) % DPY_ROWS;
            s->row_end = (s->cmd_data[2]<<8|s->cmd_data[3]) % DPY_ROWS;
            // printf("RASET %d -> %d (%d -> %d) \n", s->row, s->row_end, s->cmd_data[0], s->cmd_data[1]);
            break;
        case CMD_MADCTL:
            DATA(1);
            printf("TODO: MADCTL: %02x\n",s->cmd_data[0]);
            break;
        case CMD_COLMOD:
            DATA(1);
            uint32_t value = s->cmd_data[0];
            if ((value&0x07) != 5)
                printf("st7789v: Modes other than 16bpp are not implemented.\n");
            //printf("TODO: COLMOD: %02x %02x\n", value& 0x70, value & 0x07);
            break;
        case CMD_RAMWR:
            if (s->cmd_len==0) // First one reset index.
            {
                s->row = s->row_start;
                s->col = s->col_start;
                // printf("RAWR: %d %d\n", sr->row, s->col);
                DATA(2);
            } else {// One of an unknown number of 16-bit words.    
                DATA(2);    
                word = (s->cmd_data[0]<<8|s->cmd_data[1]);
                color.r = (word & 0xF800)>>8;
                color.g = (word & 0x7E0)>> 3;
                color.b = (word & 0x1F) << 3;
                color.a = 0xFF;
                s->framebuffer[(s->col) + (s->row*DPY_COLS)] = color.full;
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

static void st7789v_update_display(void *opaque)
{
    st7789v_state *s = (st7789v_state *)opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *dest;

    if (!s->redraw)
        return;

    dest = surface_data(surface);
    memcpy(dest, s->framebuffer, sizeof(uint32_t)*DPY_ROWS*DPY_COLS);
    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0, DPY_COLS * MAGNIFY, DPY_ROWS * MAGNIFY);
}

static void st7789v_invalidate_display(void * opaque)
{
    st7789v_state *s = (st7789v_state *)opaque;
    s->redraw = 1;
}

static void st7789v_cd(void *opaque, int n, int level)
{
    st7789v_state *s = (st7789v_state *)opaque;
    s->mode = level ? ST7789V_DATA : ST7789V_CMD;
    s->byte_msb = false;
}

static void st7789v_write_png(st7789v_state *s, const char* file)
{
    FILE *handle = fopen(file, "wb");
    if (!handle)
        printf("Screenshot failed - could not open file %s\n",file);
    
    png_structrp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    png_inforp info_p = png_create_info_struct(png_ptr);

    png_init_io(png_ptr, handle);

    png_set_IHDR(png_ptr, info_p, DPY_COLS, DPY_ROWS, 8, PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE, 
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

    png_byte row[DPY_COLS*4]; 


    for (int i=0; i<DPY_ROWS; i++){
        memcpy(&row, &s->framebuffer[i*DPY_COLS], sizeof(row));
        png_write_row(png_ptr, row);
    }
  
    png_write_end(png_ptr, NULL);

    if (info_p) png_free_data(png_ptr, info_p, PNG_FREE_ALL, -1);

    fclose(handle);
}

static int st7789v_process_action(P404ScriptIF *obj, unsigned int action, script_args args)
{
    if (action == 0) {
        const char* file = scripthost_get_string(args, 0);
        printf("Saving screenshot to: %s\n",file);
        st7789v_write_png(ST7789V(obj), file);
        return ScriptLS_Finished;
    } else {
        return ScriptLS_Unhandled;
    }
}

static const GraphicHwOps st7789v_ops = {
    .invalidate  = st7789v_invalidate_display,
    .gfx_update  = st7789v_update_display,
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(st7789v_state, st7789v, ST7789V, SSI_PERIPHERAL, {TYPE_P404_SCRIPTABLE}, {NULL})

static void st7789v_finalize(Object *obj)
{
    printf("Disp_finalize\n");
}


static void st7789v_init(Object *obj)
{
}


static void st7789v_realize(SSIPeripheral *d, Error **errp)
{
    DeviceState *dev = DEVICE(d);
    st7789v_state *s = ST7789V(d);

    s->col_end = DPY_COLS-1;
    s->row_end = DPY_ROWS-1;
    s->con = graphic_console_init(dev, 0, &st7789v_ops, s);
    qemu_console_resize(s->con, DPY_COLS * MAGNIFY, DPY_ROWS * MAGNIFY);

    qdev_init_gpio_in(dev, st7789v_cd, 1);

    script_handle pScript = script_instance_new(P404_SCRIPTABLE(s), TYPE_ST7789V);

    script_register_action(pScript, "Screenshot", "Takes a screenshot to the specified file.", 0);
    script_add_arg_string(pScript, 0);
    scripthost_register_scriptable(pScript);

}

static int st7789v_post_load(void *opaque, int version) {
    st7789v_invalidate_display(opaque);
    return 0;
}

static const VMStateDescription vmstate_st7789v = {
    .name = TYPE_ST7789V,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = st7789v_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(ssidev,st7789v_state),
        VMSTATE_UINT32(cmd_len,st7789v_state),
        VMSTATE_INT32(cmd,st7789v_state),
        VMSTATE_BOOL(byte_msb,st7789v_state),
        VMSTATE_INT32_ARRAY(cmd_data,st7789v_state,8),
        VMSTATE_INT32(row,st7789v_state),
        VMSTATE_INT32(row_start,st7789v_state),
        VMSTATE_INT32(row_end,st7789v_state),
        VMSTATE_INT32(col,st7789v_state),
        VMSTATE_INT32(col_start,st7789v_state),
        VMSTATE_INT32(col_end,st7789v_state),
        VMSTATE_INT32(redraw,st7789v_state),
        VMSTATE_INT32(remap,st7789v_state),
        VMSTATE_UINT32(mode,st7789v_state),
        VMSTATE_UINT32_ARRAY(framebuffer,st7789v_state,DPY_ROWS*DPY_COLS),
        VMSTATE_END_OF_LIST()
    }
};

static void st7789v_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_st7789v;

    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = st7789v_process_action;

    k->realize = st7789v_realize;
    k->transfer = st7789v_transfer;
    k->cs_polarity = SSI_CS_LOW;
}
