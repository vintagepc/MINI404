/*
 * ST7789V OLED controller with OSRAM Pictiva 128x64 display.
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

/* The controller can support a variety of different displays, but we only
   implement one.  Most of the commends relating to brightness and geometry
   setup are ignored. */

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
    SSISlave ssidev;
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

union st7789v_cmd { 
    uint16_t params[4];
    uint16_t cmd;

} st7789v_cmd;

#define TYPE_ST7789V "st7789v"
OBJECT_DECLARE_SIMPLE_TYPE(st7789v_state, ST7789V)

static uint32_t st7789v_transfer(SSISlave *dev, uint32_t data)
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
    } color;
    color.full = 0;
    uint16_t word = 0;
   // printf("BYTE: %02x\n", data);
 
        if (s->mode == ST7789V_CMD) {
            s->cmd = data;
            s->cmd_len=0;
        } else {
            s->cmd_data[s->cmd_len] = data;
            s->cmd_len++;
        }
        //printf("cmd 0x%02x\n", s->cmd);
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
    // }
    // return 0;
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

/* Command/data input.  */
static void st7789v_cd(void *opaque, int n, int level)
{
    st7789v_state *s = (st7789v_state *)opaque;
    // printf("st7789v mode %s\n", level ? "Data" : "Command");
    s->mode = level ? ST7789V_DATA : ST7789V_CMD;
    s->byte_msb = false;
}
void st7789v_write_png(st7789v_state *s, const char* file);
void st7789v_write_png(st7789v_state *s, const char* file)
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

// static int st7789v_post_load(void *opaque, int version_id)
// {
//     st7789v_state *s = (st7789v_state *)opaque;

//     if (s->cmd_len > ARRAY_SIZE(s->cmd_data)) {
//         return -EINVAL;
//     }
//     if (s->row < 0 || s->row >= 80) {
//         return -EINVAL;
//     }
//     if (s->row_start < 0 || s->row_start >= 80) {
//         return -EINVAL;
//     }
//     if (s->row_end < 0 || s->row_end >= 80) {
//         return -EINVAL;
//     }
//     if (s->col < 0 || s->col >= 64) {
//         return -EINVAL;
//     }
//     if (s->col_start < 0 || s->col_start >= 64) {
//         return -EINVAL;
//     }
//     if (s->col_end < 0 || s->col_end >= 64) {
//         return -EINVAL;
//     }
//     if (s->mode != ST7789V_CMD && s->mode != ST7789V_DATA) {
//         return -EINVAL;
//     }

//     return 0;
// }

// static const VMStateDescription vmstate_st7789v = {
//     .name = "st7789v",
//     .version_id = 2,
//     .minimum_version_id = 2,
//     .post_load = st7789v_post_load,
//     .fields      = (VMStateField []) {
//         VMSTATE_UINT32(cmd_len, st7789v_state),
//         VMSTATE_INT32(cmd, st7789v_state),
//         VMSTATE_INT32_ARRAY(cmd_data, st7789v_state, 8),
//         VMSTATE_INT32(row, st7789v_state),
//         VMSTATE_INT32(row_start, st7789v_state),
//         VMSTATE_INT32(row_end, st7789v_state),
//         VMSTATE_INT32(col, st7789v_state),
//         VMSTATE_INT32(col_start, st7789v_state),
//         VMSTATE_INT32(col_end, st7789v_state),
//         VMSTATE_INT32(redraw, st7789v_state),
//         VMSTATE_INT32(remap, st7789v_state),
//         VMSTATE_UINT32(mode, st7789v_state),
//         VMSTATE_BUFFER(framebuffer, st7789v_state),
//         VMSTATE_SSI_SLAVE(ssidev, st7789v_state),
//         VMSTATE_END_OF_LIST()
//     }
// };

static const GraphicHwOps st7789v_ops = {
    .invalidate  = st7789v_invalidate_display,
    .gfx_update  = st7789v_update_display,
};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(st7789v_state, st7789v, ST7789V, SSI_SLAVE, {TYPE_P404_SCRIPTABLE}, {NULL})

static void st7789v_finalize(Object *obj)
{
    printf("Disp_finalize\n");
}


static void st7789v_init(Object *obj)
{
}


static void st7789v_realize(SSISlave *d, Error **errp)
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

static void st7789v_class_init(ObjectClass *klass, void *data)
{
    // DeviceClass *dc = DEVICE_CLASS(klass);
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);

    P404ScriptIFClass *sc = P404_SCRIPTABLE_CLASS(klass);
    sc->ScriptHandler = st7789v_process_action;

    k->realize = st7789v_realize;
    k->transfer = st7789v_transfer;
    k->cs_polarity = SSI_CS_LOW;
   // dc->vmsd = &vmstate_st7789v;
}
