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

// static int st7889v_set_cs(SSISlave *dev, bool select)
// {
//     st7789v_state *s = ST7789V(dev);
//     if (!select) // act low.
//     { 
//         s->cmd_in.cmd = 0;
//         for (int i=0; i<4; i++)
//         {
//              s->cmd_in.params[i]=0;
//         }
//     }
//     else // H, command finished.
//     {
//         parseCommand();
//     }
//     return 0;
    
// }
// static uint32_t st7789v_transfer(SSISlave *dev, uint32_t data)
// {
//     st7789v_state *s = ST7789V(dev);

// }

static uint32_t st7789v_transfer(SSISlave *dev, uint32_t data)
{
    st7789v_state *s = ST7789V(dev);
   // switch (s->mode) {
    // case ST7789V_DATA:
        // printf("data 0x%02x\n", data);
        // s->framebuffer[s->col + s->row * DPY_COLS] = data;
        // if (s->remap & REMAP_VERTICAL) {
        //     s->row++;
        //     if (s->row > s->row_end) {
        //         s->row = s->row_start;
        //         s->col++;
        //     }
        //     if (s->col > s->col_end) {
        //         s->col = s->col_start;
        //     }
        // } else {
        //     s->col++;
        //     if (s->col > s->col_end) {
        //         s->row++;
        //         s->col = s->col_start;
        //     }
        //     if (s->row > s->row_end) {
        //         s->row = s->row_start;
        //     }
        // }
        // s->redraw = 1;
        // break;
    // case ST7789V_CMD:
        if (s->mode == ST7789V_CMD) {
            if (s->byte_msb)
            {
               s->cmd |= data << 8;
                printf("cmd 0x%02x\n", s->cmd);
            } else {
                s->cmd = data;
            }
            s->cmd_len=0;
        } else {
            if (s->byte_msb)
             {       
               s->cmd_data[s->cmd_len] |= data; // Note data mode sends 8 bits only in the right order.
             //   if (s->cmd_data[s->cmd_len]!=0) printf("param 0x%02x\n", s->cmd_data[s->cmd_len]);
                s->cmd_len++;
            } else {
                s->cmd_data[s->cmd_len] = data << 8;
            }
        }
        s->byte_msb = !s->byte_msb;
        if (s->byte_msb)
            return 0; // Are waiting for MSB, continue.
        switch (s->cmd) {
#define DATA(x) if (s->cmd_len < (x)) return 0
#define DATA16(x) if (s->cmd_len < (x*2)) return 0
        case CMD_NOP:
            break;
        case CMD_SLPOUT:
            break;
        case CMD_CASET: /* Set column.  */
            DATA(2);
            s->col = s->col_start = s->cmd_data[0] % DPY_COLS;
            s->col_end = s->cmd_data[1] % DPY_COLS;
            break;
        case CMD_RASET: /* Set row.  */
            DATA(2);
            s->row = s->row_start = s->cmd_data[0] % DPY_ROWS;
            s->row_end = s->cmd_data[1] % DPY_ROWS;
            break;
        case CMD_MADCTL:
            DATA(1);
            printf("TODO: MADCTL: %02x\n",s->cmd_data[0]>>8);
            break;
        case CMD_COLMOD:
            DATA(1);
            uint32_t value = s->cmd_data[0]>>8;
            if ((value&0x07) != 5)
                printf("st7789v: Modes other than 16bpp are not implemented.\n");
            // printf("TODO: COLMOD: %02x %02x\n", value& 0x70, value & 0x07);
            break;
        case CMD_RAMWR:
            if (s->cmd_len==0) // First one reset index.
            {
                s->row = s->row_start;
                s->col = s->col_start;
                DATA(1);
            } else {// One of an unknown number of bytes.               
                s->framebuffer[s->col + (s->row * DPY_COLS)] = s->cmd_data[0];
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
                DATA(1);
            }
            break;
        // case 0x81: /* Set contrast */
        //     DATA(1);
        //     break;
        // case 0x84: case 0x85: case 0x86: /* Max current.  */
        //     DATA(0);
        //     break;
        // case 0xa0: /* Set remapping.  */
        //     /* FIXME: Implement this.  */
        //     DATA(1);
        //     s->remap = s->cmd_data[0];
        //     break;
        // case 0xa1: /* Set display start line.  */
        // case 0xa2: /* Set display offset.  */
        //     /* FIXME: Implement these.  */
        //     DATA(1);
        //     break;
        // case 0xa4: /* Normal mode.  */
        // case 0xa5: /* All on.  */
        // case 0xa6: /* All off.  */
        // case 0xa7: /* Inverse.  */
        //     /* FIXME: Implement these.  */
        //     DATA(0);
        //     break;
        // case 0xa8: /* Set multiplex ratio.  */
        // case 0xad: /* Set DC-DC converter.  */
        //     DATA(1);
        //     /* Ignored.  Don't care.  */
        //     break;
        // case 0xae: /* Display off.  */
        case CMD_DISPON: /* Display on.  */
            /* TODO: Implement power control.  */
            break;
        // case 0xb1: /* Set phase length.  */
        // case 0xb2: /* Set row period.  */
        // case 0xb3: /* Set clock rate.  */
        // case 0xbc: /* Set precharge.  */
        // case 0xbe: /* Set VCOMH.  */
        // case 0xbf: /* Set segment low.  */
        //     DATA(1);
        //     /* Ignored.  Don't care.  */
        //     break;
        // case 0xb8: /* Set grey scale table.  */
        //     /* FIXME: Implement this.  */
        //     DATA(8);
        //     break;
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
    const int width = DPY_COLS, height = DPY_ROWS;
    st7789v_state *s = (st7789v_state *)opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *dest;
    uint16_t *src;
    int x;
    int y;
    int i;
    int line;
    char *colors[16];
    char colortab[MAGNIFY * 64];
    char *p;
    int dest_width;


    if (!s->redraw)
        return;

    switch (surface_bits_per_pixel(surface)) {
    case 0:
        return;
    case 15:
        dest_width = 2;
        break;
    case 16:
        dest_width = 2;
        break;
    case 24:
        dest_width = 3;
        break;
    case 32:
        dest_width = 4;
        break;
    default:
        BADF("Bad color depth\n");
        return;
    }
    p = colortab;
    for (i = 0; i < 16; i++) {
        int n;
        colors[i] = p;
        switch (surface_bits_per_pixel(surface)) {
        case 15:
            n = i * 2 + (i >> 3);
            p[0] = n | (n << 5);
            p[1] = (n << 2) | (n >> 3);
            break;
        case 16:
            n = i * 2 + (i >> 3);
            p[0] = n | (n << 6) | ((n << 1) & 0x20);
            p[1] = (n << 3) | (n >> 2);
            break;
        case 24:
        case 32:
            n = (i << 4) | i;
            p[0] = p[1] = p[2] = n;
            break;
        default:
            BADF("Bad color depth\n");
            return;
        }
        p += dest_width;
    }
    /* TODO: Implement row/column remapping.  */
    dest = surface_data(surface);
    memcpy(dest, s->framebuffer, sizeof(uint32_t)*DPY_ROWS*DPY_COLS);
    // for ( line y = 0; y<height; y++)
    // {
    //     memcpy()
    // }
    // for (y = 0; y < height; y++) {
    //     line = y;
    //     src = s->framebuffer + (width * line);
    //     for (x = 0; x < height; x++) {
    //         int val;
    //         val = *src >> 4;
    //         for (i = 0; i < MAGNIFY; i++) {
    //             memcpy(dest, colors[val], dest_width);
    //             dest += dest_width;
    //         }
    //         val = *src & 0xf;
    //         for (i = 0; i < MAGNIFY; i++) {
    //             memcpy(dest, colors[val], dest_width);
    //             dest += dest_width;
    //         }
    //         src++;
    //     }
    //     for (i = 1; i < MAGNIFY; i++) {
    //         memcpy(dest, dest - dest_width * MAGNIFY * width,
    //                dest_width * width * MAGNIFY);
    //         dest += dest_width * width * MAGNIFY;
    //     }
    // }
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
    printf("st7789v mode %s\n", level ? "Data" : "Command");
    s->mode = level ? ST7789V_DATA : ST7789V_CMD;
    s->byte_msb = false;
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

static void st7789v_realize(SSISlave *d, Error **errp)
{
    DeviceState *dev = DEVICE(d);
    st7789v_state *s = ST7789V(d);

    s->col_end = DPY_COLS;
    s->row_end = DPY_ROWS;
    s->con = graphic_console_init(dev, 0, &st7789v_ops, s);
    qemu_console_resize(s->con, DPY_COLS * MAGNIFY, DPY_ROWS * MAGNIFY);

    qdev_init_gpio_in(dev, st7789v_cd, 1);
}

static void st7789v_class_init(ObjectClass *klass, void *data)
{
    // DeviceClass *dc = DEVICE_CLASS(klass);
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);

    k->realize = st7789v_realize;
    k->transfer = st7789v_transfer;
    k->cs_polarity = SSI_CS_LOW;
   // dc->vmsd = &vmstate_st7789v;
}

static const TypeInfo st7789v_info = {
    .name          = TYPE_ST7789V,
    .parent        = TYPE_SSI_SLAVE,
    .instance_size = sizeof(st7789v_state),
    .class_init    = st7789v_class_init,
};

static void st7789v2_register_types(void)
{
    type_register_static(&st7789v_info);
}

type_init(st7789v2_register_types)
