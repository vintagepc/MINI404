/*
 * buddy_visuals OLED controller with OSRAM Pictiva 128x64 display.
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
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "qom/object.h"

//#define DEBUG_buddy_visuals 1

#ifdef DEBUG_buddy_visuals
#define DPRINTF(fmt, ...) \
do { printf("buddy_visuals: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { \
    fprintf(stderr, "buddy_visuals: error: " fmt , ## __VA_ARGS__); abort(); \
} while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "buddy_visuals: error: " fmt , ## __VA_ARGS__);} while (0)
#endif

#define DPY_ROWS 320
#define DPY_COLS 500

struct buddy_visuals_state {
    SysBusDevice parent;
    QemuConsole *con;

    FILE *fd_pipe;

    bool is_opened;

    int32_t redraw;

};

#define TYPE_BUDDY_VISUALS "buddy-visuals"

OBJECT_DECLARE_SIMPLE_TYPE(buddy_visuals_state, BUDDY_VISUALS)


static void buddy_visuals_update_display(void *opaque)
{
    const int width = DPY_COLS, height = DPY_ROWS;
    buddy_visuals_state *s = (buddy_visuals_state *)opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *dest;

    if (!s->redraw)
        return;

    dest = surface_data(surface);
    memset(dest, 0xCC, sizeof(uint32_t)*DPY_ROWS*DPY_COLS);
    // memcpy(dest, s->framebuffer, sizeof(uint32_t)*DPY_ROWS*DPY_COLS);
    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0, DPY_COLS, DPY_ROWS);
}

// static void buddy_visuals_invalidate_display(void * opaque)
// {
//     buddy_visuals_state *s = (buddy_visuals_state *)opaque;
//     s->redraw = 1;
// }



// static int buddy_visuals_post_load(void *opaque, int version_id)
// {
//     buddy_visuals_state *s = (buddy_visuals_state *)opaque;

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
//     if (s->mode != buddy_visuals_CMD && s->mode != buddy_visuals_DATA) {
//         return -EINVAL;
//     }

//     return 0;
// }

// static const VMStateDescription vmstate_buddy_visuals = {
//     .name = "buddy_visuals",
//     .version_id = 2,
//     .minimum_version_id = 2,
//     .post_load = buddy_visuals_post_load,
//     .fields      = (VMStateField []) {
//         VMSTATE_UINT32(cmd_len, buddy_visuals_state),
//         VMSTATE_INT32(cmd, buddy_visuals_state),
//         VMSTATE_INT32_ARRAY(cmd_data, buddy_visuals_state, 8),
//         VMSTATE_INT32(row, buddy_visuals_state),
//         VMSTATE_INT32(row_start, buddy_visuals_state),
//         VMSTATE_INT32(row_end, buddy_visuals_state),
//         VMSTATE_INT32(col, buddy_visuals_state),
//         VMSTATE_INT32(col_start, buddy_visuals_state),
//         VMSTATE_INT32(col_end, buddy_visuals_state),
//         VMSTATE_INT32(redraw, buddy_visuals_state),
//         VMSTATE_INT32(remap, buddy_visuals_state),
//         VMSTATE_UINT32(mode, buddy_visuals_state),
//         VMSTATE_BUFFER(framebuffer, buddy_visuals_state),
//         VMSTATE_SSI_SLAVE(ssidev, buddy_visuals_state),
//         VMSTATE_END_OF_LIST()
//     }
// };

// static const GraphicHwOps buddy_visuals_ops = {
//     .invalidate  = buddy_visuals_invalidate_display,
//     .gfx_update  = buddy_visuals_update_display,
// };

static void buddy_visuals_add_motor(buddy_visuals_state *s, unsigned char index, unsigned char label, uint32_t steps_per_mm, int32_t max_steps, bool is_simple)
{
    fprintf(s->fd_pipe,"%cAM",2);

    fprintf(s->fd_pipe,"%cM%cL%c",4,index+'0', label);
    fprintf(s->fd_pipe,"%cM%cS%c",4,index+'0', is_simple+'0');
    
    fprintf(s->fd_pipe,"%cM%cU",7,index+'0');
    for (int i=3; i>=0; i--)
        fputc(steps_per_mm>>(8*i),s->fd_pipe);

    fprintf(s->fd_pipe,"%cM%cX",7,index+'0');
    for (int i=3; i>=0; i--)
        fputc(max_steps>>(8*i),s->fd_pipe);
    fflush(s->fd_pipe);
}

static void buddy_visuals_set_indicator_logic(void *opaque, int n, int value)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
    fprintf(s->fd_pipe, "%cI%cV%c",4, n+'0', (255*(value>0)));
    fflush(s->fd_pipe);
}

static void buddy_visuals_set_indicator_analog(void *opaque, int n, int value)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;

    fprintf(s->fd_pipe, "%cI%cV%c",4, n+'0', value&0xFF);
    fflush(s->fd_pipe);
}


static void buddy_visuals_add_indicator(buddy_visuals_state *s, unsigned char index, unsigned char label, uint32_t color)
{
    fprintf(s->fd_pipe,"%cAI%c",3,label);

    fprintf(s->fd_pipe,"%cI%cC",7,index+'0');
    for (int i=3; i>=0; i--)
        fputc(color>>(8*i),s->fd_pipe);

    buddy_visuals_set_indicator_logic(s, index, 0);
}

static void buddy_visuals_step_in(void *opaque, int n, int level)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;

    fprintf(s->fd_pipe, "%cM%cP",7, n+'0');
    int32_t pos = level;
    for (int i=3; i>=0; i--)
        fputc(level>>(8*i),s->fd_pipe);
    fflush(s->fd_pipe);
}


static void buddy_visuals_enable_in(void *opaque, int n, int level)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;

    fprintf(s->fd_pipe, "%cM%cE%c",4, n+'0','0' +(level>0));
    fflush(s->fd_pipe);
}


static void buddy_visuals_realize(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    buddy_visuals_state *s = BUDDY_VISUALS(obj);
    qdev_init_gpio_in_named(DEVICE(obj), buddy_visuals_step_in, "motor-step",4);
    qdev_init_gpio_in_named(DEVICE(obj), buddy_visuals_enable_in, "motor-enable",4);
    qdev_init_gpio_in_named(DEVICE(obj), buddy_visuals_set_indicator_analog, "indicator-analog",4);
    qdev_init_gpio_in_named(DEVICE(obj), buddy_visuals_set_indicator_logic, "indicator-logic",4);


    const char IPC_FILE[] = "MK404.IPC";
    s->fd_pipe = fopen(IPC_FILE, "w");
    if (s->fd_pipe==NULL)
    {
        s->is_opened = false;
        printf("Could not open IPC file\n");
        return;
    }
    s->is_opened = true;
    // Pipe opened, configure the motor....


    buddy_visuals_add_motor(s, 0, 'X', 16*100, 16*100*182,false);
    buddy_visuals_add_motor(s, 1, 'Y', 16*100, 16*100*183,false);
    buddy_visuals_add_motor(s, 2, 'Z', 16*400, 16*400*185,false);
    buddy_visuals_add_motor(s, 3, 'E', 16*320, 0, true);

    buddy_visuals_add_indicator(s, 0,'X', 0xFF000000);
    buddy_visuals_add_indicator(s, 1,'Y', 0xFF0000);
    buddy_visuals_add_indicator(s, 2,'Z', 0xFF00);
    buddy_visuals_add_indicator(s, 3,'E', 0xFFFFFF00);

}



static void buddy_visuals_class_init(ObjectClass *klass, void *data)
{

}

static const TypeInfo buddy_visuals_info = {
    .name          = TYPE_BUDDY_VISUALS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(buddy_visuals_state),
    .class_init    = buddy_visuals_class_init,
    .instance_init = buddy_visuals_realize
};

static void buddy_visuals2_register_types(void)
{
    type_register_static(&buddy_visuals_info);
}

type_init(buddy_visuals2_register_types)
