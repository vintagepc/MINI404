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
#include "qemu/timer.h"
// #include "ui/console.h"
#include "qom/object.h"

#define MQ 0
#define FILE 0
#define SHM 1
#if MQ
#include <mqueue.h>
#elif SHM
#include "../3rdParty/shmemq-blog/shmemq.h"
#endif
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

// 20 kB

struct buddy_visuals_state {
    SysBusDevice parent;
    // QemuConsole *con;
#if MQ
    mqd_t queue;
    char mq_buffer[8];
#elif FILE
#define BUFFER_SIZE 1024U*20U 
    FILE *fd_pipe;
    char buffer[BUFFER_SIZE];
#else
    shmemq_t *queue;
#endif
    bool is_opened;

    int32_t redraw;
    QEMUTimer *timer_flush;

};

#define TYPE_BUDDY_VISUALS "buddy-visuals"

OBJECT_DECLARE_SIMPLE_TYPE(buddy_visuals_state, BUDDY_VISUALS)


// static void buddy_visuals_update_display(void *opaque)
// {
//     const int width = DPY_COLS, height = DPY_ROWS;
//     buddy_visuals_state *s = (buddy_visuals_state *)opaque;
//     DisplaySurface *surface = qemu_console_surface(s->con);
//     uint8_t *dest;

//     if (!s->redraw)
//         return;

//     dest = surface_data(surface);
//     memset(dest, 0xCC, sizeof(uint32_t)*DPY_ROWS*DPY_COLS);
//     // memcpy(dest, s->framebuffer, sizeof(uint32_t)*DPY_ROWS*DPY_COLS);
//     s->redraw = 0;
//     dpy_gfx_update(s->con, 0, 0, DPY_COLS, DPY_ROWS);
// }

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
#if MQ    
    if (mq_send(s->queue, "AM", 3,0)) {
#elif SHM
    char addmsg[] = {'A','M', 0,0,0,0,0};
    if (!shmemq_try_enqueue(s->queue, addmsg, sizeof(addmsg))) { 
#endif           
        fprintf(stderr,"Failed to send IPC message!\n");
        perror("AM");
    }
  
    // fprintf(s->fd_pipe,"%cAM",2);

#if MQ    
    char msg[] = {'M','0' + index, 'L', label };
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    char msg[] = {'M','0' + index, 'L', label,0,0,0 };
    if (!shmemq_try_enqueue(s->queue, msg, sizeof(msg))) { 
#endif           
        fprintf(stderr,"Failed to send IPC message!\n");
    }
  
    msg[2] = 'S';
    msg[3] = '0' + is_simple;
#if MQ
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, msg, sizeof(msg))) { 
#endif           
        fprintf(stderr,"Failed to send IPC message!\n");
    }

    // fprintf(s->fd_pipe,"%cM%cL%c",4,index+'0', label);
    // fprintf(s->fd_pipe,"%cM%cS%c",4,index+'0', is_simple+'0');
    
    char msg2[] = {'M','0' + index , 'U', (steps_per_mm>>24)&0xFF, (steps_per_mm>>16)&0xFF, (steps_per_mm>>8)&0xFF, (steps_per_mm)&0xFF};
#if MQ    
    if (mq_send(s->queue, msg2, sizeof(msg2),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, msg2, sizeof(msg2))) { 
#endif           
        fprintf(stderr,"Failed to send IPC message!\n");
    }

    // fprintf(s->fd_pipe,"%cM%cU",7,index+'0');
    // for (int i=3; i>=0; i--)
    //     fputc(steps_per_mm>>(8*i),s->fd_pipe);
    msg2[2] = 'X';
    msg2[3] = (max_steps>>24)&0xFF;
    msg2[4] = (max_steps>>16)&0xFF;
    msg2[5] = (max_steps>>8)&0xFF;
    msg2[6] = (max_steps)&0xFF;
#if MQ    
    if (mq_send(s->queue, msg2, sizeof(msg2),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, msg2, sizeof(msg2))) { 
#endif   
        fprintf(stderr,"Failed to send IPC message!\n");
    }
    //fprintf(s->fd_pipe,"%cM%cX",7,index+'0');
    // for (int i=3; i>=0; i--)
    //     msg[3+(max_steps>>(8*i),s->fd_pipe);
    // fflush(s->fd_pipe);
}
inline static void buddy_visuals_schedule_flush(buddy_visuals_state *s)
{
    // if (!timer_pending(s->timer_flush))
    // {
    //     timer_mod(s->timer_flush, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+50);
    // }
}

static void buddy_visuals_set_indicator_logic(void *opaque, int n, int value)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
#if MQ
    char msg[] = {'I','0'+n,'V', (255*(value>0)) };
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    char msg[] = {'I','0'+n,'V', (255*(value>0)),0,0,0 };
    if (!shmemq_try_enqueue(s->queue, msg, sizeof(msg))) { 
#endif   
        fprintf(stderr,"Failed to send IPC message!\n");
    }
  
    // fprintf(s->fd_pipe, "%cI%cV%c",4, n+'0', (255*(value>0)));
    // buddy_visuals_schedule_flush(s);
}

static void buddy_visuals_set_indicator_analog(void *opaque, int n, int value)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
  
#if MQ    
    char msg[] = {'I','0'+n,'V', value&0xFF };
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    char msg[] = {'I','0'+n,'V', value&0xFF,0,0,0 };
    if (!shmemq_try_enqueue(s->queue, msg, sizeof(msg))) { 
#endif        
        fprintf(stderr,"Failed to send IPC message!\n");
    }
  
    // fprintf(s->fd_pipe, "%cI%cV%c",4, n+'0', value&0xFF);
    // buddy_visuals_schedule_flush(s);
}


static void buddy_visuals_add_indicator(buddy_visuals_state *s, unsigned char index, unsigned char label, uint32_t color)
{
    // fprintf(s->fd_pipe,"%cAI%c",3,label);
#if MQ
    char msg[] = {'A','I', label };
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    char msg[] = {'A','I', label ,0,0,0,0};
    if (!shmemq_try_enqueue(s->queue, msg, sizeof(msg))) { 
#endif
        fprintf(stderr,"Failed to send IPC message!\n");
    }

    char msg2[] = {'I','0' + index , 'C', (color>>24)&0xFF, (color>>16)&0xFF, (color>>8)&0xFF, (color)&0xFF};
#if MQ
    if (mq_send(s->queue, msg2, sizeof(msg2),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, msg2, sizeof(msg2))) { 
#endif        
        fprintf(stderr,"Failed to send IPC message!\n");
    }
    // fprintf(s->fd_pipe,"%cI%cC",7,index+'0');
    // for (int i=3; i>=0; i--)
    //     fputc(color>>(8*i),s->fd_pipe);

    buddy_visuals_set_indicator_logic(s, index, 0);
}

// Used to flush the queue at regular intervals when stepping, as high step rates can slog things down.
// static void
// buddy_visuals_flush_timer(void *opaque)
// {
//     buddy_visuals_state *s = opaque;
// //    fflush(s->fd_pipe);
// }


static void buddy_visuals_inverted_step_in(void *opaque, int n, int level)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
    int temp = -level;
    char msg[] = {'M','0' + n, 'P', 0,0,0,0};
    memcpy(msg+3, &temp, 4);
#if MQ
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, msg, sizeof(msg))) { 
#endif
        fprintf(stderr,"Failed to send IPC message!\n");
    }
    // fprintf(s->fd_pipe, "%cM%cP",7, n+'0');
    // // int32_t pos = level;
    // for (int i=3; i>=0; i--)
    //     fputc(level>>(8*i),s->fd_pipe);

    // buddy_visuals_schedule_flush(s);
  
}

static void buddy_visuals_step_in(void *opaque, int n, int level)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
    
    char msg[] = {'M','0' + n, 'P', 0,0,0,0};
    memcpy(msg+3, &level, 4);
#if MQ
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, msg, sizeof(msg))) { 
#endif
        fprintf(stderr,"Failed to send IPC message!\n");
    }
    // fprintf(s->fd_pipe, "%cM%cP",7, n+'0');
    // // int32_t pos = level;
    // for (int i=3; i>=0; i--)
    //     fputc(level>>(8*i),s->fd_pipe);

    // buddy_visuals_schedule_flush(s);
  
}


static void buddy_visuals_enable_in(void *opaque, int n, int level)
{
    buddy_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
#if MQ
    char msg[] = {'M','0' + n, 'E', '0'+ (level>0)};
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    char msg[] = {'M','0' + n, 'E', '0'+ (level>0),0,0,0};
    if (!shmemq_try_enqueue(s->queue, msg, sizeof(msg))) { 
#endif        
        fprintf(stderr,"Failed to send IPC message!\n");
    }
//    fflush(s->fd_pipe);
}

static void buddy_visuals_realize(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    buddy_visuals_state *s = BUDDY_VISUALS(obj);
    qdev_init_gpio_in_named(dev, buddy_visuals_step_in, "motor-step",4);
    qdev_init_gpio_in_named(dev, buddy_visuals_inverted_step_in, "motor-invstep",4);
    qdev_init_gpio_in_named(dev, buddy_visuals_enable_in, "motor-enable",4);
    qdev_init_gpio_in_named(dev, buddy_visuals_set_indicator_analog, "indicator-analog",10);
    qdev_init_gpio_in_named(dev, buddy_visuals_set_indicator_logic, "indicator-logic",10);
    const char IPC_FILE[] = "/MK404IPC";
    
#if MQ
    s->queue = mq_open(IPC_FILE, O_WRONLY );
    if (s->queue == (mqd_t)-1) {
        printf("Could not open message queue. Skipping connection.\n");
        return;
    }
    mq_send(s->queue, "C",1,0);
#elif FILE
    struct stat file;
    if (stat(IPC_FILE, &file)<0)
    {
        printf("MK404.IPC file not found. Skipping connection.\n");
        return;
    }
    s->fd_pipe = fopen(IPC_FILE, "w");
    if (s->fd_pipe==NULL)
    {
        s->is_opened = false;
        printf("Could not open IPC file\n");
        return;
    }
    memset(s->buffer, 0,BUFFER_SIZE);
    setbuffer(s->fd_pipe, s->buffer,BUFFER_SIZE);
    s->timer_flush = timer_new_ms(QEMU_CLOCK_VIRTUAL, buddy_visuals_flush_timer, s);
#else
    s->queue = shmemq_new(IPC_FILE, 500, 7);

    char MSG[] = {'C', 0,0,0,0,0,0};
    shmemq_try_enqueue(s->queue, MSG, sizeof(MSG));

#endif
    s->is_opened = true;
    // Pipe opened, configure the motor....


    buddy_visuals_add_motor(s, 0, 'X', 16*100, 16*100*182,false);
    buddy_visuals_add_motor(s, 1, 'Y', 16*100, 16*100*183,false);
    buddy_visuals_add_motor(s, 2, 'Z', 16*400, 16*400*185,false);
    buddy_visuals_add_motor(s, 3, 'E', 16*320, 0, true);

    buddy_visuals_add_indicator(s, 0,'X', 0xFF000000); // DIAG pins (temporary)
    buddy_visuals_add_indicator(s, 1,'Y', 0xFF0000);
    buddy_visuals_add_indicator(s, 2,'Z', 0xFF00);
    buddy_visuals_add_indicator(s, 3,'E', 0xFFFFFF00);
    buddy_visuals_add_indicator(s, 4,'E', 0xFF0000); // E fan
    buddy_visuals_add_indicator(s, 5,'P', 0xFF0000); // P fan
    buddy_visuals_add_indicator(s, 6,'F', 0xFFFF0000); // Fsensor
    buddy_visuals_add_indicator(s, 7,'M', 0xFF000000); // Z-probe/minda
    buddy_visuals_add_indicator(s, 8,'H', 0xFF000000); // E heater
    buddy_visuals_add_indicator(s, 9,'B', 0xFF000000); // Bed heater

#if MQ
    struct mq_attr attr, attr2;
    mq_getattr(s->queue, &attr);
    mq_getattr(s->queue, &attr2);
    attr.mq_flags = O_NONBLOCK; // Change to nonblock now that setup is done.
    mq_setattr(s->queue, &attr, &attr2);
#endif


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
