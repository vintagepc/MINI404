/*
	mini_visuals.c
	
    SHM IPC interface for Mini404 to use MK404 for extended display.
    
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
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define MQ 0
#define FILE 0
#define SHM 1
#if MQ
#include <mqueue.h>
#elif SHM
#include "../3rdParty/shmemq404/shmemq.h"
#endif

struct mini_visuals_state {
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
    QEMUTimer *timer_flush;

};

#define TYPE_MINI_VISUALS "mini-visuals"

OBJECT_DECLARE_SIMPLE_TYPE(mini_visuals_state, MINI_VISUALS)

static void mini_visuals_add_motor(mini_visuals_state *s, unsigned char index, unsigned char label, uint32_t steps_per_mm, int32_t max_steps, bool is_simple)
{
#if MQ    
    if (mq_send(s->queue, "AM", 3,0)) {
#elif SHM
    shm404_msg_t addmsg = SHM_ADD_MOTOR;
    if (!shmemq_try_enqueue(s->queue, &addmsg)) { 
#endif           
        fprintf(stderr,"Failed to send IPC message!\n");
    }
  
    // fprintf(s->fd_pipe,"%cAM",2);

#if MQ    
    char msg[] = {'M','0' + index, 'L', label };
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    shm404_msg_t msg = SHM_SET_MOTOR_ID;
    msg[1] += index;
    msg[3] = label;
    if (!shmemq_try_enqueue(s->queue, &msg)) { 
#endif           
        fprintf(stderr,"Failed to send IPC message!\n");
    }
  
    msg[2] = 'S';
    msg[3] = '0' + is_simple;
#if MQ
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, &msg)) { 
#endif           
        fprintf(stderr,"Failed to send IPC message!\n");
    }

    // fprintf(s->fd_pipe,"%cM%cL%c",4,index+'0', label);
    // fprintf(s->fd_pipe,"%cM%cS%c",4,index+'0', is_simple+'0');
    shm404_msg_t msg2 = SHM_SET_MOTOR_USTEPS;
    msg2[1] += index;

    msg2[3] = (steps_per_mm>>24)&0xFF;
    msg2[4] = (steps_per_mm>>16)&0xFF;
    msg2[5] = (steps_per_mm>>8)&0xFF; 
    msg2[6] = (steps_per_mm)&0xFF;

#if MQ    
    if (mq_send(s->queue, msg2, sizeof(msg2),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, &msg2)) { 
#endif           
        fprintf(stderr,"Failed to send IPC message!\n");
    }

    // fprintf(s->fd_pipe,"%cM%cU",7,index+'0');
    // for (int i=3; i>=0; i--)
    //     fputc(steps_per_mm>>(8*i),s->fd_pipe);
    shm404_msg_t msg3 = SHM_SET_MOTOR_MSTEPS;
    msg3[1] += index;
    msg3[3] = (max_steps>>24)&0xFF;
    msg3[4] = (max_steps>>16)&0xFF;
    msg3[5] = (max_steps>>8)&0xFF;
    msg3[6] = (max_steps)&0xFF;
#if MQ    
    if (mq_send(s->queue, msg2, sizeof(msg2),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, &msg3)) { 
#endif   
        fprintf(stderr,"Failed to send IPC message!\n");
    }
    //fprintf(s->fd_pipe,"%cM%cX",7,index+'0');
    // for (int i=3; i>=0; i--)
    //     msg[3+(max_steps>>(8*i),s->fd_pipe);
    // fflush(s->fd_pipe);
}
inline static void mini_visuals_schedule_flush(mini_visuals_state *s)
{
    // if (!timer_pending(s->timer_flush))
    // {
    //     timer_mod(s->timer_flush, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+50);
    // }
}

static void mini_visuals_set_indicator_logic(void *opaque, int n, int value)
{
    mini_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
#if MQ
    char msg[] = {'I','0'+n,'V', (255*(value>0)) };
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    shm404_msg_t msg = SHM_SET_INDICATOR;
    msg[1] += n;
    msg[3] = (255*(value>0));
    if (!shmemq_try_enqueue(s->queue, &msg)) { 
#endif   
        fprintf(stderr,"Failed to send IPC message!\n");
    }
  
    // fprintf(s->fd_pipe, "%cI%cV%c",4, n+'0', (255*(value>0)));
    // mini_visuals_schedule_flush(s);
}

static void mini_visuals_set_indicator_analog(void *opaque, int n, int value)
{
    mini_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
  
#if MQ    
    char msg[] = {'I','0'+n,'V', value&0xFF };
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    shm404_msg_t msg = SHM_SET_INDICATOR;
    msg[1] += n;
    msg[3] = value&0xFF;
    if (!shmemq_try_enqueue(s->queue, &msg)) { 
#endif        
        fprintf(stderr,"Failed to send IPC message!\n");
    }
  
    // fprintf(s->fd_pipe, "%cI%cV%c",4, n+'0', value&0xFF);
    // mini_visuals_schedule_flush(s);
}


static void mini_visuals_add_indicator(mini_visuals_state *s, unsigned char index, unsigned char label, uint32_t color)
{
    // fprintf(s->fd_pipe,"%cAI%c",3,label);
#if MQ
    char msg[] = {'A','I', label };
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    shm404_msg_t msg = SHM_ADD_INDICATOR;
    msg[2] = label;
    if (!shmemq_try_enqueue(s->queue, &msg)) { 
#endif
        fprintf(stderr,"Failed to send IPC message!\n");
    }
    shm404_msg_t msg2 = SHM_SET_IND_COLOUR;
    msg2[1] += index;
    msg2[3] = (color>>24)&0xFF;
    msg2[4] = (color>>16)&0xFF;
    msg2[5] = (color>>8)&0xFF;
    msg2[6] = (color)&0xFF;
#if MQ
    if (mq_send(s->queue, msg2, sizeof(msg2),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, &msg2)) { 
#endif        
        fprintf(stderr,"Failed to send IPC message!\n");
    }
    // fprintf(s->fd_pipe,"%cI%cC",7,index+'0');
    // for (int i=3; i>=0; i--)
    //     fputc(color>>(8*i),s->fd_pipe);

    mini_visuals_set_indicator_logic(s, index, 0);
}
static void mini_visuals_step_in(void *opaque, int n, int level)
{
    mini_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
    
    shm404_msg_t msg = SHM_SET_MOTOR_SPOS;
    msg[1] += n;
    memcpy(msg+3, &level, 4);
#if MQ
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    if (!shmemq_try_enqueue(s->queue, &msg)) { 
#endif
        fprintf(stderr,"Failed to send IPC message!\n");
    }
    // fprintf(s->fd_pipe, "%cM%cP",7, n+'0');
    // // int32_t pos = level;
    // for (int i=3; i>=0; i--)
    //     fputc(level>>(8*i),s->fd_pipe);

    // mini_visuals_schedule_flush(s);
  
}


static void mini_visuals_enable_in(void *opaque, int n, int level)
{
    mini_visuals_state *s = opaque;
    if (!s->is_opened)
        return;
#if MQ
    char msg[] = {'M','0' + n, 'E', '0'+ (level>0)};
    if (mq_send(s->queue, msg, sizeof(msg),0)) {
#elif SHM
    shm404_msg_t msg = SHM_SET_MOTOR_EN;
    msg[1] +=n ;
    msg[3] = level>0;
    if (!shmemq_try_enqueue(s->queue, &msg)) { 
#endif        
        fprintf(stderr,"Failed to send IPC message!\n");
    }
//    fflush(s->fd_pipe);
}

static void mini_visuals_realize(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    mini_visuals_state *s = MINI_VISUALS(obj);
    qdev_init_gpio_in_named(dev, mini_visuals_step_in, "motor-step",4);
    qdev_init_gpio_in_named(dev, mini_visuals_enable_in, "motor-enable",4);
    qdev_init_gpio_in_named(dev, mini_visuals_set_indicator_analog, "indicator-analog",10);
    qdev_init_gpio_in_named(dev, mini_visuals_set_indicator_logic, "indicator-logic",10);
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
    s->timer_flush = timer_new_ms(QEMU_CLOCK_VIRTUAL, mini_visuals_flush_timer, s);
#else
    s->queue = shmemq_open(IPC_FILE);
    if (s->queue == NULL)
    {
        printf("Could not open SHM queue. Skipping connection.\n");
        return; // Queue not available.
    }

    shm404_msg_t msg = SHM_CLEAR;
    shmemq_try_enqueue(s->queue, &msg);

#endif
    s->is_opened = true;
    // Pipe opened, configure the motor....


    mini_visuals_add_motor(s, 0, 'X', 16*100, 16*100*182,false);
    mini_visuals_add_motor(s, 1, 'Y', 16*100, 16*100*183,false);
    mini_visuals_add_motor(s, 2, 'Z', 16*400, 16*400*185,false);
    mini_visuals_add_motor(s, 3, 'E', 16*320, 0, true);

    mini_visuals_add_indicator(s, 0,'X', 0xFF000000); // DIAG pins (temporary)
    mini_visuals_add_indicator(s, 1,'Y', 0xFF0000);
    mini_visuals_add_indicator(s, 2,'Z', 0xFF00);
    mini_visuals_add_indicator(s, 3,'E', 0xFFFFFF00);
    mini_visuals_add_indicator(s, 4,'E', 0xFF0000); // E fan
    mini_visuals_add_indicator(s, 5,'P', 0xFF0000); // P fan
    mini_visuals_add_indicator(s, 6,'F', 0xFFFF0000); // Fsensor
    mini_visuals_add_indicator(s, 7,'M', 0xFF000000); // Z-probe/minda
    mini_visuals_add_indicator(s, 8,'H', 0xFF000000); // E heater
    mini_visuals_add_indicator(s, 9,'B', 0xFF000000); // Bed heater

#if MQ
    struct mq_attr attr, attr2;
    mq_getattr(s->queue, &attr);
    mq_getattr(s->queue, &attr2);
    attr.mq_flags = O_NONBLOCK; // Change to nonblock now that setup is done.
    mq_setattr(s->queue, &attr, &attr2);
#endif


}

static void mini_visuals_class_init(ObjectClass *klass, void *data)
{

}

static const TypeInfo mini_visuals_info = {
    .name          = TYPE_MINI_VISUALS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(mini_visuals_state),
    .class_init    = mini_visuals_class_init,
    .instance_init = mini_visuals_realize
};

static void mini_visuals2_register_types(void)
{
    type_register_static(&mini_visuals_info);
}

type_init(mini_visuals2_register_types)
