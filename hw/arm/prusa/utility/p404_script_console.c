/*
 * scriptcon OLED controller with OSRAM Pictiva 128x64 display.
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
#include "qemu/option.h"
#include "qapi/error.h"
#include "qom/object.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "../utility/macros.h"
#include "hw/qdev-properties.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "qemu/readline.h"
#include "ui/console.h"

struct ScriptConsoleState {
    DeviceState parent;

    Chardev *input_source;

    CharBackend be;

    char* inputtype;

    bool is_vc;

    bool is_busy; 

    int cmd_len;
    char cmd[255];

    ReadLineState *rl_state;
};


#define TYPE_P404_SCRIPT_CONSOLE "p404-scriptcon"
OBJECT_DECLARE_SIMPLE_TYPE(ScriptConsoleState, P404_SCRIPT_CONSOLE)


static int scriptcon_can_read(void* opaque) {
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);

    if ((!s->is_busy) && s->cmd_len<sizeof(s->cmd)) {
        return (sizeof(s->cmd)-s->cmd_len);
    } else {
        return 0;
    }

}

static void scriptcon_execute(void *opaque, const char *cmdline,
                               void *readline_opaque)
{
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);

    s->is_busy = true;    
    printf("incoming cmd: %s\n",cmdline);
    s->is_busy = false;
}

static void scriptcon_auto_return(void *opaque, const char* cmd_completed)
{
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);
    readline_add_completion(s->rl_state, cmd_completed);
}

// in ScriptHost.cpp
extern void scripthost_autocomplete(void* p, const char* cmdline, void(*add_func)(void*,const char*));

static void scriptcon_autocomplete(void *opaque,
                                    const char *cmdline)
{
    scripthost_autocomplete(opaque, cmdline, scriptcon_auto_return);
}

static void GCC_FMT_ATTR(2, 3) scriptcon_printf(void *opaque,
                                                       const char *fmt, ...)
{
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);
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

static void scriptcon_flush(void *opaque)
{
    // ScriptConsoleState *s = opaque;
    // uint8_t buf[1] = {'\r'};
    // qemu_chr_fe_write(&s->be, buf, 1);
}

static void scriptcon_event(void *opaque, QEMUChrEvent event)
{
    ScriptConsoleState *s = opaque;

    switch (event) {
        case CHR_EVENT_OPENED:
            scriptcon_printf(s, "P404 Script console. Use Tab for completion options.\r\n");
            readline_restart(s->rl_state);
            readline_show_prompt(s->rl_state);
        break;
        default:
        break;
    }
}

static void scriptcon_read_command(ScriptConsoleState *s)
{
    if (!s->rl_state) {
        return;
    }
    readline_start(s->rl_state, "P404> ", 0, scriptcon_execute, NULL);
    readline_show_prompt(s->rl_state);
}

static void scriptcon_read(void *opaque, const uint8_t *buf, int size){
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);

    if (s->rl_state) {
        for (int i = 0; i < size; i++) {
            readline_handle_byte(s->rl_state, buf[i]);
        }
        return;
    } 
    printf("err: no readline??\n");
    // printf("Input: %02x len %d\n", *buf, size);
    // if (*buf == 0x08){
    //     printf("bksp\n");
    //     if (s->cmd_len>0){
    //         s->cmd[s->cmd_len]=0;
    //         s->cmd_len--;
    //     }
    // } else if (*buf == 0x0A) { // return
    //     printf("enter\n");
    // } else if (*buf == 0x09) { // tab
    // } else {
    //     s->cmd[s->cmd_len++] = *buf;
    // }
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(ScriptConsoleState, scriptcon, P404_SCRIPT_CONSOLE, DEVICE, {NULL})

static void scriptcon_finalize(Object *obj)
{
    printf("Disp_finalize\n");
}

static void scriptcon_init(Object *obj)
{
}

// static const GraphicHwOps scriptcon_ops = {
//   //  .gfx_update = scriptcon_update_gfx,
//     .text_update = scriptcon_update,
//     .invalidate = scriptcon_invalidate,
// };



static void scriptcon_realize(DeviceState *d, Error **errp)
{
    //DeviceState *dev = DEVICE(d);
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(d);
    // if (g_strcmp0(s->inputtype,"vc") ==0 ){
    //     printf("Creating new scripting VC\n");
    //     s->input_source = qemu_chr_new("P404-script-console", "vc:80Cx24C",NULL);  //qemu_chardev_new("P404 Script Console", "chardev-vc",
    //            //           NULL,  NULL, errp);
    //     s->is_vc = true;
    // }
    s->input_source = qemu_chr_find("p404-scriptcon");
    if (!s->input_source) {
        return;
    }
    qemu_chr_fe_init(&s->be, s->input_source, errp);
    qemu_chr_fe_set_handlers(&s->be, scriptcon_can_read, scriptcon_read, scriptcon_event, NULL, s, NULL, true);
        s->rl_state = readline_init(scriptcon_printf,
                            scriptcon_flush,
                            s,
                            scriptcon_autocomplete);
    scriptcon_read_command(s);


    // if (false) graphic_console_set_hwops(s->con, &scriptcon_ops, s);

    // s->con = qemu_console_lookup_by_device(DEVICE_STATE(s->input), 0);
    // if (s->con == NULL) {
    //     printf("NULL console\n");
    // }
    // graphic_console_set_hwops(s->con, &scriptcon_ops, s);

    // qemu_chr_fe_set_echo(&s->be, true);

    // TODO- need to find a way to get a chardevbackend in here. Maybe turn this 
    // into a chardev parent item?
   //s->con = graphic_console_init(dev, 0, &scriptcon_ops, s);
    
   
}

static Property scriptcon_properties[] = {
    DEFINE_PROP_STRING("inputtype", ScriptConsoleState, inputtype),
    DEFINE_PROP_END_OF_LIST(),
};


static void scriptcon_class_init(ObjectClass *klass, void *data)
{
   // ChardevClass *cc = CHARDEV_CLASS(klass);   
    // cc->chr_write = scriptcon_read;
    // cc->chr_accept_input 
  //  cc->parse = scriptcon_parse;
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = scriptcon_realize;
    dc->user_creatable = true;
   
//    //dc->reset = scriptcon_reset;
    device_class_set_props(dc, scriptcon_properties);
}
