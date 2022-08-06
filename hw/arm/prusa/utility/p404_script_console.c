/*
    p404_script_console.c  - Script console object used to bind scripting engine.

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
#include "qapi/error.h"
#include "qom/object.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "../utility/macros.h"
#include "../utility/ArgHelper.h"
#include "hw/qdev-properties.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "qemu/readline.h"
#include "ui/console.h"

struct ScriptConsoleState {
    SysBusDevice parent;

    Chardev *input_source;

    CharBackend be;

    bool disable_echo;

    bool is_vc;

    bool is_busy; 

    bool show_status;

    QEMUTimer *scripting;

    ReadLineState *rl_state;
};


#define TYPE_P404_SCRIPT_CONSOLE "p404-scriptcon"
OBJECT_DECLARE_SIMPLE_TYPE(ScriptConsoleState, P404_SCRIPT_CONSOLE)


extern int scripthost_run(int64_t iTime);
extern bool scripthost_setup(const char* strScript, void *pConsole);


static int scriptcon_can_read(void* opaque) {
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);

    if (!s->is_busy) {
        return true;
    } else {
        return 0;
    }

}

extern void scripthost_autocomplete(void* p, const char* cmdline, void(*add_func)(void*,const char*));
extern void scripthost_execute(const char* cmd);

static void scriptcon_execute(void *opaque, const char *cmdline,
                               void *readline_opaque)
{
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);
    s->is_busy = true;    
    s->show_status = true;
    scripthost_execute(cmdline);
    if (timer_expired(s->scripting,qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)))
    {
        timer_mod(s->scripting, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+10);
    }
}

static void scriptcon_auto_return(void *opaque, const char* cmd_completed)
{
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);
    readline_add_completion(s->rl_state, cmd_completed);
}

// in ScriptHost.cpp

static void scriptcon_autocomplete(void *opaque,
                                    const char *cmdline)
{
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(opaque);
    readline_set_completion_index(s->rl_state, strlen(cmdline));
    scripthost_autocomplete(opaque, cmdline, scriptcon_auto_return);
}

// Dummy handler for disabled echo mode. 
static void GCC_FMT_ATTR(2, 3) scriptcon_dummy_printf(void *opaque,
                                                       const char *fmt, ...)
                                                       {

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

extern void scriptcon_print_out(void* opaque, const char *str);

extern void scriptcon_print_out(void* opaque, const char *str) {
    scriptcon_printf(opaque, "%s\n",str);
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

static const char strOK[8] = "Success";
static const char strFailed[6] = "Error";
static const char strWait[8] = "Waiting";
static const char strTimeout[10] = "Timed out";
static const char strSyntax[22] = "Syntax/Argument Error";

static void scriptcon_timer_expire(void *opaque)
{
    ScriptConsoleState *s = opaque;
    const char* messages[] = {strOK, strFailed, strWait, strTimeout, strSyntax};
    int status = scripthost_run(qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    timer_mod(s->scripting, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL)+10);
    bool should_restart = false;
    if (status !=3 && s->show_status) {
        should_restart = true;
    }
    if (s->show_status && (status ==2 || status>3)) {
        scriptcon_printf(s,"%s\n",messages[status-1]);
    }
    if (should_restart) {
        readline_restart(s->rl_state);
        readline_show_prompt(s->rl_state);
        s->show_status = false;
        s->is_busy = false;
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

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(ScriptConsoleState, scriptcon, P404_SCRIPT_CONSOLE, SYS_BUS_DEVICE, {NULL})

static void scriptcon_finalize(Object *obj)
{
    printf("Disp_finalize\n");
}

static void scriptcon_init(Object *obj)
{
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(obj);
    s->scripting = timer_new_ms(QEMU_CLOCK_VIRTUAL,
    (QEMUTimerCB *)scriptcon_timer_expire, s);


}

static void scriptcon_realize(DeviceState *d, Error **errp)
{
    //DeviceState *dev = DEVICE(d);
    ScriptConsoleState *s = P404_SCRIPT_CONSOLE(d);

    const char* script = arghelper_get_string("script");
    if (scripthost_setup(script, OBJECT(d))) // TODO- move scripthost out of this input handler?
    {
        // Start script timer
        timer_mod(s->scripting,  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
    }

    s->input_source = qemu_chr_find("p404-scriptcon");
    if (!s->input_source) {
        return;
    }
    qemu_chr_fe_init(&s->be, s->input_source, errp);
    if (s->disable_echo) {
        s->rl_state = readline_init(scriptcon_dummy_printf,
                            scriptcon_flush,
                            s,
                            scriptcon_autocomplete);
    } else {
        s->rl_state = readline_init(scriptcon_printf,
                            scriptcon_flush,
                            s,
                            scriptcon_autocomplete);
    }
    qemu_chr_fe_set_handlers(&s->be, scriptcon_can_read, scriptcon_read, scriptcon_event, NULL, s, NULL, true);
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
    DEFINE_PROP_BOOL("no_echo", ScriptConsoleState, disable_echo, false),
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
