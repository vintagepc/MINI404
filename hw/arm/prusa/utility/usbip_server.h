/*
    USBIP server module. Generic, but somewhat specific to 
    handling a USB CDC implementation in its current use.

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

#ifndef USBIP_SERVER_H
#define USBIP_SERVER_H

#include "qemu/osdep.h"
#include "qom/object.h"
#include "usbip.h"

#define TYPE_USBIP_SERVER "usbip-server"

typedef struct USBIPServerClass USBIPServerClass;

DECLARE_CLASS_CHECKERS(USBIPServerClass, USBIP_SERVER, 
        TYPE_USBIP_SERVER)
#define USBIP_SERVER(obj) \
    INTERFACE_CHECK(USBIPIF, (obj), TYPE_USBIP_SERVER)

typedef struct USBIPIF USBIPIF;

typedef struct {
    uint32_t port;
    USBIPIF* self;
    int _private_fd;
} usbip_cfg_t;

struct USBIPServerClass {
    InterfaceClass parent;

    // Called on arrival of a new incoming packet. YOU are responsible for the thread safety 
    // of handling this data from the TCP thread to your USB implementation.
    void (*usbip_handle_packet)(USBIPIF* self, USBIP_CMD_SUBMIT* cmd, USBIP_RET_SUBMIT* usb_req);
};

    // extern int (*p404_get_func(P404ScriptIF *))(P404ScriptIF *self, unsigned int iAction, script_args args);

    // The main run function for your thread that listens on the
    // socket. Pass a usbip_cfg_t to set the port number. 
    extern void* usbip_thread_run(void* run_info);

    extern void usbip_send_reply(usbip_cfg_t* cfg, USBIP_RET_SUBMIT* usb_req, const char* data, unsigned int size, unsigned int status);

    extern bool usbip_read_payload(usbip_cfg_t* cfg, char* buffer, unsigned int size);

 
#endif // USBIP_SERVER_H
