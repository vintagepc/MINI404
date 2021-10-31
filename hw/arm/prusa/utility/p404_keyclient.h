/*
    p404_keyclient.h  - Key handling interface/dispatch 
	to work with ScriptHost.cpp.

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

#ifndef P404_KEYCLIENT_H
#define P404_KEYCLIENT_H

typedef void* p404_key_handle;

#ifndef __cplusplus

#include "qemu/osdep.h"
#include "qom/object.h"

#define TYPE_P404_KEYCLIENT "Prusa404-keyclient"

typedef struct P404KeyIFClass P404KeyIFClass;

typedef unsigned char Key;

DECLARE_CLASS_CHECKERS(P404KeyIFClass, P404_KEYCLIENT, 
        TYPE_P404_KEYCLIENT)
#define P404_KEYCLIENT(obj) \
    INTERFACE_CHECK(P404KeyIF, (obj), TYPE_P404_KEYCLIENT)

typedef struct P404KeyIF P404KeyIF;

struct P404KeyIFClass {
    InterfaceClass parent;

    // Called by the scripting engine when the target should perform the defined action.
    void (*KeyHandler)(P404KeyIF *obj, Key key);
};
    
    extern p404_key_handle p404_new_keyhandler(P404KeyIF* src);

    extern void p404_register_keyhandler(p404_key_handle src, const Key key, const char* description);
    extern void p404_call_keyfunc(P404KeyIF *dst, const Key key);
#endif

 
#endif // P404_KEYCLIENT_H
