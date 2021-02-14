/*
    p404scriptable.h  - Interface for scriptable objects in QEMU
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

#ifndef P404_SCRIPTABLE_H
#define P404_SCRIPTABLE_H

#ifndef __cplusplus

#include "qemu/osdep.h"
#include "qom/object.h"

#define TYPE_P404_SCRIPTABLE "Prusa404-scriptable"

typedef struct P404ScriptIFClass P404ScriptIFClass;

DECLARE_CLASS_CHECKERS(P404ScriptIFClass, P404_SCRIPTABLE, 
        TYPE_P404_SCRIPTABLE)
#define P404_SCRIPTABLE(obj) \
    INTERFACE_CHECK(P404ScriptIF, (obj), TYPE_P404_SCRIPTABLE)

typedef struct P404ScriptIF P404ScriptIF;

struct P404ScriptIFClass {
    InterfaceClass parent;

    // Called by the scripting engine when the target should perform the defined action.
    int (*ScriptHandler)(P404ScriptIF *self, unsigned int action, const void* args);
};

// DANGER WILL ROBINSON! These MUST match the order in IScriptable!
enum 
{
    ScriptLS_Error,
    ScriptLS_Timeout,
    ScriptLS_Waiting,
    ScriptLS_HoldExec,
    ScriptLS_Running,
    ScriptLS_Finished,
    ScriptLS_Unhandled
};
#endif
    extern int (*p404_get_func(P404ScriptIF *))(P404ScriptIF *self, unsigned int iAction, const void* args);
 
#endif // P404_SCRIPTABLE_H
