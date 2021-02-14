/*
	p404-scriptable.c - Stub and typedefs for Qemu interface class

	Copyright 2020 VintagePC <https://github.com/vintagepc/>
    Ported to Mini404 in 2021

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
#include "qemu-common.h"
#include "qapi/error.h"
#include "p404scriptable.h"

static const TypeInfo p404_scriptable_type_info = {
    .name = TYPE_P404_SCRIPTABLE,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(P404ScriptIFClass),
};

extern int (*p404_get_func(P404ScriptIF *src))(P404ScriptIF *self, unsigned int iAction, const void* args) {
    P404ScriptIFClass *s = P404_SCRIPTABLE_GET_CLASS(src);
    return s->ScriptHandler;
}

static void p404_scriptable_register_types(void)
{
    type_register_static(&p404_scriptable_type_info);
}

type_init(p404_scriptable_register_types)
