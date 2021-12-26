/*
    p404_motor_if.c  - Interface for motor display type objects in QEMU
	to work with the 2d dashboard

	Copyright 2021 VintagePC <https://github.com/vintagepc/>
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
#include "p404_motor_if.h"

static const TypeInfo p404_motor_if_type_info = {
    .name = TYPE_P404_MOTOR_IF,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(P404MotorIFClass),
};

extern const p404_motorif_status_t* p404_motor_if_get_status(P404MotorIF *src) {
    P404MotorIFClass *s = P404_MOTOR_IF_GET_CLASS(src);
    return s->get_current_status(src);
}

static void p404_motor_if_register_types(void)
{
    type_register_static(&p404_motor_if_type_info);
}

type_init(p404_motor_if_register_types)
