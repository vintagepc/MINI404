/*
    p404_motor_if.h  - Interface for motor display type objects in QEMU
	to work with the 2d dashboard

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

#ifndef P404_MOTOR_IF_H
#define P404_MOTOR_IF_H


#include "qemu/osdep.h"
#include "qom/object.h"

#define TYPE_P404_MOTOR_IF "prusa404-motor-if"

typedef struct P404MotorIFClass P404MotorIFClass;

DECLARE_CLASS_CHECKERS(P404MotorIFClass, P404_MOTOR_IF, 
        TYPE_P404_MOTOR_IF)
#define P404_MOTOR_IF(obj) \
    INTERFACE_CHECK(P404MotorIF, (obj), TYPE_P404_MOTOR_IF)

typedef struct P404MotorIF P404MotorIF;

typedef struct  
{
	float current_pos;
	float max_pos;
	struct {
		bool enabled;
		bool stalled;
		bool stealth;
		bool changed; // Use this to notify this region should be redrawn. 
	} status;
	char label;
} p404_motorif_status_t;

struct P404MotorIFClass {
    InterfaceClass parent;

    // Called to request the current motor position in mm.
    const p404_motorif_status_t* (*get_current_status)(P404MotorIF *self);

};

extern const p404_motorif_status_t* p404_motor_if_get_status(P404MotorIF *src);
 
#endif // P404_MOTOR_IF_H
