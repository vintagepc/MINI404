/*
	dashboard_types.h
	
    Helper header with some shared C and C++ enumeration indexes.
    
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

#pragma once

enum {
    DB_NONE,
    DB_MINI_LITE,
    DB_MINI_FULL,
};

enum {
    DB_MOTOR_X,
    DB_MOTOR_Y,
    DB_MOTOR_Z,
    DB_MOTOR_E,
    DB_MOTOR_COUNT,
};

enum {
    DB_IND_PFAN,
    DB_IND_EFAN,
    DB_IND_FSENS,
    DB_IND_ZPROBE, 
    DB_IND_BED, 
    DB_IND_HTR,
    DB_IND_MEDIA,
    DB_IND_COUNT,
};
