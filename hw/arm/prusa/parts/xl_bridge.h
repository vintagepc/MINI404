/*
    xl_bridge.h - include with defines for bridge properties.

	Copyright 2022-3 VintagePC <https://github.com/vintagepc/>

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


// Enum used for GPIO indexing etc.
enum {
	XL_DEV_XBUDDY = 0,
	XL_DEV_BED,
	XL_DEV_T0,
	XL_DEV_T1,
	XL_DEV_T2,
	XL_DEV_T3,
	XL_DEV_T4,
	XL_DEV_T5,
	XL_BRIDGE_COUNT
};

// GPIO indices. Use these to index output GPIOS directly.
// Each receiver only has one set of outputs anyway.
// For the inputs, GPIO index is interleaved pin indexes
// with devices. Not all combos make sense, of course
// but uniformity makes the logic saner since it is
// expected things only listen to events that matter to them.
// e.g.
// 0 = xbuddy e_step, 1 = xbuddy e_dir and so on.
enum {
	XLBRIDGE_PIN_E_STEP = 0,
	XLBRIDGE_PIN_E_DIR,
	XLBRIDGE_PIN_nAC_FAULT,
	XLBRIDGE_PIN_RESET,
	XLBRIDGE_PIN_Z_UM,
	XLBRIDGE_PIN_COUNT
};
