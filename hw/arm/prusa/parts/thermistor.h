/*
	thermistor.h - helpers for the thermistor model that other parts of the
	machine definition need.

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

#ifndef PRUSA_PARTS_THERMISTOR_H
#define PRUSA_PARTS_THERMISTOR_H

#include <stdbool.h>
#include <stdint.h>

// Reverse of the sensor curve: given a 10-bit ADC reading, what temperature is
// the probe at? Used to turn a configured power-on ADC value into a starting
// temperature for the thermal model, so the reading evolves from there instead
// of being pinned. Returns false if table_no is unknown or the reading is off
// the end of the table.
bool thermistor_temp_for_adc10(uint16_t table_no, uint16_t adc10, float *temp_out);

#endif
