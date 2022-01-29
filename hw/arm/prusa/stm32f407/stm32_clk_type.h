/*
 * Basic Clock Tree Building Blocks
 *
 * Copyright (C) 2012 Andre Beckus
 * Adapted for QEMU 5.2 in 2021 by VintagePC <http://github.com/vintagepc>
 *
 * Source code roughly based on omap_clk.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STM32_CLK_H
#define STM32_CLK_H

#include "qemu/osdep.h"
#include "../utility/macros.h"

// Cannot be >255 as counts are stored in uint8_ts
#define CLKTREE_MAX_IRQ 16
#define CLKTREE_MAX_OUTPUT 24
#define CLKTREE_MAX_INPUT 24

QEMU_BUILD_BUG_MSG(CLKTREE_MAX_INPUT>=256,"DEFINE EXCEEDS SIZE OF uint8_t used to store it in struct Clk");
QEMU_BUILD_BUG_MSG(CLKTREE_MAX_OUTPUT>=256, "DEFINE EXCEEDS SIZE OF uint8_t used to store it in struct Clk");
QEMU_BUILD_BUG_MSG(CLKTREE_MAX_IRQ>=256, "DEFINE EXCEEDS SIZE OF uint8_t used to store it in struct Clk");


struct Clk {                                                                    
    const char *name;                                                           
                                                                                
    bool is_initialized;
    bool enabled;                                                               
                                                                                
    uint32_t input_freq, output_freq, max_output_freq;                          
                                                                                
    uint16_t multiplier, divisor;                                               
                                                                                
    uint8_t user_count;                                                        
    qemu_irq user[CLKTREE_MAX_IRQ]; /* Who to notify on change */               
                                                                                
    uint8_t output_count;                                                      
    struct Clk *output[CLKTREE_MAX_OUTPUT];                                     
                                                                                
    uint8_t input_count;                                                       
    int16_t selected_input;                                                         
    struct Clk *input[CLKTREE_MAX_INPUT];                                       

};                                                                              

typedef struct Clk Clk_t;

#endif // STM32_CLK_H
