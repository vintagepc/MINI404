#include "stm32_rcc.h"
/* PUBLIC FUNCTIONS */

void stm32_rcc_check_periph_clk(Stm32Rcc *s, stm32_periph_t periph)
{
    Clk_p clk = &s->PERIPHCLK[periph];

    assert(clk->is_initialized);

    if(!clktree_is_enabled(clk)) {
        /* I assume writing to a peripheral register while the peripheral clock
         * is disabled is a bug and give a warning to unsuspecting programmers.
         * When I made this mistake on real hardware the write had no effect.
         */
        printf("Warning: You are attempting to use the stm32_rcc peripheral while "
                 "its clock is disabled.\n");
    }
}

void stm32_rcc_set_periph_clk_irq(
        Stm32Rcc *s,
        stm32_periph_t periph,
        qemu_irq periph_irq)
{
    Clk_p clk = &s->PERIPHCLK[periph];

    assert(clk->is_initialized);

    clktree_adduser(clk, periph_irq);
}

uint32_t stm32_rcc_get_periph_freq(
        Stm32Rcc *s,
        stm32_periph_t periph)
{
    Clk_p clk = &s->PERIPHCLK[periph];

    assert(clk->is_initialized);

    return clktree_get_output_freq(clk);
}
