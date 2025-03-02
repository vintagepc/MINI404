/*
 * QTest testcase for the STM32 CRC module.
 *
 * Copyright 2023 VintagePC <https://github.com/vintagepc>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#include "../stm32_chips/stm32g070xx.h"
#include "../stm32_common/stm32_usart_regdata.h"

// default delay at 9600 for one byte.
#define DEFAULT_DELAY 1041250U

static void test_tx_disabled(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_UART1].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/UART1", "byte-out");

	g_assert_cmphex(qtest_get_irq_level(ts, 0),==, 0);

	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xFF);

	g_assert_cmphex(qtest_get_irq_level(ts, 0),==, 0);

	// set UE, but not TE:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0));
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xFF);
	g_assert_cmphex(qtest_get_irq_level(ts, 0),==, 0);

	// set U TE:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(3));
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xFF);
	g_assert_cmphex(qtest_get_irq_level(ts, 0),==, 0xFF);
	qtest_quit(ts);
}

static void test_tx_irqs(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_UART1].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/UART1", "sysbus-irq");

	g_assert_false(qtest_get_irq_level(ts, 0));

	// Enable and send byte without any interrupts enabled.
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(3));
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xFF);
	g_assert_false(qtest_get_irq_level(ts, 0));

	// Now enable TC and check we get an IRQ. Note we have no baud rate set so the transmit is instant.
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(3) | BIT(6));
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xFF);
	g_assert_true(qtest_get_irq_level(ts, 0));
	// Clear the flag:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ICR), BIT(6));
	g_assert_false(qtest_get_irq_level(ts, 0));

	// Check TXE:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(3) | BIT(7));
	g_assert_true(qtest_get_irq_level(ts, 0));
	// Clear it - shouldn't work since the buffer is still empty...
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ICR), BIT(7));
	g_assert_true(qtest_get_irq_level(ts, 0));
	// Disable:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(3));
	g_assert_false(qtest_get_irq_level(ts, 0));

	qtest_quit(ts);
}

static void test_rx_disabled(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_UART1].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	// qtest_irq_intercept_out_named(ts, "/machine/soc/UART1", "byte-out");

	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xDE);

	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RDR)),==, 0);

	// set UE, but not RE:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0));

	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xAD);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RDR)),==, 0);


	// set U RE:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(2));

	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xBE);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RDR)),==, 0xBE);

	qtest_quit(ts);
}

static void test_dmar(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_UART1].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/UART1", "dmar");
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(2) | BIT(3) );

	// Input a byte without DMA.
	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xAD);
	g_assert_false(qtest_get_irq_level(ts, DMAR_P2M));
	g_assert_false(qtest_get_irq_level(ts, DMAR_M2P));
	// Remove byte.
	qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RDR));

	// Enable DMAR:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR3), BIT(6));
	g_assert_false(qtest_get_irq_level(ts, DMAR_P2M));
	g_assert_false(qtest_get_irq_level(ts, DMAR_M2P));

	// Input a byte and check the DMA request fired.
	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xAD);

	g_assert_cmphex(qtest_get_irq_level(ts, DMAR_P2M), ==, STM32_RI_ADDRESS(base, RI_RDR));
	g_assert_true(qtest_get_irq_pulsed(ts, DMAR_P2M));
	// Unfortunately due to how DMAR is handled and Qtest not re-raising the same level,
	// it's not really possible to check whether DMA is re-raised correctly at this time.
	// qtest_irq_clear_pulses(ts);
	// g_assert_false(qtest_get_irq_pulsed(ts, DMAR_P2M));
	// g_assert_false(qtest_get_irq_level(ts, DMAR_M2P));

	// read and Input another byte and make sure it re-raised:
	qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RDR));
	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xF0);

	g_assert_cmphex(qtest_get_irq_level(ts, DMAR_P2M), ==, STM32_RI_ADDRESS(base, RI_RDR));
	// g_assert_true(qtest_get_irq_pulsed(ts, DMAR_P2M));
	qtest_irq_clear_pulses(ts);

	// Now enable DMAT:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR3), BIT(7));
	g_assert_cmphex(qtest_get_irq_level(ts, DMAR_M2P), ==, STM32_RI_ADDRESS(base, RI_TDR));
	g_assert_true(qtest_get_irq_pulsed(ts, DMAR_M2P));
	// qtest_irq_clear_pulses(ts);
	// g_assert_false(qtest_get_irq_pulsed(ts, DMAR_M2P));

	// Write TDR and make sure it fires the IRQ again:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xDE);
	g_assert_cmphex(qtest_get_irq_level(ts, DMAR_M2P), ==, STM32_RI_ADDRESS(base, RI_TDR));
	// g_assert_true(qtest_get_irq_pulsed(ts, DMAR_M2P));
	// qtest_irq_clear_pulses(ts);
	// g_assert_false(qtest_get_irq_pulsed(ts, DMAR_M2P));

	qtest_quit(ts);
}

static void test_idle_rto(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_UART1].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	// qtest_irq_intercept_out_named(ts, "/machine/soc/UART1", "byte-out");

	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xDE);

	// Enable UART clock:
	qtest_writel(ts, stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(14));

	// Set baud rate to 9600
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_BRR), 0x682);

	// set U RE:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(2));
	// Enable RTO
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR2), BIT(23));

	// Set RTO to 2:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RTOR), 2);


	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xBE);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RDR)),==, 0xBE);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & (BIT(4) | BIT(11)) ,==, 0);

	// Advance the clock:
	int64_t ns = qtest_clock_step_next(ts);
	printf("# delay: %ld ns\n", ns);

	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(4),==, BIT(4));
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(11),==, 0);

	int64_t ns2 = qtest_clock_step_next(ts);
	printf("# delay: %ld ns\n", ns);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(11),==, BIT(11));

	g_assert_cmpint(ns, ==, ns2/2);

	// Clear flags:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_ICR), BIT(11) | BIT(4));


	// Now try again with RTO5:
	// Set RTO to 2:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_RTOR), 5);


	qtest_set_irq_in(ts, "/machine/soc/UART1", "byte-in", 0, 0xBE);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_RDR)),==, 0xBE);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & (BIT(4) | BIT(11)) ,==, 0);
	ns = qtest_clock_step(ts, 0);

	ns2 = qtest_clock_step_next(ts); // IDLE
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(4),==, BIT(4));
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(11),==, 0);

	ns2 = qtest_clock_step_next(ts); // RTOF
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(11),==, BIT(11));

	g_assert_cmpint(ns2 - ns, ==, 5*DEFAULT_DELAY);

	qtest_quit(ts);
}

static void test_baud_rate(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_UART1].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/UART1", "byte-out");

	// Enable UART clock:
	qtest_writel(ts, stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(14));

	// Set baud rate to 9600
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_BRR), 0x682);

	// Enable and send byte
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(3));
	// Check TXE:
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(7), ==, BIT(7));
	// Send byte:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xFF);
	// Check TC and TXE:
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(6), ==, 0);
	// TXE isn't cleared because there's a double buffer that can "store" two bytes.
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(7), ==, BIT(7));

	// Advance the clock:
	int64_t ns = qtest_clock_step_next(ts);
	printf("# delay: %ld ns\n", ns);

	// Check TC and TXE:
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(6), ==, BIT(6));
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(7), ==, BIT(7));

	// Send two bytes in succession:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xEE);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xDD);
	// Check TC and TXE:
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(6), ==, 0);
	// TXE isn't cleared because there's a double buffer that can "store" two bytes.
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(7), ==, 0);
	g_assert_cmphex(qtest_get_irq_level(ts, 0),==, 0xEE);
	// Advance the clock:
	ns = qtest_clock_step_next(ts) - ns;
	printf("# delay: %ld ns\n", ns);
	g_assert_cmphex(qtest_get_irq_level(ts, 0),==, 0xDD);
	// Check TC and TXE:
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(6), ==, 0);
	// TXE should now be empty since we've just moved byte 2 from DR to the output register.
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(7), ==, BIT(7));

	ns = qtest_clock_step_next(ts) - ns;
	// Check TC and TXE:
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(6), ==, BIT(6));
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(7), ==, BIT(7));

	qtest_quit(ts);
}

static void test_prescale(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_UART1].base_addr;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/UART1", "byte-out");

	// Enable UART clock:
	qtest_writel(ts, stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(14));

	// Set baud rate to 9600
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_BRR), 0x682);

	// Enable TX
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR1), BIT(0) | BIT(3));

	const uint16_t PRESCALE_DIV[] = { // Note anything 11-15 should be 256...
		1U, 2U, 4U, 6U, 8U, 10U, 12U, 16U, 32U, 64U, 128U, 256U, 256U, 256U, 256U, 256U
	};

	for (int i=0; i<ARRAY_SIZE(PRESCALE_DIV); i++)
	{
		// Technically we should disable UE before changing this...
		qtest_writel(ts, STM32_RI_ADDRESS(base, RI_PRESC), i);
        int64_t current_ns = qtest_clock_step(ts, 0);
		qtest_writel(ts, STM32_RI_ADDRESS(base, RI_TDR), 0xFF);
		g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(6), ==, 0);
		// Advance the clock:
        int64_t ns = qtest_clock_step_next(ts);
		// Check TC:
		g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(6), ==, BIT(6));
		g_assert_cmpint(ns - current_ns, ==, DEFAULT_DELAY*PRESCALE_DIV[i]);
		// printf("# delay: %ld ns\n", ns);
	}

	qtest_quit(ts);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("/stm32_uart/test_tx_disabled", test_tx_disabled);
    qtest_add_func("/stm32_uart/test_rx_disabled", test_rx_disabled);
    qtest_add_func("/stm32_uart/test_dma", test_dmar);
    qtest_add_func("/stm32_uart/test_idle_and_rto", test_idle_rto);
    qtest_add_func("/stm32_uart/test_tx_irqs", test_tx_irqs);
    qtest_add_func("/stm32_uart/test_baud_rate", test_baud_rate);
   qtest_add_func("/stm32_uart/test_prescale", test_prescale);

    ret = g_test_run();

    return ret;
}
