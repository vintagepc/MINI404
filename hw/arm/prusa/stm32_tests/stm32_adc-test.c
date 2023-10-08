/*
 * QTest testcase for the STM32 Common ADC module.
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

#include "../../hw/arm/prusa/stm32_chips/stm32g070xx.h"
#include "../../hw/arm/prusa/stm32_common/stm32_adc_regdata.h"

static void test_resolution(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_ADC1].base_addr;

	uint16_t expected_vals[] = {2047, 511, 127, 31};
	// Enable the ADC clock
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(20) );

	// Expected sampling times (ns) based on resolution change.
	// Determined by G070 datasheet at 16KHz ADC clock.
	const uint16_t expected_rates[] = {875, 750, 625, 500};

	for (int i=0; i<4; i++)
	{
		// Ensure disabled.
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CR))&1, ==, 0);
		qtest_set_irq_in(global_qtest, "/machine/soc/ADC1", "adc_data_in", 0, expected_vals[0]);
		writel(STM32_RI_ADDRESS(base, RI_CHSELR), 0x1 );
		writel(STM32_RI_ADDRESS(base, RI_CFGR1), i << 3U );
	 	//Enable ADC:
		writel(STM32_RI_ADDRESS(base, RI_CR), 0x1 );
	 	// Start
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CHSELR)), !=, 0);
		writel(STM32_RI_ADDRESS(base, RI_CR), BIT(2));
		int64_t start =	qtest_clock_step(global_qtest, 0), end = 0;
		// Wait for EOC

		while ( (readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
		{
			//Also check we don't get a reading before the adc is done:
			g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 0);
			end = clock_step_next();
		}
		g_assert_cmpint(end - start, ==, expected_rates[i]);
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, expected_vals[i]);
		// Ensure the bit was cleared by the DR read.
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2), ==, 0);


		// Disable it for next round. ADDIS is not implemented right now, use RCC reset.
		writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x30, BIT(20) );
	}
}

static void test_align(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_ADC1].base_addr;

	// Per G070 datasheet sec. 14.5.1 on alignment within a given res.
	uint16_t expected_vals[] = {2047 << 4U, 511 << 6U, 127<< 8U, 31<< 2U};
	// Enable the ADC clock
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(20) );

	for (int i=0; i<4; i++)
	{
		// Ensure disabled.
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CR))&1, ==, 0);
		qtest_set_irq_in(global_qtest, "/machine/soc/ADC1", "adc_data_in", 0, 2047);
		writel(STM32_RI_ADDRESS(base, RI_CHSELR), 0x1 );
		writel(STM32_RI_ADDRESS(base, RI_CFGR1), i << 3U | BIT(5) );
	 	//Enable ADC:
		writel(STM32_RI_ADDRESS(base, RI_CR), 0x1 );
	 	// Start
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CHSELR)), !=, 0);
		writel(STM32_RI_ADDRESS(base, RI_CR), BIT(2));
		// Wait for EOC

		while ( (readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
		{
			//Also check we don't get a reading before the adc is done:
			clock_step_next();
		}
		g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, expected_vals[i]);

		// Disable it for next round. ADDIS is not implemented right now, use RCC reset.
		writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x30, BIT(20) );
	}
}

static void test_irqs(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_ADC1].base_addr;

	// Enable the ADC clock
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(20) );

	// Ensure disabled.
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CR))&1, ==, 0);
	qtest_set_irq_in(global_qtest, "/machine/soc/ADC1", "adc_data_in", 0, 4095);
	writel(STM32_RI_ADDRESS(base, RI_CHSELR), 0x1 );
	//Enable ADC:
	writel(STM32_RI_ADDRESS(base, RI_CR), 0x1 );
	// Start
	writel(STM32_RI_ADDRESS(base, RI_CR), BIT(2));
	// Wait for EOC

	while ( (readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
	{
		g_assert_false(get_irq(0));
		clock_step_next();
	}
	// Ensure still false, EOCIE is disabled.
	g_assert_false(get_irq(0));
	readl(STM32_RI_ADDRESS(base, RI_DR));
	// Ensure the bit was cleared by the DR read.
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2), ==, 0);

	// now set EOCIE:
	writel(STM32_RI_ADDRESS(base, RI_IER), BIT(2) );

	writel(STM32_RI_ADDRESS(base, RI_CR), BIT(2) );
	// Wait for EOC

	while ( (readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
	{
		g_assert_false(get_irq(0));
		clock_step_next();
	}
	g_assert_true(get_irq(0));

	// Disable it for next round. ADDIS is not implemented right now, use RCC reset.
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x30, BIT(20) );
}

static void test_dmar(void)
{

	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/ADC1", "dmar");
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_ADC1].base_addr;

	// Enable the ADC clock
	qtest_writel(ts, stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(20) );

	// Ensure disabled.
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CR))&1, ==, 0);
	qtest_set_irq_in(ts, "/machine/soc/ADC1", "adc_data_in", 0, 4095);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHSELR), 0x1 );
	//Enable ADC:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR), 0x1 );
	// Start
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CR), BIT(2));
	// Wait for EOC

	while ( (qtest_readl(ts, STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
	{
		g_assert_false(qtest_get_irq(ts, DMAR_P2M));
		g_assert_false(qtest_get_irq(ts, DMAR_M2P));
		qtest_clock_step_next(ts);
	}
	// Ensure still false, DMAR is disabled.
	g_assert_false(qtest_get_irq(ts, DMAR_P2M));
	g_assert_false(qtest_get_irq(ts, DMAR_M2P));
	qtest_readl(ts,STM32_RI_ADDRESS(base, RI_DR));
	// Ensure the bit was cleared by the DR read.
	g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2), ==, 0);

	// now set DMA:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CFGR1), BIT(0) | BIT(13) );

	qtest_writel(ts,STM32_RI_ADDRESS(base, RI_CR), BIT(2) );
	// Wait for dma

	while ( (qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
	{
		g_assert_false(qtest_get_irq(ts, DMAR_P2M));
		g_assert_false(qtest_get_irq(ts, DMAR_M2P));
		qtest_clock_step_next(ts);
	}
	g_assert_cmphex(qtest_get_irq_level(ts, DMAR_P2M), ==, STM32_RI_ADDRESS(base, RI_DR) );
	g_assert_false(qtest_get_irq(ts, DMAR_M2P));
	qtest_readl(ts,STM32_RI_ADDRESS(base, RI_DR));

	// Check that CONT mode fires again.
	while ( (qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
	{
		qtest_clock_step_next(ts);
	}
	g_assert_cmphex(qtest_get_irq_level(ts, DMAR_P2M), ==, STM32_RI_ADDRESS(base, RI_DR) );
	g_assert_false(qtest_get_irq(ts, DMAR_M2P));

	// Disable it for next round. ADDIS is not implemented right now, use RCC reset.
	qtest_writel(ts,stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x30, BIT(20) );
	qtest_quit(ts);
}

static void test_samplerates(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_ADC1].base_addr;

	// Enable the ADC clock
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(20) );

	// Ensure disabled.
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CR))&1, ==, 0);
	qtest_set_irq_in(global_qtest, "/machine/soc/ADC1", "adc_data_in", 0, 4095);
	writel(STM32_RI_ADDRESS(base, RI_CHSELR), 0x1 );
	//Enable ADC:
	writel(STM32_RI_ADDRESS(base, RI_CR), 0x1 );
	int64_t t = qtest_clock_step(global_qtest, 0), t2 =0;

	// Expected G070 sample rates in nsec.
	// Calculated by converting 12.5+(smpr) clock cycles at 16 KHz to ns.
	const uint16_t expected_smprs[] = { 875, 1000, 1250, 1562, 2000, 3250, 5750, 10812 };

	for (int i=0; i<8; i++)
	{
		// Set SMPR:
		writel(STM32_RI_ADDRESS(base, RI_SMPR), i);
		// Start
		writel(STM32_RI_ADDRESS(base, RI_CR), BIT(2));
		// Wait for EOC
		while ( (readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
		{
			t2 = clock_step_next();
		}
		g_assert_cmpint(t2 - t, ==, expected_smprs[i]);
		t = t2;
		readl(STM32_RI_ADDRESS(base, RI_DR));
	}
	// Ensure still false, EOCIE is disabled.

	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2), ==, 0);

	// Disable it for next round. ADDIS is not implemented right now, use RCC reset.
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x30, BIT(20) );
}

static void test_channels(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_ADC1].base_addr;

	// Enable the ADC clock
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(20) );

	// Ensure disabled.
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CR))&1, ==, 0);

	//Enable ADC:
	writel(STM32_RI_ADDRESS(base, RI_CR), 0x1 );

	for (int i=0; i<19; i++)
	{
		qtest_set_irq_in(global_qtest, "/machine/soc/ADC1", "adc_data_in", i, 16*i);
		writel(STM32_RI_ADDRESS(base, RI_CHSELR), BIT(i) );
		// Start
		writel(STM32_RI_ADDRESS(base, RI_CR), BIT(2));
		// Wait for EOC
		while ( (readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
		{
			clock_step_next();
		}
		g_assert_cmpint(readl(STM32_RI_ADDRESS(base, RI_DR)), ==, 16*i);
	}

	// Disable it for next round. ADDIS is not implemented right now, use RCC reset.
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x30, BIT(20) );
}

static void test_prescale(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_ADC1].base_addr;

	// Enable the ADC clock
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x40, BIT(20) );

	// Ensure disabled.
	g_assert_cmphex(readl(STM32_RI_ADDRESS(base, RI_CR))&1, ==, 0);
	qtest_set_irq_in(global_qtest, "/machine/soc/ADC1", "adc_data_in", 0, 4095);
	writel(STM32_RI_ADDRESS(base, RI_CHSELR), 0x1 );
	//Enable ADC:
	writel(STM32_RI_ADDRESS(base, RI_CR), 0x1 );

	static uint16_t divisors[] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
	const uint16_t expected_smpr =  875; // Default  value for 12 bit at fastest smpr.

	for (int i=0; i<ARRAY_SIZE(divisors); i++)
	{
		// Set prescale:
		writel(stm32g070xx_cfg.perhipherals[STM32_P_ADCC].base_addr, i << 18U);
		g_assert_cmphex(readl(stm32g070xx_cfg.perhipherals[STM32_P_ADCC].base_addr), ==, i << 18);

		// Start
		writel(STM32_RI_ADDRESS(base, RI_CR), BIT(2));
		int64_t start = qtest_clock_step(global_qtest, 0), end =0;

		// Wait for EOC
		while ( (readl(STM32_RI_ADDRESS(base, RI_ISR)) & BIT(2)) != BIT(2) )
		{
			end = clock_step_next();
		}
		g_assert_cmpint(end - start, ==, expected_smpr*divisors[i]);
		readl(STM32_RI_ADDRESS(base, RI_DR));
	}

	// Disable it for next round. ADDIS is not implemented right now, use RCC reset.
	writel(stm32g070xx_cfg.perhipherals[STM32_P_RCC].base_addr + 0x30, BIT(20) );
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("/stm32_adc/resolution", test_resolution);
    qtest_add_func("/stm32_adc/align", test_align);
    qtest_add_func("/stm32_adc/irqs", test_irqs);
    qtest_add_func("/stm32_adc/dmar", test_dmar);
    qtest_add_func("/stm32_adc/samplerates", test_samplerates);
    qtest_add_func("/stm32_adc/channels", test_channels);
    qtest_add_func("/stm32_adc/adcc_prescale", test_prescale);

    qtest_start("-machine stm32g070xB");
	qtest_irq_intercept_out_named(global_qtest, "/machine/soc/ADC1", "sysbus-irq");

    ret = g_test_run();
    qtest_end();

    return ret;
}
