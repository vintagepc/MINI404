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


#define STM32_COM_DMA_MAX_CHAN 7
#define STM32_COM_DMA_CHAN_REGS 5

#include "../../hw/arm/prusa/stm32_chips/stm32g070xx.h"
#include "../../hw/arm/prusa/stm32_common/stm32_dma_regdata.h"


static void mem_data(QTestState *ts)
{
	for (int i=0; i<255; i++)
	{
		qtest_writeb(ts, stm32g070xx_cfg.sram_base + i , i+1);
	}
}

static void test_inc(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_DMA1].base_addr;
	uint32_t sram = stm32g070xx_cfg.sram_base;
	QTestState *ts = qtest_init("-machine stm32g070xB");

	mem_data(ts);

	// Setup for an M2P transfer of size 10, bytes, no increment.
	// We use sram and "pretend" to have a peripheral at that address...
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR), sram );
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CPAR), sram + 0x100 );
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR), 10);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR), BIT(4) | BIT(0));
	for (int i=0; i<10; i++)
	{
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", DMAR_M2P, sram + 0x100);
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x01);
		// Check NDTR countdown.
		g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR)), ==, 9-i);

	}
	g_assert_cmphex(qtest_readl(ts, sram + 0x101), ==, 0x0);
	// Check CHDIS when done.
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR)), ==, BIT(4));

	// Now setup for MINC:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR), 10);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR), BIT(7) | BIT(4) | BIT(0));
	for (int i=0; i<10; i++)
	{
		g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR)), ==, sram + i);
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", DMAR_M2P, sram + 0x100);
		g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR)), ==, sram + i +1);
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, i+1);
	}
	g_assert_cmphex(qtest_readl(ts, sram + 0x101), ==, 0x0);


	// Now setup for PINC (cmar needs reset...)
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR), sram );
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR), 10);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR), BIT(6) | BIT(4) | BIT(0));
	for (int i=0; i<10; i++)
	{
		g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CPAR)), ==, sram  + 0x100 + i);
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", DMAR_M2P, sram + 0x100 + i);
		g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR)), ==, sram );
		g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CPAR)), ==, sram  + 0x100 + i + 1);
		g_assert_cmphex(qtest_readb(ts, sram + 0x100 + i), ==, 1);
	}
	g_assert_cmphex(qtest_readl(ts, sram + 0x10B), ==, 0x0);

	qtest_quit(ts);
}

static void do_transfer4(QTestState* ts, uint8_t msize, uint8_t psize, bool dir)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_DMA1].base_addr;
	uint32_t sram = stm32g070xx_cfg.sram_base;
	uint32_t cpar = sram + (dir ? 0x100 : 0x000);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR), sram + (dir ? 0x000 : 0x100));
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CPAR), cpar );
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR), 4);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR), msize << 10 | psize << 8 | BIT(7) | BIT(6) | (dir ? BIT(4) : 0) | BIT(0));
	for (int i=0; i<4; i++)
	{
		printf("# xfer m %x p %x dir %d\n",
			qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR)),
			qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CPAR)),
			dir
		);
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", dir, cpar + (i*(1U << (psize))));
	}
}

static void test_sizes(bool dir)
{
	uint32_t sram = stm32g070xx_cfg.sram_base;
	QTestState *ts = qtest_init("-machine stm32g070xB");

	mem_data(ts);

	// These are literally the test cases/examples from the RM0454 datasheet for DIR=1.
	// For DIR = 0 the expected results are the same as the transfer with m/psize swapped because the direction (and sizes) are swapped.
	do_transfer4(ts, 0,0, dir);
	g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x04030201);

	do_transfer4(ts, 0,1, dir);
	if (dir)
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x00020001);
		g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x00040003);
	}
	else
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x07050301);
	}

	do_transfer4(ts, 0,2, dir);
	if (dir)
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x01);
		g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x02);
		g_assert_cmphex(qtest_readl(ts, sram + 0x108), ==, 0x03);
		g_assert_cmphex(qtest_readl(ts, sram + 0x10C), ==, 0x04);
	}
	else
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x0D090501);
	}

	do_transfer4(ts, 1,0, dir);
	if (!dir) // inverse of 0,1
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x00020001);
		g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x00040003);
	}
	else
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x07050301);
	}

	do_transfer4(ts, 1,1, dir);
	g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x04030201);
	g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x08070605);

	do_transfer4(ts, 1,2, dir);
	if (dir)
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x0201);
		g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x0403);
		g_assert_cmphex(qtest_readl(ts, sram + 0x108), ==, 0x0605);
		g_assert_cmphex(qtest_readl(ts, sram + 0x10C), ==, 0x0807);
	}
	else
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x06050201);
		g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x0E0D0A09);
	}

	do_transfer4(ts, 2,0, dir);
	if (!dir) // inverse of 0,2
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x01);
		g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x02);
		g_assert_cmphex(qtest_readl(ts, sram + 0x108), ==, 0x03);
		g_assert_cmphex(qtest_readl(ts, sram + 0x10C), ==, 0x04);
	}
	else
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x0D090501);
	}

	do_transfer4(ts, 2,1, dir);
	if (!dir)
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x0201);
		g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x0403);
		g_assert_cmphex(qtest_readl(ts, sram + 0x108), ==, 0x0605);
		g_assert_cmphex(qtest_readl(ts, sram + 0x10C), ==, 0x0807);
	}
	else
	{
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x06050201);
		g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x0E0D0A09);
	}

	do_transfer4(ts, 2,2, dir);
	g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x04030201);
	g_assert_cmphex(qtest_readl(ts, sram + 0x104), ==, 0x08070605);
	g_assert_cmphex(qtest_readl(ts, sram + 0x108), ==, 0x0C0B0A09);
	g_assert_cmphex(qtest_readl(ts, sram + 0x10C), ==, 0x100F0E0D);

	qtest_quit(ts);
}

static void test_size(void)
{
	test_sizes(true);
}

static void test_dir(void)
{
	test_sizes(false);
}


static void test_irqs(gconstpointer data)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_DMA1].base_addr;
	uint32_t sram = stm32g070xx_cfg.sram_base;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/DMA1", "sysbus-irq");
	mem_data(ts);
	intptr_t chan = (intptr_t)data;

	uint8_t ch_base = RI_CHAN1 + (STM32_COM_DMA_CHAN_REGS*chan);

	// No interrupts:
	qtest_writel(ts, STM32_RI_ADDRESS(base, ch_base+CH_OFF_CMAR), sram );
	qtest_writel(ts, STM32_RI_ADDRESS(base, ch_base+CH_OFF_CPAR), sram + 0x100 );
	qtest_writel(ts, STM32_RI_ADDRESS(base, ch_base+CH_OFF_CNDTR), 10);
	qtest_writel(ts, STM32_RI_ADDRESS(base, ch_base+CH_OFF_CCR), BIT(7) | 0b10001);
	for (int i=0; i<STM32_COM_DMA_MAX_CHAN; i++)
	{
		g_assert_false(qtest_get_irq(ts,i));
	}
	for (int i=0; i<10; i++)
	{
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", DMAR_M2P, sram + 0x100);
		if (i==4)
		{
			g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)), ==, BIT(2 + (4*chan)) | BIT(0 + (4*chan)));
		}
		else if (i==9)
		{
			g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)), ==, BIT(1 + (4*chan)) | BIT(0 + (4*chan)));
		}
		for (int j=0; j<STM32_COM_DMA_MAX_CHAN; j++)
		{
			g_assert_false(qtest_get_irq(ts,j));
		}
		// Could get GCIF here...
		qtest_writel(ts,STM32_RI_ADDRESS(base, RI_IFCR), UINT32_MAX);
		g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)), ==, 0);
	}

	// Setup with flags enabled:
	qtest_writel(ts, STM32_RI_ADDRESS(base, ch_base+CH_OFF_CMAR), sram );
	qtest_writel(ts, STM32_RI_ADDRESS(base, ch_base+CH_OFF_CPAR), sram + 0x100 );
	qtest_writel(ts, STM32_RI_ADDRESS(base, ch_base+CH_OFF_CNDTR), 10);
	qtest_writel(ts, STM32_RI_ADDRESS(base, ch_base+CH_OFF_CCR), BIT(7) | 0b10111);
	for (int i=0; i<STM32_COM_DMA_MAX_CHAN; i++)
	{
		g_assert_false(qtest_get_irq(ts,i));
	}
	for (int i=0; i<10; i++)
	{
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", DMAR_M2P, sram + 0x100);
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, i+1 );
		if (i==4)
		{
			// Did the interrupt fire?
			g_assert_true(qtest_get_irq(ts,chan));
			// is GIF set along with HTIF?
			g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)), ==, BIT(2 + (4*chan)) | BIT(0 + (4*chan)));

			// Is ISR read-only?
			qtest_writel(ts,STM32_RI_ADDRESS(base, RI_ISR), ~(BIT(2 + (4*chan))));
			g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)) & (BIT(2 + (4*chan))), ==, BIT(2 + (4*chan)));

			g_assert_true(qtest_get_irq(ts,chan));

			qtest_writel(ts,STM32_RI_ADDRESS(base, RI_IFCR), (BIT(2 + (4*chan))));
			// Did the HTIF and GIF clear?
			g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)) , ==, 0);
			// Did the IRQ clear?
			g_assert_false(qtest_get_irq(ts,chan));
		}
		else if (i==9)
		{
			// Did the interrupt fire?
			g_assert_true(qtest_get_irq(ts,chan));
			// is GIF set?
			g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)), ==, BIT(1 + (4*chan)) | BIT(0 + (4*chan)));

			// Is ISR read-only?
			qtest_writel(ts,STM32_RI_ADDRESS(base, RI_ISR), ~(BIT(1 + (4*chan))));
			g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)) & (BIT(1 + (4*chan))), ==, BIT(1 + (4*chan)));

			g_assert_true(qtest_get_irq(ts,chan));

			qtest_writel(ts,STM32_RI_ADDRESS(base, RI_IFCR), (BIT(1 + (4*chan))));
			// Did the TCIF and GIF clear?
			g_assert_cmphex(qtest_readl(ts,STM32_RI_ADDRESS(base, RI_ISR)), ==, 0);
			// Did the IRQ clear?
			g_assert_false(qtest_get_irq(ts,chan));
		}
		else
		{
			for (int j=0; j<STM32_COM_DMA_MAX_CHAN; j++)
			{
				g_assert_false(qtest_get_irq(ts,j));
			}
		}

	}
	qtest_quit(ts);
}

static void test_disabled(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_DMA1].base_addr;
	uint32_t sram = stm32g070xx_cfg.sram_base;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/DMA1", "sysbus-irq");
	mem_data(ts);

	// No interrupts:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR), sram );
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CPAR), sram + 0x100 );
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR), 10);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR), 0b1000);

	// Make sure nothing happens when disabled.
	for (int i=0; i<5; i++)
	{
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", DMAR_M2P, sram + 0x100);
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x00);
	}
	// Now enable it but send the wrong IRQ direction
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR), 0b1001);
	for (int i=0; i<5; i++)
	{
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", DMAR_P2M, sram + 0x100);
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, 0x00);
	}

	qtest_quit(ts);
}

static void test_circ(void)
{
	uint32_t base = stm32g070xx_cfg.perhipherals[STM32_P_DMA1].base_addr;
	uint32_t sram = stm32g070xx_cfg.sram_base;
	QTestState *ts = qtest_init("-machine stm32g070xB");
	qtest_irq_intercept_out_named(ts, "/machine/soc/DMA1", "sysbus-irq");
	mem_data(ts);

	// No interrupts:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR), sram );
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CPAR), sram + 0x100 );
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR), 10);
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR), BIT(7) | 0b110001);

	// Check the counters reset when it loops around:
	for (int i=0; i<15; i++)
	{
		qtest_set_irq_in(ts, "/machine/soc/DMA1", "dmar-in", DMAR_M2P, sram + 0x100);
		g_assert_cmphex(qtest_readl(ts, sram + 0x100), ==, (i%10)+1);
		if (i==9)
			g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR)), ==, 10);
		else
			g_assert_cmpint(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CNDTR)), ==, (19-i)%10);
	}

	// Now stop the channel:
	qtest_writel(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CCR), BIT(7) | 0b1000);
	// Check the address reset:
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CMAR)), ==, sram);
	g_assert_cmphex(qtest_readl(ts, STM32_RI_ADDRESS(base, RI_CHAN1+CH_OFF_CPAR)), ==, sram + 0x100);

	qtest_quit(ts);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("/stm32_dma/test_minc_pinc", test_inc);
    qtest_add_func("/stm32_dma/test_msize_psize", test_size);
    qtest_add_func("/stm32_dma/test_dir", test_dir);
    qtest_add_func("/stm32_dma/test_disabled", test_disabled);
    qtest_add_func("/stm32_dma/test_circ", test_circ);
    qtest_add_data_func("/stm32_dma/test_irq_ch1", (void *)(intptr_t)0, test_irqs);
    qtest_add_data_func("/stm32_dma/test_irq_ch2", (void *)(intptr_t)1, test_irqs);
    qtest_add_data_func("/stm32_dma/test_irq_ch3", (void *)(intptr_t)2, test_irqs);
    qtest_add_data_func("/stm32_dma/test_irq_ch4", (void *)(intptr_t)3, test_irqs);
    qtest_add_data_func("/stm32_dma/test_irq_ch5", (void *)(intptr_t)4, test_irqs);
    qtest_add_data_func("/stm32_dma/test_irq_ch6", (void *)(intptr_t)5, test_irqs);
    qtest_add_data_func("/stm32_dma/test_irq_ch7", (void *)(intptr_t)6, test_irqs);

    ret = g_test_run();

    return ret;
}
