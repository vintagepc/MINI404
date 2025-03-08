/*
 * MINI404 - HX717 Test
 *
 * This file is part of the MINI404 project, an open-source 3D printer simulator.
 * Copyright 2024 VintagePC <https://github.com/vintagepc>
 *
 * MINI404 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * MINI404 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MINI404. If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define QOM_PATH "/machine/peripheral/at21csxx"

#define MICROS(x) (x * 1000)

#define LOW1_NS 1500
#define LOW0_NS 10000
#define BIT_TIME 25000

#define READ_NS 1000

#define WRITE_CMD 0xA0
#define READ_CMD 0xA1

static bool do_setup(QTestState *ts)
{
    qtest_irq_intercept_out(ts, QOM_PATH);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 0);
    qtest_clock_step(ts, MICROS(150));
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    g_assert_cmpint(qtest_get_irq_level(ts,0), ==, 1);
    // Check device ACK:
    qtest_clock_step(ts, MICROS(100));
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 0);
    qtest_clock_step(ts, MICROS(1) + 1);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_clock_step(ts, MICROS(3));
    bool ack = !qtest_get_irq_level(ts,0);
    qtest_clock_step(ts, MICROS(150));
    g_assert_cmpint(ack, ==, 1);
    return ack;
}

static bool read_bit(QTestState *ts)
{
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 0);
    qtest_clock_step(ts, READ_NS);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_clock_step(ts, 800);
    bool bit = qtest_get_irq_level(ts,0);
    qtest_clock_step(ts, BIT_TIME - READ_NS - 800);
    return bit;
}

static void send_bit(QTestState *ts, bool bit)
{
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 0);
    qtest_clock_step(ts, bit ? LOW1_NS : LOW0_NS);
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_clock_step(ts, BIT_TIME - (bit ? LOW1_NS : LOW0_NS));
}

static void send_start(QTestState *ts)
{
    qtest_set_irq_in(ts, QOM_PATH, NULL, 0, 1);
    qtest_clock_step(ts, MICROS(500) + 1);
}

static void send_byte(QTestState *ts, uint8_t byte)
{
    for(uint8_t mask = 0x80; mask >0; mask >>= 1)
    {
        send_bit(ts, byte & mask);
    }
}

// Sends a byte and checks ACK
static bool write_byte(QTestState *ts, uint8_t byte)
{
    send_byte(ts, byte);
    return !read_bit(ts);
}

static uint8_t read_byte(QTestState *ts, bool send_ack)
{
    uint8_t result = 0;
    for (int i=7; i >= 0; i--)
    {
        bool bit = read_bit(ts);
        //printf("# Read bit %d: %d\n", i, bit);
        result |= bit << i;
    }
    send_bit(ts, !send_ack);

    return result;
}

static void test_at21csxx_discovery_test(void)
{
    QTestState *ts = qtest_init("-machine prusa-mk4-027c");

    do_setup(ts);

    qtest_quit(ts);
}

static void test_at21csxx_read_test(void)
{
    QTestState *ts = qtest_init("-machine prusa-mk4-027c");

    do_setup(ts);

    send_start(ts);

    write_byte(ts, READ_CMD);

    g_assert_cmphex(read_byte(ts, true), ==, 0x02); // ver
    g_assert_cmphex(read_byte(ts, true), ==, 0x20); // size LSB
    g_assert_cmphex(read_byte(ts, true), ==,0x00); // size MSB
    g_assert_cmphex(read_byte(ts, false), ==, 0x1F); // bomID


    qtest_quit(ts);
}

static void fill_eeprom(QTestState* ts)
{

    g_assert_true(write_byte(ts, WRITE_CMD));
    g_assert_true(write_byte(ts, 0)); // set address pointer

    // fill EEPROM with data:

    for (int i=0; i<128; i++)
    {
        g_assert_true(write_byte(ts, i+128));
    }
}

static void test_at21csxx_fill_test(void)
{
    QTestState *ts = qtest_init("-machine prusa-mk4-027c");

    do_setup(ts);

    fill_eeprom(ts);

    send_start(ts);

    write_byte(ts, READ_CMD);

    for (int i=0; i<128; i++)
    {
        g_assert_cmphex(read_byte(ts, true), ==, i+128);
    }

    // Now do a random read:
    send_start(ts);
    g_assert_true(write_byte(ts, WRITE_CMD));
    g_assert_true(write_byte(ts, 0x50)); // set address pointer
    send_start(ts);
    write_byte(ts, READ_CMD);
    g_assert_cmphex(read_byte(ts, true), ==, 0x50 + 128);
    qtest_quit(ts);
}

static void test_at21csxx_loopback_test(void)
{
    QTestState *ts = qtest_init("-machine prusa-mk4-027c");

    do_setup(ts);

    fill_eeprom(ts);
    write_byte(ts, 0xFF);

    send_start(ts);

    g_assert_true(write_byte(ts, WRITE_CMD));
    g_assert_true(write_byte(ts, 0x00)); // set address pointer
    send_start(ts);
    write_byte(ts, READ_CMD);
    g_assert_cmphex(read_byte(ts, true), ==, 0xFF);
    qtest_quit(ts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/at21csxx/discovery", test_at21csxx_discovery_test);
    qtest_add_func("/at21csxx/read_byte", test_at21csxx_read_test);
    qtest_add_func("/at21csxx/fill_data", test_at21csxx_fill_test);
    qtest_add_func("/at21csxx/loopback", test_at21csxx_loopback_test);
    return g_test_run();
}
