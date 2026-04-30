/*
 * MINI404 - Scriptcon helper functions during testing.
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

#ifndef SCRIPTCON_HELPER_H
#define SCRIPTCON_HELPER_H

#include "qemu/osdep.h"
#include "libqtest-single.h"

static void send_scriptcmd(const char* cmd, int fd, QTestState* ts)
{
    // Flush the buffer:
    char response[5] = {0};
    //while(recv(fd, response, sizeof(response), MSG_DONTWAIT) > 0);
    g_assert_true(send(fd, cmd, strlen(cmd), 0) == strlen(cmd));
    int len = 0;
    while((len = recv(fd, response, sizeof(response), MSG_DONTWAIT)) == -1)
    {
        qtest_clock_step(ts, 100);
    };
    g_assert_cmpint(len, ==, 5);
    g_assert_cmpstr(response, ==, "ACK\r\n");
}


#endif