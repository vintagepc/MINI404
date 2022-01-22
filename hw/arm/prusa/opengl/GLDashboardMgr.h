/*
	GLDashboardMgr.cpp - C++ interface layer for separate legacy
	GL window in QEMU.

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

 	This file is part of MINI404.

	MINI404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	MINI404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with MINI404.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifdef __cplusplus

#include <cstdint>          // for uint32_t
#include "../parts/dashboard_types.h"
#include <memory>
#include <pthread.h>
class MK3SGL;

class GLDashboardMgr
{
    public:
        explicit GLDashboardMgr(int iType);

		~GLDashboardMgr();

        void Start();

        void UpdateMotor(int motor, int pos);    

        void UpdateIndicator(int iInd, int level);    

    private:

        void SetupHardware();

        void* RunThread(void *p);

        pthread_t m_glThread;
        pthread_cond_t m_glReady;
        pthread_mutex_t m_glMtx;

        int m_iType = DB_NONE;

        static GLDashboardMgr* g_pGLDashboardMgr;

        std::unique_ptr<MK3SGL> m_p3DVis {nullptr};

};

extern "C" {
#endif

extern void * gl_dashboard_start(int type);

extern void gl_dashboard_reset(void* pDashboard);

extern void gl_dashboard_run(void* pDashboard);

extern void gl_dashboard_update_motor(void* pDashboard, int motor, int pos);

extern void gl_dashboard_update_indicator(void *pDashboard, int indicator, int level);

#ifdef __cplusplus
};
#endif
