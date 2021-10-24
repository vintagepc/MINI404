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
#include "GLMotor.h"
#include "IPCIndicator.h"
#include <array>
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

        void UpdateMotorStall(int motor, int level);

        void UpdateMotorEnable(int motor, int level);

    private:

        void SetupHardware();

        // Draws the visuals within the current GL transformation context.
        void Draw();

        void SetWindow(int iWin) { m_iWindow = iWin;};

		void ResizeCB(int w, int h);

        void TimerCB(int i);

        void OnGlutClose();

        void* RunThread(void *p);

        int m_iWindow = 0;
        pthread_t m_glThread;
        pthread_cond_t m_glReady;
        pthread_mutex_t m_glMtx;

        void (*m_fcnTimer)(int i) = nullptr;

        int m_iWinW = 500, m_iWinH = 200;

        int m_iType = DB_NONE;

        int m_iTic = 0, m_iLast = 0, m_iFrCount = 0;

        static GLDashboardMgr* g_pGLDashboardMgr;

        GLMotor m_X{'X'}, m_Y{'Y'}, m_Z{'Z'}, m_E{'E',true};

        std::array<GLMotor*,4> m_motors = {&m_X, &m_Y, &m_Z, &m_E};

        IPCIndicator m_EFan {'E'}, m_PFan{'P'}, m_FSens {'F'}, m_ZProbe{'Z'}, m_Bed{'B'}, m_Htr{'H'};

        std::array<IPCIndicator*, 6> m_indicators = {&m_PFan, &m_EFan, &m_FSens, &m_ZProbe, &m_Bed, &m_Htr};

        bool m_bQuit = false;

        std::unique_ptr<MK3SGL> m_p3DVis {nullptr};

};

extern "C" {
#endif

extern void * gl_dashboard_start(int type);

extern void gl_dashboard_reset(void* pDashboard);

extern void gl_dashboard_run(void* pDashboard);

extern void gl_dashboard_update_motor(void* pDashboard, int motor, int pos);
extern void gl_dashboard_update_motor_enable(void* pDashboard, int motor, int pos);
extern void gl_dashboard_update_motor_stall(void* pDashboard, int motor, int pos);

extern void gl_dashboard_update_indicator(void *pDashboard, int indicator, int level);

#ifdef __cplusplus
};
#endif
