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

#include "GLDashboardMgr.h"
#include "../3rdParty/gsl-lite.hpp"
#include "MK3SGL.h"
#include <GL/glew.h>         // NOLINT for glTranslatef
#include <GL/freeglut_std.h> //
#include <GL/freeglut_ext.h>
#include <atomic>            // for atomic, atomic_bool, atomic_int
#include <iostream>
#include <utility>

extern "C" {
	extern void * gl_dashboard_start(int type) {
		void * pDashboard = nullptr;
		if (type > 0) {
			std::cout << "Creating new GLDashboard\n";
			pDashboard = new GLDashboardMgr(type);
		}
		return pDashboard;
	}

	extern void gl_dashboard_reset(void* pDashboard) {

	}

	extern void gl_dashboard_run(void* pDashboard) {
		GLDashboardMgr* p = static_cast<GLDashboardMgr*>(pDashboard);
		if (p) {
			p->Start();
		}
	}

	extern void gl_dashboard_update_motor(void* pDashboard, int motor, int pos) {
		GLDashboardMgr* p = static_cast<GLDashboardMgr*>(pDashboard);
		if (p) {
			p->UpdateMotor(motor,pos);
		}
	}

	extern void gl_dashboard_update_indicator(void* pDashboard, int indicator, int level) {
		GLDashboardMgr* p = static_cast<GLDashboardMgr*>(pDashboard);
		if (p) {
			p->UpdateIndicator(indicator,level);
		}
	}
};

GLDashboardMgr* GLDashboardMgr::g_pGLDashboardMgr = nullptr;


GLDashboardMgr::GLDashboardMgr(int iType):m_iType(iType) {
	if (g_pGLDashboardMgr) {
		std::cerr << "Error: Cannot instantiate more than one GLDashboardManager instance\n";
		return;
	}
	g_pGLDashboardMgr = this;
}

GLDashboardMgr::~GLDashboardMgr() {
}   

void GLDashboardMgr::Start() {
	auto fcnRun = [](void *p) { return g_pGLDashboardMgr->RunThread(p); };
	pthread_cond_init(&m_glReady, nullptr);
	pthread_mutex_init(&m_glMtx, nullptr);
	pthread_create(&m_glThread, nullptr, fcnRun, nullptr);
	// Wait for the GL thread to start so that the script host can be set up. 
	pthread_mutex_lock(&m_glMtx);
	pthread_cond_wait(&m_glReady, &m_glMtx);
	pthread_mutex_unlock(&m_glMtx);
}

void GLDashboardMgr::UpdateMotor(int motor, int pos) {
	if (motor>=0 && motor<DB_MOTOR_COUNT) {
		if (m_p3DVis) {
			m_p3DVis->OnMotorStep(motor,pos);
		}
	} else {
		std::cerr << "Error: Invalid motor index: "<< std::to_string(motor) <<"!\n";
	}
}

void GLDashboardMgr::UpdateIndicator(int iInd, int level) {
	bool bIsFan = iInd == DB_IND_EFAN || iInd == DB_IND_PFAN;
	// bool bIsDigital = iInd == DB_IND_FSENS || iInd == DB_IND_ZPROBE;
	if (iInd>=0 && iInd<DB_IND_COUNT) {
		if (m_p3DVis) {
			m_p3DVis->OnBoolChanged(iInd, bIsFan ? level : level>0);
		}
	} else {
		std::cerr << "Error: Invalid indicator index!\n";
	}
}

void GLDashboardMgr::SetupHardware() {
	switch (m_iType) {
		case DB_MINI_LITE:
		case DB_MINI_FULL:
		{
		}
		break;
		default: 
			std::cerr << "Invalid dashboard type specified!\n";
	}
}

void* GLDashboardMgr::RunThread(void *p) {
	SetupHardware();
	// Also set up the 3D visuals. 
	switch (m_iType) {
		case DB_MINI_FULL:
			m_p3DVis.reset(new MK3SGL("full",false));
			m_p3DVis->SetStepsPerMM(100*16, 100*16, 400*16, 320*16);
			break;
		case DB_MINI_LITE:
			m_p3DVis.reset(new MK3SGL("lite",false));
			m_p3DVis->SetStepsPerMM(100*16, 100*16, 400*16, 320*16);
			break;
		default:
			break;
	}
	pthread_cond_signal(&m_glReady);
	glutMainLoop();
	return p;
}
