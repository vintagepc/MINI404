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

	extern void gl_dashboard_update_motor_stall(void* pDashboard, int motor, int level) {
		GLDashboardMgr* p = static_cast<GLDashboardMgr*>(pDashboard);
		if (p) {
			p->UpdateMotorStall(motor,level);
		}
	}

	extern void gl_dashboard_update_motor_enable(void* pDashboard, int motor, int level) {
		GLDashboardMgr* p = static_cast<GLDashboardMgr*>(pDashboard);
		if (p) {
			p->UpdateMotorEnable(motor,level);
		}
	}
	extern void gl_dashboard_update_indicator(void* pDashboard, int indicator, int level) {
		GLDashboardMgr* p = static_cast<GLDashboardMgr*>(pDashboard);
		if (p) {
			p->UpdateIndicator(indicator,level);
		}
	}

	void GLAPIENTRY
	GLErrorCB( GLenum, //source,
				GLenum type,
				GLuint id,
				GLenum severity,
				GLsizei, // length
				const GLchar* message,
				const void*) // userparam )
	{
		std::cerr <<  "GL:";
		if (type == GL_DEBUG_TYPE_ERROR)
		{
			std::cerr << "** GL ERROR **";
		}
		std::cerr << "ID:" << id << " type = " << type << " sev = " << severity << " message = " << message << '\n';
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
	m_bQuit = true;
	glutLeaveMainLoop();
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
		m_motors.at(motor)->SetCurrentPos(pos);
		if (m_p3DVis) {
			m_p3DVis->OnMotorStep(motor,pos);
			m_p3DVis->OnPosChanged(motor, m_motors.at(motor)->GetCurrentPos());
		}
	} else {
		std::cerr << "Error: Invalid motor index: "<< std::to_string(motor) <<"!\n";
	}
}

void GLDashboardMgr::UpdateMotorEnable(int motor, int pos) {
	if (motor>=0 && motor<DB_MOTOR_COUNT) {
		m_motors.at(motor)->SetEnable(pos>0);
	} else {
		std::cerr << "Error: Invalid motor index: "<< std::to_string(motor) <<"!\n";
	}
}

void GLDashboardMgr::UpdateMotorStall(int motor, int pos) {
	if (motor>=0 && motor<DB_MOTOR_COUNT) {
		m_motors.at(motor)->SetStall(pos>0);
	} else {
		std::cerr << "Error: Invalid motor index: "<< std::to_string(motor) <<"!\n";
	}
}

void GLDashboardMgr::UpdateIndicator(int iInd, int level) {
	bool bIsFan = iInd == DB_IND_EFAN || iInd == DB_IND_PFAN;
	bool bIsDigital = iInd == DB_IND_FSENS || iInd == DB_IND_ZPROBE;
	if (iInd>=0 && iInd<DB_IND_COUNT) {
		m_indicators.at(iInd)->SetValue(gsl::narrow<uint8_t>(bIsDigital ? level*255U : level));

		if (m_p3DVis) {
			m_p3DVis->OnBoolChanged(iInd, bIsFan ? level : level>0);
		}
	} else {
		std::cerr << "Error: Invalid indicator index!\n";
	}
}

void GLDashboardMgr::Draw() {	
	if (m_bQuit) {
		return;
	}
	glClear((GL_COLOR_BUFFER_BIT) | (GL_DEPTH_BUFFER_BIT));
	glLoadIdentity();
	glScalef(500.f/350, -4, 1);
	for (auto motor : m_motors) {
		motor->Draw();
		glTranslatef(0,10,0);
	}
	for (auto ind : m_indicators) {
		ind->Draw();
		glTranslatef(20,0,0);
	}
	glutSwapBuffers();
	m_iFrCount++;
	m_iTic=glutGet(GLUT_ELAPSED_TIME);
	auto iDiff = m_iTic - m_iLast;
	if (iDiff > 1000) {
		int iFPS = m_iFrCount*1000.f/(iDiff);
		m_iLast = m_iTic;
		m_iFrCount = 0;
		std::string strFPS = "Mini404 GL Dashboard (" +std::to_string(iFPS) + " FPS)";
		glutSetWindowTitle(strFPS.c_str());
	}
}

void GLDashboardMgr::TimerCB(int i) {
	if (m_bQuit) {
		return;
	}
	glutSetWindow(m_iWindow);
	glutTimerFunc(32, m_fcnTimer, i);
	glutPostRedisplay();
	if(m_p3DVis) {
		m_p3DVis->FlagForRedraw();
	}
}

void GLDashboardMgr::SetupHardware() {
	m_EFan.SetColor(0xFF0000);
	m_PFan.SetColor(0xFF0000);
	m_ZProbe.SetColor(0xFFFF0000);
	m_FSens.SetColor(0xFF00);
	m_Htr.SetColor(0xFF000000);
	m_Bed.SetColor(0xFF000000);
	switch (m_iType) {
		case DB_MINI_LITE:
		case DB_MINI_FULL:
		case DB_MINI_DB:
		{
			m_X.SetStepsPerMM(100*16);
			m_X.SetMaxPos(100*16*182);
			m_Y.SetStepsPerMM(100*16);
			m_Y.SetMaxPos(100*16*183);
			m_Z.SetStepsPerMM(400*16);
			m_Z.SetMaxPos(400*16*185);
			m_E.SetStepsPerMM(320*16);

		}
		break;
		default: 
			std::cerr << "Invalid dashboard type specified!\n";
	}
}

void* GLDashboardMgr::RunThread(void *p) {
	int argc = 0;
	char **argv = nullptr;

	SetupHardware();

	glutInit(&argc, argv);		/* initialize GLUT system */
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutInitContextVersion(3,0);
	glutSetOption(GLUT_MULTISAMPLE,2);
	glutInitDisplayMode((GLUT_RGB) | (GLUT_DOUBLE) | (GLUT_MULTISAMPLE));
	glutInitWindowSize(m_iWinW, m_iWinH);		/* width=400pixels height=500pixels */
	m_iWindow = glutCreateWindow("Mini404 GL Dashboard");	/* create window */

	glewInit();
	std::cout << "GL_VERSION   : " << glGetString(GL_VERSION) << '\n';
	std::cout << "GL_VENDOR    : " << glGetString(GL_VENDOR) << '\n';
	std::cout << "GL_RENDERER  : " << glGetString(GL_RENDERER) << '\n';
	std::cout << "GLEW_VERSION : " << glewGetString(GLEW_VERSION) << '\n';
#if !defined(__APPLE__)
		glDebugMessageCallback( GLErrorCB, nullptr );
		if (true)
		{
			glDebugMessageControl(GL_DONT_CARE,
						GL_DONT_CARE,
						GL_DEBUG_SEVERITY_NOTIFICATION,
						0, nullptr, GL_FALSE);
		}
		glEnable(GL_DEBUG_OUTPUT);
#endif
			// Set up projection matrix
	auto fcnDraw = []() { g_pGLDashboardMgr->Draw();};
	glutDisplayFunc(fcnDraw);
	auto fcnClose = []() { g_pGLDashboardMgr->OnGlutClose();};
	glutCloseFunc(fcnClose);
	m_fcnTimer = [](int i) { g_pGLDashboardMgr->TimerCB(i);};
	glutTimerFunc(1000, m_fcnTimer, 0);
	auto fcnResize = [](int w, int h) { g_pGLDashboardMgr->ResizeCB(w,h);};
	glutReshapeFunc(fcnResize);
	glEnable(GL_MULTISAMPLE);
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 0.f, 0.f, 1.0f);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
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

void GLDashboardMgr::OnGlutClose() {
	m_bQuit = true;
}


void GLDashboardMgr::ResizeCB(int w, int h) {
	if (m_bQuit) {
		return;
	}
	glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 10);
	glTranslatef(0, h, 0);
	//glScalef(fScale,-fScale,1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
