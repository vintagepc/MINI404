/*
	MK3SGL.cpp - Printer visualization for a MK3S, with MMU and print.

	Copyright 2020 VintagePC <https://github.com/vintagepc/>

 	This file is part of MK404.

	MK404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	MK404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with MK404.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MK3SGL.h"
#include "GLPrint.h"          // for GLPrint
#include "../utility/KeyController.h"
#include "Mini_Lite.h"        // for MK3S_Lite
#include "Mini_Full.h"        // for MK3S_Lite
#include "Macros.h"
#include "OBJCollection.h"    // for OBJCollection, OBJCollection::ObjClass
#include "../3rdParty/gsl-lite.hpp"
#include "../parts/dashboard_types.h"
//NOLINTNEXTLINE _std must come before _ext.
#include <GL/freeglut_std.h>  // for glutSetWindow, GLUT_DOWN, GLUT_UP, glut...
#ifndef TEST_MODE
#include <GL/freeglut_ext.h>  // for glutSetOption
#endif
#include <GL/glew.h>          // for glTranslatef, glPopMatrix, glPushMatrix
#include <cstdlib>           // for exit
#include <cstring>
#include <iostream>             // for size_t, fprintf, printf, stderr
#include <vector>             // for vector

MK3SGL* MK3SGL::g_pMK3SGL = nullptr;

extern "C"
{
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
}

MK3SGL::~MK3SGL()
{
	m_bQuit = true;
	glutLeaveMainLoop();
	g_pMK3SGL = nullptr;
}


MK3SGL::MK3SGL(const std::string &strModel, bool bMMU):Scriptable("3DVisuals"), IKeyClient(),m_bMMU(bMMU)
{
	if (g_pMK3SGL)
	{
		std::cerr << "ERROR: Cannot have multiple MK3SGL instances due to freeglut limitations." << '\n';
		exit(1);
	}
	g_pMK3SGL = this;

	RegisterActionAndMenu("ClearPrint","Clears rendered print objects",ActClear);
	RegisterActionAndMenu("ToggleNozzleCam","Toggles between normal and nozzle cam mode.",ActToggleNCam);
	RegisterActionAndMenu("ResetCamera","Resets camera view to default",ActResetView);
	RegisterAction("MouseBtn", "Simulates a mouse button (# = GL button enum, gl state,x,y)", ActMouse, {ArgType::Int,ArgType::Int,ArgType::Int,ArgType::Int});
	RegisterAction("MouseMove", "Simulates a mouse move (x,y)", ActMouseMove, {ArgType::Int,ArgType::Int});
	RegisterActionAndMenu("NonLinearX", "Toggle motor nonlinearity on X", ActNonLinearX);
	RegisterActionAndMenu("NonLinearY", "Toggle motor nonlinearity on Y", ActNonLinearY);
	RegisterActionAndMenu("NonLinearZ", "Toggle motor nonlinearity on Z", ActNonLinearZ);
	RegisterActionAndMenu("NonLinearE", "Toggle motor nonlinearity on E", ActNonLinearE);
    RegisterActionAndMenu("ExportPLY", "Export high resolution extrusion print to a PLY file (Export.ply)", ActExportPLY);
	RegisterAction("ExportPLYFile", "Export high resolution extrusion print to a PLY file (filename)", ActExportPLYFile, {ArgType::String});

	RegisterKeyHandler('`', "Reset camera view to default");
	RegisterKeyHandler('n',"Toggle Nozzle-Cam Mode");
	RegisterKeyHandler('l',"Clears any print on the bed. May cause graphical glitches if used while printing.");
	RegisterKeyHandler('w',"");
	RegisterKeyHandler('s',"");

	RegisterKeyHandler('5',"");
	RegisterKeyHandler('6',"");
	RegisterKeyHandler('7',"");
	RegisterKeyHandler('8',"");
	RegisterKeyHandler('9',"");
	RegisterKeyHandler('0',"");

	int argc = 0;
	char **argv = nullptr;
	glutInit(&argc, argv);		/* initialize GLUT system */
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutInitContextVersion(3,0);
#ifdef TEST_MODE
	glutInitDisplayMode( US(GLUT_RGB) | US(GLUT_DOUBLE) | US(GLUT_DEPTH)) ;
#else
	glutSetOption(GLUT_MULTISAMPLE,4);
	glutInitDisplayMode( US(GLUT_RGB) | US(GLUT_DOUBLE) | US(GLUT_DEPTH) | US(GLUT_MULTISAMPLE)) ;
#endif
	glutInitWindowSize(800,800);		/* width=400pixels height=500pixels */
	std::string strTitle = std::string("Fancy Graphics");
	m_iWindow = glutCreateWindow(strTitle.c_str());	/* create window */
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

	auto fcnDraw = []() { if (g_pMK3SGL) g_pMK3SGL->Draw();};
	glutDisplayFunc(fcnDraw);

	glutKeyboardFunc(KeyController::GLKeyReceiver); // same func as main window.

	auto fwd = [](int button, int state, int x, int y) { if (g_pMK3SGL)g_pMK3SGL->MouseCB(button,state,x,y);};
	glutMouseFunc(fwd);

	auto fcnMove = [](int x, int y) { if (g_pMK3SGL)g_pMK3SGL->MotionCB(x,y);};
	glutMotionFunc(fcnMove);

	auto fcnResize = [](int x, int y) { if (g_pMK3SGL)g_pMK3SGL->ResizeCB(x,y);};
	glutReshapeFunc(fcnResize);

	m_fcnTimer = [](int i) { if (g_pMK3SGL)g_pMK3SGL->TimerCB(i);};
	glutTimerFunc(1000, m_fcnTimer, 0);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_MULTISAMPLE);

	ResetCamera();
	if (strModel == "lite")
	{
		m_Objs = new Mini_Lite();
	} 
	else if (strModel == "full") 
	{
		m_Objs = new Mini_Full();
	}

	m_Objs->Load();
}

void MK3SGL::ResetCamera()
{
	m_camera = Camera();
	m_camera.setWindowSize(800,800);
	m_camera.setEye(0,0.5,3);
	m_camera.setCenter(0,0,0);
}

void MK3SGL::ResizeCB(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, static_cast<float>(w) / static_cast<float>(h), 0.01f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void MK3SGL::TimerCB(int i) {
	if (m_bQuit) {
		return;
	}
	FlagForRedraw();
	glutTimerFunc(32, m_fcnTimer, i);
}

void MK3SGL::OnKeyPress(const Key& key)
{
	std::cout <<"KEY:" << key << '\n';
	switch (key)
	{
		case 'l':
			ClearPrint();
			break;
		case 'n':
			ToggleNozzleCam();
		break;
		case '`':
			ResetCamera();
			break;
		case 'w':
		case 's':
			TwistKnob(key=='w');
			break;
		case '5':
		case '6':
			m_flDbg = m_flDbg + (key=='5'?0.001f:-0.001f);
			break;
		case '7':
		case '8':
			m_flDbg2 = m_flDbg2 + (key=='7'?0.001f:-0.001f);
			break;
		case '9':
		case '0':
			m_flDbg3 = m_flDbg3 + (key=='9'?0.001f:-0.001f);
			break;

	}
	// Decomment this block and use the flDbg variables
	// as your position translation. Then, you can move the
	// object into place using the numpad, and just read off
	// the correct position values to use when finished.
	// 	m_iDbg ++;
	// 	m_MMUBase.SetSubobjectVisible(m_iDbg,false);
	// }
	// else if (c == '8')
	// {
	// 	m_MMUBase.SetSubobjectVisible(m_iDbg,true);
	// 	m_iDbg --;
	// }
	// printf("Int: %d\n",m_iDbg.load());
	printf("Offsets: %03f, %03f, %03f,\n",m_flDbg.load(),m_flDbg2.load(), m_flDbg3.load());
}

void MK3SGL::Init()
{
	m_bDirty = true;
}


Scriptable::LineStatus MK3SGL::ProcessAction(unsigned int iAct, const std::vector<std::string> &vArgs)
{
	if (m_iQueuedAct>=0) // Don't clobber a queued item...
	{
		return LineStatus::Waiting;
	}
	else if (m_iQueuedAct == -2)
	{
		m_iQueuedAct = -1;
		return LineStatus::Finished;
	}
	switch (iAct)
	{
		case ActMouse:
		case ActMouseMove:
			m_vArgs = vArgs;
			/* FALLTHRU */
		case ActResetView:
			m_iQueuedAct = iAct;
			return LineStatus::Waiting;
		case ActToggleNCam:
			m_bFollowNozzle = !m_bFollowNozzle;
			return LineStatus::Finished;
		case ActClear:
			ClearPrint();
			return LineStatus::Finished;
		case ActNonLinearX: // NOTE: we only do this for T0 for now. Not hard to extend with for(auto &print: m_vPrints)...
			std::cout << "Nonlinear X: " << std::to_string(m_Print.ToggleNLX()) << '\n';
			return LineStatus::Finished;
		case ActNonLinearY:
			std::cout << "Nonlinear Y: " << std::to_string(m_Print.ToggleNLY()) << '\n';
			return LineStatus::Finished;
		case ActNonLinearZ: // NOTE: we only do this for T0 for now. Not hard to extend with for(auto &print: m_vPrints)...
			std::cout << "Nonlinear Z: " << std::to_string(m_Print.ToggleNLZ()) << '\n';
			return LineStatus::Finished;
		case ActNonLinearE:
			std::cout << "Nonlinear E: " << std::to_string(m_Print.ToggleNLE()) << '\n';
			return LineStatus::Finished;
		case ActExportPLYFile:
			m_pExportFN = vArgs.data(); // This *should* be thread safe since the script persists from start to finish, and it's read-only.
			/* FALLTHRU */
        case ActExportPLY:
			if (m_iExportPLYResult > 0 )
			{
				auto bResult = m_iExportPLYResult == 1;
				m_iExportPLYResult = 0;
				return bResult ? LineStatus::Timeout : LineStatus::Finished;
			}
			if (m_iExportPLYResult==0 && !m_bExportPLY)
			{
            	std::cout << "ExportPLY...\n";
            	m_bExportPLY = true;
			}
            return LineStatus::HoldExec;
		default:
			return LineStatus::Unhandled;

	}
}

void MK3SGL::ProcessAction_GL()
{
	switch (m_iQueuedAct.load())
	{
		case ActResetView:
			ResetCamera();
			break;
		case ActMouse:
			MouseCB(std::stoi(m_vArgs.at(0)),std::stoi(m_vArgs.at(1)),std::stoi(m_vArgs.at(2)),std::stoi(m_vArgs.at(3)));
			break;
		case ActMouseMove:
			MotionCB(std::stoi(m_vArgs.at(0)),std::stoi(m_vArgs.at(1)));
			break;
	}
	m_iQueuedAct = -2;
}

void MK3SGL::TwistKnob(bool bDir)
{
	if (bDir)
	{
		m_iKnobPos = (m_iKnobPos+18)%360;
	}
	else
	{
		m_iKnobPos = (m_iKnobPos + 342)%360;
	}
}

void MK3SGL::OnBoolChanged(int source, uint32_t value)
{
	bool bVal = value>0;
	switch(source)
	{
		case DB_IND_BED:
			m_bBedOn = bVal;
			break;
		// case IRQ::SHEET_IN:
		// 	m_bPrintSurface = bVal;
		// 	break;
		case DB_IND_EFAN:
			m_iFanPWM = value;
			break;
		case DB_IND_PFAN:
			m_iPFanPWM = value;
			break;
		case DB_IND_MEDIA:
			m_bSDCard = !bVal; // CS is inverted.
			break;
		case DB_IND_ZPROBE:
			m_bPINDAOn = bVal;
			break;
		case DB_IND_HTR:
			break;
		default:
		std::cout << "NOTE: MK3SGL: Unhandled Bool IRQ " << std::to_string(source) << '\n';
			break;
	}
	m_bDirty = true;
}

void MK3SGL::OnMotorStep(int source, uint32_t value)
{
	switch (source)
	{
		case DB_MOTOR_X:
			m_vPrints[m_iCurTool]->OnXStep(value);
			break;
		case DB_MOTOR_Y:
			m_vPrints[m_iCurTool]->OnYStep(value);
			break;
		case DB_MOTOR_Z:
			m_vPrints[m_iCurTool]->OnZStep(value);
			break;
		case DB_MOTOR_E:
			// Yeah, this is a bit of a kludge; I need to come up with a better way to handle the
			// scenario where the last E tick is very far in the past (Typically the first tick of a print)
			// if ((m_pAVR->cycle - m_lastETick)>UINT32_MAX)
			// {
			// 	m_lastETick = m_pAVR->cycle - 10000;
			// }
			// The narrow will assert if there is an overflow/underflow condition.
			// Time should never be negative...
			m_vPrints[m_iCurTool]->OnEStep(value, 0 /*gsl::narrow<uint32_t>(m_pAVR->cycle - m_lastETick)*/);
			//m_lastETick = m_pAVR->cycle;
			break;
	}
	OnPosChanged(source, static_cast<float>(value)/static_cast<float>(m_uiStepsPerMM[source]));
}

void MK3SGL::OnPosChanged(int source, float value)
{
	switch (source)
	{
		case DB_MOTOR_X:
			m_fXPos = value/1000.f;
			break;
		case DB_MOTOR_Y:
			m_fYPos = value/1000.f;
			break;
		case DB_MOTOR_Z:
			m_fZPos = value/1000.f;
			break;
		case DB_MOTOR_E:
			m_fEPos = value/1000.f;
			break;
		default:
			std::cout << "NOTE: MK3SGL: Unhandled IRQ " << std::to_string(source) << '\n';
			break;
	}
	m_bDirty = true;
}

void MK3SGL::OnToolChanged(int source, uint32_t iIdx)
{
	// Need to stop the old tool and start the new one at the right location.
	m_iCurTool = iIdx;
};

static constexpr float fMM2M = 1.f/1000.f;

void MK3SGL::Draw()
{
		//if (!m_bDirty)
		//    return;
	if (m_bClearPrints) // Needs to be done in the GL loop for thread safety.
	{
		for (int i=0; i<5; i++) m_vPrints[i]->Clear();
		m_bClearPrints = false;
	}

    if( m_bExportPLY ){
        // export buffered GL structures into PLY - must be done in GL loop for thread safety
		if (m_pExportFN !=nullptr)
		{
			bool bResult = m_vPrints[0]->ExportPLY(*m_pExportFN);
			m_pExportFN = nullptr;
			m_iExportPLYResult = (bResult? 2 : 1);
		}
		else
		{
        	bool bResult = m_vPrints[0]->ExportPLY();
			m_iExportPLYResult = (bResult? 2 : 1);
		}
        m_bExportPLY = false;
    }

	if (m_iQueuedAct>=0)
	{
		// This may have issues if draw() is reentrant but I don't think it is as GLUT
		// waits for the draw call to finish before it kicks off another one from PostRedisplay
		ProcessAction_GL();
	}
	// No AVR cycling here, we only need this in the GL context.
	KeyController::GetController().OnAVRCycle();

	glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
	glClearDepth(1.0f);
	glClear( US(GL_COLOR_BUFFER_BIT) | US(GL_DEPTH_BUFFER_BIT));

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glLoadIdentity();

		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		float fExtent = m_Objs->GetScaleFactor();
		m_Objs->SetupLighting();

		glEnable(GL_LIGHT0);

		glEnable(GL_LIGHTING);
		glMatrixMode(GL_MODELVIEW);
		glEnable(GL_NORMALIZE);
		glLoadIdentity();

		if (!m_bFollowNozzle)
		{
			float fSize = 0.1f;
			float fBase[3] = {0,0,0};
			glLineWidth(2.f);
			// If a mouse button is pressed, draw the "look at" axis.
			glPushMatrix();
				glEnable(GL_COLOR_MATERIAL);
				glBegin(GL_LINES);
					glColor3f(1,0,0);
					glVertex3f(fBase[0]-fSize, fBase[1], fBase[2]); glVertex3f(fBase[0]+fSize,fBase[1], fBase[2]);
					glColor3f(0,1,0);
					glVertex3f(fBase[0], fBase[1]-fSize,  fBase[2]); glVertex3f(fBase[0], fBase[1]+fSize, fBase[2]);
					glColor3f(0,0,1);
					glVertex3f(fBase[0],fBase[1], fBase[2]-fSize); glVertex3f(fBase[0],fBase[1], fBase[2]+fSize);
				glEnd();
				glDisable(GL_COLOR_MATERIAL);
			glPopMatrix();
			glMultMatrixf(m_camera.getViewMatrix());
		}

		glScalef(1.0f / fExtent, 1.0f / fExtent, 1.0f / fExtent);

		float _fTransform[3] = {0,0,0};
		gsl::span<float> fTransform {_fTransform};
		m_Objs->GetBaseCenter(fTransform);
		// Centerize object.
		glTranslatef (fTransform[0], fTransform[1], fTransform[2]);
		if (m_bFollowNozzle)
		{
				float _fLook[3] = {0,0,0};
				gsl::span<float> fLook {_fLook};
				m_Objs->GetNozzleCamPos(fLook);
				fLook[0]+=fTransform[0]=m_fXPos;
				fLook[1]+=m_fZPos;
				gluLookAt(fLook[0]+.001, fLook[1]+.003 ,fLook[2]+.08, fLook[0],fLook[1],fLook[2] ,0,1,0);
		}
		m_Objs->SetNozzleCam(m_bFollowNozzle);
		glPushMatrix();
			glTranslatef(0,m_fZPos,0);
			m_Objs->Draw(OBJCollection::ObjClass::Z);
			glPushMatrix();
				glTranslatef(m_fXPos,0,0);
				m_Objs->Draw(OBJCollection::ObjClass::X);
				if (!m_bPINDAOn)
				{
					glPushMatrix();
						m_Objs->ApplyPLEDTransform();
						DrawRoundLED();
					glPopMatrix();
				}
				glPushMatrix();
					m_iPFanPos = (m_iPFanPos + (m_iPFanPWM/10))%360;
					m_Objs->DrawPFan(m_iPFanPos);
				glPopMatrix();
				glPushMatrix();
					m_iFanPos = (m_iFanPos + (m_iFanPWM/10))%360;
					m_Objs->DrawEFan(m_iFanPos);
				glPopMatrix();
				glPushMatrix();
					m_Objs->DrawEVis(m_fEPos);
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(0,0,(m_fYPos));
			m_Objs->Draw(OBJCollection::ObjClass::Y);
			if (m_bPrintSurface)
			{
				m_Objs->Draw(OBJCollection::ObjClass::PrintSurface);
			}
			glPushMatrix();
				glScalef(1,1,-1);
				m_Objs->ApplyPrintTransform();
				for (auto &c : m_vPrints)
				{
					c->Draw();
				}
			glPopMatrix();
			if (m_bBedOn)
			{
				m_Objs->ApplyBedLEDTransform();
				DrawLED(1,0,0);
			}
		glPopMatrix();
		m_Objs->Draw(OBJCollection::ObjClass::Fixed);
		glPushMatrix();
			m_Objs->DrawKnob(m_iKnobPos);
		glPopMatrix();
		if (m_bSDCard) //if card present
		{
			m_Objs->Draw(OBJCollection::ObjClass::Media); // Draw removable media (SD, USB, etc)
		}
		if (m_Objs->SupportsMMU() && m_bMMU)
		{
			// DrawMMU();
		}
		// m_snap.OnDraw();
		glutSwapBuffers();
		m_iFrCount++;
		m_iTic=glutGet(GLUT_ELAPSED_TIME);
		auto iDiff = m_iTic - m_iLast;
		if (iDiff > 1000) {
			int iFPS = m_iFrCount*1000.f/(iDiff);
			m_iLast = m_iTic;
			m_iFrCount = 0;
			std::string strFPS = std::string("Fancy Graphics: ") + m_Objs->GetName() + " (" +std::to_string(iFPS) + " FPS)";
			glutSetWindowTitle(strFPS.c_str());
		}
		m_bDirty = false;
}

void MK3SGL::DrawRoundLED()
{
	std::vector<float> vLED = {1,0,0,1};
	std::vector<float> vNone = {0,0,0,1};
	glMaterialfv(GL_FRONT_AND_BACK, US(GL_AMBIENT_AND_DIFFUSE) | US(GL_SPECULAR) ,vNone.data());
	glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,vLED.data());
	glPushMatrix();
		glTranslatef(0.092,0.3355,0.274);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f( 0,   0,    0);
			glVertex3f(-0.003,0,0);
			glVertex3f(-0.002,0,0.002);
			glVertex3f( 0,   0,     0.003);
			glVertex3f( 0.002,0,0.002);
			glVertex3f( 0.003,  0,  0);
			glVertex3f(0.002, 0,   -0.002);
			glVertex3f( 0,   0,    -0.003);
			glVertex3f(-0.002, 0, -0.002);
			glVertex3f(-0.003,0,0);
		glEnd();
	glPopMatrix();
	glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,vNone.data());
}

void MK3SGL::DrawLED(float r, float g, float b)
{
		std::vector<float> vLED = {r,g,b,1};
		std::vector<float> vNone = {0,0,0,1};
		glMaterialfv(GL_FRONT_AND_BACK, US(GL_AMBIENT_AND_DIFFUSE) | US(GL_SPECULAR),vNone.data());
				glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,vLED.data());
				glBegin(GL_QUADS);
					glVertex3f(0,0,0);
					glVertex3f(0.004,0,0);
					glVertex3f(0.004,0.003,0.003);
					glVertex3f(0,0.003,0.003);
				glEnd();
				glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION, vNone.data());
}

void MK3SGL::MouseCB(int button, int action, int x, int y)
{
	auto w = glutGet(GLUT_WINDOW_WIDTH);
	auto h = glutGet(GLUT_WINDOW_HEIGHT);
	m_camera.setWindowSize(w, h);
	m_camera.setCurrentMousePos(x,y);
 	if (button == GLUT_LEFT_BUTTON) {
		if (action == GLUT_DOWN)
		{
			m_camera.beginRotate();
		}
		else if (action == GLUT_UP)
		{
			m_camera.endRotate();
		}
	}
	if (button == GLUT_RIGHT_BUTTON) {
		if (action == GLUT_DOWN)
		{
			m_camera.beginPan();
		}
		else if (action == GLUT_UP)
		{
			m_camera.endPan();
		}

	}
	if (button == GLUT_MIDDLE_BUTTON) {
		if (action == GLUT_DOWN)
		{
			m_camera.beginZoom();
		}
		else if (action == GLUT_UP)
		{
			m_camera.endZoom();
		}

	}
	if (button==3)
	{
		m_camera.zoom(0.5f);
	}
	if (button==4)
	{
		m_camera.zoom(-0.5f);
	}

	m_bDirty = true;
}

void MK3SGL::MotionCB(int x, int y)
{
 	m_camera.setCurrentMousePos(x, y);
	m_bDirty = true;
}
