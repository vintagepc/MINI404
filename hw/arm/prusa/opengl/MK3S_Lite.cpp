/*
	MK3S_Lite.h - Object collection for the "lite" visuals.

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

#include "MK3S_Lite.h"
#include "GLObj.h"
#include "OBJCollection.h"
#include <map>              // for map
#include <string>           // for string
#include <vector>           // for vector

constexpr float ZCorrect = -0.179;

MK3S_Lite::MK3S_Lite(bool /*bMMU*/):OBJCollection("Lite")
{
	constexpr float XCorrect = -0.092;

	AddObject(ObjClass::Y, "assets/Mini_Y.obj", MM_TO_M);
	m_mObjs.at(ObjClass::Y).at(0).get()->ForceDissolveTo1(true);
	m_mObjs.at(ObjClass::Y).at(0).get()->SetReverseWinding(true);
	m_mObjs.at(ObjClass::Y).at(0).get()->SetSwapMode(GLObj::SwapMode::YMINUSZ);
	AddObject(ObjClass::Z, "assets/Mini_X.obj",0,ZCorrect,0, MM_TO_M);
	m_mObjs.at(ObjClass::Z).at(0).get()->ForceDissolveTo1(true);
	m_mObjs.at(ObjClass::Z).at(0).get()->SetReverseWinding(true);
	m_mObjs.at(ObjClass::Z).at(0).get()->SetSwapMode(GLObj::SwapMode::YMINUSZ);
	AddObject(ObjClass::PrintSurface, "assets/Mini_Sheet.obj", MM_TO_M);
	m_mObjs.at(ObjClass::PrintSurface).at(0).get()->ForceDissolveTo1(true);
	m_mObjs.at(ObjClass::PrintSurface).at(0).get()->SetReverseWinding(true);
	m_mObjs.at(ObjClass::PrintSurface).at(0).get()->SetSwapMode(GLObj::SwapMode::YMINUSZ);
	//AddObject(ObjClass::Media, "assets/SDCard.obj",0,0,0,MM_TO_M)->SetKeepNormalsIfScaling(true);
	m_pE = AddObject(ObjClass::X, "assets/Mini_Extruder.obj",XCorrect,ZCorrect,0, MM_TO_M);
	m_pE->ForceDissolveTo1(true);
	m_pE->SetReverseWinding(true);
	m_pE->SetSwapMode(GLObj::SwapMode::YMINUSZ);
	//m_pKnob = AddObject(ObjClass::Other, "assets/LCD-knobR2.obj");
	m_pPFan = AddObject(ObjClass::Other, "assets/Mini_PFan.obj",XCorrect,ZCorrect,0, MM_TO_M);
	m_pPFan->ForceDissolveTo1(true);
	m_pPFan->SetReverseWinding(true);
	m_pPFan->SetSwapMode(GLObj::SwapMode::YMINUSZ);
	m_pEFan = AddObject(ObjClass::Other, "assets/Mini_EFan.obj",XCorrect,ZCorrect,0,MM_TO_M);
	m_pEFan->ForceDissolveTo1(true);
	m_pEFan->SetReverseWinding(true);
	m_pEFan->SetSwapMode(GLObj::SwapMode::YMINUSZ);
	//m_pEFan->SetKeepNormalsIfScaling(true);
	m_pEVis = AddObject(ObjClass::Other,"assets/Triangles.obj",MM_TO_M);
	m_pEVis->SetKeepNormalsIfScaling(true);
	m_pBaseObj = AddObject(ObjClass::Fixed, "assets/Mini_Z.obj",MM_TO_M);
	m_pBaseObj->ForceDissolveTo1(true);
	m_pBaseObj->SetReverseWinding(true);
	m_pBaseObj->SetSwapMode(GLObj::SwapMode::YMINUSZ);
};

void MK3S_Lite::OnLoadComplete()
{
	// m_mObjs.at(ObjClass::Y).at(0)->SetAllVisible(false);
	// m_mObjs.at(ObjClass::Y).at(0)->SetSubobjectVisible(2); // heatbed, sheet
	// auto pExtruder = m_mObjs.at(ObjClass::X).at(0);
	// pExtruder->SetAllVisible(false);
	// pExtruder->SetSubobjectVisible(19); // V6
	// //pExtruder->SetSubobjectVisible(20);
	// pExtruder->SetSubobjectVisible(1); // PINDA
	// pExtruder->SetSubobjectVisible(2);
}

void MK3S_Lite::SetupLighting()
{
	float fAmb[] = {.7,.7,.7,1};
	float fSpec[] = {.4,.4,.4,.5};
	float fDiff[] = {1,1,1,1};
	float fPos[] = {2,2,2,0};
	glLightfv(GL_LIGHT0,GL_AMBIENT, 	static_cast<float*>(fAmb));
	glLightfv(GL_LIGHT0,GL_SPECULAR, 	static_cast<float*>(fSpec));
	glLightfv(GL_LIGHT0,GL_DIFFUSE, 	static_cast<float*>(fDiff));
	glLightfv(GL_LIGHT0,GL_POSITION, 	static_cast<float*>(fPos));
}

// void MK3S_Lite::GetBaseCenter(gsl::span<float> fTrans)
// {
// 	// Values stolen from the full model so we don't have to load the frame:
// 	// m_mObjs.at(ObjClass::Y).G
// 	fTrans[0] = -0.154;
// 	fTrans[1] = -0.204;
// 	fTrans[2] = -0.3134;
// }

void MK3S_Lite::DrawKnob(int iRotation)
{
	if (m_pKnob != nullptr)
	{
		glPushMatrix();
			glTranslatef(0.215,0.051,0.501);
			glRotatef(-45.f,1,0,0);
			glPushMatrix();
				glRotatef(static_cast<float>(iRotation),0,0,1);
				m_pKnob->Draw();
			glPopMatrix();
		glPopMatrix();
	}
}

void MK3S_Lite::DrawEVis(float fEPos)
{
	float fTransform[3];
	glTranslatef(-0.044,-0.210,0.f);
	m_pEVis->GetCenteringTransform(fTransform);
	fTransform[1] +=.0015f;
	glTranslatef (-fTransform[0] , -fTransform[1], -fTransform[2]);
	glRotatef((-36.f/28.f)*6.f*(fEPos*1000.f),0,0,1);
	glTranslatef (fTransform[0], fTransform[1], fTransform[2]);
	m_pEVis->Draw();
}

void MK3S_Lite::DrawEFan(int iRotation)
{
	float fTransform[3];
	m_pEFan->GetCenteringTransform(fTransform);
	glPushMatrix();
		glTranslatef (-fTransform[0], -fTransform[1] + ZCorrect, -fTransform[2]);
		glRotatef(static_cast<float>(iRotation),1,0,0);
		glTranslatef (fTransform[0], fTransform[1] - ZCorrect, fTransform[2]);
		m_pEFan->Draw();
	glPopMatrix();
}

void MK3S_Lite::DrawPFan(int iRotation)
{
	float fTransform[3];
	m_pPFan->GetCenteringTransform(fTransform);
	glPushMatrix();
		glTranslatef (-fTransform[0], -fTransform[1] + ZCorrect , -fTransform[2]);
		glRotatef(static_cast<float>(iRotation),1,0,0);
		glTranslatef (fTransform[0], fTransform[1] - ZCorrect , fTransform[2]);
		m_pPFan->Draw();
	glPopMatrix();
}
