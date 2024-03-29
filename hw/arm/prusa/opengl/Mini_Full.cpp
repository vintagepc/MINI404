/*
	Mini_Full.h - Object collection for the "lite" visuals.

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

#include "Mini_Full.h"
#include "GLObj.h"
#include "OBJCollection.h"
#include <map>              // for map
#include <string>           // for string
#include <vector>           // for vector

constexpr float ZCorrect = -0.179;

Mini_Full::Mini_Full():Mini_Lite(false)
{
	AddObject(ObjClass::Y, "assets/Mini_Y.obj", MM_TO_M);
	m_mObjs.at(ObjClass::Y).at(0).get()->ForceDissolveTo1(true);
	m_mObjs.at(ObjClass::Y).at(0).get()->SetReverseWinding(true);
	m_mObjs.at(ObjClass::Y).at(0).get()->SetSwapMode(GLObj::SwapMode::YMINUSZ);
	AddObject(ObjClass::Z, "assets/Mini_X.obj",0,ZCorrect,0, MM_TO_M);
	m_mObjs.at(ObjClass::Z).at(0).get()->ForceDissolveTo1(true);
	m_mObjs.at(ObjClass::Z).at(0).get()->SetReverseWinding(true);
	m_mObjs.at(ObjClass::Z).at(0).get()->SetSwapMode(GLObj::SwapMode::YMINUSZ);
};

void Mini_Full::OnLoadComplete()
{

}
