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

#pragma once

#include "Mini_Lite.h"
#include "OBJCollection.h"
#include "../3rdParty/gsl-lite.hpp"
#include <GL/glew.h>
#include <memory>

class GLObj;

class Mini_Full: public Mini_Lite
{
	public:
		explicit Mini_Full();


		void OnLoadComplete() override;
};
