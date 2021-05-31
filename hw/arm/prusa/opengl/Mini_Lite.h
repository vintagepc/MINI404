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

#include "OBJCollection.h"
#include "../3rdParty/gsl-lite.hpp"
#include <GL/glew.h>
#include <memory>

class GLObj;

class Mini_Lite: public OBJCollection
{
	public:
		explicit Mini_Lite(bool bLiteExtruder = true);


		void OnLoadComplete() override;

		void SetupLighting() override;

		inline bool SupportsMMU() override { return false; }

		inline void ApplyLCDTransform() override { glTranslatef(0.101,0.0549,0.4925); }

		inline void ApplyPLEDTransform() override {glTranslatef(-0.273,-0.278,-0.139f);};

		inline void ApplyBedLEDTransform() override {glTranslatef( 0.013000, 0.020000, -0.056000);};

		inline void ApplyPrintTransform() override { glTranslatef(-0.151000, 0.021000, -0.131500); };

		// void GetBaseCenter(gsl::span<float> fTrans) override;

		float GetScaleFactor() override { return 0.210874f; }

		void DrawKnob(int iRotation) override;

		inline void GetNozzleCamPos(gsl::span<float> fPos) override
		{
			fPos[0] = -.155f;
			fPos[1] = -0.127f;
			fPos[2] = -0.024f;
		}

		void DrawEVis(float fEPos) override;

		void DrawEFan(int iRotation) override;

		void DrawPFan(int iRotation) override;

	protected:

		std::shared_ptr<GLObj> m_pKnob = nullptr, m_pEFan = nullptr, m_pPFan = nullptr, m_pEVis = nullptr, m_pE = nullptr;

};
