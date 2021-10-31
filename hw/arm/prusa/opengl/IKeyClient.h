/*
	IKeyClient.h - Mixin interface class for components/objects that have
	key actions. To use, mix in this class and call AddKeyControl()
	for each key you wish to get notified of. Multiple handlers can
	act on the same key by design.

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

#include <string>


class KeyController;

using Key = unsigned char;
using P404KeyIF = struct P404KeyIF;


class IKeyClient
{
	friend KeyController;

    public: 
        explicit IKeyClient(P404KeyIF* src = nullptr):m_pObj(src){};
        virtual ~IKeyClient() = default;

        inline void OnKeyPress_C(const Key& uiKey) { OnKeyPress(uiKey); }

        inline void RegisterKeyHandler_C(const Key& uiKey, const std::string strDesc)
        {
            RegisterKeyHandler(uiKey, strDesc);
        }

        inline bool IsP404KeyInput(){ return m_pObj != nullptr; }

	protected:
		virtual void OnKeyPress(const Key &uiKey);

		void RegisterKeyHandler(const Key uiKey, const std::string &strDesc);

        P404KeyIF* m_pObj = nullptr;
};

