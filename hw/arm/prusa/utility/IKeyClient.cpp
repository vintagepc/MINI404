/*
	IKeyClient.cpp - Mixin interface class for components/objects that have
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

#include "IKeyClient.h"
#include "KeyController.h"
#include <string>
#include "../utility/p404_keyclient.h"

void IKeyClient::RegisterKeyHandler(const Key uiKey, const std::string &strDesc)
{
	KeyController::GetController().AddKeyClient(this, uiKey, strDesc);
};


extern "C"
{

    extern p404_key_handle p404_new_keyhandler(P404KeyIF* src){
        auto p = new IKeyClient(src);
		KeyController::GetController().AddNewClient_C(p);
		return p;
    }

    extern void p404_register_keyhandler(p404_key_handle src, const Key key, const char* description)
    {
        IKeyClient* p = static_cast<IKeyClient*>(src);
        p->RegisterKeyHandler_C(key, description);
    }

	extern void p404_call_keyfunc(P404KeyIF *dst, const Key key);
}


void IKeyClient::OnKeyPress(const Key &uiKey) {
	if (m_pObj)
	{
		p404_call_keyfunc(m_pObj, uiKey);
	}
}
