/*
	KeyController.cpp - Wrangler for dispatching keypresses to KeyClients.

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

#include "KeyController.h"
#include "IKeyClient.h"
#include <iostream>
#include <scoped_allocator>  // for allocator_traits<>::value_type
#include <string>
#include <utility>
#include <vector>

KeyController& KeyController::GetController()
{
	static KeyController k {};
	return k;
}

extern "C"
{
	extern void p404_keyctl_handle_key(int keycode)
	{
		KeyController::GetController().OnKeyPressed_C(keycode);
	}
}


void KeyController::OnKeyPressed_C(int keycode)
{
	// Maps qemu keys to standard character types:
	if (keycode == 42) 
	{
		m_bShift = true;
	}
	else if (keycode == 170)
	{
		m_bShift = false;
	}	
	else if (!m_qemu2char.count({keycode, m_bShift}))
	{
		// std::cout << "KeyController: Unknown QEMU keycode " << std::to_string(keycode) << "\n";
	}
	else if (m_mClients.count(m_qemu2char.at({keycode, m_bShift})))
	{
		auto key = m_qemu2char.at({keycode, m_bShift});
		for (auto c : m_mClients.at(key))
		{
			if (c->IsP404KeyInput()) // This can happen here because it's already the same thread, and the GL event loop may not be running.
			{
				c->OnKeyPress(key);
			}
		}
		KeyController::GetController().OnKeyPressed(m_qemu2char.at({keycode, m_bShift}));
	}
}

KeyController::KeyController():Scriptable("KeyCtl", false)
{
	RegisterAction("Key","Simulates a keypress",0, {ArgType::String});
}

IScriptable::LineStatus KeyController::ProcessAction(unsigned int /*iAction*/, const std::vector<std::string> &vArgs)
{
	Key key = vArgs.at(0).at(0);
	if (vArgs.at(0)=="enter")
	{
		key = 0xd;
	}
	OnKeyPressed(key);
	return LineStatus::Finished;
}

void KeyController::AddKeyClient(IKeyClient *pClient, const unsigned char key, const std::string &strDesc)
{
	m_mClients[key].push_back(pClient);
	if (!strDesc.empty())
	{
		m_mDescrs[key].append(strDesc + " ");
	}
}

void KeyController::PrintKeys(bool bMarkdown)
{
	std::cout << "Available Key Controls:\n";
	for (auto it : m_mDescrs)
	{
		std::cout << (bMarkdown?"- `":"\t");
		PutNiceKeyName(it.first);
		std::cout << (bMarkdown?"` - ":":\t") << it.second << '\n';
	}

}

void KeyController::PutNiceKeyName(unsigned char key)
{
	switch(key)
	{
		case 0xd:
			std::cout << "Enter";
			break;
		default:
			std::cout << key;
			break;
	}
}

void KeyController::OnAVRCycle()
{
	auto key = m_key.load();

	if (key==0)
	{
		return;
	}

	m_key.store(0);

	if (m_mClients.count(key)>0)
	{
		std::vector<IKeyClient*>& vClients = m_mClients.at(key);
		for (auto c: vClients)
		{
			if (!c->IsP404KeyInput())
			{
				c->OnKeyPress(key);
			}
		}
	}
}
