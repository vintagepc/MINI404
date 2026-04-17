/*
	KeyController.h - Wrangler for dispatching keypresses to KeyClients.

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

#include "../utility/IScriptable.h"
#include "Scriptable.h"
#include <atomic>
#include <map>               // for map
#include <string>            // for string
#include <vector>            // for vector

// We include this here again because I can't directly include the header from C without include hell or other C/C++ issues. 

typedef enum QKeyCode {
    Q_KEY_CODE_UNMAPPED,
    Q_KEY_CODE_SHIFT,
    Q_KEY_CODE_SHIFT_R,
    Q_KEY_CODE_ALT,
    Q_KEY_CODE_ALT_R,
    Q_KEY_CODE_CTRL,
    Q_KEY_CODE_CTRL_R,
    Q_KEY_CODE_MENU,
    Q_KEY_CODE_ESC,
    Q_KEY_CODE_1,
    Q_KEY_CODE_2,
    Q_KEY_CODE_3,
    Q_KEY_CODE_4,
    Q_KEY_CODE_5,
    Q_KEY_CODE_6,
    Q_KEY_CODE_7,
    Q_KEY_CODE_8,
    Q_KEY_CODE_9,
    Q_KEY_CODE_0,
    Q_KEY_CODE_MINUS,
    Q_KEY_CODE_EQUAL,
    Q_KEY_CODE_BACKSPACE,
    Q_KEY_CODE_TAB,
    Q_KEY_CODE_Q,
    Q_KEY_CODE_W,
    Q_KEY_CODE_E,
    Q_KEY_CODE_R,
    Q_KEY_CODE_T,
    Q_KEY_CODE_Y,
    Q_KEY_CODE_U,
    Q_KEY_CODE_I,
    Q_KEY_CODE_O,
    Q_KEY_CODE_P,
    Q_KEY_CODE_BRACKET_LEFT,
    Q_KEY_CODE_BRACKET_RIGHT,
    Q_KEY_CODE_RET,
    Q_KEY_CODE_A,
    Q_KEY_CODE_S,
    Q_KEY_CODE_D,
    Q_KEY_CODE_F,
    Q_KEY_CODE_G,
    Q_KEY_CODE_H,
    Q_KEY_CODE_J,
    Q_KEY_CODE_K,
    Q_KEY_CODE_L,
    Q_KEY_CODE_SEMICOLON,
    Q_KEY_CODE_APOSTROPHE,
    Q_KEY_CODE_GRAVE_ACCENT,
    Q_KEY_CODE_BACKSLASH,
    Q_KEY_CODE_Z,
    Q_KEY_CODE_X,
    Q_KEY_CODE_C,
    Q_KEY_CODE_V,
    Q_KEY_CODE_B,
    Q_KEY_CODE_N,
    Q_KEY_CODE_M,
    Q_KEY_CODE_COMMA,
    Q_KEY_CODE_DOT,
    Q_KEY_CODE_SLASH,
    Q_KEY_CODE_ASTERISK,
    Q_KEY_CODE_SPC,
    Q_KEY_CODE_CAPS_LOCK,
    Q_KEY_CODE_F1,
    Q_KEY_CODE_F2,
    Q_KEY_CODE_F3,
    Q_KEY_CODE_F4,
    Q_KEY_CODE_F5,
    Q_KEY_CODE_F6,
    Q_KEY_CODE_F7,
    Q_KEY_CODE_F8,
    Q_KEY_CODE_F9,
    Q_KEY_CODE_F10,
    Q_KEY_CODE_NUM_LOCK,
    Q_KEY_CODE_SCROLL_LOCK,
    Q_KEY_CODE_KP_DIVIDE,
    Q_KEY_CODE_KP_MULTIPLY,
    Q_KEY_CODE_KP_SUBTRACT,
    Q_KEY_CODE_KP_ADD,
    Q_KEY_CODE_KP_ENTER,
    Q_KEY_CODE_KP_DECIMAL,
    Q_KEY_CODE_SYSRQ,
    Q_KEY_CODE_KP_0,
    Q_KEY_CODE_KP_1,
    Q_KEY_CODE_KP_2,
    Q_KEY_CODE_KP_3,
    Q_KEY_CODE_KP_4,
    Q_KEY_CODE_KP_5,
    Q_KEY_CODE_KP_6,
    Q_KEY_CODE_KP_7,
    Q_KEY_CODE_KP_8,
    Q_KEY_CODE_KP_9,
    Q_KEY_CODE_LESS,
    Q_KEY_CODE_F11,
    Q_KEY_CODE_F12,
    Q_KEY_CODE_PRINT,
    Q_KEY_CODE_HOME,
    Q_KEY_CODE_PGUP,
    Q_KEY_CODE_PGDN,
    Q_KEY_CODE_END,
    Q_KEY_CODE_LEFT,
    Q_KEY_CODE_UP,
    Q_KEY_CODE_DOWN,
    Q_KEY_CODE_RIGHT,
    Q_KEY_CODE_INSERT,
    Q_KEY_CODE_DELETE,
    Q_KEY_CODE_STOP,
    Q_KEY_CODE_AGAIN,
    Q_KEY_CODE_PROPS,
    Q_KEY_CODE_UNDO,
    Q_KEY_CODE_FRONT,
    Q_KEY_CODE_COPY,
    Q_KEY_CODE_OPEN,
    Q_KEY_CODE_PASTE,
    Q_KEY_CODE_FIND,
    Q_KEY_CODE_CUT,
    Q_KEY_CODE_LF,
    Q_KEY_CODE_HELP,
    Q_KEY_CODE_META_L,
    Q_KEY_CODE_META_R,
    Q_KEY_CODE_COMPOSE,
    Q_KEY_CODE_PAUSE,
    Q_KEY_CODE_RO,
    Q_KEY_CODE_HIRAGANA,
    Q_KEY_CODE_HENKAN,
    Q_KEY_CODE_YEN,
    Q_KEY_CODE_MUHENKAN,
    Q_KEY_CODE_KATAKANAHIRAGANA,
    Q_KEY_CODE_KP_COMMA,
    Q_KEY_CODE_KP_EQUALS,
    Q_KEY_CODE_POWER,
    Q_KEY_CODE_SLEEP,
    Q_KEY_CODE_WAKE,
    Q_KEY_CODE_AUDIONEXT,
    Q_KEY_CODE_AUDIOPREV,
    Q_KEY_CODE_AUDIOSTOP,
    Q_KEY_CODE_AUDIOPLAY,
    Q_KEY_CODE_AUDIOMUTE,
    Q_KEY_CODE_VOLUMEUP,
    Q_KEY_CODE_VOLUMEDOWN,
    Q_KEY_CODE_MEDIASELECT,
    Q_KEY_CODE_MAIL,
    Q_KEY_CODE_CALCULATOR,
    Q_KEY_CODE_COMPUTER,
    Q_KEY_CODE_AC_HOME,
    Q_KEY_CODE_AC_BACK,
    Q_KEY_CODE_AC_FORWARD,
    Q_KEY_CODE_AC_REFRESH,
    Q_KEY_CODE_AC_BOOKMARKS,
    Q_KEY_CODE_LANG1,
    Q_KEY_CODE_LANG2,
    Q_KEY_CODE_F13,
    Q_KEY_CODE_F14,
    Q_KEY_CODE_F15,
    Q_KEY_CODE_F16,
    Q_KEY_CODE_F17,
    Q_KEY_CODE_F18,
    Q_KEY_CODE_F19,
    Q_KEY_CODE_F20,
    Q_KEY_CODE_F21,
    Q_KEY_CODE_F22,
    Q_KEY_CODE_F23,
    Q_KEY_CODE_F24,
    Q_KEY_CODE__MAX,
} QKeyCode;

class IKeyClient;

class KeyController: private Scriptable
{
	friend IKeyClient;

	public:
		static KeyController& GetController();

		// Called by the key handler to notify a key was pressed.
		inline void OnKeyPressed(unsigned char key) { m_key.store(key); };

        void OnKeyPressed_C(int keycode, bool bDown);

        // Keep unique clients so they can be cleaned up later.
        void AddNewClient_C(IKeyClient* src);

		// Called by the printer so key events happen "safely" on AVR cycles.
		void OnAVRCycle();

		void PrintKeys(bool bMarkdown);

		static inline void GLKeyReceiver(unsigned char key, int /*x*/, int /*y*/) { KeyController::GetController().OnKeyPressed(key); };

	protected:
		KeyController();
		~KeyController();

		// Invoked by IKeyClient to add a client.
		void AddKeyClient(IKeyClient *pClient, const unsigned char key, const std::string &strDesc);

		LineStatus ProcessAction(unsigned int iAction, const std::vector<std::string> &args) override;

	private:
		void PutNiceKeyName(unsigned char key);

		std::map<unsigned char, std::vector<IKeyClient*> > m_mClients {};
		std::map<unsigned char, std::string> m_mDescrs {};
        std::vector<IKeyClient*> m_vAllClients {};
        // tuple is code, shift, release
        using KeyTuple = std::tuple<unsigned char, bool, bool>;
        std::map<KeyTuple, unsigned char> m_qemu2char
        {
            { {Q_KEY_CODE_S         ,true,  false} , 'S'},
			{ {Q_KEY_CODE_W         ,false, true} , 'w'|0x80 },
			{ {Q_KEY_CODE_S         ,false, true} , 's'|0x80 },
            { {Q_KEY_CODE_W         ,false, false} , 'w'}, // shared with arrow keys for up/down
            { {Q_KEY_CODE_UP        ,false, false} , 'w'}, // shared with arrow keys for up/down
            { {Q_KEY_CODE_S         ,false, false} , 's'},
            { {Q_KEY_CODE_DOWN      ,false, false} , 's'},
            { {Q_KEY_CODE_RET       ,false, false} ,  0xd},
            { {Q_KEY_CODE_KP_ENTER  ,false, false} ,  0xd},
			{ {Q_KEY_CODE_F	        ,false, false} , 'f'},
			{ {Q_KEY_CODE_1         ,false, false}, '1'},
			{ {Q_KEY_CODE_2         ,false, false}, '2'},
			{ {Q_KEY_CODE_3         ,false, false}, '3'},
			{ {Q_KEY_CODE_4         ,false, false}, '4'},
			{ {Q_KEY_CODE_5         ,false, false}, '5'},
			{ {Q_KEY_CODE_6         ,false, false}, '6'},
			{ {Q_KEY_CODE_P         ,false, false} , 'p'},
			{ {Q_KEY_CODE_T	        ,false, false} , 't'},
			{ {Q_KEY_CODE_L	        ,false, false} , 'l'},
            { {Q_KEY_CODE_Z	        ,false, false} , 'z'},
            { {Q_KEY_CODE_X	        ,false, false} , 'x'},
            { {Q_KEY_CODE_C	        ,false, false} , 'c'},
            { {Q_KEY_CODE_V	        ,false, false} , 'v'},
            { {Q_KEY_CODE_B	        ,false, false} , 'b'},
            { {Q_KEY_CODE_M	        ,false, false} , 'm'},
        };
		std::atomic_uchar m_key {0};
        bool m_bShift = false;

};
