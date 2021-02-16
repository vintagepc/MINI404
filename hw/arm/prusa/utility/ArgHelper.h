/*
	ArgHelper.h - Convenience class for turning the -append
    CLI arg into things we can use within the context of Mini404

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#ifdef __cplusplus

#include <map>
#include <string>
#include <utility>
#include <vector>

class ArgHelper
{
    public:
        void SetArgs(std::string strArgs);
        bool Parse();
        
        static ArgHelper& Get()
		{
			static ArgHelper h;
			return h;
		}

        bool IsArg(std::string strArg);

        const char* GetString(std::string strArg);

    private:
		std::map<std::string,std::string> m_map;
};

extern "C" {
#endif 
// Sets the arguments for future use. 
extern void arghelper_setargs(const char* args);

// Processes the args after startup. Return code indicates whether
// the machine should continue execution (T) or quit (F)
// e.g. after some kind of help output.  
extern bool arghelper_parseargs(void);

// Returns whether a given argument was observed. 
extern bool arghelper_is_arg(const char* arg);

// Returns the string value of the given argument. 
extern const char* arghelper_get_string(const char* arg);

#ifdef __cplusplus
}
#endif
