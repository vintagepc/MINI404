/*
	ScriptHost_C.h - C bridge header for C++ ScriptHost/IScriptable classes. 

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

typedef P404ScriptIF P404ScriptIF;

// Registers a script action for the given src. 
extern void scripthost_register_scriptable(void *src);

// gets a new scripting context for an object. 
extern void* script_instance_new(P404ScriptIF *src, const char *strName);

// Registers an action with the scripting context. 
extern bool script_register_action(void *src, const char* strAction, const char* strDesc, int iID);

// Adds an argument of the given type to the specified action.
extern void script_add_arg_int(void *src, int iID);
extern void script_add_arg_string(void *src, int iID);

extern int scripthost_get_int(const void *pArgs, uint8_t iIdx);

// returns pointer to C string of the arg. If you want to keep it for later you MUST copy it!
extern const char* scripthost_get_string(const void *pArgs, uint8_t iIdx);


// Initializes the script host with the given script. 
extern bool scripthost_setup(const char* strScript);

// Runs one cycle of script processing. 
extern void scripthost_run(int64_t iTime);
// adds an argument requirement to the given action ID.
// static void scripthost_add_action_arg(P404ScriptIF *src, int id, int argtype);
